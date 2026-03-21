#!/usr/bin/env python3
"""
S2S Trot 数据收集与可视化脚本
收集策略输入输出数据并绘制曲线
"""

import math
import numpy as np
import mujoco
import mujoco.viewer
from collections import deque
from scipy.spatial.transform import Rotation as R
from pathlib import Path
import onnxruntime as ort
import argparse
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import time

from joystick_interface import JoystickInterface

# ---------------------------------------------------------------------------- #
#                               Remapping Indices                              #
# ---------------------------------------------------------------------------- #

sim2policy_indices = np.array(
    [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11], dtype=np.int64
)
policy2sim_indices = np.array(
    [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11], dtype=np.int64
)

# ---------------------------------------------------------------------------- #
#                                 Configuration                                #
# ---------------------------------------------------------------------------- #


class Sim2simCfg:
    class sim_config:
        base_dir = Path(__file__).resolve().parent.parent  # simulation -> dog
        mujoco_model_path = (base_dir / "leggedrobot_flat.xml").as_posix()
        sim_duration = 120.0
        dt = 0.005
        decimation = 4

    class robot_config:
        num_actions = 12
        default_dof_pos = np.zeros(12, dtype=np.double)
        kps = np.full(12, 25.0, dtype=np.double)
        kds = np.full(12, 0.5, dtype=np.double)
        tau_limit = np.array([17, 17, 25] * 4, dtype=np.double)

    class normalization:
        class isaac_obs_scales:
            lin_vel = 1.0
            ang_vel = 1.0
            projected_gravity = 1.0
            commands = 1.0
            joint_pos = 1.0
            joint_vel = 1.0
            actions = 1.0

        clip_observations = 100.0
        clip_actions = 100.0

    class env:
        frame_stack = 10
        num_single_obs = 45

    class control:
        action_scale = 0.25


# ---------------------------------------------------------------------------- #
#                               Data Collector                                 #
# ---------------------------------------------------------------------------- #


class DataCollector:
    """数据收集器，记录所有输入输出"""

    def __init__(self, max_samples=10000):
        self.max_samples = max_samples
        self.reset()

    def reset(self):
        self.timestamps = []
        # 观测数据
        self.omega = []  # 角速度 (3)
        self.gravity = []  # 重力投影 (3)
        self.commands = []  # 命令 (3)
        self.joint_pos = []  # 关节位置 (12)
        self.joint_vel = []  # 关节速度 (12)
        self.last_action = []  # 上一帧动作 (12)
        # 输出数据
        self.action = []  # 策略输出动作 (12)
        self.target_q = []  # 目标关节位置 (12)
        self.tau = []  # 扭矩 (12)
        # 状态数据
        self.base_pos = []  # 基座位置 (3)
        self.base_quat = []  # 基座四元数 (4)
        self.base_vel = []  # 基座速度 (3)
        self.foot_contact = []  # 足端接触 (4)

    def record(
        self,
        t,
        omega,
        gravity,
        cmd,
        joint_pos,
        joint_vel,
        last_action,
        action,
        target_q,
        tau,
        base_pos,
        base_quat,
        base_vel,
        foot_contact,
    ):
        if len(self.timestamps) >= self.max_samples:
            return
        self.timestamps.append(t)
        self.omega.append(omega.copy())
        self.gravity.append(gravity.copy())
        self.commands.append(cmd.copy())
        self.joint_pos.append(joint_pos.copy())
        self.joint_vel.append(joint_vel.copy())
        self.last_action.append(last_action.copy())
        self.action.append(action.copy())
        self.target_q.append(target_q.copy())
        self.tau.append(tau.copy())
        self.base_pos.append(base_pos.copy())
        self.base_quat.append(base_quat.copy())
        self.base_vel.append(base_vel.copy())
        self.foot_contact.append(foot_contact.copy())

    def to_numpy(self):
        """转换为numpy数组"""
        return {
            "timestamps": np.array(self.timestamps),
            "omega": np.array(self.omega),
            "gravity": np.array(self.gravity),
            "commands": np.array(self.commands),
            "joint_pos": np.array(self.joint_pos),
            "joint_vel": np.array(self.joint_vel),
            "last_action": np.array(self.last_action),
            "action": np.array(self.action),
            "target_q": np.array(self.target_q),
            "tau": np.array(self.tau),
            "base_pos": np.array(self.base_pos),
            "base_quat": np.array(self.base_quat),
            "base_vel": np.array(self.base_vel),
            "foot_contact": np.array(self.foot_contact),
        }


# ---------------------------------------------------------------------------- #
#                               Util Functions                                 #
# ---------------------------------------------------------------------------- #


def get_gravity_orientation(quat):
    r = R.from_quat(quat)
    gravity_vec = np.array([0.0, 0.0, -1.0])
    return r.apply(gravity_vec, inverse=True)


def get_obs(data):
    q_sim = data.qpos[7:].astype(np.double)
    dq_sim = data.qvel[6:].astype(np.double)
    q_policy = q_sim[sim2policy_indices]
    dq_policy = dq_sim[sim2policy_indices]
    mj_quat = data.qpos[3:7]
    quat = np.array([mj_quat[1], mj_quat[2], mj_quat[3], mj_quat[0]])
    omega = data.sensor("angular-velocity").data.astype(np.double)
    return q_policy, dq_policy, quat, omega


def pd_control(target_q, q, kp, target_dq, dq, kd, default_pos):
    return (target_q - q) * kp + (target_dq - dq) * kd


# ---------------------------------------------------------------------------- #
#                              Plotting Functions                              #
# ---------------------------------------------------------------------------- #


def plot_data(data, save_path=None):
    """绘制所有数据曲线"""
    t = data["timestamps"]
    if len(t) == 0:
        print("No data to plot!")
        return

    joint_names = [
        "LF_HipA",
        "LR_HipA",
        "RF_HipA",
        "RR_HipA",
        "LF_HipF",
        "LR_HipF",
        "RF_HipF",
        "RR_HipF",
        "LF_Knee",
        "LR_Knee",
        "RF_Knee",
        "RR_Knee",
    ]

    fig = plt.figure(figsize=(20, 16))
    gs = GridSpec(5, 3, figure=fig, hspace=0.3, wspace=0.3)

    # 1. 基座位置
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(t, data["base_pos"][:, 0], label="X", color="r")
    ax1.plot(t, data["base_pos"][:, 1], label="Y", color="g")
    ax1.plot(t, data["base_pos"][:, 2], label="Z", color="b")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Position (m)")
    ax1.set_title("Base Position")
    ax1.legend()
    ax1.grid(True)

    # 2. 基座速度
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t, data["base_vel"][:, 0], label="Vx", color="r")
    ax2.plot(t, data["base_vel"][:, 1], label="Vy", color="g")
    ax2.plot(t, data["base_vel"][:, 2], label="Vz", color="b")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Velocity (m/s)")
    ax2.set_title("Base Velocity (Body Frame)")
    ax2.legend()
    ax2.grid(True)

    # 3. 命令 vs 实际速度
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(t, data["commands"][:, 0], label="Cmd X", color="r", linestyle="--")
    ax3.plot(t, data["base_vel"][:, 0], label="Real X", color="r")
    ax3.plot(t, data["commands"][:, 1], label="Cmd Y", color="g", linestyle="--")
    ax3.plot(t, data["base_vel"][:, 1], label="Real Y", color="g")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Velocity (m/s)")
    ax3.set_title("Command vs Real Velocity")
    ax3.legend()
    ax3.grid(True)

    # 4. 角速度
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.plot(t, data["omega"][:, 0], label="ωx", color="r")
    ax4.plot(t, data["omega"][:, 1], label="ωy", color="g")
    ax4.plot(t, data["omega"][:, 2], label="ωz", color="b")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Angular Velocity (rad/s)")
    ax4.set_title("Angular Velocity")
    ax4.legend()
    ax4.grid(True)

    # 5. 重力投影
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.plot(t, data["gravity"][:, 0], label="gx", color="r")
    ax5.plot(t, data["gravity"][:, 1], label="gy", color="g")
    ax5.plot(t, data["gravity"][:, 2], label="gz", color="b")
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Gravity Projection")
    ax5.set_title("Projected Gravity")
    ax5.legend()
    ax5.grid(True)

    # 6. 足端接触
    ax6 = fig.add_subplot(gs[1, 2])
    foot_names = ["LF", "LR", "RF", "RR"]
    colors = ["r", "g", "b", "orange"]
    for i, (name, color) in enumerate(zip(foot_names, colors)):
        ax6.plot(t, data["foot_contact"][:, i], label=name, color=color)
    ax6.set_xlabel("Time (s)")
    ax6.set_ylabel("Contact")
    ax6.set_title("Foot Contact")
    ax6.legend()
    ax6.grid(True)

    # 7-9. 关节位置 (分三组)
    for group_idx, (start, end, title) in enumerate(
        [
            (0, 4, "HipA Joint Position"),
            (4, 8, "HipF Joint Position"),
            (8, 12, "Knee Joint Position"),
        ]
    ):
        ax = fig.add_subplot(gs[2, group_idx])
        for i in range(start, end):
            ax.plot(t, data["joint_pos"][:, i], label=joint_names[i])
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (rad)")
        ax.set_title(title)
        ax.legend(fontsize=6)
        ax.grid(True)

    # 10-12. 关节速度
    for group_idx, (start, end, title) in enumerate(
        [
            (0, 4, "HipA Joint Velocity"),
            (4, 8, "HipF Joint Velocity"),
            (8, 12, "Knee Joint Velocity"),
        ]
    ):
        ax = fig.add_subplot(gs[3, group_idx])
        for i in range(start, end):
            ax.plot(t, data["joint_vel"][:, i], label=joint_names[i])
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (rad/s)")
        ax.set_title(title)
        ax.legend(fontsize=6)
        ax.grid(True)

    # 13-15. 扭矩
    for group_idx, (start, end, title) in enumerate(
        [
            (0, 4, "HipA Torque"),
            (4, 8, "HipF Torque"),
            (8, 12, "Knee Torque"),
        ]
    ):
        ax = fig.add_subplot(gs[4, group_idx])
        for i in range(start, end):
            ax.plot(t, data["tau"][:, i], label=joint_names[i])
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Torque (Nm)")
        ax.set_title(title)
        ax.legend(fontsize=6)
        ax.grid(True)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Figure saved to {save_path}")

    plt.show()


def plot_action_analysis(data, save_path=None):
    """分析策略动作输出"""
    t = data["timestamps"]
    if len(t) == 0:
        print("No data to plot!")
        return

    joint_names = [
        "LF_HipA",
        "LR_HipA",
        "RF_HipA",
        "RR_HipA",
        "LF_HipF",
        "LR_HipF",
        "RF_HipF",
        "RR_HipF",
        "LF_Knee",
        "LR_Knee",
        "RF_Knee",
        "RR_Knee",
    ]

    fig, axes = plt.subplots(3, 4, figsize=(16, 10))
    fig.suptitle("Policy Action Output Analysis", fontsize=14)

    for i, ax in enumerate(axes.flat):
        if i < 12:
            ax.plot(t, data["action"][:, i], label="Action", color="blue", alpha=0.7)
            ax.plot(
                t, data["target_q"][:, i], label="Target Q", color="red", alpha=0.7
            )
            ax.plot(
                t, data["joint_pos"][:, i], label="Actual Q", color="green", alpha=0.7
            )
            ax.set_title(joint_names[i])
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Position (rad)")
            ax.legend(fontsize=6)
            ax.grid(True)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Figure saved to {save_path}")

    plt.show()


def print_data_summary(data):
    """打印数据统计摘要"""
    print("\n" + "=" * 60)
    print("Data Summary")
    print("=" * 60)
    print(f"Total samples: {len(data['timestamps'])}")
    print(f"Duration: {data['timestamps'][-1]:.2f} s" if len(data['timestamps']) > 0 else "No data")

    print("\n--- Base Position ---")
    print(f"X: min={data['base_pos'][:, 0].min():.3f}, max={data['base_pos'][:, 0].max():.3f}")
    print(f"Y: min={data['base_pos'][:, 1].min():.3f}, max={data['base_pos'][:, 1].max():.3f}")
    print(f"Z: min={data['base_pos'][:, 2].min():.3f}, max={data['base_pos'][:, 2].max():.3f}")

    print("\n--- Base Velocity (Body Frame) ---")
    print(f"Vx: mean={data['base_vel'][:, 0].mean():.3f}, std={data['base_vel'][:, 0].std():.3f}")
    print(f"Vy: mean={data['base_vel'][:, 1].mean():.3f}, std={data['base_vel'][:, 1].std():.3f}")

    print("\n--- Commands ---")
    print(f"Cmd X: mean={data['commands'][:, 0].mean():.3f}")
    print(f"Cmd Y: mean={data['commands'][:, 1].mean():.3f}")
    print(f"Cmd Yaw: mean={data['commands'][:, 2].mean():.3f}")

    print("\n--- Joint Torque (Nm) ---")
    print(f"Max torque: {np.abs(data['tau']).max():.2f}")
    print(f"Mean torque: {np.abs(data['tau']).mean():.2f}")


# ---------------------------------------------------------------------------- #
#                                 Main Loop                                    #
# ---------------------------------------------------------------------------- #


def run_mujoco(onnx_session, cfg, collect_data=True, max_samples=5000):
    joy = JoystickInterface(device_path="/dev/input/js0", max_v_x=2.0, max_v_y=1.0, max_omega=1.5)

    model = mujoco.MjModel.from_xml_path(cfg.sim_config.mujoco_model_path)
    model.opt.timestep = cfg.sim_config.dt
    data = mujoco.MjData(model)

    data.qpos[7:] = cfg.robot_config.default_dof_pos
    mujoco.mj_step(model, data)

    # 数据收集器
    collector = DataCollector(max_samples=max_samples) if collect_data else None

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -45
        viewer.cam.lookat[:] = np.array([0.0, -0.25, 0.824])

        action_policy = np.zeros(cfg.robot_config.num_actions, dtype=np.double)
        target_q_sim = np.zeros(cfg.robot_config.num_actions, dtype=np.double)

        hist_obs = deque(maxlen=cfg.env.frame_stack)
        for _ in range(cfg.env.frame_stack):
            hist_obs.append(np.zeros(cfg.env.num_single_obs, dtype=np.float32))

        count_lowlevel = 0
        scales = cfg.normalization.isaac_obs_scales

        # 记录上一帧action用于收集
        last_action_for_record = np.zeros(12, dtype=np.float32)

        while viewer.is_running():
            q_policy, dq_policy, quat, omega = get_obs(data)

            vel_world = data.qvel[:3]
            r_temp = R.from_quat(quat)
            vel_body = r_temp.apply(vel_world, inverse=True)

            # 获取足端接触状态
            foot_contact = np.array([
                data.sensor("LF_touch").data[0] if data.sensor("LF_touch") else 0,
                data.sensor("LR_touch").data[0] if data.sensor("LR_touch") else 0,
                data.sensor("RF_touch").data[0] if data.sensor("RF_touch") else 0,
                data.sensor("RR_touch").data[0] if data.sensor("RR_touch") else 0,
            ])

            if count_lowlevel % cfg.sim_config.decimation == 0:
                obs_list = []
                obs_list.append(omega * scales.ang_vel)
                gravity = get_gravity_orientation(quat) * scales.projected_gravity
                obs_list.append(gravity)

                cmd_x, cmd_y, cmd_yaw = joy.get_command()
                current_cmd = np.array([cmd_x, cmd_y, cmd_yaw], dtype=np.double)
                obs_list.append(current_cmd * scales.commands)

                dof_pos_rel = q_policy - cfg.robot_config.default_dof_pos
                obs_list.append(dof_pos_rel * scales.joint_pos)
                obs_list.append(dq_policy * scales.joint_vel)
                obs_list.append(action_policy * scales.actions)

                current_obs = np.concatenate(obs_list).astype(np.float32)
                current_obs = np.clip(
                    current_obs,
                    -cfg.normalization.clip_observations,
                    cfg.normalization.clip_observations,
                )
                hist_obs.append(current_obs)

                policy_input = np.concatenate(hist_obs)[None, :]
                input_name = onnx_session.get_inputs()[0].name
                raw_action = onnx_session.run(None, {input_name: policy_input})[0][0]

                action_policy = np.clip(
                    raw_action,
                    -cfg.normalization.clip_actions,
                    cfg.normalization.clip_actions,
                )

                action_sim = action_policy[policy2sim_indices]
                target_q_sim = (
                    action_sim * cfg.control.action_scale
                    + cfg.robot_config.default_dof_pos
                )

            q_sim_raw = data.qpos[7:]
            dq_sim_raw = data.qvel[6:]

            tau = pd_control(
                target_q_sim,
                q_sim_raw,
                cfg.robot_config.kps,
                np.zeros_like(dq_sim_raw),
                dq_sim_raw,
                cfg.robot_config.kds,
                0.0,
            )
            tau = np.clip(tau, -cfg.robot_config.tau_limit, cfg.robot_config.tau_limit)

            data.ctrl[:] = tau
            mujoco.mj_step(model, data)

            # 收集数据
            if collector and count_lowlevel % cfg.sim_config.decimation == 0:
                collector.record(
                    t=data.time,
                    omega=omega.astype(np.float32),
                    gravity=gravity.astype(np.float32),
                    cmd=current_cmd.astype(np.float32),
                    joint_pos=q_policy.astype(np.float32),
                    joint_vel=dq_policy.astype(np.float32),
                    last_action=last_action_for_record,
                    action=action_policy.astype(np.float32),
                    target_q=target_q_sim.astype(np.float32),
                    tau=tau.astype(np.float32),
                    base_pos=data.qpos[:3].copy().astype(np.float32),
                    base_quat=data.qpos[3:7].copy().astype(np.float32),
                    base_vel=vel_body.astype(np.float32),
                    foot_contact=foot_contact.astype(np.float32),
                )
                last_action_for_record = action_policy.copy()

            viewer.sync()

            if count_lowlevel % 10 == 0:
                print(
                    f"\r"
                    f"Samples: {len(collector.timestamps) if collector else 0}/{max_samples} | "
                    f"Cmd: x={cmd_x:.2f} y={cmd_y:.2f} yaw={cmd_yaw:.2f} | "
                    f"Real: x={vel_body[0]:.2f} y={vel_body[1]:.2f}"
                    f"\033[K",
                    end="",
                    flush=True,
                )

            count_lowlevel += 1

            # 数据收集满后自动退出
            if collector and len(collector.timestamps) >= max_samples:
                print(f"\n\nCollected {max_samples} samples, stopping...")
                break

    joy.stop()

    if collector:
        return collector.to_numpy()
    return None


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--load_model", type=str, required=True, help="Path to ONNX model")
    parser.add_argument("--max_samples", type=int, default=5000, help="Max samples to collect")
    parser.add_argument("--no_plot", action="store_true", help="Don't show plots after collection")
    args = parser.parse_args()

    try:
        ort_session = ort.InferenceSession(args.load_model)
        print("ONNX model loaded successfully.")
    except Exception as e:
        print(f"Error loading ONNX model: {e}")
        exit()

    print(f"\nStarting data collection (max {args.max_samples} samples)...")
    print("Use joystick to control the robot. Data will be collected automatically.\n")

    data = run_mujoco(ort_session, Sim2simCfg(), collect_data=True, max_samples=args.max_samples)

    if data:
        print_data_summary(data)

        # 保存数据
        save_path = Path(__file__).parent / "collected_data.npz"
        np.savez(save_path, **data)
        print(f"\nData saved to {save_path}")

        if not args.no_plot:
            print("\nGenerating plots...")
            plot_data(data, save_path=Path(__file__).parent / "data_plot.png")
            plot_action_analysis(data, save_path=Path(__file__).parent / "action_plot.png")