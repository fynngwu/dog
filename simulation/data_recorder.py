#!/usr/bin/env python3
"""
详细数据记录脚本 - 用于调试真实部署问题
记录所有输入输出数据，便于与真实部署对比
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
import json
import time
import signal
import sys

from joystick_interface import JoystickInterface
from ros2_interface import ROS2Interface, ROS2_AVAILABLE

# 全局变量用于信号处理
_recorder = None
_output_path = None


def signal_handler(sig, frame):
    """处理 Ctrl+C 信号，保存数据后退出"""
    print("\n\n[Interrupt] 收到中断信号，正在保存数据...")
    global _recorder, _output_path
    if _recorder and len(_recorder.data['timestamps']) > 0:
        try:
            _recorder.print_summary()
        except Exception as e:
            print(f"[Warning] 打印摘要时出错: {e}")
        _recorder.save(_output_path)
        print(f"[Interrupt] 数据已保存，程序退出")
    else:
        print("[Interrupt] 没有数据需要保存")
    sys.exit(0)

# ---------------------------------------------------------------------------- #
#                               Remapping Indices                              #
# ---------------------------------------------------------------------------- #

# MuJoCo 顺序: [LF_A, LF_F, LF_K, LR_A, LR_F, LR_K, RF_A, RF_F, RF_K, RR_A, RR_F, RR_K]
# Policy 顺序: [LF_A, LR_A, RF_A, RR_A, LF_F, LR_F, RF_F, RR_F, LF_K, LR_K, RF_K, RR_K]

sim2policy_indices = np.array(
    [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11], dtype=np.int64
)
policy2sim_indices = np.array(
    [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11], dtype=np.int64
)

# 关节名称 (Policy 顺序)
JOINT_NAMES_POLICY = [
    "LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",  # 0-3
    "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",  # 4-7
    "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee",  # 8-11
]

# 关节名称 (MuJoCo/Sim 顺序)
JOINT_NAMES_SIM = [
    "LF_HipA", "LF_HipF", "LF_Knee",  # 0-2
    "LR_HipA", "LR_HipF", "LR_Knee",  # 3-5
    "RF_HipA", "RF_HipF", "RF_Knee",  # 6-8
    "RR_HipA", "RR_HipF", "RR_Knee",  # 9-11
]


# ---------------------------------------------------------------------------- #
#                                 Configuration                                #
# ---------------------------------------------------------------------------- #

class Sim2simCfg:
    class sim_config:
        base_dir = Path(__file__).resolve().parent.parent
        mujoco_model_path = ""
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
#                               Data Recorder                                  #
# ---------------------------------------------------------------------------- #

class DataRecorder:
    """详细数据记录器"""

    def __init__(self, max_samples=10000):
        self.max_samples = max_samples
        self.data = {
            # 时间戳
            'timestamps': [],
            # === 网络输入 (45维) ===
            'obs_angular_vel': [],      # 角速度 (3) - 网络输入
            'obs_gravity': [],          # 重力投影 (3) - 网络输入
            'obs_commands': [],         # 命令 (3) - 网络输入
            'obs_joint_pos': [],        # 关节位置 (12) - 网络输入
            'obs_joint_vel': [],        # 关节速度 (12) - 网络输入
            'obs_last_action': [],      # 上一帧动作 (12) - 网络输入
            # === 网络输出 ===
            'action_raw': [],           # 原始网络输出 (12)
            'action_clipped': [],       # 裁剪后的动作 (12)
            # === 后处理 ===
            'action_scaled': [],        # 缩放后的动作 (12)
            'target_q_sim': [],         # 目标关节位置 - Sim顺序 (12)
            'target_q_policy': [],      # 目标关节位置 - Policy顺序 (12)
            # === PD控制输出 ===
            'tau': [],                  # 扭矩 - Sim顺序 (12)
            'tau_clipped': [],          # 裁剪后的扭矩 (12)
            # === 真实状态 ===
            'joint_pos_sim': [],        # 实际关节位置 - Sim顺序 (12)
            'joint_pos_policy': [],     # 实际关节位置 - Policy顺序 (12)
            'joint_vel_sim': [],        # 实际关节速度 - Sim顺序 (12)
            'joint_vel_policy': [],     # 实际关节速度 - Policy顺序 (12)
            # === 基座状态 ===
            'base_pos': [],             # 基座位置 (3)
            'base_quat_wxyz': [],       # 基座四元数 [w,x,y,z] (4)
            'base_quat_xyzw': [],       # 基座四元数 [x,y,z,w] (4) - scipy格式
            'base_vel_world': [],       # 基座速度 - 世界坐标系 (3)
            'base_vel_body': [],        # 基座速度 - 机体坐标系 (3)
            # === IMU原始数据 ===
            'imu_angular_vel': [],      # IMU角速度 (3)
            'imu_gravity': [],          # IMU重力投影 (3)
            # === 足端接触 ===
            'foot_contact': [],         # 足端接触状态 (4)
        }

    def record(self, **kwargs):
        """记录一帧数据"""
        if len(self.data['timestamps']) >= self.max_samples:
            return False

        for key, value in kwargs.items():
            if key in self.data:
                self.data[key].append(value.copy() if isinstance(value, np.ndarray) else value)

        return True

    def is_full(self):
        return len(self.data['timestamps']) >= self.max_samples

    def save(self, filepath):
        """保存数据到npz文件"""
        # 转换为numpy数组
        np_data = {}
        for key, value in self.data.items():
            if len(value) > 0:
                np_data[key] = np.array(value)

        np.savez(filepath, **np_data)
        print(f"[Recorder] Data saved to {filepath}")
        print(f"[Recorder] Total samples: {len(np_data['timestamps'])}")

    def print_summary(self):
        """打印数据摘要"""
        if len(self.data['timestamps']) == 0:
            print("No data recorded!")
            return

        print("\n" + "="*70)
        print("数据记录摘要")
        print("="*70)
        print(f"总样本数: {len(self.data['timestamps'])}")
        print(f"时长: {self.data['timestamps'][-1]:.2f} 秒")

        print("\n--- 网络输入统计 ---")
        obs_angular_vel = np.array(self.data['obs_angular_vel'])
        print(f"角速度 (rad/s):")
        print(f"  ωx: mean={obs_angular_vel[:,0].mean():.4f}, std={obs_angular_vel[:,0].std():.4f}, "
              f"min={obs_angular_vel[:,0].min():.4f}, max={obs_angular_vel[:,0].max():.4f}")
        print(f"  ωy: mean={obs_angular_vel[:,1].mean():.4f}, std={obs_angular_vel[:,1].std():.4f}, "
              f"min={obs_angular_vel[:,1].min():.4f}, max={obs_angular_vel[:,1].max():.4f}")
        print(f"  ωz: mean={obs_angular_vel[:,2].mean():.4f}, std={obs_angular_vel[:,2].std():.4f}, "
              f"min={obs_angular_vel[:,2].min():.4f}, max={obs_angular_vel[:,2].max():.4f}")

        obs_gravity = np.array(self.data['obs_gravity'])
        print(f"\n重力投影:")
        print(f"  gx: mean={obs_gravity[:,0].mean():.4f}, std={obs_gravity[:,0].std():.4f}")
        print(f"  gy: mean={obs_gravity[:,1].mean():.4f}, std={obs_gravity[:,1].std():.4f}")
        print(f"  gz: mean={obs_gravity[:,2].mean():.4f}, std={obs_gravity[:,2].std():.4f}")

        obs_joint_pos = np.array(self.data['obs_joint_pos'])
        print(f"\n关节位置 (Policy顺序, rad):")
        for i, name in enumerate(JOINT_NAMES_POLICY):
            print(f"  {name}: mean={obs_joint_pos[:,i].mean():.4f}, std={obs_joint_pos[:,i].std():.4f}, "
                  f"min={obs_joint_pos[:,i].min():.4f}, max={obs_joint_pos[:,i].max():.4f}")

        print("\n--- 网络输出统计 ---")
        action_raw = np.array(self.data['action_raw'])
        print(f"原始动作:")
        for i, name in enumerate(JOINT_NAMES_POLICY):
            print(f"  {name}: mean={action_raw[:,i].mean():.4f}, std={action_raw[:,i].std():.4f}, "
                  f"min={action_raw[:,i].min():.4f}, max={action_raw[:,i].max():.4f}")

        print("\n--- 扭矩统计 ---")
        tau = np.array(self.data['tau'])
        if len(tau) > 0:
            print(f"扭矩 (Nm):")
            print(f"  最大扭矩: {np.abs(tau).max():.2f}")
            print(f"  平均扭矩: {np.abs(tau).mean():.2f}")
        else:
            print("扭矩: 无数据")


# ---------------------------------------------------------------------------- #
#                               Util Functions                                 #
# ---------------------------------------------------------------------------- #

def get_gravity_orientation(quat_xyzw):
    """计算重力在基座坐标系下的投影"""
    r = R.from_quat(quat_xyzw)  # [x, y, z, w]
    gravity_vec = np.array([0.0, 0.0, -1.0])
    return r.apply(gravity_vec, inverse=True)


def get_obs(data):
    """从 MuJoCo 数据中提取观测值"""
    # 关节位置和速度 (Sim Order)
    q_sim = data.qpos[7:].astype(np.double)
    dq_sim = data.qvel[6:].astype(np.double)

    # 转换为 Policy Order
    q_policy = q_sim[sim2policy_indices]
    dq_policy = dq_sim[sim2policy_indices]

    # 四元数: MuJoCo [w, x, y, z] -> scipy [x, y, z, w]
    mj_quat = data.qpos[3:7]
    quat_xyzw = np.array([mj_quat[1], mj_quat[2], mj_quat[3], mj_quat[0]])

    # 角速度
    omega = data.sensor("angular-velocity").data.astype(np.double)

    return q_sim, q_policy, dq_sim, dq_policy, quat_xyzw, mj_quat, omega


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """PD控制"""
    return (target_q - q) * kp + (target_dq - dq) * kd


# ---------------------------------------------------------------------------- #
#                                 Main Loop                                    #
# ---------------------------------------------------------------------------- #

def run_mujoco(onnx_session, cfg, recorder, use_ros2=False):
    """运行仿真并记录数据"""

    # 初始化控制接口
    if use_ros2:
        if not ROS2_AVAILABLE:
            print("[Error] ROS2 not available")
            return
        controller = ROS2Interface(max_v_x=2.0, max_v_y=1.0, max_omega=1.5)
        print("[Info] Using ROS2 cmd_vel control")
    else:
        controller = JoystickInterface(device_path="/dev/input/js0", max_v_x=2.0, max_v_y=1.0, max_omega=1.5)

    # 加载模型
    model = mujoco.MjModel.from_xml_path(cfg.sim_config.mujoco_model_path)
    model.opt.timestep = cfg.sim_config.dt
    data = mujoco.MjData(model)

    # 初始化关节位置
    data.qpos[7:] = cfg.robot_config.default_dof_pos
    mujoco.mj_step(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -45
        viewer.cam.lookat[:] = np.array([0.0, -0.25, 0.824])

        # 状态变量
        action_policy = np.zeros(cfg.robot_config.num_actions, dtype=np.double)
        target_q_sim = np.zeros(cfg.robot_config.num_actions, dtype=np.double)

        # 历史观测
        hist_obs = deque(maxlen=cfg.env.frame_stack)
        for _ in range(cfg.env.frame_stack):
            hist_obs.append(np.zeros(cfg.env.num_single_obs, dtype=np.float32))

        count_lowlevel = 0
        scales = cfg.normalization.isaac_obs_scales

        # 用于记录的变量
        last_action_for_obs = np.zeros(12, dtype=np.float32)
        gravity_for_record = np.zeros(3, dtype=np.float32)
        current_cmd = np.zeros(3, dtype=np.float32)

        while viewer.is_running() and not recorder.is_full():
            # 获取物理数据
            q_sim, q_policy, dq_sim, dq_policy, quat_xyzw, quat_wxyz, omega = get_obs(data)

            # 计算基座速度
            vel_world = data.qvel[:3]
            r_temp = R.from_quat(quat_xyzw)
            vel_body = r_temp.apply(vel_world, inverse=True)

            # 获取足端接触
            foot_contact = np.array([
                data.sensor("LF_touch").data[0],
                data.sensor("LR_touch").data[0],
                data.sensor("RF_touch").data[0],
                data.sensor("RR_touch").data[0],
            ], dtype=np.float32)

            # 策略推理 (50Hz)
            if count_lowlevel % cfg.sim_config.decimation == 0:
                # 构建观测
                obs_list = []

                # 1. 角速度
                obs_omega = omega * scales.ang_vel
                obs_list.append(obs_omega)

                # 2. 重力投影
                gravity = get_gravity_orientation(quat_xyzw)
                obs_gravity = gravity * scales.projected_gravity
                obs_list.append(obs_gravity)
                gravity_for_record = gravity.astype(np.float32)

                # 3. 命令
                cmd_x, cmd_y, cmd_yaw = controller.get_command()
                current_cmd = np.array([cmd_x, cmd_y, cmd_yaw], dtype=np.float32)
                obs_list.append(current_cmd * scales.commands)

                # 4. 关节位置 (Policy Order, 相对于默认位置)
                dof_pos_rel = q_policy - cfg.robot_config.default_dof_pos
                obs_list.append(dof_pos_rel * scales.joint_pos)

                # 5. 关节速度 (Policy Order)
                obs_list.append(dq_policy * scales.joint_vel)

                # 6. 上一帧动作 (Policy Order)
                obs_list.append(action_policy * scales.actions)

                # 构建当前帧观测
                current_obs = np.concatenate(obs_list).astype(np.float32)
                current_obs = np.clip(current_obs, -cfg.normalization.clip_observations, cfg.normalization.clip_observations)
                hist_obs.append(current_obs)

                # 推理
                policy_input = np.concatenate(hist_obs)[None, :]
                input_name = onnx_session.get_inputs()[0].name
                raw_action = onnx_session.run(None, {input_name: policy_input})[0][0]

                # 裁剪动作
                action_clipped = np.clip(raw_action, -cfg.normalization.clip_actions, cfg.normalization.clip_actions)
                action_policy = action_clipped.copy()

                # 缩放动作
                action_scaled = action_policy * cfg.control.action_scale

                # Policy -> Sim 重排
                action_sim = action_policy[policy2sim_indices]
                target_q_sim = action_sim * cfg.control.action_scale + cfg.robot_config.default_dof_pos

                # 记录数据 (包含扭矩)
                # 先计算扭矩用于记录
                q_sim_raw = data.qpos[7:]
                dq_sim_raw = data.qvel[6:]
                tau_for_record = pd_control(target_q_sim, q_sim_raw, cfg.robot_config.kps,
                                            np.zeros_like(dq_sim_raw), dq_sim_raw, cfg.robot_config.kds)
                tau_clipped_for_record = np.clip(tau_for_record, -cfg.robot_config.tau_limit, cfg.robot_config.tau_limit)

                recorder.record(
                    timestamps=data.time,
                    # 网络输入
                    obs_angular_vel=obs_omega.astype(np.float32),
                    obs_gravity=obs_gravity.astype(np.float32),
                    obs_commands=current_cmd.astype(np.float32),
                    obs_joint_pos=dof_pos_rel.astype(np.float32),
                    obs_joint_vel=dq_policy.astype(np.float32),
                    obs_last_action=last_action_for_obs,
                    # 网络输出
                    action_raw=raw_action.astype(np.float32),
                    action_clipped=action_clipped.astype(np.float32),
                    # 后处理
                    action_scaled=action_scaled.astype(np.float32),
                    target_q_sim=target_q_sim.astype(np.float32),
                    target_q_policy=(action_policy * cfg.control.action_scale).astype(np.float32),
                    # PD控制输出
                    tau=tau_for_record.astype(np.float32),
                    tau_clipped=tau_clipped_for_record.astype(np.float32),
                    # 真实状态
                    joint_pos_sim=q_sim.astype(np.float32),
                    joint_pos_policy=q_policy.astype(np.float32),
                    joint_vel_sim=dq_sim.astype(np.float32),
                    joint_vel_policy=dq_policy.astype(np.float32),
                    # 基座状态
                    base_pos=data.qpos[:3].astype(np.float32),
                    base_quat_wxyz=quat_wxyz.astype(np.float32),
                    base_quat_xyzw=quat_xyzw.astype(np.float32),
                    base_vel_world=vel_world.astype(np.float32),
                    base_vel_body=vel_body.astype(np.float32),
                    # IMU
                    imu_angular_vel=omega.astype(np.float32),
                    imu_gravity=gravity_for_record,
                    # 足端
                    foot_contact=foot_contact,
                )

                last_action_for_obs = action_policy.copy()

            # PD控制
            q_sim_raw = data.qpos[7:]
            dq_sim_raw = data.qvel[6:]

            tau = pd_control(target_q_sim, q_sim_raw, cfg.robot_config.kps,
                           np.zeros_like(dq_sim_raw), dq_sim_raw, cfg.robot_config.kds)
            tau_clipped = np.clip(tau, -cfg.robot_config.tau_limit, cfg.robot_config.tau_limit)

            data.ctrl[:] = tau_clipped
            mujoco.mj_step(model, data)

            viewer.sync()

            # 打印状态
            if count_lowlevel % 50 == 0:
                print(f"\rSamples: {len(recorder.data['timestamps'])}/{recorder.max_samples} | "
                      f"Cmd: x={cmd_x:.2f} y={cmd_y:.2f} yaw={cmd_yaw:.2f} | "
                      f"Real: x={vel_body[0]:.2f} y={vel_body[1]:.2f}"
                      f"\033[K", end="", flush=True)

            count_lowlevel += 1

    controller.stop()
    return recorder


# ---------------------------------------------------------------------------- #
#                                    Main                                      #
# ---------------------------------------------------------------------------- #

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="详细数据记录脚本")
    parser.add_argument("--load_model", type=str, required=True, help="ONNX模型路径")
    parser.add_argument("--max_samples", type=int, default=5000, help="最大样本数")
    parser.add_argument("--output", type=str, default="sim_data.npz", help="输出文件名")
    parser.add_argument("--ros2", action="store_true", help="使用ROS2控制")
    args = parser.parse_args()

    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler)

    # 设置模型路径
    Sim2simCfg.sim_config.mujoco_model_path = (
        Path(__file__).resolve().parent.parent / "leggedrobot_flat.xml"
    ).as_posix()

    # 加载ONNX模型
    try:
        ort_session = ort.InferenceSession(args.load_model)
        print("ONNX model loaded successfully.")
    except Exception as e:
        print(f"Error loading ONNX model: {e}")
        exit(1)

    # 创建记录器
    recorder = DataRecorder(max_samples=args.max_samples)

    # 设置全局变量（用于信号处理）
    _recorder = recorder
    _output_path = Path(__file__).parent / args.output

    print(f"\n开始数据收集 (最大 {args.max_samples} 样本)...")
    print("按 Ctrl+C 可随时停止并保存数据\n")

    # 运行仿真
    run_mujoco(ort_session, Sim2simCfg(), recorder, use_ros2=args.ros2)

    # 正常结束时打印摘要并保存
    recorder.print_summary()
    recorder.save(_output_path)