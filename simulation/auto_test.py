#!/usr/bin/env python3
"""
自动化测试脚本 - 只记录网络输入输出和目标位置
测试序列: 静止1s -> 前进2s -> 后退2s -> 左移1s -> 右移1s -> 左转1s -> 右转1s -> 静止1s
"""

import numpy as np
import mujoco
import mujoco.viewer
from collections import deque
from scipy.spatial.transform import Rotation as R
from pathlib import Path
import onnxruntime as ort
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# ---------------------------------------------------------------------------- #
#                               配置参数                                       #
# ---------------------------------------------------------------------------- #

# 关节映射
SIM2POLICY = np.array([0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11], dtype=np.int64)
POLICY2SIM = np.array([0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11], dtype=np.int64)

JOINT_NAMES = ["LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
               "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
               "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee"]

# 测试序列: (持续时间, cmd_x, cmd_y, cmd_yaw)
TEST_SEQUENCE = [
    (1.0, 0.0, 0.0, 0.0),    # 静止
    (2.0, 1.0, 0.0, 0.0),    # 前进
    (2.0, -1.0, 0.0, 0.0),   # 后退
    (1.0, 0.0, 0.5, 0.0),    # 左移
    (1.0, 0.0, -0.5, 0.0),   # 右移
    (1.0, 0.0, 0.0, 1.0),    # 左转
    (1.0, 0.0, 0.0, -1.0),   # 右转
    (1.0, 0.0, 0.0, 0.0),    # 静止
]
PHASE_NAMES = ["Stand", "Forward", "Backward", "Left", "Right", "TurnL", "TurnR", "Stand"]
PHASE_COLORS = ['gray', 'green', 'red', 'blue', 'cyan', 'orange', 'purple', 'gray']
TOTAL_TIME = sum(t for t, _, _, _ in TEST_SEQUENCE)

# PD参数
KP = 25.0
KD = 0.5
TAU_LIMIT = np.array([17, 17, 25] * 4)
ACTION_SCALE = 0.25

# 膝关节减速比 (与 main.cpp 一致)
# 注意：如果网络训练时没有减速比处理，这里应该设为 1.0
# 如果网络训练时有减速比处理，这里应该设为 1.667
KNEE_GEAR_RATIO = 1.0  # 暂时禁用减速比处理，与网络训练一致


# ---------------------------------------------------------------------------- #
#                               主函数                                         #
# ---------------------------------------------------------------------------- #

def main():
    script_dir = Path(__file__).parent
    xml_path = script_dir.parent / "leggedrobot_flat.xml"
    onnx_path = script_dir.parent / "policy.onnx"

    # 加载模型
    print("Loading models...")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    model.opt.timestep = 0.005
    data = mujoco.MjData(model)
    ort_session = ort.InferenceSession(str(onnx_path))

    # 打印测试序列
    print("\n" + "="*50)
    print("Automated Test Sequence")
    print("="*50)
    for i, (dur, cx, cy, cyaw) in enumerate(TEST_SEQUENCE):
        print(f"  {i}: {PHASE_NAMES[i]:8s} {dur}s, cmd=({cx:.1f}, {cy:.1f}, {cyaw:.1f})")
    print(f"Total: {TOTAL_TIME}s")
    print("="*50 + "\n")

    # 数据存储 - 只记录网络输入输出
    records = {
        't': [],
        'phase': [],
        # 网络输入 (45维)
        'obs_omega': [],        # 角速度 (3)
        'obs_gravity': [],      # 重力投影 (3)
        'obs_cmd': [],          # 命令 (3)
        'obs_joint_pos': [],    # 关节位置 (12)
        'obs_joint_vel': [],    # 关节速度 (12)
        'obs_last_action': [],  # 上一帧动作 (12)
        # 网络输出
        'action': [],           # 网络输出 (12)
        # 目标位置
        'target_q': [],         # 目标关节位置 (12)
    }

    # 初始化
    data.qpos[7:] = 0
    mujoco.mj_step(model, data)

    hist_obs = deque(maxlen=10)
    for _ in range(10):
        hist_obs.append(np.zeros(45, dtype=np.float32))

    action = np.zeros(12, dtype=np.float64)
    target_q = np.zeros(12, dtype=np.float64)
    last_phase = -1

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -45

        while viewer.is_running() and data.time < TOTAL_TIME:
            # 获取当前命令
            elapsed = 0.0
            for i, (dur, cx, cy, cyaw) in enumerate(TEST_SEQUENCE):
                if data.time < elapsed + dur:
                    cmd = np.array([cx, cy, cyaw])
                    phase = i
                    break
                elapsed += dur
            else:
                cmd = np.zeros(3)
                phase = len(TEST_SEQUENCE) - 1

            # 打印阶段变化
            if phase != last_phase:
                print(f"[{data.time:.1f}s] Phase {phase}: {PHASE_NAMES[phase]}")
                last_phase = phase

            # 获取状态
            q_sim = data.qpos[7:].copy()
            dq_sim = data.qvel[6:].copy()
            q_policy = q_sim[SIM2POLICY]
            dq_policy = dq_sim[SIM2POLICY]

            mj_quat = data.qpos[3:7]
            quat_xyzw = np.array([mj_quat[1], mj_quat[2], mj_quat[3], mj_quat[0]])
            omega = data.sensor("angular-velocity").data.copy()

            gravity = R.from_quat(quat_xyzw).apply(np.array([0, 0, -1]), inverse=True)

            # 策略推理 (50Hz)
            if int(data.time / 0.005) % 4 == 0:
                # 构建观测
                default_dof_pos = np.zeros(12, dtype=np.float32)
                dof_pos_rel = q_policy - default_dof_pos

                # === 膝关节减速比处理 (与 main.cpp observations.cpp 一致) ===
                # 观测中：膝关节位置和速度除以减速比
                dof_pos_rel_for_obs = dof_pos_rel.copy()
                dq_policy_for_obs = dq_policy.copy()
                for i in range(8, 12):  # 膝关节索引 8-11
                    dof_pos_rel_for_obs[i] = dof_pos_rel[i] / KNEE_GEAR_RATIO
                    dq_policy_for_obs[i] = dq_policy[i] / KNEE_GEAR_RATIO

                obs = np.concatenate([
                    omega,                  # 角速度 (3)
                    gravity,                # 重力投影 (3)
                    cmd,                    # 命令 (3)
                    dof_pos_rel_for_obs,    # 关节位置 (12) - 膝关节已除减速比
                    dq_policy_for_obs,      # 关节速度 (12) - 膝关节已除减速比
                    action                  # 上一帧动作 (12)
                ]).astype(np.float32)
                obs = np.clip(obs, -100, 100)
                hist_obs.append(obs)

                # 推理
                policy_input = np.concatenate(list(hist_obs))[None, :]
                raw_action = ort_session.run(None, {"obs": policy_input})[0][0]
                action = np.clip(raw_action, -100, 100)

                # === 后处理：膝关节减速比处理 (与 main.cpp 一致) ===
                # 输出中：膝关节动作乘以减速比
                action_for_target = action.copy()
                for i in range(8, 12):  # 膝关节索引 8-11
                    action_for_target[i] = action[i] * KNEE_GEAR_RATIO

                action_sim = action_for_target[POLICY2SIM]
                target_q = action_sim * ACTION_SCALE + default_dof_pos

                # 记录数据
                records['t'].append(data.time)
                records['phase'].append(phase)
                # 网络输入 (记录处理后的观测)
                records['obs_omega'].append(omega.copy())
                records['obs_gravity'].append(gravity.copy())
                records['obs_cmd'].append(cmd.copy())
                records['obs_joint_pos'].append(dof_pos_rel_for_obs.copy())
                records['obs_joint_vel'].append(dq_policy_for_obs.copy())
                records['obs_last_action'].append(action.copy())
                # 网络输出 (原始输出)
                records['action'].append(action.copy())
                # 目标位置 (转回Policy顺序便于对比)
                target_q_policy = target_q[SIM2POLICY]
                records['target_q'].append(target_q_policy.copy())

            # PD控制
            tau = (target_q - q_sim) * KP - dq_sim * KD
            tau = np.clip(tau, -TAU_LIMIT, TAU_LIMIT)
            data.ctrl[:] = tau

            mujoco.mj_step(model, data)
            viewer.sync()

            # 进度显示
            if int(data.time * 10) % 10 == 0:
                print(f"\rProgress: {data.time/TOTAL_TIME*100:4.0f}%\033[K", end="", flush=True)

    print("\n\nTest completed!")

    # 转换数据
    for k in records:
        records[k] = np.array(records[k])

    # 保存数据
    np.savez(script_dir / "auto_test_data.npz", **records)
    print(f"Data saved: {script_dir / 'auto_test_data.npz'}")

    # 绘图
    plot_results(records, script_dir / "auto_test_plot.png")


def plot_results(data, save_path):
    """绘制测试结果 - 只展示网络输入输出和目标位置"""
    t = data['t']

    fig = plt.figure(figsize=(18, 18))
    gs = GridSpec(6, 3, figure=fig, hspace=0.35, wspace=0.25)

    def add_phase_bg(ax):
        for i, (dur, _, _, _) in enumerate(TEST_SEQUENCE):
            start = sum(TEST_SEQUENCE[j][0] for j in range(i))
            ax.axvspan(start, start + dur, alpha=0.15, color=PHASE_COLORS[i])

    # ==================== 网络输入 ====================
    fig.text(0.5, 0.97, 'Network Input (Observation) - 45 dims', ha='center', fontsize=12, fontweight='bold')

    # 1. 角速度 (网络输入)
    ax = fig.add_subplot(gs[0, 0])
    add_phase_bg(ax)
    ax.plot(t, data['obs_omega'][:, 0], 'r-', label='wx')
    ax.plot(t, data['obs_omega'][:, 1], 'g-', label='wy')
    ax.plot(t, data['obs_omega'][:, 2], 'b-', label='wz')
    ax.set_ylabel('Angular Vel (rad/s)')
    ax.set_title('[Input] Angular Velocity (3)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 2. 重力投影 (网络输入)
    ax = fig.add_subplot(gs[0, 1])
    add_phase_bg(ax)
    ax.plot(t, data['obs_gravity'][:, 0], 'r-', label='gx')
    ax.plot(t, data['obs_gravity'][:, 1], 'g-', label='gy')
    ax.plot(t, data['obs_gravity'][:, 2], 'b-', label='gz')
    ax.set_ylabel('Gravity')
    ax.set_title('[Input] Projected Gravity (3)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 3. 命令 (网络输入)
    ax = fig.add_subplot(gs[0, 2])
    add_phase_bg(ax)
    ax.plot(t, data['obs_cmd'][:, 0], 'r-', label='cmd_x')
    ax.plot(t, data['obs_cmd'][:, 1], 'g-', label='cmd_y')
    ax.plot(t, data['obs_cmd'][:, 2], 'b-', label='cmd_yaw')
    ax.set_ylabel('Command')
    ax.set_title('[Input] Velocity Command (3)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 4. 关节位置观测 - HipA
    ax = fig.add_subplot(gs[1, 0])
    add_phase_bg(ax)
    for i in range(4):
        ax.plot(t, data['obs_joint_pos'][:, i], label=JOINT_NAMES[i])
    ax.set_ylabel('Position (rad)')
    ax.set_title('[Input] Joint Position - HipA (4)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 5. 关节位置观测 - HipF
    ax = fig.add_subplot(gs[1, 1])
    add_phase_bg(ax)
    for i in range(4, 8):
        ax.plot(t, data['obs_joint_pos'][:, i], label=JOINT_NAMES[i])
    ax.set_ylabel('Position (rad)')
    ax.set_title('[Input] Joint Position - HipF (4)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 6. 关节位置观测 - Knee
    ax = fig.add_subplot(gs[1, 2])
    add_phase_bg(ax)
    for i in range(8, 12):
        ax.plot(t, data['obs_joint_pos'][:, i], label=JOINT_NAMES[i])
    ax.set_ylabel('Position (rad)')
    ax.set_title('[Input] Joint Position - Knee (4)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 7. 关节速度观测 - HipA
    ax = fig.add_subplot(gs[2, 0])
    add_phase_bg(ax)
    for i in range(4):
        ax.plot(t, data['obs_joint_vel'][:, i], label=JOINT_NAMES[i])
    ax.set_ylabel('Velocity (rad/s)')
    ax.set_title('[Input] Joint Velocity - HipA (4)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 8. 关节速度观测 - HipF
    ax = fig.add_subplot(gs[2, 1])
    add_phase_bg(ax)
    for i in range(4, 8):
        ax.plot(t, data['obs_joint_vel'][:, i], label=JOINT_NAMES[i])
    ax.set_ylabel('Velocity (rad/s)')
    ax.set_title('[Input] Joint Velocity - HipF (4)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 9. 关节速度观测 - Knee
    ax = fig.add_subplot(gs[2, 2])
    add_phase_bg(ax)
    for i in range(8, 12):
        ax.plot(t, data['obs_joint_vel'][:, i], label=JOINT_NAMES[i])
    ax.set_ylabel('Velocity (rad/s)')
    ax.set_title('[Input] Joint Velocity - Knee (4)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 10. 上一帧动作 - HipA
    ax = fig.add_subplot(gs[3, 0])
    add_phase_bg(ax)
    for i in range(4):
        ax.plot(t, data['obs_last_action'][:, i], label=JOINT_NAMES[i])
    ax.set_ylabel('Action')
    ax.set_title('[Input] Last Action - HipA (4)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 11. 上一帧动作 - HipF
    ax = fig.add_subplot(gs[3, 1])
    add_phase_bg(ax)
    for i in range(4, 8):
        ax.plot(t, data['obs_last_action'][:, i], label=JOINT_NAMES[i])
    ax.set_ylabel('Action')
    ax.set_title('[Input] Last Action - HipF (4)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 12. 上一帧动作 - Knee
    ax = fig.add_subplot(gs[3, 2])
    add_phase_bg(ax)
    for i in range(8, 12):
        ax.plot(t, data['obs_last_action'][:, i], label=JOINT_NAMES[i])
    ax.set_ylabel('Action')
    ax.set_title('[Input] Last Action - Knee (4)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # ==================== 网络输出 ====================
    fig.text(0.5, 0.52, 'Network Output (Action) - 12 dims', ha='center', fontsize=12, fontweight='bold')

    # 13. 网络输出 - HipA
    ax = fig.add_subplot(gs[4, 0])
    add_phase_bg(ax)
    for i in range(4):
        ax.plot(t, data['action'][:, i], label=JOINT_NAMES[i])
    ax.set_ylabel('Action')
    ax.set_title('[Output] Action - HipA (4)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 14. 网络输出 - HipF
    ax = fig.add_subplot(gs[4, 1])
    add_phase_bg(ax)
    for i in range(4, 8):
        ax.plot(t, data['action'][:, i], label=JOINT_NAMES[i])
    ax.set_ylabel('Action')
    ax.set_title('[Output] Action - HipF (4)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 15. 网络输出 - Knee
    ax = fig.add_subplot(gs[4, 2])
    add_phase_bg(ax)
    for i in range(8, 12):
        ax.plot(t, data['action'][:, i], label=JOINT_NAMES[i])
    ax.set_ylabel('Action')
    ax.set_title('[Output] Action - Knee (4)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # ==================== 目标位置 ====================
    fig.text(0.5, 0.35, 'Target Position (Action x 0.25) - 12 dims', ha='center', fontsize=12, fontweight='bold')

    # 16. 目标位置 - HipA
    ax = fig.add_subplot(gs[5, 0])
    add_phase_bg(ax)
    for i in range(4):
        ax.plot(t, data['target_q'][:, i], label=JOINT_NAMES[i])
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (rad)')
    ax.set_title('[Target] HipA Position (4)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 17. 目标位置 - HipF
    ax = fig.add_subplot(gs[5, 1])
    add_phase_bg(ax)
    for i in range(4, 8):
        ax.plot(t, data['target_q'][:, i], label=JOINT_NAMES[i])
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (rad)')
    ax.set_title('[Target] HipF Position (4)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 18. 目标位置 - Knee
    ax = fig.add_subplot(gs[5, 2])
    add_phase_bg(ax)
    for i in range(8, 12):
        ax.plot(t, data['target_q'][:, i], label=JOINT_NAMES[i])
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (rad)')
    ax.set_title('[Target] Knee Position (4)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 图例
    legend_handles = [plt.Rectangle((0, 0), 1, 1, fc=PHASE_COLORS[i], alpha=0.3, label=PHASE_NAMES[i])
                      for i in range(len(PHASE_NAMES))]
    fig.legend(handles=legend_handles, loc='upper center', ncol=8, fontsize=9, bbox_to_anchor=(0.5, 0.99))

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"Plot saved: {save_path}")
    plt.show()


if __name__ == "__main__":
    main()