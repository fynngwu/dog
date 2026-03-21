#!/usr/bin/env python3
"""
部署数据解析与对比脚本
解析 main.cpp 记录的二进制日志，与仿真数据对比
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from pathlib import Path
import struct

# 关节名称
JOINT_NAMES = ["LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
               "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
               "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee"]

# 测试阶段
PHASE_NAMES = ["Stand", "Forward", "Backward", "Left", "Right", "TurnL", "TurnR", "Stand"]
PHASE_COLORS = ['gray', 'green', 'red', 'blue', 'cyan', 'orange', 'purple', 'gray']


def parse_deploy_log(filepath):
    """
    解析部署日志 (DetailedDataLogger 格式)

    Frame 结构 (C++):
    - timestamp: float (4 bytes)
    - obs_angular_vel[3]: float (12 bytes)
    - obs_gravity[3]: float (12 bytes)
    - obs_cmd[3]: float (12 bytes)
    - obs_joint_pos[12]: float (48 bytes)
    - obs_joint_vel[12]: float (48 bytes)
    - obs_last_action[12]: float (48 bytes)
    - action_raw[12]: float (48 bytes)
    - action_clipped[12]: float (48 bytes)
    - target_q[12]: float (48 bytes)
    - imu_gyro_raw[3]: float (12 bytes)
    - imu_quat_raw[4]: float (16 bytes)
    - joint_pos_raw[12]: float (48 bytes)
    - joint_vel_raw[12]: float (48 bytes)
    - motor_cmd[12]: float (48 bytes)

    总计: 4 + 12*3 + 48*9 + 12*2 + 16 + 48*3 = 4 + 36 + 432 + 24 + 16 + 144 = 656 bytes
    """
    # 计算帧大小
    frame_size = 4 + 12*3 + 48*9 + 12*2 + 16 + 48*3  # 656 bytes

    with open(filepath, 'rb') as f:
        data = f.read()

    num_frames = len(data) // frame_size
    print(f"[Deploy] File size: {len(data)} bytes")
    print(f"[Deploy] Frame size: {frame_size} bytes")
    print(f"[Deploy] Frames: {num_frames}")

    if num_frames == 0:
        print("[Deploy] No valid frames!")
        return None

    # 解析格式
    # 注意：需要与 C++ 结构体完全对应
    fmt = 'f'  # timestamp
    fmt += '3f'  # obs_angular_vel
    fmt += '3f'  # obs_gravity
    fmt += '3f'  # obs_cmd
    fmt += '12f'  # obs_joint_pos
    fmt += '12f'  # obs_joint_vel
    fmt += '12f'  # obs_last_action
    fmt += '12f'  # action_raw
    fmt += '12f'  # action_clipped
    fmt += '12f'  # target_q
    fmt += '3f'  # imu_gyro_raw
    fmt += '4f'  # imu_quat_raw
    fmt += '12f'  # joint_pos_raw
    fmt += '12f'  # joint_vel_raw
    fmt += '12f'  # motor_cmd

    frame_fmt = fmt
    frame_struct_size = struct.calcsize(frame_fmt)
    print(f"[Deploy] Struct size: {frame_struct_size} bytes")

    # 如果大小不匹配，尝试调整
    if frame_struct_size != frame_size:
        print(f"[Deploy] Warning: Size mismatch! Using struct size: {frame_struct_size}")
        frame_size = frame_struct_size
        num_frames = len(data) // frame_size

    records = {
        't': [],
        'obs_omega': [], 'obs_gravity': [], 'obs_cmd': [],
        'obs_joint_pos': [], 'obs_joint_vel': [], 'obs_last_action': [],
        'action': [], 'target_q': [],
        'imu_gyro_raw': [], 'imu_quat_raw': [],
        'joint_pos_raw': [], 'joint_vel_raw': [], 'motor_cmd': []
    }

    for i in range(num_frames):
        frame_data = data[i * frame_size:(i + 1) * frame_size]
        values = struct.unpack(frame_fmt, frame_data)

        idx = 0
        records['t'].append(values[idx]); idx += 1
        records['obs_omega'].append(values[idx:idx+3]); idx += 3
        records['obs_gravity'].append(values[idx:idx+3]); idx += 3
        records['obs_cmd'].append(values[idx:idx+3]); idx += 3
        records['obs_joint_pos'].append(values[idx:idx+12]); idx += 12
        records['obs_joint_vel'].append(values[idx:idx+12]); idx += 12
        records['obs_last_action'].append(values[idx:idx+12]); idx += 12
        records['action'].append(values[idx:idx+12]); idx += 12
        # 跳过 action_clipped
        idx += 12
        records['target_q'].append(values[idx:idx+12]); idx += 12
        records['imu_gyro_raw'].append(values[idx:idx+3]); idx += 3
        records['imu_quat_raw'].append(values[idx:idx+4]); idx += 4
        records['joint_pos_raw'].append(values[idx:idx+12]); idx += 12
        records['joint_vel_raw'].append(values[idx:idx+12]); idx += 12
        records['motor_cmd'].append(values[idx:idx+12]); idx += 12

    # 转换为 numpy 数组
    for k in records:
        records[k] = np.array(records[k])

    return records


def compare_with_sim(deploy_data, sim_path=None):
    """对比部署数据与仿真数据"""
    if sim_path is None:
        sim_path = Path(__file__).parent / "auto_test_data.npz"

    sim_data = np.load(sim_path)

    print("\n" + "="*70)
    print("对比分析: 部署 vs 仿真")
    print("="*70)

    # 1. 角速度对比
    print("\n--- 1. 角速度 (Angular Velocity) ---")
    print(f"{'':20s} {'部署':>20s} {'仿真':>20s}")
    for i, name in enumerate(['wx', 'wy', 'wz']):
        d_min, d_max = deploy_data['obs_omega'][:,i].min(), deploy_data['obs_omega'][:,i].max()
        s_min, s_max = sim_data['obs_omega'][:,i].min(), sim_data['obs_omega'][:,i].max()
        print(f"{name:20s} [{d_min:7.3f}, {d_max:7.3f}] [{s_min:7.3f}, {s_max:7.3f}]")

    # 2. 重力投影对比
    print("\n--- 2. 重力投影 (Projected Gravity) ---")
    print(f"{'':20s} {'部署':>20s} {'仿真':>20s}")
    for i, name in enumerate(['gx', 'gy', 'gz']):
        d_min, d_max = deploy_data['obs_gravity'][:,i].min(), deploy_data['obs_gravity'][:,i].max()
        s_min, s_max = sim_data['obs_gravity'][:,i].min(), sim_data['obs_gravity'][:,i].max()
        print(f"{name:20s} [{d_min:7.4f}, {d_max:7.4f}] [{s_min:7.4f}, {s_max:7.4f}]")

    # 3. 关节位置对比
    print("\n--- 3. 关节位置 (Joint Position) ---")
    print(f"{'':12s} {'部署范围':>22s} {'仿真范围':>22s} {'差异':>10s}")
    for i, name in enumerate(JOINT_NAMES):
        d_min, d_max = deploy_data['obs_joint_pos'][:,i].min(), deploy_data['obs_joint_pos'][:,i].max()
        s_min, s_max = sim_data['obs_joint_pos'][:,i].min(), sim_data['obs_joint_pos'][:,i].max()
        diff = abs((d_min + d_max)/2 - (s_min + s_max)/2)
        warn = " ⚠️" if diff > 0.2 else ""
        print(f"{name:12s} [{d_min:6.3f}, {d_max:6.3f}] [{s_min:6.3f}, {s_max:6.3f}] {diff:8.3f}{warn}")

    # 4. 网络输出对比
    print("\n--- 4. 网络输出 (Network Output) ---")
    print(f"{'':12s} {'部署范围':>22s} {'仿真范围':>22s}")
    for i, name in enumerate(JOINT_NAMES):
        d_min, d_max = deploy_data['action'][:,i].min(), deploy_data['action'][:,i].max()
        s_min, s_max = sim_data['action'][:,i].min(), sim_data['action'][:,i].max()
        print(f"{name:12s} [{d_min:6.3f}, {d_max:6.3f}] [{s_min:6.3f}, {s_max:6.3f}]")

    # 5. 目标位置对比
    print("\n--- 5. 目标位置 (Target Position) ---")
    print(f"{'':12s} {'部署范围':>22s} {'仿真范围':>22s}")
    for i, name in enumerate(JOINT_NAMES):
        d_min, d_max = deploy_data['target_q'][:,i].min(), deploy_data['target_q'][:,i].max()
        s_min, s_max = sim_data['target_q'][:,i].min(), sim_data['target_q'][:,i].max()
        print(f"{name:12s} [{d_min:6.3f}, {d_max:6.3f}] [{s_min:6.3f}, {s_max:6.3f}]")


def plot_comparison(deploy_data, sim_data, save_path=None):
    """绘制对比图"""
    fig = plt.figure(figsize=(20, 12))
    gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.25)

    # 时间轴
    t_deploy = deploy_data['t'] - deploy_data['t'][0]
    t_sim = sim_data['t']

    # 1. 角速度对比
    for idx, (name, color) in enumerate([('wx', 'r'), ('wy', 'g'), ('wz', 'b')]):
        ax = fig.add_subplot(gs[0, idx])
        ax.plot(t_deploy, deploy_data['obs_omega'][:, idx], color=color, label='Deploy', alpha=0.7)
        ax.plot(t_sim, sim_data['obs_omega'][:, idx], color=color, linestyle='--', label='Sim', alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(f'{name} (rad/s)')
        ax.set_title(f'Angular Velocity: {name}')
        ax.legend()
        ax.grid(True, alpha=0.3)

    # 2. 重力投影对比
    for idx, (name, color) in enumerate([('gx', 'r'), ('gy', 'g'), ('gz', 'b')]):
        ax = fig.add_subplot(gs[1, idx])
        ax.plot(t_deploy, deploy_data['obs_gravity'][:, idx], color=color, label='Deploy', alpha=0.7)
        ax.plot(t_sim, sim_data['obs_gravity'][:, idx], color=color, linestyle='--', label='Sim', alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(name)
        ax.set_title(f'Projected Gravity: {name}')
        ax.legend()
        ax.grid(True, alpha=0.3)

    # 3. 关节位置对比 (HipA)
    ax = fig.add_subplot(gs[2, 0])
    for i in range(4):
        ax.plot(t_deploy, deploy_data['obs_joint_pos'][:, i], label=f'{JOINT_NAMES[i]} (Deploy)')
    ax.set_title('Deploy: HipA Joint Position')
    ax.legend(fontsize=6)
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(gs[2, 1])
    for i in range(4):
        ax.plot(t_sim, sim_data['obs_joint_pos'][:, i], label=f'{JOINT_NAMES[i]} (Sim)')
    ax.set_title('Sim: HipA Joint Position')
    ax.legend(fontsize=6)
    ax.grid(True, alpha=0.3)

    # 4. 网络输出对比
    ax = fig.add_subplot(gs[2, 2])
    for i in range(4):
        ax.plot(t_deploy, deploy_data['action'][:, i], label=f'{JOINT_NAMES[i]} (D)', alpha=0.7)
        ax.plot(t_sim, sim_data['action'][:, i], linestyle='--', label=f'{JOINT_NAMES[i]} (S)', alpha=0.7)
    ax.set_title('Network Output: HipA')
    ax.legend(fontsize=5)
    ax.grid(True, alpha=0.3)

    plt.suptitle('Deploy vs Simulation Comparison', fontsize=14)
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[Plot] Saved: {save_path}")

    plt.show()


def check_data_validity(data):
    """检查数据有效性"""
    print("\n" + "="*70)
    print("数据有效性检查")
    print("="*70)

    issues = []

    # 1. 检查 NaN 和 Inf
    for key in ['obs_omega', 'obs_gravity', 'obs_joint_pos', 'obs_joint_vel', 'action', 'target_q']:
        arr = data[key]
        nan_count = np.isnan(arr).sum()
        inf_count = np.isinf(arr).sum()
        if nan_count > 0 or inf_count > 0:
            issues.append(f"[{key}] NaN: {nan_count}, Inf: {inf_count}")

    # 2. 检查重力投影
    gz = data['obs_gravity'][:, 2]
    if np.abs(gz.mean() - (-1.0)) > 0.1:
        issues.append(f"[Gravity] gz mean = {gz.mean():.4f}, expected ≈ -1.0")

    # 3. 检查关节位置范围
    joint_pos = data['obs_joint_pos']
    if np.abs(joint_pos).max() > 1.5:
        issues.append(f"[Joint Pos] Max abs value = {np.abs(joint_pos).max():.3f}, may be too large")

    # 4. 检查网络输出范围
    action = data['action']
    if np.abs(action).max() > 10:
        issues.append(f"[Action] Max abs value = {np.abs(action).max():.3f}, may be too large")

    # 5. 检查时间戳连续性
    t = data['t']
    dt = np.diff(t)
    if dt.max() > 0.1:  # 超过100ms
        issues.append(f"[Timestamp] Max dt = {dt.max():.3f}s, may have gaps")

    if issues:
        print("\n⚠️ 发现以下问题:")
        for issue in issues:
            print(f"  - {issue}")
    else:
        print("\n✅ 数据有效性检查通过")

    return len(issues) == 0


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="部署数据解析与对比")
    parser.add_argument("--deploy", type=str, required=True, help="部署日志文件路径")
    parser.add_argument("--sim", type=str, help="仿真数据文件路径 (默认: auto_test_data.npz)")
    parser.add_argument("--no_plot", action="store_true", help="不显示图表")
    args = parser.parse_args()

    # 解析部署数据
    deploy_data = parse_deploy_log(args.deploy)
    if deploy_data is None:
        exit(1)

    # 检查数据有效性
    check_data_validity(deploy_data)

    # 对比仿真数据
    sim_path = Path(args.sim) if args.sim else None
    compare_with_sim(deploy_data, sim_path)

    # 绘制对比图
    if not args.no_plot:
        sim_data = np.load(sim_path if sim_path else Path(__file__).parent / "auto_test_data.npz")
        plot_comparison(deploy_data, sim_data)