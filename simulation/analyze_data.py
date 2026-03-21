#!/usr/bin/env python3
"""
数据对比分析脚本
对比仿真数据和真实部署数据，找出差异点
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from pathlib import Path
import argparse

# 关节名称 (Policy 顺序)
JOINT_NAMES_POLICY = [
    "LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
    "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
    "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee",
]


def load_sim_data(filepath):
    """加载仿真数据"""
    data = np.load(filepath)
    print(f"[Sim] Loaded {filepath}")
    print(f"[Sim] Keys: {list(data.keys())}")
    return data


def load_deploy_data(filepath, obs_dim=450, action_dim=12):
    """
    加载真实部署数据 (二进制格式)

    假设格式:
    - 每帧: [obs_dim floats] + [action_dim floats]
    - 总共: frame_count * (obs_dim + action_dim) * 4 bytes
    """
    data = np.fromfile(filepath, dtype=np.float32)

    frame_size = obs_dim + action_dim
    frame_count = len(data) // frame_size

    if frame_count == 0:
        print(f"[Deploy] No valid frames in {filepath}")
        return None

    data = data[:frame_count * frame_size].reshape(frame_count, frame_size)

    obs = data[:, :obs_dim]
    action = data[:, obs_dim:obs_dim + action_dim]

    print(f"[Deploy] Loaded {filepath}")
    print(f"[Deploy] Frames: {frame_count}")

    return {
        'obs': obs,
        'action': action,
        'frame_count': frame_count,
    }


def parse_single_obs(obs_array):
    """
    解析单帧观测数据 (45维)

    格式:
    - angular_vel: 3
    - gravity: 3
    - commands: 3
    - joint_pos: 12
    - joint_vel: 12
    - last_action: 12
    """
    return {
        'angular_vel': obs_array[0:3],
        'gravity': obs_array[3:6],
        'commands': obs_array[6:9],
        'joint_pos': obs_array[9:21],
        'joint_vel': obs_array[21:33],
        'last_action': obs_array[33:45],
    }


def compare_data(sim_data, deploy_data, output_dir=None):
    """对比仿真和部署数据"""

    print("\n" + "="*70)
    print("数据对比分析")
    print("="*70)

    # 1. 对比角速度
    print("\n--- 1. 角速度对比 ---")
    sim_omega = sim_data['obs_angular_vel']
    deploy_omega = deploy_data['obs'][:, 0:3]

    print(f"仿真角速度统计:")
    print(f"  ωx: mean={sim_omega[:,0].mean():.4f}, std={sim_omega[:,0].std():.4f}")
    print(f"  ωy: mean={sim_omega[:,1].mean():.4f}, std={sim_omega[:,1].std():.4f}")
    print(f"  ωz: mean={sim_omega[:,2].mean():.4f}, std={sim_omega[:,2].std():.4f}")

    print(f"\n部署角速度统计:")
    print(f"  ωx: mean={deploy_omega[:,0].mean():.4f}, std={deploy_omega[:,0].std():.4f}")
    print(f"  ωy: mean={deploy_omega[:,1].mean():.4f}, std={deploy_omega[:,1].std():.4f}")
    print(f"  ωz: mean={deploy_omega[:,2].mean():.4f}, std={deploy_omega[:,2].std():.4f}")

    # 2. 对比重力投影
    print("\n--- 2. 重力投影对比 ---")
    sim_gravity = sim_data['obs_gravity']
    deploy_gravity = deploy_data['obs'][:, 3:6]

    print(f"仿真重力投影:")
    print(f"  gx: mean={sim_gravity[:,0].mean():.4f}, std={sim_gravity[:,0].std():.4f}")
    print(f"  gy: mean={sim_gravity[:,1].mean():.4f}, std={sim_gravity[:,1].std():.4f}")
    print(f"  gz: mean={sim_gravity[:,2].mean():.4f}, std={sim_gravity[:,2].std():.4f}")

    print(f"\n部署重力投影:")
    print(f"  gx: mean={deploy_gravity[:,0].mean():.4f}, std={deploy_gravity[:,0].std():.4f}")
    print(f"  gy: mean={deploy_gravity[:,1].mean():.4f}, std={deploy_gravity[:,1].std():.4f}")
    print(f"  gz: mean={deploy_gravity[:,2].mean():.4f}, std={deploy_gravity[:,2].std():.4f}")

    # 3. 对比关节位置
    print("\n--- 3. 关节位置对比 ---")
    sim_joint_pos = sim_data['obs_joint_pos']
    deploy_joint_pos = deploy_data['obs'][:, 9:21]

    print("仿真关节位置 (Policy顺序):")
    for i, name in enumerate(JOINT_NAMES_POLICY):
        print(f"  {name}: mean={sim_joint_pos[:,i].mean():.4f}, std={sim_joint_pos[:,i].std():.4f}")

    print("\n部署关节位置 (Policy顺序):")
    for i, name in enumerate(JOINT_NAMES_POLICY):
        print(f"  {name}: mean={deploy_joint_pos[:,i].mean():.4f}, std={deploy_joint_pos[:,i].std():.4f}")

    # 4. 对比网络输出
    print("\n--- 4. 网络输出对比 ---")
    sim_action = sim_data['action_raw']
    deploy_action = deploy_data['action']

    print("仿真网络输出:")
    for i, name in enumerate(JOINT_NAMES_POLICY):
        print(f"  {name}: mean={sim_action[:,i].mean():.4f}, std={sim_action[:,i].std():.4f}")

    print("\n部署网络输出:")
    for i, name in enumerate(JOINT_NAMES_POLICY):
        print(f"  {name}: mean={deploy_action[:,i].mean():.4f}, std={deploy_action[:,i].std():.4f}")


def plot_comparison(sim_data, deploy_data, output_path=None):
    """绘制对比图"""

    fig = plt.figure(figsize=(20, 16))
    gs = GridSpec(4, 3, figure=fig, hspace=0.3, wspace=0.3)

    # 时间轴
    t_sim = np.arange(len(sim_data['timestamps'])) * 0.02  # 50Hz
    t_deploy = np.arange(deploy_data['frame_count']) * 0.02

    # 1. 角速度对比
    for idx, (name, color) in enumerate([('ωx', 'r'), ('ωy', 'g'), ('ωz', 'b')]):
        ax = fig.add_subplot(gs[0, idx])
        ax.plot(t_sim, sim_data['obs_angular_vel'][:, idx], label='Sim', color=color, alpha=0.7)
        ax.plot(t_deploy, deploy_data['obs'][:, idx], label='Deploy', color=color, linestyle='--', alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(name)
        ax.set_title(f'Angular Velocity: {name}')
        ax.legend()
        ax.grid(True)

    # 2. 重力投影对比
    for idx, (name, color) in enumerate([('gx', 'r'), ('gy', 'g'), ('gz', 'b')]):
        ax = fig.add_subplot(gs[1, idx])
        ax.plot(t_sim, sim_data['obs_gravity'][:, idx], label='Sim', color=color, alpha=0.7)
        ax.plot(t_deploy, deploy_data['obs'][:, 3+idx], label='Deploy', color=color, linestyle='--', alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(name)
        ax.set_title(f'Gravity Projection: {name}')
        ax.legend()
        ax.grid(True)

    # 3. 关节位置对比 (HipA)
    ax = fig.add_subplot(gs[2, 0])
    for i in range(4):
        ax.plot(t_sim, sim_data['obs_joint_pos'][:, i], label=f'{JOINT_NAMES_POLICY[i]} (Sim)')
    ax.set_title('Sim: HipA Joint Position')
    ax.legend(fontsize=6)
    ax.grid(True)

    ax = fig.add_subplot(gs[2, 1])
    for i in range(4):
        ax.plot(t_deploy, deploy_data['obs'][:, 9+i], label=f'{JOINT_NAMES_POLICY[i]} (Deploy)')
    ax.set_title('Deploy: HipA Joint Position')
    ax.legend(fontsize=6)
    ax.grid(True)

    # 4. 网络输出对比
    ax = fig.add_subplot(gs[2, 2])
    for i in range(4):
        ax.plot(t_sim, sim_data['action_raw'][:, i], label=f'{JOINT_NAMES_POLICY[i]} (Sim)')
    ax.set_title('Sim: Action (HipA)')
    ax.legend(fontsize=6)
    ax.grid(True)

    ax = fig.add_subplot(gs[3, 0])
    for i in range(4):
        ax.plot(t_deploy, deploy_data['action'][:, i], label=f'{JOINT_NAMES_POLICY[i]} (Deploy)')
    ax.set_title('Deploy: Action (HipA)')
    ax.legend(fontsize=6)
    ax.grid(True)

    # 5. 关节速度对比
    ax = fig.add_subplot(gs[3, 1])
    for i in range(4):
        ax.plot(t_sim, sim_data['obs_joint_vel'][:, i], label=f'{JOINT_NAMES_POLICY[i]} (Sim)')
    ax.set_title('Sim: HipA Joint Velocity')
    ax.legend(fontsize=6)
    ax.grid(True)

    ax = fig.add_subplot(gs[3, 2])
    for i in range(4):
        ax.plot(t_deploy, deploy_data['obs'][:, 21+i], label=f'{JOINT_NAMES_POLICY[i]} (Deploy)')
    ax.set_title('Deploy: HipA Joint Velocity')
    ax.legend(fontsize=6)
    ax.grid(True)

    plt.suptitle('Simulation vs Deployment Data Comparison', fontsize=14)
    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150)
        print(f"[Plot] Saved to {output_path}")

    plt.show()


def plot_sim_data(sim_data, output_path=None):
    """绘制仿真数据详细图"""

    t = np.arange(len(sim_data['timestamps'])) * 0.02

    fig = plt.figure(figsize=(20, 24))
    gs = GridSpec(6, 3, figure=fig, hspace=0.3, wspace=0.3)

    # 1. 基座位置
    ax = fig.add_subplot(gs[0, 0])
    ax.plot(t, sim_data['base_pos'][:, 0], label='X', color='r')
    ax.plot(t, sim_data['base_pos'][:, 1], label='Y', color='g')
    ax.plot(t, sim_data['base_pos'][:, 2], label='Z', color='b')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (m)')
    ax.set_title('Base Position')
    ax.legend()
    ax.grid(True)

    # 2. 基座速度
    ax = fig.add_subplot(gs[0, 1])
    ax.plot(t, sim_data['base_vel_body'][:, 0], label='Vx', color='r')
    ax.plot(t, sim_data['base_vel_body'][:, 1], label='Vy', color='g')
    ax.plot(t, sim_data['base_vel_body'][:, 2], label='Vz', color='b')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Base Velocity (Body Frame)')
    ax.legend()
    ax.grid(True)

    # 3. 命令 vs 实际
    ax = fig.add_subplot(gs[0, 2])
    ax.plot(t, sim_data['obs_commands'][:, 0], label='Cmd X', color='r', linestyle='--')
    ax.plot(t, sim_data['base_vel_body'][:, 0], label='Real X', color='r')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Command vs Real Velocity')
    ax.legend()
    ax.grid(True)

    # 4. 角速度
    ax = fig.add_subplot(gs[1, 0])
    ax.plot(t, sim_data['obs_angular_vel'][:, 0], label='ωx', color='r')
    ax.plot(t, sim_data['obs_angular_vel'][:, 1], label='ωy', color='g')
    ax.plot(t, sim_data['obs_angular_vel'][:, 2], label='ωz', color='b')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angular Velocity (rad/s)')
    ax.set_title('Angular Velocity (Network Input)')
    ax.legend()
    ax.grid(True)

    # 5. 重力投影
    ax = fig.add_subplot(gs[1, 1])
    ax.plot(t, sim_data['obs_gravity'][:, 0], label='gx', color='r')
    ax.plot(t, sim_data['obs_gravity'][:, 1], label='gy', color='g')
    ax.plot(t, sim_data['obs_gravity'][:, 2], label='gz', color='b')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Gravity Projection')
    ax.set_title('Projected Gravity (Network Input)')
    ax.legend()
    ax.grid(True)

    # 6. 足端接触
    ax = fig.add_subplot(gs[1, 2])
    foot_names = ['LF', 'LR', 'RF', 'RR']
    colors = ['r', 'g', 'b', 'orange']
    for i, (name, color) in enumerate(zip(foot_names, colors)):
        ax.plot(t, sim_data['foot_contact'][:, i], label=name, color=color)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Contact')
    ax.set_title('Foot Contact')
    ax.legend()
    ax.grid(True)

    # 7-9. 关节位置
    for group_idx, (start, end, title) in enumerate([
        (0, 4, 'HipA Joint Position'),
        (4, 8, 'HipF Joint Position'),
        (8, 12, 'Knee Joint Position'),
    ]):
        ax = fig.add_subplot(gs[2, group_idx])
        for i in range(start, end):
            ax.plot(t, sim_data['obs_joint_pos'][:, i], label=JOINT_NAMES_POLICY[i])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (rad)')
        ax.set_title(f'{title} (Network Input)')
        ax.legend(fontsize=6)
        ax.grid(True)

    # 10-12. 关节速度
    for group_idx, (start, end, title) in enumerate([
        (0, 4, 'HipA Joint Velocity'),
        (4, 8, 'HipF Joint Velocity'),
        (8, 12, 'Knee Joint Velocity'),
    ]):
        ax = fig.add_subplot(gs[3, group_idx])
        for i in range(start, end):
            ax.plot(t, sim_data['obs_joint_vel'][:, i], label=JOINT_NAMES_POLICY[i])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (rad/s)')
        ax.set_title(f'{title} (Network Input)')
        ax.legend(fontsize=6)
        ax.grid(True)

    # 13-15. 网络输出 (原始动作)
    for group_idx, (start, end, title) in enumerate([
        (0, 4, 'HipA Action'),
        (4, 8, 'HipF Action'),
        (8, 12, 'Knee Action'),
    ]):
        ax = fig.add_subplot(gs[4, group_idx])
        for i in range(start, end):
            ax.plot(t, sim_data['action_raw'][:, i], label=JOINT_NAMES_POLICY[i])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Action')
        ax.set_title(f'{title} (Network Output)')
        ax.legend(fontsize=6)
        ax.grid(True)

    # 16-18. 扭矩
    for group_idx, (start, end, title) in enumerate([
        (0, 4, 'HipA Torque'),
        (4, 8, 'HipF Torque'),
        (8, 12, 'Knee Torque'),
    ]):
        ax = fig.add_subplot(gs[5, group_idx])
        for i in range(start, end):
            ax.plot(t, sim_data['tau'][:, i], label=JOINT_NAMES_POLICY[i])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Torque (Nm)')
        ax.set_title(title)
        ax.legend(fontsize=6)
        ax.grid(True)

    plt.suptitle('Simulation Data Details', fontsize=14)
    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150)
        print(f"[Plot] Saved to {output_path}")

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="数据对比分析")
    parser.add_argument("--sim_data", type=str, default="sim_data.npz", help="仿真数据文件 (.npz)")
    parser.add_argument("--deploy_data", type=str, help="部署数据文件 (.bin)")
    parser.add_argument("--plot_sim", action="store_true", help="仅绘制仿真数据")
    parser.add_argument("--output", type=str, default="comparison.png", help="输出图片路径")
    args = parser.parse_args()

    # 获取脚本所在目录
    script_dir = Path(__file__).parent

    # 处理相对路径
    sim_data_path = Path(args.sim_data)
    if not sim_data_path.is_absolute():
        sim_data_path = script_dir / sim_data_path

    if args.plot_sim:
        if sim_data_path.exists():
            sim_data = load_sim_data(sim_data_path)
            plot_sim_data(sim_data, script_dir / args.output)
        else:
            print(f"文件不存在: {sim_data_path}")
    elif args.deploy_data:
        deploy_data_path = Path(args.deploy_data)
        if not deploy_data_path.is_absolute():
            deploy_data_path = script_dir / deploy_data_path

        if sim_data_path.exists() and deploy_data_path.exists():
            sim_data = load_sim_data(sim_data_path)
            deploy_data = load_deploy_data(deploy_data_path)
            if deploy_data:
                compare_data(sim_data, deploy_data)
                plot_comparison(sim_data, deploy_data, script_dir / args.output)
        else:
            if not sim_data_path.exists():
                print(f"仿真数据文件不存在: {sim_data_path}")
            if not deploy_data_path.exists():
                print(f"部署数据文件不存在: {deploy_data_path}")
    else:
        print("用法:")
        print("  仅查看仿真数据: uv run python analyze_data.py --plot_sim")
        print("  对比数据:       uv run python analyze_data.py --deploy_data /path/to/deploy_log.bin")