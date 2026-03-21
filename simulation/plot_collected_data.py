#!/usr/bin/env python3
"""
加载已收集的数据并绘制曲线
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from pathlib import Path
import argparse


def load_data(path):
    """加载npz数据文件"""
    data = np.load(path)
    return {key: data[key] for key in data.files}


def plot_data(data, save_path=None):
    """绘制所有数据曲线"""
    t = data["timestamps"]
    if len(t) == 0:
        print("No data to plot!")
        return

    joint_names = [
        "LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
        "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
        "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee",
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

    # 7-9. 关节位置
    for group_idx, (start, end, title) in enumerate([
        (0, 4, "HipA Joint Position"),
        (4, 8, "HipF Joint Position"),
        (8, 12, "Knee Joint Position"),
    ]):
        ax = fig.add_subplot(gs[2, group_idx])
        for i in range(start, end):
            ax.plot(t, data["joint_pos"][:, i], label=joint_names[i])
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (rad)")
        ax.set_title(title)
        ax.legend(fontsize=6)
        ax.grid(True)

    # 10-12. 关节速度
    for group_idx, (start, end, title) in enumerate([
        (0, 4, "HipA Joint Velocity"),
        (4, 8, "HipF Joint Velocity"),
        (8, 12, "Knee Joint Velocity"),
    ]):
        ax = fig.add_subplot(gs[3, group_idx])
        for i in range(start, end):
            ax.plot(t, data["joint_vel"][:, i], label=joint_names[i])
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (rad/s)")
        ax.set_title(title)
        ax.legend(fontsize=6)
        ax.grid(True)

    # 13-15. 扭矩
    for group_idx, (start, end, title) in enumerate([
        (0, 4, "HipA Torque"),
        (4, 8, "HipF Torque"),
        (8, 12, "Knee Torque"),
    ]):
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
        "LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
        "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
        "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee",
    ]

    fig, axes = plt.subplots(3, 4, figsize=(16, 10))
    fig.suptitle("Policy Action Output Analysis", fontsize=14)

    for i, ax in enumerate(axes.flat):
        if i < 12:
            ax.plot(t, data["action"][:, i], label="Action", color="blue", alpha=0.7)
            ax.plot(t, data["target_q"][:, i], label="Target Q", color="red", alpha=0.7)
            ax.plot(t, data["joint_pos"][:, i], label="Actual Q", color="green", alpha=0.7)
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


def plot_observation_components(data, save_path=None):
    """绘制观测向量各分量"""
    t = data["timestamps"]
    if len(t) == 0:
        print("No data to plot!")
        return

    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle("Policy Observation Components (Single Frame)", fontsize=14)

    # 角速度
    axes[0, 0].plot(t, data["omega"][:, 0], label="ωx")
    axes[0, 0].plot(t, data["omega"][:, 1], label="ωy")
    axes[0, 0].plot(t, data["omega"][:, 2], label="ωz")
    axes[0, 0].set_title("Angular Velocity (3D)")
    axes[0, 0].legend()
    axes[0, 0].grid(True)

    # 重力投影
    axes[0, 1].plot(t, data["gravity"][:, 0], label="gx")
    axes[0, 1].plot(t, data["gravity"][:, 1], label="gy")
    axes[0, 1].plot(t, data["gravity"][:, 2], label="gz")
    axes[0, 1].set_title("Projected Gravity (3D)")
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    # 命令
    axes[0, 2].plot(t, data["commands"][:, 0], label="cmd_x")
    axes[0, 2].plot(t, data["commands"][:, 1], label="cmd_y")
    axes[0, 2].plot(t, data["commands"][:, 2], label="cmd_yaw")
    axes[0, 2].set_title("Commands (3D)")
    axes[0, 2].legend()
    axes[0, 2].grid(True)

    # 关节位置相对值 (需要计算)
    default_pos = np.zeros(12)
    dof_pos_rel = data["joint_pos"] - default_pos
    axes[1, 0].plot(t, dof_pos_rel)
    axes[1, 0].set_title("Joint Position Relative (12D)")
    axes[1, 0].grid(True)

    # 关节速度
    axes[1, 1].plot(t, data["joint_vel"])
    axes[1, 1].set_title("Joint Velocity (12D)")
    axes[1, 1].grid(True)

    # 上一帧动作
    axes[1, 2].plot(t, data["last_action"])
    axes[1, 2].set_title("Last Action (12D)")
    axes[1, 2].grid(True)

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
    if len(data['timestamps']) > 0:
        print(f"Duration: {data['timestamps'][-1]:.2f} s")
        print(f"Sample rate: {len(data['timestamps']) / data['timestamps'][-1]:.1f} Hz")

    print("\n--- Base Position ---")
    print(f"X: min={data['base_pos'][:, 0].min():.3f}, max={data['base_pos'][:, 0].max():.3f}")
    print(f"Y: min={data['base_pos'][:, 1].min():.3f}, max={data['base_pos'][:, 1].max():.3f}")
    print(f"Z: min={data['base_pos'][:, 2].min():.3f}, max={data['base_pos'][:, 2].max():.3f}")

    print("\n--- Base Velocity (Body Frame) ---")
    print(f"Vx: mean={data['base_vel'][:, 0].mean():.3f}, std={data['base_vel'][:, 0].std():.3f}")
    print(f"Vy: mean={data['base_vel'][:, 1].mean():.3f}, std={data['base_vel'][:, 1].std():.3f}")
    print(f"Vz: mean={data['base_vel'][:, 2].mean():.3f}, std={data['base_vel'][:, 2].std():.3f}")

    print("\n--- Commands ---")
    print(f"Cmd X: mean={data['commands'][:, 0].mean():.3f}, max={data['commands'][:, 0].max():.3f}")
    print(f"Cmd Y: mean={data['commands'][:, 1].mean():.3f}, max={data['commands'][:, 1].max():.3f}")
    print(f"Cmd Yaw: mean={data['commands'][:, 2].mean():.3f}, max={data['commands'][:, 2].max():.3f}")

    print("\n--- Joint Torque (Nm) ---")
    print(f"Max torque: {np.abs(data['tau']).max():.2f}")
    print(f"Mean torque: {np.abs(data['tau']).mean():.2f}")

    print("\n--- Observation Dimensions ---")
    print(f"omega: {data['omega'].shape[1]} (angular velocity)")
    print(f"gravity: {data['gravity'].shape[1]} (projected gravity)")
    print(f"commands: {data['commands'].shape[1]} (velocity commands)")
    print(f"joint_pos: {data['joint_pos'].shape[1]} (joint positions)")
    print(f"joint_vel: {data['joint_vel'].shape[1]} (joint velocities)")
    print(f"last_action: {data['last_action'].shape[1]} (previous action)")
    print(f"Total single frame obs: {sum([data[k].shape[1] for k in ['omega', 'gravity', 'commands', 'joint_pos', 'joint_vel', 'last_action']])}")


def main():
    parser = argparse.ArgumentParser(description="Plot collected S2S data")
    parser.add_argument("--data", type=str, default="collected_data.npz", help="Path to data file")
    parser.add_argument("--save", action="store_true", help="Save plots to files")
    parser.add_argument("--no_show", action="store_true", help="Don't display plots")
    args = parser.parse_args()

    data_path = Path(__file__).parent / args.data
    if not data_path.exists():
        print(f"Data file not found: {data_path}")
        return

    print(f"Loading data from {data_path}...")
    data = load_data(data_path)

    print_data_summary(data)

    if args.no_show:
        import matplotlib
        matplotlib.use('Agg')

    save_dir = Path(__file__).parent if args.save else None

    print("\nGenerating plots...")
    plot_data(data, save_path=save_dir / "data_plot.png" if save_dir else None)
    plot_action_analysis(data, save_path=save_dir / "action_plot.png" if save_dir else None)
    plot_observation_components(data, save_path=save_dir / "obs_plot.png" if save_dir else None)


if __name__ == "__main__":
    main()