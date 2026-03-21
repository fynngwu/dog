#!/usr/bin/env python3
"""
查看自动化测试数据 - 只展示网络输入输出和目标位置
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from pathlib import Path

JOINT_NAMES = ["LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
               "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
               "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee"]

PHASE_NAMES = ["Stand", "Forward", "Backward", "Left", "Right", "TurnL", "TurnR", "Stand"]
PHASE_COLORS = ['gray', 'green', 'red', 'blue', 'cyan', 'orange', 'purple', 'gray']

TEST_SEQUENCE = [
    (1.0, 0.0, 0.0, 0.0),
    (2.0, 1.0, 0.0, 0.0),
    (2.0, -1.0, 0.0, 0.0),
    (1.0, 0.0, 0.5, 0.0),
    (1.0, 0.0, -0.5, 0.0),
    (1.0, 0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0, -1.0),
    (1.0, 0.0, 0.0, 0.0),
]


def plot_data(data_path=None, save_path=None):
    """绘制测试数据 - 只展示网络输入输出和目标位置"""
    if data_path is None:
        data_path = Path(__file__).parent / "auto_test_data.npz"

    data = np.load(data_path)
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

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {save_path}")

    plt.show()


def print_summary(data_path=None):
    """打印数据摘要"""
    if data_path is None:
        data_path = Path(__file__).parent / "auto_test_data.npz"

    data = np.load(data_path)

    print("\n" + "="*60)
    print("Data Summary - Network Input/Output")
    print("="*60)
    print(f"Samples: {len(data['t'])}")
    print(f"Duration: {data['t'][-1]:.2f}s")

    print("\n" + "-"*60)
    print("Network Input (Observation)")
    print("-"*60)

    print("\n[Angular Velocity] (3 dims)")
    print(f"  wx: [{data['obs_omega'][:,0].min():.3f}, {data['obs_omega'][:,0].max():.3f}] rad/s")
    print(f"  wy: [{data['obs_omega'][:,1].min():.3f}, {data['obs_omega'][:,1].max():.3f}] rad/s")
    print(f"  wz: [{data['obs_omega'][:,2].min():.3f}, {data['obs_omega'][:,2].max():.3f}] rad/s")

    print("\n[Projected Gravity] (3 dims)")
    print(f"  gx: [{data['obs_gravity'][:,0].min():.4f}, {data['obs_gravity'][:,0].max():.4f}]")
    print(f"  gy: [{data['obs_gravity'][:,1].min():.4f}, {data['obs_gravity'][:,1].max():.4f}]")
    print(f"  gz: [{data['obs_gravity'][:,2].min():.4f}, {data['obs_gravity'][:,2].max():.4f}]")

    print("\n[Velocity Command] (3 dims)")
    print(f"  cmd_x: [{data['obs_cmd'][:,0].min():.2f}, {data['obs_cmd'][:,0].max():.2f}] m/s")
    print(f"  cmd_y: [{data['obs_cmd'][:,1].min():.2f}, {data['obs_cmd'][:,1].max():.2f}] m/s")
    print(f"  cmd_yaw: [{data['obs_cmd'][:,2].min():.2f}, {data['obs_cmd'][:,2].max():.2f}] rad/s")

    print("\n[Joint Position] (12 dims, relative to default)")
    for i, name in enumerate(JOINT_NAMES):
        print(f"  {name}: [{data['obs_joint_pos'][:,i].min():.3f}, {data['obs_joint_pos'][:,i].max():.3f}] rad")

    print("\n[Joint Velocity] (12 dims)")
    for i, name in enumerate(JOINT_NAMES):
        print(f"  {name}: [{data['obs_joint_vel'][:,i].min():.3f}, {data['obs_joint_vel'][:,i].max():.3f}] rad/s")

    print("\n[Last Action] (12 dims)")
    for i, name in enumerate(JOINT_NAMES):
        print(f"  {name}: [{data['obs_last_action'][:,i].min():.3f}, {data['obs_last_action'][:,i].max():.3f}]")

    print("\n" + "-"*60)
    print("Network Output (Action)")
    print("-"*60)
    for i, name in enumerate(JOINT_NAMES):
        print(f"  {name}: [{data['action'][:,i].min():.3f}, {data['action'][:,i].max():.3f}]")

    print("\n" + "-"*60)
    print("Target Position (Action x 0.25)")
    print("-"*60)
    for i, name in enumerate(JOINT_NAMES):
        print(f"  {name}: [{data['target_q'][:,i].min():.3f}, {data['target_q'][:,i].max():.3f}] rad")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=str, help="Data file path")
    parser.add_argument("--no_plot", action="store_true", help="Don't show plot")
    args = parser.parse_args()

    data_path = Path(args.data) if args.data else None

    print_summary(data_path)

    if not args.no_plot:
        plot_data(data_path)