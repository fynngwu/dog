#!/usr/bin/env python3
"""
日志可视化分析工具

用法:
    python analyze_log.py <log_dir> [options]

示例:
    python analyze_log.py logs/session_1234567890
    python analyze_log.py logs/session_1234567890 --plot action
    python analyze_log.py logs/session_1234567890 --plot joint --joint 0
    python analyze_log.py logs/session_1234567890 --stat all
"""

import argparse
import os
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# 关节名称
JOINT_NAMES = [
    'LF_HipA', 'LF_HipF', 'LF_Knee',
    'LR_HipA', 'LR_HipF', 'LR_Knee',
    'RF_HipA', 'RF_HipF', 'RF_Knee',
    'RR_HipA', 'RR_HipF', 'RR_Knee'
]

# 观测维度布局
OBS_LAYOUT = {
    'gyro': (0, 3),        # 角速度
    'gravity': (3, 6),     # 重力投影
    'command': (6, 9),     # 命令
    'joint_pos': (9, 21),  # 关节位置
    'joint_vel': (21, 33), # 关节速度
    'prev_action': (33, 45) # 上一拍动作
}


def parse_vector(s):
    """解析逗号分隔的向量字符串"""
    if pd.isna(s) or s == '':
        return np.array([])
    return np.array([float(x) for x in s.split(',')])


def load_log(log_dir):
    """加载日志数据"""
    log_path = Path(log_dir)
    
    # 加载 CSV
    csv_path = log_path / 'frames.csv'
    if not csv_path.exists():
        raise FileNotFoundError(f"frames.csv not found in {log_dir}")
    
    df = pd.read_csv(csv_path)
    
    # 加载 meta.json
    meta_path = log_path / 'meta.json'
    meta = {}
    if meta_path.exists():
        with open(meta_path, 'r') as f:
            meta = json.load(f)
    
    # 加载 events.log
    events_path = log_path / 'events.log'
    events = []
    if events_path.exists():
        with open(events_path, 'r') as f:
            events = f.readlines()
    
    return df, meta, events


def parse_vector_columns(df):
    """解析向量列"""
    vector_cols = [
        'single_obs', 'raw_action', 'desired_abs', 'clipped_abs',
        'cmd_sent_abs', 'cmd_minus_pos', 'cmd_delta',
        'motor_pos_abs', 'motor_vel_abs', 'motor_torque'
    ]
    
    parsed = {}
    for col in vector_cols:
        if col in df.columns:
            parsed[col] = np.array([parse_vector(x) for x in df[col]])
    
    return parsed


def print_summary(df, parsed, meta):
    """打印摘要信息"""
    print("=" * 60)
    print("日志摘要")
    print("=" * 60)
    
    # 基本信息
    print(f"\n模式: {meta.get('mode', 'unknown')}")
    print(f"输入源: {meta.get('input', 'unknown')}")
    print(f"总帧数: {len(df)}")
    print(f"运行时间: {df['t_sec'].iloc[-1]:.2f} 秒")
    
    # 控制统计
    if 'control_enabled' in df.columns:
        ctrl_frames = df['control_enabled'].sum()
        print(f"控制启用帧数: {ctrl_frames} ({100*ctrl_frames/len(df):.1f}%)")
    
    # 推理统计
    if 'infer_ms' in df.columns:
        print(f"\n推理耗时: mean={df['infer_ms'].mean():.2f}ms, max={df['infer_ms'].max():.2f}ms")
    
    # 跟踪误差
    if 'max_track_err' in df.columns:
        print(f"最大跟踪误差: mean={df['max_track_err'].mean():.4f}, max={df['max_track_err'].max():.4f} rad")
    
    # Clamp 统计
    if 'clamp_count' in df.columns:
        total_clamp = df['clamp_count'].sum()
        print(f"Clamp 总次数: {total_clamp}")
    
    # 传感器状态
    if 'imu_fresh' in df.columns:
        imu_ok = df['imu_fresh'].mean() * 100
        print(f"IMU 新鲜率: {imu_ok:.1f}%")
    if 'motors_fresh' in df.columns:
        motors_ok = df['motors_fresh'].mean() * 100
        print(f"电机新鲜率: {motors_ok:.1f}%")
    
    # Action 统计
    if 'raw_action' in parsed and len(parsed['raw_action']) > 0:
        actions = np.vstack(parsed['raw_action'])
        print(f"\nAction 统计:")
        print(f"  max_abs: {np.abs(actions).max():.4f}")
        print(f"  mean_abs: {np.abs(actions).mean():.4f}")
        print(f"  各关节 max_abs: {np.abs(actions).max(axis=0).round(3)}")
    
    print("=" * 60)


def plot_timeline(df, parsed, save_path=None):
    """绘制时间线图"""
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    
    t = df['t_sec']
    
    # 1. 推理耗时和控制状态
    ax = axes[0]
    ax.plot(t, df['infer_ms'], 'b-', alpha=0.7, label='infer_ms')
    ax.set_ylabel('Infer Time (ms)')
    ax.legend(loc='upper right')
    ax.set_title('Inference Time & Control Status')
    
    # 标记控制启用区域
    if 'control_enabled' in df.columns:
        ctrl_on = df['control_enabled'] == 1
        for i in range(len(ctrl_on)):
            if ctrl_on.iloc[i]:
                ax.axvspan(t.iloc[i], t.iloc[min(i+1, len(t)-1)], alpha=0.2, color='green')
    
    # 2. 跟踪误差和 clamp
    ax = axes[1]
    if 'max_track_err' in df.columns:
        ax.plot(t, df['max_track_err'], 'r-', label='max_track_err')
    if 'clamp_count' in df.columns:
        ax2 = ax.twinx()
        ax2.plot(t, df['clamp_count'], 'orange', alpha=0.7, label='clamp_count')
        ax2.set_ylabel('Clamp Count', color='orange')
    ax.set_ylabel('Max Track Err (rad)')
    ax.legend(loc='upper left')
    ax.set_title('Tracking Error & Clamp Count')
    
    # 3. Action 范围
    ax = axes[2]
    if 'raw_action' in parsed and len(parsed['raw_action']) > 0:
        actions = np.vstack(parsed['raw_action'])
        ax.plot(t, np.abs(actions).max(axis=1), 'purple', label='max_abs_action')
        ax.plot(t, np.abs(actions).mean(axis=1), 'purple', alpha=0.5, label='mean_abs_action')
    ax.set_ylabel('Action Magnitude')
    ax.legend(loc='upper right')
    ax.set_title('Action Statistics')
    
    # 4. 传感器状态
    ax = axes[3]
    if 'imu_fresh' in df.columns:
        ax.plot(t, df['imu_fresh'], 'b-', alpha=0.7, label='imu_fresh')
    if 'motors_fresh' in df.columns:
        ax.plot(t, df['motors_fresh'], 'g-', alpha=0.7, label='motors_fresh')
    ax.set_ylabel('Sensor Status')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right')
    ax.set_title('Sensor Freshness')
    ax.set_ylim(-0.1, 1.1)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"保存图片: {save_path}")
    
    plt.show()


def plot_action(df, parsed, save_path=None):
    """绘制 action 详情"""
    if 'raw_action' not in parsed or len(parsed['raw_action']) == 0:
        print("没有 action 数据")
        return
    
    actions = np.vstack(parsed['raw_action'])
    t = df['t_sec']
    
    fig, axes = plt.subplots(3, 4, figsize=(16, 10), sharex=True)
    axes = axes.flatten()
    
    for i in range(12):
        ax = axes[i]
        ax.plot(t, actions[:, i], 'b-', alpha=0.7)
        ax.set_title(JOINT_NAMES[i])
        ax.set_ylabel('Action')
        ax.grid(True, alpha=0.3)
    
    for ax in axes[8:]:
        ax.set_xlabel('Time (s)')
    
    plt.suptitle('Raw Action per Joint', fontsize=14)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"保存图片: {save_path}")
    
    plt.show()


def plot_joint(df, parsed, joint_idx=0, save_path=None):
    """绘制单个关节的详细数据"""
    if 'motor_pos_abs' not in parsed or len(parsed['motor_pos_abs']) == 0:
        print("没有关节数据")
        return
    
    t = df['t_sec']
    
    motor_pos = np.vstack(parsed['motor_pos_abs'])[:, joint_idx]
    motor_vel = np.vstack(parsed['motor_vel_abs'])[:, joint_idx]
    motor_torque = np.vstack(parsed['motor_torque'])[:, joint_idx]
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    
    # 位置
    ax = axes[0]
    ax.plot(t, motor_pos, 'b-', label='motor_pos')
    if 'cmd_sent_abs' in parsed and len(parsed['cmd_sent_abs']) > 0:
        cmd_sent = np.vstack(parsed['cmd_sent_abs'])[:, joint_idx]
        ax.plot(t, cmd_sent, 'r--', alpha=0.7, label='cmd_sent')
    if 'desired_abs' in parsed and len(parsed['desired_abs']) > 0:
        desired = np.vstack(parsed['desired_abs'])[:, joint_idx]
        ax.plot(t, desired, 'g:', alpha=0.7, label='desired')
    ax.set_ylabel('Position (rad)')
    ax.legend(loc='upper right')
    ax.set_title(f'{JOINT_NAMES[joint_idx]} - Position')
    ax.grid(True, alpha=0.3)
    
    # 速度
    ax = axes[1]
    ax.plot(t, motor_vel, 'b-')
    ax.set_ylabel('Velocity (rad/s)')
    ax.set_title('Velocity')
    ax.grid(True, alpha=0.3)
    
    # 力矩
    ax = axes[2]
    ax.plot(t, motor_torque, 'b-')
    ax.set_ylabel('Torque (Nm)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Torque')
    ax.grid(True, alpha=0.3)
    
    plt.suptitle(f'Joint {joint_idx} ({JOINT_NAMES[joint_idx]}) Detail', fontsize=14)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"保存图片: {save_path}")
    
    plt.show()


def plot_tracking_error(df, parsed, save_path=None):
    """绘制跟踪误差热图"""
    if 'cmd_minus_pos' not in parsed or len(parsed['cmd_minus_pos']) == 0:
        print("没有跟踪误差数据")
        return
    
    t = df['t_sec']
    err = np.vstack(parsed['cmd_minus_pos'])
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # 热图
    ax = axes[0]
    im = ax.imshow(err.T, aspect='auto', cmap='RdBu_r', 
                   extent=[t.iloc[0], t.iloc[-1], 11.5, -0.5])
    ax.set_yticks(range(12))
    ax.set_yticklabels(JOINT_NAMES)
    ax.set_xlabel('Time (s)')
    ax.set_title('Tracking Error Heatmap (cmd - pos)')
    plt.colorbar(im, ax=ax, label='Error (rad)')
    
    # 误差分布
    ax = axes[1]
    ax.boxplot(err, labels=JOINT_NAMES, vert=True)
    ax.set_ylabel('Tracking Error (rad)')
    ax.set_title('Tracking Error Distribution per Joint')
    ax.tick_params(axis='x', rotation=45)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"保存图片: {save_path}")
    
    plt.show()


def plot_obs(df, parsed, save_path=None):
    """绘制观测数据"""
    if 'single_obs' not in parsed or len(parsed['single_obs']) == 0:
        print("没有观测数据")
        return
    
    obs = np.vstack(parsed['single_obs'])
    t = df['t_sec']
    
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    
    # Gyro
    ax = axes[0, 0]
    ax.plot(t, obs[:, 0], 'r-', label='gyro_x (forward)')
    ax.plot(t, obs[:, 1], 'g-', label='gyro_y (left)')
    ax.plot(t, obs[:, 2], 'b-', label='gyro_z (up)')
    ax.set_ylabel('Angular Velocity (rad/s)')
    ax.legend(loc='upper right')
    ax.set_title('Gyroscope')
    ax.grid(True, alpha=0.3)
    
    # Gravity
    ax = axes[0, 1]
    ax.plot(t, obs[:, 3], 'r-', label='grav_x')
    ax.plot(t, obs[:, 4], 'g-', label='grav_y')
    ax.plot(t, obs[:, 5], 'b-', label='grav_z')
    ax.set_ylabel('Gravity Projection')
    ax.legend(loc='upper right')
    ax.set_title('Projected Gravity')
    ax.grid(True, alpha=0.3)
    
    # Command
    ax = axes[1, 0]
    ax.plot(t, obs[:, 6], 'r-', label='vx')
    ax.plot(t, obs[:, 7], 'g-', label='vy')
    ax.plot(t, obs[:, 8], 'b-', label='yaw')
    ax.set_ylabel('Command')
    ax.legend(loc='upper right')
    ax.set_title('Command')
    ax.grid(True, alpha=0.3)
    
    # Joint Position (mean and std)
    ax = axes[1, 1]
    joint_pos = obs[:, 9:21]
    ax.plot(t, joint_pos.mean(axis=1), 'b-', label='mean')
    ax.fill_between(t, joint_pos.min(axis=1), joint_pos.max(axis=1), alpha=0.3, label='range')
    ax.set_ylabel('Joint Position (rad)')
    ax.legend(loc='upper right')
    ax.set_title('Joint Positions (relative)')
    ax.grid(True, alpha=0.3)
    
    # Joint Velocity
    ax = axes[2, 0]
    joint_vel = obs[:, 21:33]
    ax.plot(t, joint_vel.mean(axis=1), 'b-', label='mean')
    ax.fill_between(t, joint_vel.min(axis=1), joint_vel.max(axis=1), alpha=0.3, label='range')
    ax.set_ylabel('Joint Velocity (rad/s)')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right')
    ax.set_title('Joint Velocities')
    ax.grid(True, alpha=0.3)
    
    # Prev Action
    ax = axes[2, 1]
    prev_action = obs[:, 33:45]
    ax.plot(t, prev_action.mean(axis=1), 'b-', label='mean')
    ax.fill_between(t, prev_action.min(axis=1), prev_action.max(axis=1), alpha=0.3, label='range')
    ax.set_ylabel('Previous Action')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right')
    ax.set_title('Previous Actions')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"保存图片: {save_path}")
    
    plt.show()


def check_clamp(df, parsed):
    """检查 clamp 情况"""
    if 'desired_abs' not in parsed or 'clipped_abs' not in parsed:
        print("没有 desired/clipped 数据")
        return
    
    desired = np.vstack(parsed['desired_abs'])
    clipped = np.vstack(parsed['clipped_abs'])
    
    diff = np.abs(desired - clipped)
    clamp_mask = diff > 0.001
    
    print("\n" + "=" * 60)
    print("Clamp 分析")
    print("=" * 60)
    
    for i in range(12):
        clamp_count = clamp_mask[:, i].sum()
        if clamp_count > 0:
            mean_diff = diff[clamp_mask[:, i], i].mean()
            max_diff = diff[:, i].max()
            print(f"  {JOINT_NAMES[i]:10s}: clamp {clamp_count:4d} 次, mean_diff={mean_diff:.4f}, max_diff={max_diff:.4f}")
    
    if clamp_mask.sum() == 0:
        print("  没有检测到 clamp")
    
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description='日志可视化分析工具')
    parser.add_argument('log_dir', help='日志目录路径')
    parser.add_argument('--plot', choices=['timeline', 'action', 'joint', 'tracking', 'obs', 'all'],
                        default='timeline', help='绘制类型')
    parser.add_argument('--joint', type=int, default=0, help='关节索引 (0-11)')
    parser.add_argument('--stat', choices=['summary', 'clamp', 'all'], default='summary',
                        help='统计类型')
    parser.add_argument('--save', action='store_true', help='保存图片到日志目录')
    
    args = parser.parse_args()
    
    # 加载数据
    print(f"加载日志: {args.log_dir}")
    df, meta, events = load_log(args.log_dir)
    parsed = parse_vector_columns(df)
    
    # 统计
    if args.stat in ['summary', 'all']:
        print_summary(df, parsed, meta)
    if args.stat in ['clamp', 'all']:
        check_clamp(df, parsed)
    
    # 绘图
    save_path = None
    if args.save:
        save_path = os.path.join(args.log_dir, f'{args.plot}.png')
    
    if args.plot == 'timeline' or args.plot == 'all':
        plot_timeline(df, parsed, save_path)
    if args.plot == 'action' or args.plot == 'all':
        plot_action(df, parsed, save_path)
    if args.plot == 'joint' or args.plot == 'all':
        plot_joint(df, parsed, args.joint, save_path)
    if args.plot == 'tracking' or args.plot == 'all':
        plot_tracking_error(df, parsed, save_path)
    if args.plot == 'obs' or args.plot == 'all':
        plot_obs(df, parsed, save_path)


if __name__ == '__main__':
    main()