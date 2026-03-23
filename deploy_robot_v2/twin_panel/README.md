# twin_panel

四足机器人数字孪生验证工具包。

## 架构：双线程直连模式

```
┌─────────────────────────────────────────────┐
│           Qt 主线程 (UI)                     │
│  ┌─────────┐  ┌─────────┐  ┌─────────────┐  │
│  │ Sliders │  │ Buttons │  │   Table     │  │
│  └────┬────┘  └────┬────┘  └─────────────┘  │
│       │            │                         │
│       ▼            ▼                         │
│  set_action()  send_command()                │
└───────┬─────────────┬───────────────────────┘
        │             │
        ▼             ▼
┌───────────────┐  ┌───────────────────────┐
│ Sim Worker    │  │ SSH Worker            │
│ (MuJoCo)      │  │ (hw_action_cli)       │
│               │  │                       │
│ mj_step()     │  │ ssh robot@ip set ...  │
│ state_ready   │  │ state_ready           │
└───────────────┘  └───────────────────────┘
```

**关键设计**：
- 控制链路：Qt → SSH Worker（直连）
- ROS2：可选，仅做观测转发，不影响控制

## 文件结构

```
local/
├── __init__.py
├── twin_topics.py          # 常量定义
├── ui_models.py            # 数据模型
├── sim_bridge_worker.py    # MuJoCo 仿真线程
├── hw_ssh_bridge_worker.py # SSH 调用线程
├── ros_qt_node.py          # ROS2 节点（可选，观测转发）
├── qt_ros2_twin_panel.py   # 主面板（入口）
└── robot_fixed.xml         # 固定基座模型
```

## 运行

```bash
# 基本运行
uv run python local/qt_ros2_twin_panel.py --xml local/robot_fixed.xml

# 自定义 SSH 目标
uv run python local/qt_ros2_twin_panel.py --ssh robot@192.168.1.2 --xml local/robot_fixed.xml

# 启用 ROS2 观测转发（需先 source ROS2 环境）
source /opt/ros/humble/setup.bash
uv run python local/qt_ros2_twin_panel.py --xml local/robot_fixed.xml
```

## 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--ssh` | `ares@10.20.105.185` | SSH 目标 |
| `--xml` | `local/robot_fixed.xml` | MuJoCo 模型路径 |
| `--no-ros2` | - | 禁用 ROS2（默认自动检测） |

## UI 说明

### 表格列

| 列 | 说明 |
|----|------|
| Joint | 关节名称 |
| Slider | 滑动条 |
| Target | 目标关节相对角 (rad) |
| Sim | MuJoCo 关节相对角 (rad) |
| HW | 真机关节相对角 (rad) |
| Diff | HW - Sim 差值 (rad) |

### 状态栏

- ✓ 绿色：成功，显示 RTT 和在线电机数
- ⚠ 橙色：成功但反馈不新鲜
- ✗ 红色：失败，显示错误信息

### 按钮

| 按钮 | 命令 |
|------|------|
| Enable | 使能电机 |
| Go Offset | 移动到 offset 姿态 |
| Disable | 失能电机 |
| Hold | 保持当前姿态 |
| Get State | 读取状态 |
| Auto Report | 开启自动上报 |
| Stop Report | 关闭自动上报 |
| Zero All | 清零所有 slider |

## 注意事项

1. **SSH 超时**：默认 4 秒，关闭窗口时会等待 5 秒
2. **发布频率**：5 Hz，适合 SSH 短命命令
3. **模型验证**：支持固定基座 (nq=12) 和浮动基座 (nq=19, nv=18)
4. **ROS2 可选**：不影响控制链路，仅做观测转发