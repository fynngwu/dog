# 给 Gemini 的 Prompt：生成基于 Qt 的高效现代化前端

你现在要为一个四足机器人调试系统生成一个**可运行的 PySide6 桌面前端工程**。  
请严格按照下面约束生成，不要自行改协议，不要擅自更换技术栈。

---

## 目标

生成一个**现代化、高效、可维护**的 Qt 前端，用于连接现有机器人后端。  
这个前端主要服务两个核心功能：

1. 单电机调试
2. CSV 录制结果回放控制

后端协议和 Python 适配层已经存在，你的前端必须复用它们，而不是自己重写底层控制逻辑。

---

## 技术栈要求

- Python 3.10+
- PySide6
- pyqtgraph 做实时曲线
- 使用 signal/slot
- 使用后台 worker 线程处理阻塞 I/O
- UI 主线程绝不能阻塞
- 不要使用 webview，不要做网页，不要换成 Electron
- 风格要求现代化、清爽、工程化

---

## 项目结构要求

请生成如下结构，尽量完整：

```text
dog_qt_frontend/
  README.md
  requirements.txt
  main.py
  app/
    __init__.py
    models/
      app_state.py
      joint_mapping.py
    services/
      backend_service.py
      replay_service.py
      state_stream_worker.py
      command_worker.py
    widgets/
      status_bar.py
      joint_selector.py
      json_viewer.py
      plot_panel.py
      replay_controls.py
    views/
      dashboard_page.py
      joint_debug_page.py
      replay_page.py
      diagnostics_page.py
    main_window.py
    styles.py
```

---

## 必须复用的现有后端接口

假设当前工程里已经有这几个 Python 文件可直接 import：

- `experiment_manager.py`
- `robot_backend.py`
- `replay_controller.py`
- `robot_client.py`

请优先通过 `ExperimentManager` 和 `ReplayController` 调用后端，不要直接在 UI 层拼 TCP 协议。

---

## 后端协议与语义

### 连接
- 命令端口：47001
- 状态推流端口：47002
- 状态推流频率：20Hz
- 状态是 JSON line

### 关节索引
```text
0 LF_HipA
1 LR_HipA
2 RF_HipA
3 RR_HipA
4 LF_HipF
5 LR_HipF
6 RF_HipF
7 RR_HipF
8 LF_Knee
9 LR_Knee
10 RF_Knee
11 RR_Knee
```

### 单电机调试接口
- `joint_test(indices, target_rad)`
- `joint_sine(indices, amp_rad, freq_hz, duration_s)`

语义：
- 角度单位是 relative-to-offset joint rad
- sine 是绕 offset=0 摆动
- 支持多关节同时选中

### 回放接口
- `stage_and_load_csv(filename, csv_text)`
- `start()`
- `stop()`
- `step()`
- `prev()`
- `seek(frame_idx)`
- `status()`

语义：
- `seek` 只移动游标，不发送动作
- `step` 是按帧
- speed factor 固定为 1.0，不需要做动态调速 UI

### 状态包结构
顶层包含：
- `mode`
- `seq`
- `state`
- `motion`
- `replay`
- `fault`

其中 `state` 包含：
- `joint_positions`
- `joint_velocities`
- `joint_torques`
- `target_joint_positions`
- `offline_motors`

---

## UI 功能要求

### 1. Dashboard 页面
要有：
- Host、cmd_port、state_port 输入
- Connect / Disconnect
- Init / Enable / Disable
- 连接状态灯
- 状态流在线灯
- 当前 mode
- 当前 motion 状态
- replay 状态摘要
- 最近 fault 摘要

### 2. Joint Debug 页面
要有：
- 12 关节多选列表，显示 joint name
- target_rad 输入框
- amp/freq/duration 输入框
- “Apply Joint Test” 按钮
- “Start Sine” 按钮
- 实时数据显示：
  - current position
  - target
  - error
  - velocity
  - torque
- 实时曲线，建议用 pyqtgraph：
  - real position
  - target position
  - error
  - velocity

### 3. Replay 页面
要有：
- CSV 文件选择
- Upload + Load 按钮
- Start / Stop / Step / Prev 按钮
- Seek slider + spinbox
- 当前 cursor / total_frames
- replay status
- 当前 CSV 路径
- 原始返回日志区域

### 4. Diagnostics 页面
要有：
- offline motors 列表
- fault 详情卡片
- motion.last_error
- 最近 50 条命令日志
- 最近 50 条错误日志
- 原始 JSON 状态查看器

---

## 交互约束

- 当 `motion.active == true` 时，禁用冲突按钮
- 当 `replay.loaded == false` 时，禁用 Start / Step / Prev / Seek
- 当 `mode == "disabled"` 时，禁用发运动命令的控件
- 如果 `offline_motors` 非空，高亮这些关节
- 如果存在 `fault`，在顶部显示红色告警条

---

## 架构要求

请采用以下模式：

### 分层
- View：Qt 页面和控件
- Service：与 ExperimentManager/ReplayController 交互
- Model：保存当前状态快照与 UI 状态
- Worker：处理阻塞命令与状态流监听

### 明确要求
- 不要在 widget 里直接写 socket 调用
- 不要把业务逻辑塞进按钮回调
- 要用 dataclass 或清晰的状态对象
- 代码要易读、可扩展、文件划分清晰

---

## 代码质量要求

- 每个类和关键函数都写 docstring
- 给主要 signal 写注释
- 异常处理明确
- 日志输出清楚
- `README.md` 要写清楚如何安装、如何启动、工程结构、每个页面做什么

---

## 输出要求

请直接给出**完整工程代码**，按文件逐个输出。  
不要只给设计图，不要只给伪代码。  
每个文件都要完整，可直接保存运行。  
如果内容很多，就按文件分段连续输出，直到整个工程结束。
