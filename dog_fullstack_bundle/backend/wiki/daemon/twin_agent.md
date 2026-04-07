# twin_agent.cpp / twin_agent.hpp

## 角色

这是 daemon 的核心编排器。  
它同时负责：

- TCP 命令服务器
- TCP 状态推流服务器
- 命令解析
- 单电机测试与 replay motion worker
- 状态快照拼装
- replay CSV 生命周期

## 关键成员

### 连接与线程
- `cmd_port_`
- `state_port_`
- `cmd_server_fd_`
- `state_server_fd_`
- `cmd_thread_`
- `state_thread_`
- `motion_thread_`

### 控制状态
- `enabled_`
- `last_joint_targets_`
- `motion_active_`
- `motion_abort_`
- `active_motion_name_`
- `last_motion_error_`

### replay 状态
- `replay_samples_`
- `replay_csv_path_`
- `replay_cursor_`
- `replay_speed_factor_`
- `replay_loaded_`

### fault
- `last_fault_`

---

## 核心方法

## `Start()`
- 初始化 `MotorIO`
- 建立命令端口
- 建立状态推流端口
- 启动 `cmd_thread_` / `state_thread_`

## `Stop()`
- `running_ = false`
- 中止当前 motion
- 关闭 socket
- join 所有线程
- `DisableAll`

## `CommandLoop()`
- `accept` 命令连接
- 每个连接交给 `HandleCommand()`

## `HandleCommand(int client_fd)`
- 按行读取文本命令
- 每行调用 `ProcessCommand()`
- 把 JSON 回复写回

## `StateLoop()`
- 接受一个状态连接
- 每 50ms 推送一次 `SnapshotToJson()`

注意：当前实现是**单状态客户端**串行模型。

## `ProcessCommand(const std::string&)`
解析所有命令并执行业务逻辑。

## `SnapshotToJson()`
拼装前端使用的统一状态包。

---

## 已实现命令

- `ping`
- `get_state`
- `enable`
- `disable`
- `init`
- `goto_offset`
- `set_joint`
- `joint_test`
- `joint_sine`
- `load_replay_csv`
- `replay_start`
- `replay_stop`
- `replay_step`
- `replay_prev`
- `replay_seek`
- `replay_status`
- `replay`
- `set_mit_param`

---

## motion worker 机制

通过 `LaunchMotionThread(...)` 启动统一 worker。

### 统一特征
- 有名字：如 `joint_sine` / `replay`
- 有 duration
- 每 10ms tick 一次
- 有 watchdog
- 可被 `disable` / `replay_stop` 中止

### 互斥规则
一个时刻只允许一个 motion 运行。  
否则返回 `motion_active`。

---

## replay 机制

### 加载
`load_replay_csv` 调 `LoadReplayCsv()` 把 CSV 读到 `replay_samples_`

### 单帧发送
`SendReplayFrameAtCursor()`
- 先做 feedback freshness 检查
- 再 clamp
- 再发送 12 关节目标

### 连续播放
`StartReplayMotion()`
- 从 `replay_cursor_` 对应的时间开始
- 根据 `speed_factor` 计算 replay 时间
- 找到应该播放的 CSV 帧
- 更新 `replay_cursor_`

---

## 你读这个文件时最该关注的点

1. 命令互斥
2. replay 游标语义
3. `SnapshotToJson()` 输出结构
4. motion watchdog
5. `disable` 的中断语义

---

## 已知局限

1. 状态推流当前只服务一个客户端
2. 命令处理也是同步串行 accept/读写模型
3. 大 CSV replay 时，`StartReplayMotion()` 会把 `replay_samples_` 复制到 lambda capture，中大文件会有一次额外内存复制
