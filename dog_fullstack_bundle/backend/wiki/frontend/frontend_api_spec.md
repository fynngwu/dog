# 前端总对接 Spec

本页是给 Qt 前端直接使用的统一接口说明。  
后端当前由两层组成：

1. **Jetson C++ daemon**  
   - 命令端口：`47001`
   - 状态推流端口：`47002`
2. **Python 前端适配层**  
   - `RobotBackend`
   - `ReplayController`
   - `ExperimentManager`

对于 **PySide6 / Qt for Python** 前端，推荐直接复用 Python 层，不要在 UI 线程直接写 socket。

---

## 1. 连接模型

### 命令通道
- TCP
- 单行文本命令
- UTF-8
- 必须以 `\n` 结尾
- 每条命令返回一条 JSON 行

### 状态推流
- TCP
- 服务端每 **50ms** 推一次，也就是 **20Hz**
- 每条状态是一条 JSON 行
- 前端应该常驻连接并持续消费

---

## 2. 前端推荐调用路径

### 方案 A：Qt 前端直接复用 Python 包
推荐程度：**最高**

UI 层调用关系：

```text
Qt Widget / ViewModel
    ↓
ExperimentManager
    ↓
RobotBackend + ReplayController
    ↓
RobotClient
    ↓
TCP cmd/state sockets
    ↓
Jetson daemon
```

这样做的好处：
- UI 不直接碰底层协议
- CSV 上传/缓存、回放命令、状态监听已经有现成封装
- 以后换 Web UI 时也能保留这层

### 方案 B：Qt 前端自己实现协议
只建议在你明确要用 C++/QML 且不走 Python 时再做。  
那时应严格遵守：
- [命令协议](command_protocol.md)
- [状态推流协议](state_stream_spec.md)

---

## 3. 功能域

### 单电机调试
已支持：
- `joint_test <idx[,idx...]> <target_rad>`
- `joint_sine <idx[,idx...]> <amp_rad> <freq_hz> <duration_sec>`

语义：
- `idx` 范围：`0..11`
- 角度单位：**relative-to-offset joint rad**
- `joint_sine`：**绕 offset=0 做正弦摆动**
- 可以同时选多个关节

### 回放
已支持：
- `load_replay_csv`
- `replay_start`
- `replay_stop`
- `replay_step`
- `replay_prev`
- `replay_seek`
- `replay_status`

语义：
- `seek` 只移动游标，不发命令
- `step` 是按 CSV **一帧**
- `speed_factor` 只允许在 `replay_start` 指定，运行中不可动态改

### 状态展示
前端应从状态流或 `get_state` 中读取：
- `state.joint_positions`
- `state.joint_velocities`
- `state.joint_torques`
- `state.target_joint_positions`
- `state.offline_motors`
- `motion`
- `replay`
- `fault`

---

## 4. 关节索引表

| idx | joint_name |
|---:|---|
| 0 | LF_HipA |
| 1 | LR_HipA |
| 2 | RF_HipA |
| 3 | RR_HipA |
| 4 | LF_HipF |
| 5 | LR_HipF |
| 6 | RF_HipF |
| 7 | RR_HipF |
| 8 | LF_Knee |
| 9 | LR_Knee |
| 10 | RF_Knee |
| 11 | RR_Knee |

前端展示建议：
- 对用户显示 `joint_name`
- 内部仍然使用 `idx`

---

## 5. 推荐 UI 页面

### 页面 1：连接与基础控制
- Host / cmd port / state port
- Connect / Disconnect
- Init / Enable / Disable
- 状态灯：connected / state stream / mode / motion active

### 页面 2：单电机调试
- 多选关节列表
- target 输入
- sine 参数输入：amp / freq / duration
- 实时曲线：
  - target
  - position
  - error
  - velocity
  - torque

### 页面 3：回放
- CSV 选择
- 上传到 Jetson 固定目录
- Load
- Start / Stop / Step / Prev / Seek
- 当前帧、总帧、播放状态

### 页面 4：诊断
- offline motors
- last fault
- motion last_error
- replay status
- 原始 JSON 查看器

---

## 6. CSV 约定

最小必需表头：

- `timestamp_ms` **或** `sim_time`
- `scaled_action_0`
- `scaled_action_1`
- ...
- `scaled_action_11`

只有满足这个格式，daemon 才能加载回放。

---

## 7. 错误处理

错误返回统一形式：

```json
{
  "ok": false,
  "msg": "human readable message",
  "error": {
    "code": "bad_command",
    "message": "human readable message",
    "detail": "optional extra detail"
  }
}
```

前端建议：
- `error.code` 作为分支依据
- `error.message` 直接弹给用户
- `detail` 放在折叠日志区

---

## 8. 最推荐的前端接入 API

如果前端是 Python + PySide6，最推荐只用这几个对象：

### `ExperimentManager`
- `connect()`
- `disconnect()`
- `start_state_stream(callback)`
- `stop_state_stream()`
- `init_robot(duration_s=2.5)`
- `enable()`
- `disable()`
- `get_state()`
- `set_mit_param(...)`
- `joint_test(indices, target_rad)`
- `joint_sine(indices, amp_rad, freq_hz, duration_s)`

### `ReplayController`
- `stage_and_load_csv(filename, csv_text)`
- `start()`
- `stop()`
- `step()`
- `prev()`
- `seek(frame_idx)`
- `status()`

---

## 9. 前端线程建议

不要把 socket 或长时间 I/O 放在 UI 主线程。

推荐：
- 状态监听：后台线程
- CSV 上传：后台线程
- 曲线刷新：主线程定时器，读取最近状态快照
- 命令调用：worker + signal/slot

---

## 10. 已知边界

当前实现里有几个事实，前端要知道：

1. 状态推流服务当前是 **单客户端串行 accept 模型**  
   通常让 Qt 前端独占这一条连接最稳。

2. 命令口也更适合由一个长期连接的客户端独占。  
   不建议 GUI、CLI、脚本同时混连。

3. `joint_torques` 来自底层控制器反馈，当前语义更接近 **电机/控制器原始 torque telemetry**，不是严格做过关节侧传动比换算后的“纯 joint torque”。
