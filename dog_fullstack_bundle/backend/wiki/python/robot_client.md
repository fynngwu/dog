# robot_client.py

## 角色

这是最底层 TCP 客户端。  
它同时处理：

- 命令 socket
- 状态推流 socket
- JSON 解析
- 持续监听线程
- 最近状态缓存

## 关键属性

- `host`
- `cmd_port`
- `state_port`
- `timeout_s`
- `_cmd_sock`
- `_state_sock`
- `_state_thread`
- `_latest_state`
- `_last_error`

## 主方法

### 连接
- `connect()`
- `connect_state_stream()`
- `close()`

### 命令
- `send_command(command)`
- `ping()`
- `get_state_once()`
- `enable()`
- `disable()`
- `init(duration_s)`
- `set_joint(targets_rad)`
- `joint_test(indices, target_rad)`
- `joint_sine(indices, amp_rad, freq_hz, duration_s)`
- `load_replay_csv(csv_path)`
- `replay_start(speed_factor)`
- `replay_stop()`
- `replay_step()`
- `replay_prev()`
- `replay_seek(frame_idx)`
- `replay_status()`
- `replay(csv_path, speed_factor)`
- `set_mit_param(...)`

### 状态流
- `start_state_listener(callback=None)`
- `stop_state_listener()`

---

## 使用建议

前端不要直接把这个类暴露给每个 widget。  
应该由一个 service 层统一持有它。

---

## 已知局限

1. 状态流没有自动重连
2. 更适合“一个客户端长期持有”
3. 作为包使用时，最好再包一层 service，别让 UI 直接依赖它
