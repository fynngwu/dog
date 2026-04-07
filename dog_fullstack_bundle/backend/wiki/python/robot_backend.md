# robot_backend.py

## 角色

业务友好的薄封装。

它把 `RobotClient` 重新命名成更偏“后端服务接口”的形式，方便给前端或更上层 orchestration 调用。

## 主方法

- `connect()`
- `disconnect()`
- `start_state_stream(callback)`
- `stop_state_stream()`
- `init(duration_s=2.5)`
- `enable()`
- `disable()`
- `get_state()`
- `set_joint_targets(targets_rad)`
- `joint_test(indices, target_rad)`
- `joint_sine(indices, amp_rad, freq_hz, duration_s)`
- `set_mit_param(...)`
- `load_replay_csv(csv_path)`
- `replay_start()`
- `replay_stop()`
- `replay_step()`
- `replay_prev()`
- `replay_seek(frame_idx)`
- `replay_status()`

## 适用位置

- Qt service
- FastAPI / gRPC service
- 简单自动化脚本

## 评价

这个文件是合适的。  
如果以后服务更多，建议继续保持“只做业务命名和参数整理，不塞复杂逻辑”。
