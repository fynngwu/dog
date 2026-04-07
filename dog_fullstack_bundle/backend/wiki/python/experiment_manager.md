# experiment_manager.py

## 角色

最上层编排器。  
它把前端最常用的动作收束成一个入口。

## 内部组合

- `self.backend = RobotBackend(...)`
- `self.replay = ReplayController(self.backend)`

## 主方法

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

## 适用性

Qt 前端最适合直接持有这个对象。  
然后把：
- 单电机调试动作走 `ExperimentManager`
- 回放动作走 `ExperimentManager.replay`

## 评价

当前它还比较薄，但这正合适。  
未来增加：
- 自动重连
- 任务状态
- 更完整错误映射
可以优先加在这一层。
