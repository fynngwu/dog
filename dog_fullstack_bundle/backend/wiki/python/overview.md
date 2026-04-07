# python 层总览

Python 层不是控制底层，而是**前端适配层**。

## 分层

### 最底层：`robot_client.py`
直接负责 TCP 命令和状态流连接。

### 适配层：`robot_backend.py`
把 `RobotClient` 包成更清晰的业务方法。

### 回放层：`replay_controller.py`
负责把前端传来的 CSV 文本存盘并 load 到 daemon。

### 编排层：`experiment_manager.py`
给前端暴露更稳定的一组 workflow API。

### 工具
- `dog_cli.py`
- `visualize.py`
- `motor_config.py`
- `recorder.py`

## 前端推荐只依赖哪几层

- `ExperimentManager`
- `ReplayController`

这两层对 Qt 前端最友好。
