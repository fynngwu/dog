# Compatibility Fix Summary

## 修复项

- 修复 `BackendService` 未调用 `ExperimentManager.connect()` 的问题。
- 修复 `ReplayService` 直接错误构造 `ReplayController(host=..., cmd_port=...)` 的问题。
- 修复 Dashboard 连接时未初始化 replay 服务的问题。
- 修复 `RobotState` 对 `offline_motors` / `fault` 的解析与后端实际 JSON 不一致的问题。
- 修复 Replay seek 滑条在拖动过程中持续发命令的问题。
- 修复 UI 逻辑对 `ok=false` 响应不报错的问题。
- 将前端与后端整合到同一文件夹，启动前端时无需额外设置 `PYTHONPATH`。

## 已完成验证

- 后端 C++ 工程已重新本地编译通过。
- 使用 mock daemon 完成：
  - ping
  - init
  - joint_test
  - joint_sine
  - load_replay_csv
  - replay_step / replay_prev / replay_seek / replay_status
  - 20Hz 状态流接收
- 前端源码已完成 Python 语法级检查。

## 受环境限制未完成的验证

- 当前容器中没有 Jetson CAN 设备，因此无法真实启动 C++ daemon 进入硬件控制环。
- 当前容器中没有 PySide6/pyqtgraph，因此无法在此容器内直接把 Qt 窗口拉起来做 GUI 交互验证。
