# 前端与后端兼容性审查

## 初版前端的关键不兼容点

1. **命令连接未建立**
   - 初版 `BackendService.connect_backend()` 只创建 `ExperimentManager`，没有调用 `connect()`。
   - 结果：点击 Init/Enable/Disable 时会在后台线程里报 `RuntimeError: not connected`。

2. **ReplayService 构造器错误**
   - 后端真实签名：`ReplayController(backend: RobotBackend, store_dir=...)`
   - 初版前端写法：`ReplayController(host=..., cmd_port=...)`
   - 结果：Replay 页面加载 CSV 会直接抛异常。

3. **Dashboard 只连接了 BackendService，没有连接 ReplayService**
   - 结果：Replay 页面始终不可用。

4. **状态字段解析不匹配**
   - 后端 `offline_motors` 实际是字符串数组。
   - 后端 `fault` 只在有故障时出现，字段是 `code/joint_name/motor_index/feedback_age_ms/message`。
   - 初版前端期待 `has_fault/msg`，会把故障状态解析错。

5. **Replay seek 过于频繁**
   - 初版使用 `slider.valueChanged -> seek()`。
   - 结果：拖动滑条时会高频发网络命令。

## 本版修复后的结论

- **协议兼容：通过**
- **Python API 兼容：通过**
- **真机运行前提：后端 daemon 成功启动**
- **无硬件时：可用 mock daemon 做全链路协议测试**
