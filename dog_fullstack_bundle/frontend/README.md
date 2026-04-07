# Dog Qt Frontend

这是修正过、可直接接到本仓库后端的 PySide6 前端。

## 已修复的兼容问题

相较于你提供的初版前端代码，这版已经修掉几个会直接影响联调的问题：

1. `BackendService.connect_backend()` 现在会真正建立命令通道连接，而不只是实例化对象。
2. `ReplayService` 现在按后端真实签名构造 `RobotBackend + ReplayController`，不再错误地把 `host/cmd_port` 直接传给 `ReplayController`。
3. Dashboard 连接时会同时连接 replay 服务和状态流。
4. 前端状态模型现在按后端真实返回解析：
   - `offline_motors` 是字符串列表，不是整数列表。
   - `fault` 是可选对象，不带 `has_fault/msg` 字段。
5. Replay 的 seek 滑条不再在拖动时对每个 value change 都发命令，而是只在释放滑条后发一次 `replay_seek`。
6. 成功/失败判断现在基于后端 JSON 的 `ok` 字段，不再把所有返回都当作成功。
7. 本仓库内启动时会自动加入 `../backend/python` 到 `sys.path`，不用再手动设 `PYTHONPATH`。

## 安装

```bash
cd frontend
uv venv
source .venv/bin/activate
uv pip install -r requirements.txt
python main.py
```

## 启动前说明

- 真机联调：先启动 `../backend/build/dog_debug_daemon`
- 无硬件联调：先启动 `../tools/mock_daemon.py`

默认端口：
- cmd: `47001`
- state: `47002`
