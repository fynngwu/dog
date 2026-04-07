# Dog Fullstack Bundle

这是一个已经整合好的完整目录，包含：

- `backend/`：C++ daemon + Python backend adapter + wiki 文档
- `frontend/`：修复过兼容性的 PySide6 前端
- `tools/mock_daemon.py`：无硬件时的协议模拟器
- `tests/run_mock_integration.py`：可直接运行的 mock 联调脚本
- `reports/compatibility_fix_summary.md`：修复与验证摘要

---

## 你先看哪些文件

### 第一层：快速理解整体
1. `README.md`
2. `reports/compatibility_fix_summary.md`
3. `backend/wiki/frontend/frontend_api_spec.md`
4. `frontend/docs/compatibility_audit.md`

### 第二层：理解前端如何调后端
1. `frontend/app/services/backend_service.py`
2. `frontend/app/services/replay_service.py`
3. `frontend/app/services/state_stream_worker.py`
4. `backend/python/experiment_manager.py`
5. `backend/python/robot_backend.py`
6. `backend/python/replay_controller.py`
7. `backend/python/robot_client.py`

### 第三层：理解 C++ daemon 真正支持什么命令
1. `backend/daemon/twin_agent.cpp`
2. `backend/daemon/twin_protocol.hpp`
3. `backend/daemon/motor_io.cpp`

---

## 一步一步测试

## A. 先做无硬件测试（推荐先跑）

这一步不需要 Jetson CAN，不需要真机。

### 1) 运行 mock 集成测试

```bash
cd dog_fullstack_bundle
python tests/run_mock_integration.py
```

你应该看到：
- `PING OK`
- `ALL MOCK INTEGRATION CHECKS PASSED`

这一步验证的是：
- Python backend adapter 是否能工作
- Replay CSV 上传与控制是否能工作
- 20Hz 状态流消费是否能工作

### 2) 启动 mock daemon

```bash
cd dog_fullstack_bundle
python tools/mock_daemon.py
```

### 3) 启动 Qt 前端

```bash
cd dog_fullstack_bundle/frontend
uv venv
source .venv/bin/activate
uv pip install -r requirements.txt
python main.py
```

### 4) 前端里这样连

- Host: `127.0.0.1`
- Cmd Port: `47001`
- State Port: `47002`

然后依次测试：
- Dashboard -> Connect
- Init Robot
- Enable / Disable
- Joint Debug -> 选择关节 -> Apply Joint Test
- Joint Debug -> Start Sine
- Replay Controller -> 加载 `../tests/sample_replay.csv`
- Start / Step / Prev / Seek

---

## B. 再做 Jetson 真机测试

### 1) 构建后端

```bash
cd dog_fullstack_bundle/backend
./build.sh
```

### 2) 启动 daemon

```bash
./build/dog_debug_daemon --cmd-port 47001 --state-port 47002
```

如果这里报 CAN 初始化失败，说明：
- 不是前端问题
- 是 Jetson 上的 CAN 设备或网卡名还没准备好

### 3) 启动前端

```bash
cd dog_fullstack_bundle/frontend
uv venv
source .venv/bin/activate
uv pip install -r requirements.txt
python main.py
```

### 4) 真机联调顺序

推荐严格按这个顺序：

1. Connect
2. Init Robot
3. 看 Dashboard 是否收到 20Hz 状态流
4. 单关节 `joint_test`
5. 单关节 `joint_sine`
6. 加载 CSV，先 `replay_step`
7. 再 `replay_prev`
8. 再 `replay_seek`
9. 最后 `replay_start / replay_stop`

---

## 当前环境下我已经完成的检查

- 后端 C++ 本地编译通过
- 前端源码按修复版重新组织完成
- 前端与后端 Python API 签名重新对齐
- 用 mock daemon 跑通协议级联调脚本

## 当前环境下无法代替你完成的部分

当前容器没有 Jetson 的 CAN 设备，所以无法替你把真实 C++ daemon 在这里跑进硬件控制环。
这部分你需要在 Jetson 上按上面的顺序测试。
