# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Dog Fullstack Bundle is a **12-DOF quadruped robot** control stack with three layers: a **C++ daemon** (hardware/CAN), a **Python adapter** (TCP protocol), and a **PySide6 Qt frontend** (GUI). Documentation is primarily in Chinese.

```
[Qt Frontend] ←TCP→ [Python Adapter] ←TCP→ [C++ Daemon] ←CAN→ [Motors]
    PySide6        robot_backend.py       twin_agent.cpp
    20Hz plots     robot_client.py        motor_io.cpp
                   experiment_manager.py  can_interface.cpp
```

## Build & Run Commands

### C++ Backend Daemon (requires Jetson CAN hardware)
```bash
cd backend && ./build.sh          # cmake + make, produces build/dog_debug_daemon
./build/dog_debug_daemon --cmd-port 47001 --state-port 47002
```

### Python Frontend
```bash
cd frontend
uv venv && source .venv/bin/activate
uv pip install -r requirements.txt
python main.py
```

### Mock Daemon (no hardware needed)
```bash
python tools/mock_daemon.py        # Listens on 47001/47002
python tests/run_mock_integration.py  # Automated protocol validation
```

### Headless Frontend (no display)
```bash
xvfb-run python main.py
```

## Architecture

### TCP Protocol (two ports)
- **Command port (47001)**: newline-delimited JSON request/reply, single client
- **State port (47002)**: 20Hz JSON push stream, single client

### C++ Daemon (`backend/daemon/`)
- `twin_agent.cpp` — main command handler and state streaming server
- `motor_io.cpp` — 12-motor interface with joint offsets, limits, and PD control
- `can_interface.cpp` — SocketCAN communication
- `robstride.cpp` — Robstride motor protocol (RS485/CAN framing)
- `twin_protocol.hpp` — JSON command parsing utilities

### Python Adapter (`backend/python/`)
- `robot_client.py` — thread-safe TCP client (send_command / state stream)
- `robot_backend.py` — thin facade over RobotClient for frontend use
- `experiment_manager.py` — orchestrates connect, init, enable/disable, joint control, state stream
- `replay_controller.py` — CSV upload, stage, seek, step, start/stop playback
- `mock_daemon.py` (in `tools/`) — protocol simulator for testing without hardware

### Qt Frontend (`frontend/app/`)
- `main.py` → `MainWindow` with 4 tab pages
- `services/backend_service.py` — async command execution via CommandWorker (QRunnable)
- `services/replay_service.py` — async replay control
- `services/state_stream_worker.py` — background TCP client consuming 20Hz state stream
- `views/` — dashboard, joint_debug, replay, diagnostics pages
- `models/` — RobotState data model, joint name mapping

### Frontend Async Pattern
Commands use `QRunnable` workers (not threads). Service methods accept a method name string + args dict; `CommandWorker` executes in thread pool and emits signals on completion. **Never call deleteLater on CommandWorker** — it auto-manages lifecycle.

### Call Chain for Frontend → Backend
```
View → BackendService.run_async("init_robot", {"duration_s": 2.5})
    → CommandWorker → ExperimentManager.init_robot() → RobotClient.send_command()
        → TCP → C++ daemon processes and replies
```

## Joint Mapping (0-indexed)

| 0-3 | 4-7 | 8-11 |
|-----|-----|------|
| LF/LR/RF/RR HipA (roll) | LF/LR/RF/RR HipF (pitch) | LF/LR/RF/RR Knee |

Angles are in **relative-to-offset radians**.

## CSV Replay Convention

Required columns: `timestamp_ms` (or `sim_time`), `scaled_action_0` through `scaled_action_11`.

## Key Constraints

- Command and state ports are **single-client** — no concurrent GUI/CLI connections
- `joint_torques` is raw controller telemetry, not joint-side computed torque
- `speed_factor` can only be set at `replay_start`, not changed during playback
- Frontend auto-adds `../backend/python` to `sys.path` — no PYTHONPATH needed
- C++ daemon requires SocketCAN (Jetson hardware); use mock_daemon.py for development

## Wiki & Docs

Detailed backend documentation lives in `backend/wiki/`:
- `backend/wiki/frontend/frontend_api_spec.md` — complete frontend API spec
- `backend/wiki/frontend/command_protocol.md` — TCP command protocol
- `backend/wiki/frontend/state_stream_spec.md` — state stream format
- `backend/wiki/daemon/` — per-file C++ daemon documentation
- `backend/wiki/python/` — per-file Python adapter documentation
