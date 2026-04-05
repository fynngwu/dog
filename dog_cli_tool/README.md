# dog_cli_tool

Real-robot debug toolkit built to stay wire-compatible with the provided `twin_agent` reference daemon and the MuJoCo policy tooling. It has two parts:

- `daemon/`: enhanced C++17 daemon for Jetson, speaking the same TCP protocol and SocketCAN Robstride format as the reference implementation
- `python/`: host-side CLI, TCP client, plotting utilities, and shared motor constants

## Architecture

```text
PC / Laptop
  ├─ python/dog_cli.py           interactive CLI / one-shot commands
  ├─ python/robot_client.py      TCP command + state stream client
  └─ python/visualize.py         CSV plotting / bound inspection
          │
          │ newline-delimited TCP JSON
          ▼
Jetson / Robot
  └─ dog_debug_daemon
        ├─ command socket :47001
        ├─ state socket   :47002
        ├─ daemon/twin_agent.cpp   command dispatch + motion worker
        ├─ daemon/motor_io.cpp     joint-space clamping / offsets / smoothing
        ├─ daemon/robstride.cpp    Robstride MIT CAN encoding / feedback parsing
        └─ daemon/can_interface.cpp SocketCAN wrappers for candle0..candle3
                │
                ▼
             SocketCAN
                │
                ▼
             Robstride motors
```

## Prerequisites

### Jetson / daemon side

- Ubuntu with SocketCAN enabled
- `candle0`, `candle1`, `candle2`, `candle3` interfaces available
- CMake >= 3.10
- g++ with C++17 support

### PC / CLI side

- Python 3.9+
- `numpy`, `matplotlib`

Install host Python deps with:

```bash
cd dog_cli_tool/python
python -m pip install -r requirements.txt
```

## Build and install on Jetson

```bash
cd dog_cli_tool
./build.sh
```

The binary will be created at `build/dog_debug_daemon`.

Helper scripts:

- `scripts/build_jetson.sh`: wrapper around `build.sh`
- `scripts/install_daemon.sh`: copies the built binary to `/usr/local/bin/dog_debug_daemon` when permissions allow
- `scripts/example_session.sh`: sample daemon + CLI session

Start the daemon:

```bash
./build/dog_debug_daemon --cmd-port 47001 --state-port 47002
```

## Quick start

假设 Jetson IP 为 `10.20.127.185`，在 PC 端操作：

```bash
cd dog_cli_tool/python

# 1. 使能电机并平滑移动到站立位置（enable + smooth move to offset + hold）
python dog_cli.py --host 10.20.127.185 init 2.5

# 2. 停止：失能所有电机（任何运动中的回放也会立即中止）
python dog_cli.py --host 10.20.127.185 disable

# 3. 回放 CSV 动作序列
#    先把 CSV 传到 Jetson 上，路径必须是 Jetson 本地路径
scp /path/to/recording.csv ares@10.20.127.185:~/recording.csv
python dog_cli.py --host 10.20.127.185 init 2.5
python dog_cli.py --host 10.20.127.185 replay /home/ares/recording.csv 1.0
```

`init` 响应示例：

```json
// 全部电机正常
{"ok": true, "msg": "moved to offset and holding"}

// 部分电机 offline（仍会执行，但响应中会列出失败的电机）
{"ok": true, "msg": "moved to offset and holding", "offline_motors": ["RF_HipA", "RF_HipF", "RF_Knee"]}
```

## CLI reference

All commands keep the same TCP framing as the reference daemon: plain-text command lines, JSON replies, newline terminated.

| Command | Description | Example |
|---|---|---|
| `ping` | 测试连接 | `python dog_cli.py --host 10.20.127.185 ping` |
| `get_state` | 获取关节状态 | `python dog_cli.py --host 10.20.127.185 get_state` |
| `init [sec]` | 使能电机并平滑到 offset（默认 2.5s） | `python dog_cli.py --host 10.20.127.185 init 2.5` |
| `enable` / `disable` | 使能/失能所有电机 | `python dog_cli.py --host 10.20.127.185 enable` |
| `set_joint <12 floats>` | 直接设置 12 个关节角度 (rad) | `python dog_cli.py --host 10.20.127.185 set_joint 0 0 0 0 0 0 0 0 0 0 0 0` |
| `set_mit_param <kp> <kd> <vel_limit> <torque_limit>` | 运行时修改 MIT 参数 | `python dog_cli.py --host 10.20.127.185 set_mit_param 40 0.5 44 17` |
| `replay <csv> [speed] [--no-feedback-check]` | 回放 CSV 动作序列（路径为 Jetson 本地） | `python dog_cli.py --host 10.20.127.185 replay /home/ares/recording.csv 1.0` |

不带命令进入交互 REPL：`python dog_cli.py --host 10.20.127.185`

## Visualization guide

`python/visualize.py` reads one or more CSV files from `sim_record` and produces a 3-row × 4-column joint plot (4 legs across columns, HipA/HipF/Knee down the rows).

```bash
python visualize.py recording.csv --highlight-bounds
python visualize.py run_a.csv run_b.csv --overlay --highlight-bounds --output compare.png
```

Features:

- phase-colored backgrounds
- position vs target overlay
- joint-limit lines
- optional limit-violation shading / near-bound markers
- stdout summary of per-joint bound hits

## Safety and troubleshooting

- Every daemon-side target is clamped to the configured joint limits before transmission.
- Motion commands run in a 100 Hz worker thread; `disable` aborts any active motion immediately.
- Motors older than 500 ms are treated as offline and skipped during motion playback.
- A 5 s motion watchdog aborts and holds the current pose if the worker stops making progress.
- If a motion command returns “motion is active”, either wait for completion or send `disable`.
- If `motors_online` stays below 12 after `init`, inspect CAN interface names and Robstride feedback first.
- For replay, make sure the CSV path is valid **on the Jetson**, not only on the PC running the CLI.
