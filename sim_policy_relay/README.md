# sim_policy_relay

MuJoCo simulation + ONNX policy relay for the quadruped robot. Runs a trained RL policy in simulation and optionally relays actions to the real robot via TCP, with real-time sim-vs-real joint comparison and safety monitoring.

## Modes

| Mode | What it does | Requires |
|---|---|---|
| **GUI (sim-only)** | MuJoCo sim + policy, sliders/gamepad/ROS2 for commands | XML + ONNX |
| **GUI (robot)** | Above + TCP relay to Jetson, sim/real divergence monitoring | XML + ONNX + Jetson |
| **Headless** | Same as GUI sim-only but no GUI, ROS2 cmd_vel input | XML + ONNX |

## Quick Start

```bash
# Install deps (uv required)
uv venv --system-site-packages   # --system-site-packages only needed for ROS2
uv sync

# Run GUI
uv run python sim_policy_relay.py
```

GUI workflow: **Connect** -> **Init Robot** -> **Start Policy** -> control via keyboard (WASDQE), sliders, gamepad, or ROS2 cmd_vel.

## Architecture

```
sim_policy_relay/
  sim_policy_relay.py   # tkinter GUI
  relay_engine.py       # core: sim loop, policy inference, safety, TCP relay
  sim_headless.py       # headless CLI runner
  twin_client.py        # TCP client to Jetson twin_agent
  joystick_input.py     # optional gamepad input
  ros2_interface.py     # optional ROS2 /cmd_vel subscriber
  leggedrobot_flat.xml  # MuJoCo robot model (uses meshes/ for STL files)
  meshes/               # robot STL mesh files (git-tracked)
  bench_sim.py          # performance benchmark tool
  pyproject.toml        # uv project config
```

## Sim Loop Timing

The sim loop targets **200 Hz physics** with **50 Hz policy inference** (decimation=4).

| Component | Avg cost | Notes |
|---|---|---|
| `mj_step` (physics) | ~52 us | MuJoCo forward + step |
| ONNX inference | ~22 us | per policy tick (every 4th sim step) |
| Observation build | ~8 us | concatenate + clip |
| PD control | ~6 us | torque compute + clip |
| **Total per sim tick** | **~69 us** | well within 5000 us budget |

With the viewer enabled, sim steps are batched to match wall-clock time — the viewer's vsync provides natural pacing at ~1:1 realtime.

## Benchmarks

Run the benchmark to measure performance on your hardware:

```bash
uv run python bench_sim.py
```

Tested on the reference machine:
- **No viewer**: 200 Hz sim, 100% realtime
- **With viewer**: 200 Hz sim, 99.9% realtime (wall-clock matched)

## ROS2 Setup

Requires `--system-site-packages` when creating the venv so `rclpy` is visible.

```bash
source /opt/ros/humble/setup.bash
uv run python sim_policy_relay.py
```

Deadman timeout: 200 ms — publish `/cmd_vel` at >= 5 Hz to maintain control.

## Robot-Connected Mode

1. Run `twin_agent` on the Jetson (TCP ports 47001/47002)
2. Enter Jetson IP, leave Sim Only unchecked
3. Connect -> Init Robot -> Start Policy

Safety checks before every action relay: motors_online==12, feedback_age < 200ms, state stream fresh. GUI close sends hold + disable.

## Meshes

STL files live in `meshes/` with relative paths in the XML. No absolute path editing needed.
