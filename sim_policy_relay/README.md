# sim_policy_relay

MuJoCo simulation + ONNX policy relay for the quadruped robot. Runs a trained RL policy in simulation and optionally relays actions to the real robot via TCP, with real-time sim-vs-real joint comparison and safety monitoring.

## Modes

| Mode | What it does | Requires |
|---|---|---|
| **GUI (sim-only)** | MuJoCo sim + policy, sliders/gamepad/ROS2 for commands | XML + ONNX |
| **GUI (robot)** | Above + TCP relay to Jetson, sim/real divergence monitoring | XML + ONNX + Jetson |
| **Headless** | Same as GUI sim-only but no GUI, ROS2 cmd_vel input | XML + ONNX |

## Quick Start

### 1. Install uv

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### 2. Get the code

```bash
git clone <repo-url> ~/projects/dog
cd ~/projects/dog/sim_policy_relay
```

### 3. Set up the environment

```bash
uv venv --system-site-packages
uv sync
```

`--system-site-packages` is needed if you want ROS2 cmd_vel input. It lets the venv see `rclpy` from the ROS2 system install. If you don't need ROS2, you can omit it:

```bash
uv venv
uv sync
```

### 4. Prepare your model files

You need two files in `sim_policy_relay/`:

- **MuJoCo XML** — robot model (default: `leggedrobot_flat.xml`)
- **ONNX policy** — trained RL policy (default: `policy.onnx`)

The XML references STL mesh files with absolute paths. If you're on a different computer, see [Custom STL Meshes](#custom-stl-meshes) below.

### 5. Run the GUI panel

```bash
uv run python sim_policy_relay.py
```

#### Step-by-step GUI workflow

1. Check **Sim Only** if you don't have a robot connected
2. Click **Connect** — loads the MuJoCo model and ONNX policy
3. Click **Init Robot** — places the robot in the default pose (2.5s wait)
4. Click **Start Policy** — begins the simulation loop at 50 Hz policy rate
5. Control the robot:
   - **Keyboard**: W/S (forward/back), A/D (left/right), Q/E (yaw), Space (zero)
   - **Gamepad**: check "Use gamepad if available" (requires `inputs` package)
   - **ROS2**: check "Use ROS2 cmd_vel" (requires ROS2, see [ROS2 Setup](#ros2-setup))
   - **Sliders**: drag cmd_x / cmd_y / cmd_yaw manually
6. The **Joint Monitor** table shows real vs sim joint positions and highlights divergence:
   - green = small difference
   - yellow = moderate difference
   - red = exceeds threshold
7. Click **Stop Policy** to pause, **E-Stop** for emergency stop

## Headless Mode

For running without a display:

```bash
uv run python sim_headless.py --no-viewer
```

Options:

| Flag | Default | Description |
|---|---|---|
| `--xml path` | `leggedrobot_flat.xml` | Custom MuJoCo XML |
| `--onnx path` | `policy.onnx` | Custom ONNX policy |
| `--cmd-scale` | `0.5` | Velocity scaling factor |
| `--deadman-ms` | `200` | Deadman timeout in ms |
| `--no-viewer` | off | Disable MuJoCo 3D window |

Without ROS2 it runs with zero command (robot stands in place).

## ROS2 Setup

To use `/cmd_vel` as the command source:

### Install ROS2 Humble

Follow the [official guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

### Source ROS2 before running

```bash
source /opt/ros/humble/setup.bash
uv run python sim_policy_relay.py
# or
uv run python sim_headless.py
```

### Publish commands

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}" --rate 20
```

### Deadman timeout

If no `/cmd_vel` message is received for 200 ms, the command automatically resets to zero. Keep publishing at >= 5 Hz to maintain control.

## Custom STL Meshes

The MuJoCo XML references STL mesh files with absolute paths. You need to update them when moving to a new computer.

### Option A: Relative paths (recommended)

1. Copy your STL files into `sim_policy_relay/meshes/`
2. Edit `leggedrobot_flat.xml` — change every `<mesh>` from absolute to relative:

```xml
<!-- Before -->
<mesh name="base_link" file="/home/wufy/projects/dog/DOGV2.2.4.SLDASM/DOGV2.2.4.SLDASM/meshes/base_link.STL"/>

<!-- After -->
<mesh name="base_link" file="meshes/base_link.STL"/>
```

Or batch-update with sed:

```bash
sed -i 's|file="/home/wufy/projects/dog/DOGV2.2.4.SLDASM/DOGV2.2.4.SLDASM/meshes/|file="meshes/|g' leggedrobot_flat.xml
```

### Option B: Update to your absolute path

```bash
sed -i 's|/home/wufy/projects/dog/DOGV2.2.4.SLDASM|/your/path/to/meshes|g' leggedrobot_flat.xml
```

### Required mesh files

```
base_link.STL        LF_HipA_link.STL    LF_HipF_link.STL    LF_Knee_link.STL    LF_Foot_link.STL
LR_HipA_link.STL    LR_HipF_link.STL    LR_Knee_link.STL    LR_Foot_link.STL
RF_HipA_link.STL    RF_HipF_link.STL    RF_Knee_link.STL    RF_Foot_link.STL
RR_HipA_link.STL    RR_HipF_link.STL    RR_Knee_link.STL    RR_Foot_link.STL
```

## Robot-Connected Mode

To control the real robot through the GUI:

1. Run `twin_agent` on the Jetson (listens on TCP ports 47001/47002)
2. In the GUI, enter the Jetson IP in the **Jetson IP** field
3. Leave **Sim Only** unchecked
4. Click **Connect**, then **Init Robot**, then **Start Policy**

The GUI monitors sim-vs-real joint divergence. Enable **Auto-stop on divergence** to automatically halt the policy when any joint exceeds the threshold (default 0.35 rad).

## Safety Behavior

Before every action relay, the engine checks:
- `motors_online == 12`
- `feedback_age_ms < 200`
- State stream age is fresh

If any check fails, the engine triggers E-stop and sends `disable`.

On GUI close, the app sends `hold` then `disable` to the robot.

## Optional: Gamepad

```bash
uv add inputs
```

Left stick = forward/lateral, right stick horizontal = yaw.

## File Structure

```
sim_policy_relay/
  sim_policy_relay.py   # GUI application (tkinter)
  sim_headless.py       # Headless runner (CLI)
  relay_engine.py       # Core engine: sim, policy, safety, TCP relay
  joystick_input.py     # Optional gamepad input
  twin_client.py        # TCP client for Jetson communication
  leggedrobot_flat.xml  # MuJoCo robot model
  policy.onnx           # ONNX policy (not tracked in git)
  pyproject.toml        # uv project config
```
