# sim_policy_relay — Detailed Function Specification

A Tkinter control panel plus headless relay engine for **sim-to-real validation**: run a trained locomotion policy (ONNX) inside MuJoCo, then relay the policy actions to the real robot through the existing twin-agent TCP interface.

## Architecture

```
┌───────────────────────────────────────────────────────────────┐
│                     sim_policy_relay.py (GUI)                │
│  SimPolicyRelayApp — Tkinter window, buttons, sliders,       │
│                      joint monitor table, gamepad integration│
└──────────────────────┬────────────────────────────────────────┘
                       │ calls
┌──────────────────────┴────────────────────────────────────────┐
│                    relay_engine.py (Core)                     │
│  SimPolicyRelayEngine — MuJoCo sim loop + ONNX inference +   │
│                        safety checks + action relay          │
└──────┬───────────────────────────────────────────────┬────────┘
       │ imports                                       │ imports
┌──────┴──────────┐                         ┌─────────┴──────────┐
│ twin_client.py  │                         │ joystick_input.py  │
│ TwinAgentClient │                         │ OptionalGamepad    │
│ RobotState      │                         │ (optional inputs)  │
└────────┬────────┘                         └────────────────────┘
         │ TCP
┌────────┴────────────────┐
│ twin_agent (on Jetson)  │
│ cmd port :47001         │
│ state port :47002       │
└────────┬────────────────┘
         │ CAN
    12x Robstride motors
```

## Dependency Status

| Dependency | Required | Tested |
|---|---|---|
| `mujoco` | Yes | Missing — `pip install mujoco` |
| `numpy` | Yes | OK (1.21.5) |
| `onnxruntime` | Yes | Missing — `pip install onnxruntime` |
| `scipy` | Yes | OK (1.8.0) |
| `inputs` (gamepad) | No | Missing (optional) |
| `tkinter` | Yes | OK (stdlib) |
| MuJoCo XML model | Yes | User-supplied (`leggedrobot_flat.xml`) |
| ONNX policy model | Yes | User-supplied |
| twin_agent on Jetson | Yes | User-supplied (TCP server) |

## Quick Start

```bash
cd sim_policy_relay
python3 -m pip install -r requirements.txt
python3 sim_policy_relay.py
```

---

## File 1: `twin_client.py`

Persistent TCP client for the twin-agent command + state channels. No GUI or simulation logic.

### Class `RobotState`

**Purpose:** Dataclass representing a single robot state snapshot received from the twin-agent state stream.

**Fields:**

| Field | Type | Default | Description |
|---|---|---|---|
| `ok` | `bool` | `False` | Whether the state is valid |
| `mode` | `str` | `""` | `"enabled"` or `"disabled"` |
| `seq` | `int` | `0` | Monotonic sequence number |
| `motors_online` | `int` | `0` | Number of motors reporting feedback (out of 12) |
| `feedback_age_ms` | `int` | `-1` | Age of stalest motor feedback in milliseconds |
| `joint_positions` | `List[float]` | `[0.0]*12` | 12 joint angles (rad), policy order |
| `joint_velocities` | `List[float]` | `[0.0]*12` | 12 joint velocities (rad/s), policy order |
| `motor_positions` | `List[float]` | `[0.0]*12` | 12 raw motor positions, motor order |
| `motor_velocities` | `List[float]` | `[0.0]*12` | 12 raw motor velocities, motor order |
| `motor_torques` | `List[float]` | `[0.0]*12` | 12 raw motor torques, motor order |
| `target_joint_positions` | `List[float]` | `[0.0]*12` | Last commanded joint targets |
| `raw` | `dict` | `{}` | Full raw JSON dict |

**Methods:**

#### `from_json(text: str) -> RobotState` (classmethod)

Parses a JSON string from the twin-agent state stream into a `RobotState` instance. Handles missing keys gracefully with defaults. Casts all numeric fields to native Python types.

```python
state = RobotState.from_json('{"ok":true,"mode":"enabled","seq":1,...}')
assert state.ok == True
```

---

### Class `TwinAgentClient`

**Purpose:** Thread-safe TCP client with two channels: a command channel (request-response) and a state channel (subscribe-stream).

**Constructor:**

```python
TwinAgentClient(host: str, cmd_port: int = 47001, state_port: int = 47002, timeout_s: float = 3.0)
```

| Parameter | Default | Description |
|---|---|---|
| `host` | (required) | IP/hostname of the Jetson running twin_agent |
| `cmd_port` | `47001` | TCP command port |
| `state_port` | `47002` | TCP state stream port |
| `timeout_s` | `3.0` | Connection and command timeout (seconds) |

**Properties:**

| Property | Type | Description |
|---|---|---|
| `connected` | `bool` | `True` if both cmd and state sockets are open and state thread is running |
| `last_error` | `str` | Most recent error message from state thread or failed command |

**Methods:**

#### `connect() -> None`

Opens TCP connections to both command and state ports. Spawns a daemon thread `_state_loop` to continuously receive state updates. Raises `ConnectionRefusedError` or `socket.timeout` on failure.

#### `close() -> None`

Gracefully shuts down both sockets (`SHUT_RDWR` then close). Stops the state loop. Safe to call multiple times.

#### `send_command(command: str) -> dict`

Sends a text command (newline-terminated) to the command port and waits for a JSON reply. Thread-safe via `_cmd_lock`. Returns the parsed JSON dict. Sets `last_error` if reply has `"ok": false`.

```python
reply = client.send_command("ping")
# {"ok": true, "msg": "pong"}
```

#### `ping() -> bool`

Sends `"ping"`, returns `True` if reply has `"ok": true`.

#### `init_robot(seconds: float = 2.5) -> dict`

Sends `"init <seconds>"` to enable motors and smoothly move to standing offset.

#### `enable() -> dict`

Sends `"enable"` to enable all motors.

#### `disable() -> dict`

Sends `"disable"` to disable all motors.

#### `hold() -> dict`

Sends `"hold"` to hold current standing offset position.

#### `get_state_once() -> dict`

Sends `"get_state"` for a single state snapshot via the command port.

#### `set_joint(targets_rad: List[float]) -> dict`

Sends `"set_joint <12 floats>"` to command 12 joint targets. Validates that exactly 12 values are provided. Values are formatted to 6 decimal places.

```python
client.set_joint([0.1, -0.05, 0.3, 0.1, -0.05, 0.3, 0.1, -0.05, 0.3, 0.1, -0.05, 0.3])
```

#### `latest_state() -> Optional[RobotState]`

Returns the most recent `RobotState` from the state stream, or `None` if no data received yet. Thread-safe.

#### `latest_state_age_ms() -> float`

Returns milliseconds elapsed since the last state packet was received. Returns `inf` if no state has been received.

**Internal Methods:**

#### `_state_loop() -> None` (private)

Daemon thread target. Continuously receives data from the state socket, splits on newlines, parses each line as `RobotState.from_json()`, and stores the latest under `_state_lock`. Sets `last_error` on parse failure or socket errors.

---

## File 2: `joystick_input.py`

Optional gamepad input module. Reads a Linux gamepad via the `inputs` package and maps stick axes to velocity commands.

### Class `CommandState`

**Purpose:** Simple dataclass holding `(x, y, yaw)` velocity command values.

| Field | Type | Default | Description |
|---|---|---|---|
| `x` | `float` | `0.0` | Forward/backward velocity command |
| `y` | `float` | `0.0` | Left/right velocity command |
| `yaw` | `float` | `0.0` | Yaw angular velocity command |

### Module-level Constants

| Constant | Value | Description |
|---|---|---|
| `INPUTS_AVAILABLE` | `bool` | `True` if the `inputs` package is importable |

### Class `OptionalGamepad`

**Purpose:** Best-effort gamepad reader. Non-blocking. The main app works without it.

**Constructor:**

```python
OptionalGamepad(deadzone: float = 0.08)
```

| Parameter | Default | Description |
|---|---|---|
| `deadzone` | `0.08` | Axis values below this magnitude are zeroed |

**Properties:**

| Property | Type | Description |
|---|---|---|
| `available` | `bool` | Whether the `inputs` library was importable |
| `enabled` | settable `bool` | Whether gamepad input should be processed |

**Methods:**

#### `start() -> None`

Starts the background polling thread. No-op if `inputs` is not available or already running.

#### `stop() -> None`

Stops the background polling thread. Joins with 1s timeout.

#### `set_enabled(enabled: bool) -> None`

Enable or disable gamepad processing without stopping the thread.

#### `get_command() -> Tuple[float, float, float]`

Returns current `(x, y, yaw)` command values. Thread-safe via lock.

**Stick Mapping:**

| Gamepad Axis | Code | Maps to |
|---|---|---|
| Left stick vertical | `ABS_Y` | `-cmd_x` (inverted: stick up = forward) |
| Left stick horizontal | `ABS_X` | `cmd_y` |
| Right stick horizontal | `ABS_RX` | `cmd_yaw` |

**Internal Methods:**

#### `_normalize(value: int) -> float` (private)

Converts raw 16-bit axis value `[-32767, 32767]` to `[-1.0, 1.0]`. Applies deadzone.

#### `_update_state() -> None` (private)

Reads raw axis values, normalizes them, and updates the internal `CommandState` under lock.

---

## File 3: `relay_engine.py`

Core simulation and relay engine. Runs MuJoCo physics + ONNX policy inference in a background thread, with safety checks and action relay to the real robot.

### Module-level Constants

#### `JOINT_NAMES`

```python
["LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
 "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
 "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee"]
```

12 joint names in **policy order** (same as twin-agent order).

#### `SIM_TO_POLICY`

```python
np.array([0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11])
```

Index remap: MuJoCo sim order → policy order.

**MuJoCo order:** `[LF_A, LF_F, LF_K, LR_A, LR_F, LR_K, RF_A, RF_F, RF_K, RR_A, RR_F, RR_K]`
**Policy order:** `[LF_A, LR_A, RF_A, RR_A, LF_F, LR_F, RF_F, RR_F, LF_K, LR_K, RF_K, RR_K]`

#### `POLICY_TO_SIM`

```python
np.array([0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11])
```

Index remap: policy order → MuJoCo sim order (inverse of `SIM_TO_POLICY`).

### Class `EngineConfig`

**Purpose:** Configuration dataclass for `SimPolicyRelayEngine.connect_and_load()`.

| Field | Type | Default | Description |
|---|---|---|---|
| `host` | `str` | (required) | Jetson IP address |
| `cmd_port` | `int` | `47001` | twin-agent command TCP port |
| `state_port` | `int` | `47002` | twin-agent state TCP port |
| `xml_path` | `str` | `""` | Path to MuJoCo XML model file |
| `onnx_path` | `str` | `""` | Path to ONNX policy model |
| `viewer_enabled` | `bool` | `False` | Launch MuJoCo 3D viewer window |
| `sim_dt` | `float` | `0.005` | MuJoCo timestep → 200 Hz physics |
| `decimation` | `int` | `4` | Policy runs every N sim steps → 50 Hz |
| `frame_stack` | `int` | `10` | Number of observation frames to stack |
| `single_obs_dim` | `int` | `45` | Single-frame observation dimension |
| `action_scale` | `float` | `0.25` | Multiply raw policy output by this before sending to motors |
| `clip_observations` | `float` | `100.0` | Clip observation values to [-100, 100] |
| `clip_actions` | `float` | `100.0` | Clip action values to [-100, 100] |
| `max_feedback_age_ms` | `int` | `200` | E-stop if motor feedback is stale |
| `max_state_stream_age_ms` | `int` | `500` | E-stop if TCP state stream is stale |
| `divergence_threshold_rad` | `float` | `0.35` | Joint divergence threshold for auto-stop |
| `auto_stop_on_divergence` | `bool` | `False` | Auto-pause policy on divergence |
| `kp` | `List[float]` | `[25.0]*12` | PD position gains (sim order) |
| `kd` | `List[float]` | `[0.5]*12` | PD velocity damping gains (sim order) |
| `tau_limit` | `List[float]` | `[17,17,25]*4` | Torque limits per joint (sim order): HipA=17, HipF=17, Knee=25 |

### Class `EngineSnapshot`

**Purpose:** Read-only snapshot of engine state for the GUI to poll. Returned by `get_snapshot()`.

| Field | Type | Default | Description |
|---|---|---|---|
| `connected` | `bool` | `False` | TCP connection to twin-agent is active |
| `initialized` | `bool` | `False` | Robot has been initialized to standing offset |
| `policy_running` | `bool` | `False` | Policy inference loop is active |
| `sim_thread_running` | `bool` | `False` | Background sim thread is alive |
| `estopped` | `bool` | `False` | Emergency stop has been triggered |
| `viewer_enabled` | `bool` | `False` | MuJoCo 3D viewer is active |
| `last_error` | `str` | `""` | Most recent error message |
| `last_warning` | `str` | `""` | Most recent warning (e.g., divergence) |
| `status_text` | `str` | `""` | Human-readable status string |
| `motors_online` | `int` | `0` | Number of motors with recent feedback |
| `feedback_age_ms` | `int` | `-1` | Age of stalest motor feedback |
| `state_stream_age_ms` | `float` | `inf` | Age of last TCP state packet |
| `sim_fps` | `float` | `0.0` | MuJoCo physics steps per second |
| `policy_fps` | `float` | `0.0` | ONNX inference calls per second |
| `joint_names` | `List[str]` | 12 names | Policy-order joint names |
| `real_joint_positions` | `List[float]` | `[0.0]*12` | Real robot joint angles from state stream |
| `sim_joint_positions` | `List[float]` | `[0.0]*12` | MuJoCo sim joint angles (relative to default) |
| `scaled_action` | `List[float]` | `[0.0]*12` | Last action sent to robot (after action_scale) |
| `raw_action` | `List[float]` | `[0.0]*12` | Last raw policy output (before action_scale) |
| `diff_real_minus_sim` | `List[float]` | `[0.0]*12` | Per-joint divergence: real - sim |
| `cmd` | `List[float]` | `[0,0,0]` | Current velocity command `[x, y, yaw]` |

### Class `SimPolicyRelayEngine`

**Purpose:** Core engine that runs MuJoCo simulation, ONNX policy inference, and relays actions to the real robot via `TwinAgentClient`.

#### Constructor

```python
SimPolicyRelayEngine() -> None
```

Initializes all internal state to defaults. No connections are made until `connect_and_load()` is called.

#### `connect_and_load(cfg: EngineConfig) -> None`

Validates config, opens TCP connection to twin-agent (ping check), loads ONNX session, loads MuJoCo model, extracts default joint pose from `model.qpos0`, validates model layout (expects `nq >= 19`, `nv >= 18` for free-base + 12 joints), and spawns the background `_sim_loop` thread.

**Raises:** `RuntimeError` if already running, `FileNotFoundError` if XML/ONNX not found, or if ping fails.

#### `init_robot(seconds: float = 2.5) -> dict`

Sends `init <seconds>` command to twin-agent. Sets `_initialized = True` on success. Returns the command reply dict.

**Raises:** `RuntimeError` if not connected or twin-agent returns error.

#### `start_policy() -> None`

Enables the policy inference loop. The sim thread will start running ONNX inference and sending actions to the real robot.

**Raises:** `RuntimeError` if not connected, not initialized, or in E-stop state.

#### `stop_policy() -> None`

Pauses policy inference. The sim freezes (keeps running but skips policy steps). Sends the last scaled action as a frozen target to the robot.

#### `hold() -> dict`

Sends `"hold"` to twin-agent. Stops policy and tells robot to hold current offset.

#### `disable() -> dict`

Sends `"disable"` to twin-agent. Stops policy, clears initialized state.

#### `emergency_stop(reason: str = "manual E-stop") -> None`

Immediately stops policy, sets E-stop flag, and sends `"disable"` to the robot. If already E-stopped, is a no-op.

#### `reset_sim() -> None`

Requests simulation reset. The sim thread will reset MuJoCo state to defaults on the next iteration.

#### `shutdown() -> None`

Stops policy and sim thread, sends `"hold"` then `"disable"` to robot, closes TCP connection.

#### `update_command(cmd_x: float, cmd_y: float, cmd_yaw: float) -> None`

Updates the velocity command fed into the policy observation. Thread-safe.

#### `update_safety_settings(divergence_threshold_rad: float, auto_stop_on_divergence: bool) -> None`

Updates divergence threshold and auto-stop flag at runtime. Thread-safe.

#### `get_snapshot() -> EngineSnapshot`

Returns a frozen snapshot of current engine state for the GUI to display. Thread-safe.

### Internal Methods (Sim Loop)

#### `_sim_loop() -> None`

Entry point for the background sim thread. Dispatches to `_run_with_viewer()` or `_run_no_viewer()` based on config.

#### `_run_with_viewer(model, data, cfg, ...) -> None`

Launches MuJoCo passive viewer and runs the main loop at ~200 Hz. Camera positioned at distance=3.0, azimuth=90, elevation=-45, looking at `[0, -0.25, 0.824]`.

#### `_run_no_viewer(model, data, cfg, ...) -> None`

Headless sim loop without viewer. Same physics and policy logic.

#### `_run_one_iteration(model, data, cfg, ...) -> tuple`

Single sim step. Handles:
1. Pending reset requests
2. Reading latest real robot state from the state stream
3. If policy is running: calls `_step_policy_and_sim()`
4. If policy is paused: sleeps for `sim_dt`
5. FPS tracking (updated every 1 second)
6. Timing: sleeps to maintain `sim_dt` cadence

#### `_apply_reset(data: mujoco.MjData) -> None`

Resets MuJoCo `qpos` to `model.qpos0`, zeros `qvel` and `ctrl`, calls `mj_forward()`. Clears all internal state: targets, actions, obs history.

#### `_step_policy_and_sim(data: mujoco.MjData, cmd: np.ndarray, lowlevel_count: int) -> None`

Core policy + sim step. On every sim tick:
1. Reads current joint positions and velocities from MuJoCo
2. Remaps sim order → policy order via `SIM_TO_POLICY`
3. Computes relative joint positions (minus default pose)
4. **Every `decimation` ticks (50 Hz):**
   - Reads base orientation quaternion and angular velocity
   - Computes projected gravity vector
   - Builds 45-dim observation: `[omega(3), gravity(3), cmd(3), joint_pos(12), joint_vel(12), last_action(12)]`
   - Clips observation
   - Appends to history buffer (10 frames)
   - Concatenates to 450-dim stacked input
   - Runs ONNX inference
   - Validates output shape == 12
   - Clips raw action
   - Computes scaled action = raw × action_scale
   - Remaps policy order → sim order via `POLICY_TO_SIM`
   - Adds default joint pose to get absolute sim targets
   - Runs safety check `_safe_to_send()`
   - Sends scaled action to robot via `client.set_joint()`
5. **Every tick:** Computes PD torque, applies to `data.ctrl`, steps MuJoCo

**Observation layout (45-dim single frame):**

```
[0:3]   angular_velocity (base frame)
[3:6]   projected_gravity (base frame)
[6:9]   velocity_command [cmd_x, cmd_y, cmd_yaw]
[9:21]  joint_positions (relative to default, policy order)
[21:33] joint_velocities (policy order)
[33:45] last_action (raw policy output, policy order)
```

**Stacked input: 10 × 45 = 450 dims**

#### `_pd_control(target_q_sim, q_sim_abs, dq_sim) -> np.ndarray`

Computes PD torques in sim order: `tau = kp × (target - current) + kd × (0 - velocity)`. Clips to torque limits.

#### `_extract_angular_velocity(data: mujoco.MjData) -> np.ndarray`

Tries to read from MuJoCo sensor named `"angular-velocity"`. Falls back to `qvel[3:6]` (base angular velocity) if sensor not found.

#### `_projected_gravity(quat_xyzw: np.ndarray) -> np.ndarray` (static)

Computes gravity vector projected into the base frame: `R_inv × [0, 0, -1]`. Uses scipy Rotation with `(x, y, z, w)` quaternion convention.

#### `_safe_to_send(scaled_action: np.ndarray) -> bool`

Pre-send safety check. Returns `False` and triggers E-stop if any condition fails:
1. No client object → E-stop "missing twin_agent client"
2. No state received → E-stop "no real-robot state available"
3. `motors_online != 12` → E-stop with count
4. `feedback_age_ms >= max_feedback_age_ms` → E-stop with age
5. `state_stream_age_ms >= max_state_stream_age_ms` → E-stop with age
6. Joint divergence ≥ threshold → warning (auto-stops if `auto_stop_on_divergence` is enabled)

Returns `True` only if all checks pass.

---

## File 4: `sim_policy_relay.py`

Tkinter GUI application. Provides the full user interface for the sim_policy_relay system.

### Class `SimPolicyRelayApp`

**Purpose:** Main application window with connection settings, robot control buttons, velocity command sliders, and a joint monitor table.

#### Constructor

```python
SimPolicyRelayApp() -> None
```

Creates the Tkinter root window (1220×760), instantiates the engine and gamepad, builds the UI, binds keyboard shortcuts, and starts the 100ms refresh loop.

#### GUI Layout

**Connection Settings (top panel):**
- Jetson IP (text entry, default `127.0.0.1`)
- Command port (text entry, default `47001`)
- State port (text entry, default `47002`)
- MuJoCo viewer checkbox
- MuJoCo XML path (text entry + Browse button)
- ONNX Policy path (text entry + Browse button)
- Diff threshold (text entry, default `0.35`)
- Auto-stop on divergence checkbox
- Use gamepad checkbox
- **Connect** button

**Control Buttons (middle panel):**
- Init Robot, Start Policy, Stop Policy, Hold, Disable, Reset Sim
- **E-Stop** (red, large button)

**Velocity Commands panel:**
- `cmd_x` slider (-2.0 to 2.0)
- `cmd_y` slider (-1.0 to 1.0)
- `cmd_yaw` slider (-1.5 to 1.5)
- Zero Command button

**Joint Monitor Table (bottom panel):**
12 rows × 5 columns:

| Column | Content | Color Coding |
|---|---|---|
| Joint | Name (e.g., `LF_HipA`) | — |
| Real(rad) | Real robot joint angle | Green/yellow/red by diff |
| Sim(rad) | MuJoCo sim joint angle | Green/yellow/red by diff |
| Policy Action | Last scaled action sent | Green/yellow/red by diff |
| Diff(real-sim) | Divergence | Green/yellow/red by diff |

Row background colors:
- Green (`#e8f5e9`): `|diff| < 0.5 × threshold`
- Yellow (`#fff9c4`): `0.5 × threshold ≤ |diff| < threshold`
- Red (`#ffebee`): `|diff| ≥ threshold`

**Status Bar:**
- Status line: connection state, flags
- Stats line: motors_online, feedback_age_ms, policy_fps, sim_fps
- Error line: error/warning messages (red text)

#### Keyboard Shortcuts

| Key | Action |
|---|---|
| `W` | cmd_x += 0.05 (forward) |
| `S` | cmd_x -= 0.05 (backward) |
| `A` | cmd_y += 0.05 (strafe left) |
| `D` | cmd_y -= 0.05 (strafe right) |
| `Q` | cmd_yaw += 0.05 (turn left) |
| `E` | cmd_yaw -= 0.05 (turn right) |
| `Space` | Zero all commands |

#### Methods

#### `_build_ui() -> None`

Constructs all UI widgets using `tkinter.ttk`.

#### `_bind_keyboard() -> None`

Binds W/S/A/D/Q/E/Space keyboard events.

#### `_make_cmd_slider(parent, row, label_text, variable, min_v, max_v) -> None`

Creates a label + horizontal scale + text entry for one command axis.

#### `_on_command_changed(*_args) -> None`

Callback when any command slider changes. Calls `engine.update_command()`.

#### `_nudge(var: tk.DoubleVar, delta: float) -> None`

Increments a `DoubleVar` by `delta` (used by keyboard shortcuts).

#### `zero_command() -> None`

Sets all three command sliders to 0.0.

#### `pick_xml() / pick_onnx() -> None`

File picker dialogs for MuJoCo XML and ONNX model paths.

#### `on_connect() -> None`

Reads all GUI fields, constructs `EngineConfig`, calls `engine.connect_and_load()`. Shows error dialog on failure.

#### `on_init() -> None`

Calls `engine.init_robot(2.5)`.

#### `on_start_policy() -> None`

Updates safety settings from GUI, calls `engine.start_policy()`.

#### `on_stop_policy() -> None`

Calls `engine.stop_policy()`.

#### `on_hold() -> None`

Calls `engine.hold()`.

#### `on_disable() -> None`

Calls `engine.disable()`.

#### `on_estop() -> None`

Calls `engine.emergency_stop("manual E-stop from GUI")`.

#### `on_reset_sim() -> None`

Calls `engine.reset_sim()`.

#### `_set_button_states(connected, initialized, policy_running) -> None`

Enables/disables buttons based on current state machine:
- **Connect:** disabled when connected
- **Init Robot:** enabled when connected
- **Start Policy:** enabled when connected + initialized + not running
- **Stop Policy:** enabled when policy running
- **Hold/Disable/Reset:** enabled when connected
- **E-Stop:** enabled when connected

#### `_refresh_loop() -> None`

Polls at 100ms intervals:
1. If gamepad enabled and available: updates command sliders from gamepad
2. Gets engine snapshot
3. Updates button enable states
4. Updates status/stats/error text
5. Updates joint monitor table (values + color coding)

#### `on_close() -> None`

Calls `engine.shutdown()` and `gamepad.stop()`, then destroys the Tkinter root.

#### `run() -> None`

Starts the Tkinter main loop.

---

## Safety System

### Pre-send Checks (`_safe_to_send`)

Before every action relay to the real robot, the engine verifies:

| Check | Condition | Action on Failure |
|---|---|---|
| Client exists | `self._client is not None` | E-stop |
| State available | `latest_state() is not None` | E-stop |
| All motors online | `motors_online == 12` | E-stop |
| Fresh feedback | `feedback_age_ms < 200` | E-stop |
| Fresh state stream | `stream_age_ms < 500` | E-stop |
| Joint divergence | `max|real - sim| < threshold` | Warning + optional auto-stop |

### Divergence Auto-Stop

When enabled, if `max|real_joint - sim_joint| ≥ divergence_threshold_rad`:
1. Policy is paused (but sim keeps running)
2. Status text changes to `"auto-stopped on divergence"`
3. No actions are sent to the robot until user clicks Start Policy again

### Shutdown Sequence

On GUI close or `shutdown()`:
1. Stop policy
2. Send `"hold"` (robot holds offset)
3. Send `"disable"` (motors power down)
4. Join sim thread (2s timeout)
5. Close TCP sockets

---

## Joint Order Reference

```
Policy / twin-agent order:
  [LF_HipA, LR_HipA, RF_HipA, RR_HipA,
   LF_HipF, LR_HipF, RF_HipF, RR_HipF,
   LF_Knee, LR_Knee, RF_Knee, RR_Knee]
  Indices:  0       1       2       3       4       5       6       7       8       9       10      11

MuJoCo sim order:
  [LF_A, LF_F, LF_K, LR_A, LR_F, LR_K, RF_A, RF_F, RF_K, RR_A, RR_F, RR_K]
  Indices:  0     1     2     3     4     5     6     7     8     9     10    11

Remap tables:
  SIM_TO_POLICY  = [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]
  POLICY_TO_SIM  = [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]
```

**Key point:** Policy output is sent directly to `set_joint` in policy order — no remap needed between policy and twin-agent, because they share the same joint ordering.
