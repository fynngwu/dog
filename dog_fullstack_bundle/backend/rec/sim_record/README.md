# sim_record

Standalone MuJoCo policy recorder derived from the provided `auto_test.py`. It runs an ONNX locomotion policy in simulation, executes a scheduled command sequence, records policy observations/actions/targets into CSV, and optionally produces a diagnostic plot.

Its main purpose is to create replayable traces for the real-robot daemon in `dog_cli_tool`.

## What it does

- loads a MuJoCo XML and ONNX policy
- runs the same policy observation stack structure as the reference tooling
- supports a YAML schedule or a one-line inline command
- records policy-rate samples to CSV with explicit columns for replay and analysis
- optionally writes an `.npz` archive and a plot PNG

## YAML schedule format

A schedule is a YAML file with a top-level `schedule:` list. Each phase contains:

- `cmd`: `[cmd_x, cmd_y, cmd_yaw]`
- `duration`: seconds
- `name`: optional label used in the plot and CSV

Example:

```yaml
schedule:
  - name: Stand
    cmd: [0.0, 0.0, 0.0]
    duration: 3.0
  - name: Forward
    cmd: [0.5, 0.0, 0.0]
    duration: 5.0
  - name: TurnL
    cmd: [0.5, 0.0, 0.3]
    duration: 3.0
```

Example files are included in `examples/`.

## Usage examples

YAML schedule:

```bash
python sim_record.py \
  --onnx policy.onnx \
  --xml leggedrobot_flat.xml \
  --schedule examples/walk_turn.yaml \
  --output recording.csv \
  --plot-output recording.png
```

Inline one-phase command:

```bash
python sim_record.py \
  --onnx policy.onnx \
  --xml leggedrobot_flat.xml \
  --cmd 0.5 0.0 0.0 \
  --duration 10.0 \
  --output recording.csv
```

Headless mode:

```bash
python sim_record.py --onnx policy.onnx --xml leggedrobot_flat.xml \
  --schedule examples/walk_forward.yaml --output recording.csv --headless
```

## CSV format

Each row is recorded at the policy update rate (default 50 Hz, with a 0.005 s sim step and decimation 4). The CSV columns are:

- `timestamp_ms`, `step`, `sim_time`, `phase`, `phase_name`
- `cmd_x`, `cmd_y`, `cmd_yaw`
- `obs_0..obs_44`
- `raw_action_0..raw_action_11`
- `scaled_action_0..scaled_action_11`
- `target_q_0..target_q_11`
- `joint_pos_0..joint_pos_11`
- `joint_vel_0..joint_vel_11`

Important conventions:

- `scaled_action_*` is in **policy order** and is the exact column group used by daemon replay.
- `target_q_*` is the post-remap target pose in policy order after adding the sim default pose.
- `joint_pos_*` and `joint_vel_*` are policy-order simulated joint states.

## How the CSV feeds into `dog_cli.py replay`

The daemon command

```bash
python dog_cli.py --host 192.168.1.100 replay /path/to/recording.csv 1.0
```

reads `scaled_action_0..11`, reconstructs each sample timestamp from `timestamp_ms` or `sim_time`, and sends those relative joint targets to the real robot through the same clamped joint-space path used by `set_joint`.

That means the replay interface is intentionally simple:

- `sim_record` owns **recording**
- the daemon owns **real-robot replay**
- the shared contract is the `scaled_action_*` policy-order columns
