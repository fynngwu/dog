"""Benchmark: wall-clock matched viewer pacing (1:1 realtime)."""
from __future__ import annotations

import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import onnxruntime as ort

import mujoco
import mujoco.viewer

sys.path.insert(0, str(Path(__file__).resolve().parent))
from relay_engine import SIM_TO_POLICY, POLICY_TO_SIM

xml_path = Path(__file__).with_name("leggedrobot_flat.xml")
onnx_path = Path(__file__).with_name("policy.onnx")

print("Loading...")
model = mujoco.MjModel.from_xml_path(str(xml_path))
model.opt.timestep = 0.005
data = mujoco.MjData(model)
data.qpos[:] = model.qpos0
mujoco.mj_forward(model, data)

session = ort.InferenceSession(str(onnx_path))
input_name = session.get_inputs()[0].name
dummy_pi = np.zeros((1, 450), dtype=np.float32)

default_joint_sim = np.array(model.qpos0[7:19], dtype=np.float64)
default_joint_policy = default_joint_sim[SIM_TO_POLICY].copy()
target_q_sim = default_joint_sim.copy()
kp = np.array([25.0] * 12)
kd = np.array([0.5] * 12)
tau_limit = np.array([17.0, 17.0] * 6)
raw_action_policy = np.zeros(12, dtype=np.float64)
cmd = np.zeros(3, dtype=np.float32)
hist_obs = deque(maxlen=10)
for _ in range(10):
    hist_obs.append(np.zeros(45, dtype=np.float32))

for _ in range(100):
    mujoco.mj_step(model, data)
for _ in range(20):
    session.run(None, {input_name: dummy_pi})

DURATION = 5.0
dt = 0.005
decimation = 4


def do_sim_step():
    q = np.array(data.qpos[7:19], dtype=np.float64)
    dq = np.array(data.qvel[6:18], dtype=np.float64)
    tau = kp * (target_q_sim - q) + kd * (0.0 - dq)
    data.ctrl[:12] = np.clip(tau, -tau_limit, tau_limit)
    mujoco.mj_step(model, data)


def do_policy_step():
    q = np.array(data.qpos[7:19], dtype=np.float64)
    dq = np.array(data.qvel[6:18], dtype=np.float64)
    q_rel = q[SIM_TO_POLICY] - default_joint_policy
    dq_pol = dq[SIM_TO_POLICY]
    parts = [np.array(data.qvel[3:6], dtype=np.float64),
             np.array([0.0, 0.0, -1.0]), cmd, q_rel, dq_pol, raw_action_policy]
    obs = np.clip(np.concatenate(parts).astype(np.float32), -100.0, 100.0)
    hist_obs.append(obs)
    pi = np.concatenate(list(hist_obs), axis=0)[None, :]
    raw = np.clip(session.run(None, {input_name: pi})[0][0], -100.0, 100.0)
    target_q_sim[:] = (raw * 0.25)[POLICY_TO_SIM] + default_joint_sim


def reset():
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    target_q_sim[:] = default_joint_sim
    raw_action_policy[:] = 0
    hist_obs.clear()
    for _ in range(10):
        hist_obs.append(np.zeros(45, dtype=np.float32))


print(f"\nRunning {DURATION}s wall-clock matched sim with viewer...")

reset()
sim_time = 0.0
sim_ticks = 0
pol_ticks = 0
lowlevel = 0
wall_t0 = time.perf_counter()
frame_count = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.distance = 3.0
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -45
    viewer.cam.lookat[:] = np.array([0.0, -0.25, 0.824])
    while wall_t0 + DURATION > time.perf_counter() and viewer.is_running():
        wall_elapsed = time.perf_counter() - wall_t0
        while sim_time < wall_elapsed:
            do_sim_step()
            sim_ticks += 1
            if lowlevel % decimation == 0:
                do_policy_step()
                pol_ticks += 1
            lowlevel += 1
            sim_time += dt
        viewer.sync()
        frame_count += 1

wall = time.perf_counter() - wall_t0
sim_fps = sim_ticks / wall
pol_fps = pol_ticks / wall
rt = sim_ticks * dt / wall * 100
drift_ms = (sim_ticks * dt - wall) * 1000

print(f"\n  Wall time:    {wall:.3f} s")
print(f"  Sim time:     {sim_ticks * dt:.3f} s")
print(f"  Sim steps:    {sim_ticks}")
print(f"  Policy steps: {pol_ticks}")
print(f"  Viewer frames:{frame_count}")
print(f"  Sim FPS:      {sim_fps:.1f}  (target 200)")
print(f"  Policy FPS:   {pol_fps:.1f}  (target 50)")
print(f"  Realtime:     {rt:.1f}%")
print(f"  Drift:        {drift_ms:+.1f} ms")
