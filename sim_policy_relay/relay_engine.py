from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import onnxruntime as ort
from scipy.spatial.transform import Rotation as R

import mujoco
import mujoco.viewer

from twin_client import RobotState, TwinAgentClient


JOINT_NAMES = [
    "LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
    "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
    "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee",
]

SIM_TO_POLICY = np.array([0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11], dtype=np.int64)
POLICY_TO_SIM = np.array([0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11], dtype=np.int64)


@dataclass
class EngineConfig:
    sim_only: bool = False
    host: str = "127.0.0.1"
    cmd_port: int = 47001
    state_port: int = 47002
    xml_path: str = ""
    onnx_path: str = ""
    viewer_enabled: bool = False
    sim_dt: float = 0.005
    decimation: int = 4
    frame_stack: int = 10
    single_obs_dim: int = 45
    action_scale: float = 0.25
    clip_observations: float = 100.0
    clip_actions: float = 100.0
    max_feedback_age_ms: int = 200
    max_state_stream_age_ms: int = 500
    divergence_threshold_rad: float = 0.35
    auto_stop_on_divergence: bool = False
    kp: List[float] = field(default_factory=lambda: [25.0] * 12)
    kd: List[float] = field(default_factory=lambda: [0.5] * 12)
    tau_limit: List[float] = field(default_factory=lambda: [17.0, 17.0, 25.0] * 4)


@dataclass
class EngineSnapshot:
    connected: bool = False
    initialized: bool = False
    policy_running: bool = False
    sim_thread_running: bool = False
    estopped: bool = False
    viewer_enabled: bool = False
    last_error: str = ""
    last_warning: str = ""
    status_text: str = ""
    motors_online: int = 0
    feedback_age_ms: int = -1
    state_stream_age_ms: float = float("inf")
    sim_fps: float = 0.0
    policy_fps: float = 0.0
    joint_names: List[str] = field(default_factory=lambda: list(JOINT_NAMES))
    real_joint_positions: List[float] = field(default_factory=lambda: [0.0] * 12)
    sim_joint_positions: List[float] = field(default_factory=lambda: [0.0] * 12)
    scaled_action: List[float] = field(default_factory=lambda: [0.0] * 12)
    raw_action: List[float] = field(default_factory=lambda: [0.0] * 12)
    diff_real_minus_sim: List[float] = field(default_factory=lambda: [0.0] * 12)
    cmd: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


class SimPolicyRelayEngine:
    def __init__(self) -> None:
        self._cfg: Optional[EngineConfig] = None
        self._client: Optional[TwinAgentClient] = None
        self._session: Optional[ort.InferenceSession] = None
        self._model: Optional[mujoco.MjModel] = None
        self._data: Optional[mujoco.MjData] = None

        self._sim_thread: Optional[threading.Thread] = None
        self._running = False
        self._policy_running = False
        self._initialized = False
        self._connected = False
        self._estopped = False

        self._lock = threading.RLock()
        self._last_error = ""
        self._last_warning = ""
        self._status_text = "idle"

        self._cmd = np.zeros(3, dtype=np.float64)
        self._latest_real_state: Optional[RobotState] = None
        self._sim_joint_policy = np.zeros(12, dtype=np.float64)
        self._raw_action_policy = np.zeros(12, dtype=np.float64)
        self._scaled_action_policy = np.zeros(12, dtype=np.float64)

        self._default_qpos = None
        self._default_qvel = None
        self._default_joint_sim = np.zeros(12, dtype=np.float64)
        self._default_joint_policy = np.zeros(12, dtype=np.float64)
        self._target_q_sim = np.zeros(12, dtype=np.float64)
        self._hist_obs = deque(maxlen=10)

        self._request_reset = False
        self._input_name = None
        self._sim_fps = 0.0
        self._policy_fps = 0.0

    def connect_and_load(self, cfg: EngineConfig) -> None:
        if self._running:
            raise RuntimeError("engine is already connected")

        xml_path = Path(cfg.xml_path)
        onnx_path = Path(cfg.onnx_path)
        if not xml_path.exists():
            raise FileNotFoundError(f"MuJoCo XML not found: {xml_path}")
        if not onnx_path.exists():
            raise FileNotFoundError(f"ONNX model not found: {onnx_path}")

        client = None
        if not cfg.sim_only:
            client = TwinAgentClient(cfg.host, cfg.cmd_port, cfg.state_port)
            client.connect()
            if not client.ping():
                client.close()
                raise RuntimeError("twin_agent ping failed")

        session = ort.InferenceSession(str(onnx_path))
        input_meta = session.get_inputs()[0]
        self._input_name = input_meta.name
        if input_meta.shape[-1] not in (cfg.frame_stack * cfg.single_obs_dim, None, "None"):
            # Keep soft validation; dynamic shapes are acceptable.
            pass

        model = mujoco.MjModel.from_xml_path(str(xml_path))
        model.opt.timestep = cfg.sim_dt
        data = mujoco.MjData(model)

        default_qpos = np.array(model.qpos0, dtype=np.float64)
        default_qvel = np.zeros(model.nv, dtype=np.float64)
        if model.nq < 19 or model.nv < 18:
            if client is not None:
                client.close()
            raise RuntimeError(
                f"unexpected MuJoCo layout: nq={model.nq}, nv={model.nv}; expected free-base + 12 joints"
            )

        default_joint_sim = default_qpos[7:19].copy()
        default_joint_policy = default_joint_sim[SIM_TO_POLICY].copy()

        with self._lock:
            self._cfg = cfg
            self._client = client
            self._session = session
            self._model = model
            self._data = data
            self._default_qpos = default_qpos
            self._default_qvel = default_qvel
            self._default_joint_sim = default_joint_sim
            self._default_joint_policy = default_joint_policy
            self._target_q_sim = default_joint_sim.copy()
            self._sim_joint_policy = np.zeros(12, dtype=np.float64)
            self._raw_action_policy = np.zeros(12, dtype=np.float64)
            self._scaled_action_policy = np.zeros(12, dtype=np.float64)
            self._cmd[:] = 0.0
            self._reset_obs_history_locked()
            self._connected = True
            self._initialized = False
            self._estopped = False
            self._policy_running = False
            self._last_error = ""
            self._last_warning = ""
            self._status_text = "connected"
            self._request_reset = True

        self._running = True
        self._sim_thread = threading.Thread(target=self._sim_loop, name="sim-policy-relay", daemon=True)
        self._sim_thread.start()

    def init_robot(self, seconds: float = 2.5) -> dict:
        if self._cfg is not None and self._cfg.sim_only:
            with self._lock:
                self._initialized = True
                self._status_text = "sim-only: initialized"
            return {"ok": True, "msg": "sim-only mode"}
        client = self._require_client()
        reply = client.init_robot(seconds)
        if not reply.get("ok", False):
            raise RuntimeError(reply.get("error", "init failed"))
        with self._lock:
            self._initialized = True
            self._status_text = "robot initialized"
        return reply

    def start_policy(self) -> None:
        with self._lock:
            if not self._connected:
                raise RuntimeError("connect first")
            if self._cfg is not None and self._cfg.sim_only and not self._initialized:
                self._initialized = True
                self._status_text = "auto-initialized (sim-only)"
            if not self._initialized:
                raise RuntimeError("init robot first")
            if self._estopped:
                raise RuntimeError("engine is in E-stop state; reconnect to clear it")
            self._policy_running = True
            self._status_text = "policy running"
            self._last_warning = ""

    def stop_policy(self) -> None:
        with self._lock:
            frozen_target = self._scaled_action_policy.copy().tolist()
            self._policy_running = False
            self._status_text = "policy stopped; simulation frozen"
        try:
            if self._client is not None and self._connected:
                self._client.set_joint(frozen_target)
        except Exception:
            pass

    def hold(self) -> dict:
        if self._cfg is not None and self._cfg.sim_only:
            with self._lock:
                self._policy_running = False
                self._status_text = "hold (sim-only)"
            return {"ok": True, "msg": "sim-only: no-op"}
        client = self._require_client()
        reply = client.hold()
        with self._lock:
            self._policy_running = False
            self._status_text = "hold"
        return reply

    def disable(self) -> dict:
        if self._cfg is not None and self._cfg.sim_only:
            with self._lock:
                self._policy_running = False
                self._initialized = False
                self._status_text = "disabled (sim-only)"
            return {"ok": True, "msg": "sim-only: no-op"}
        client = self._require_client()
        reply = client.disable()
        with self._lock:
            self._policy_running = False
            self._initialized = False
            self._status_text = "disabled"
        return reply

    def emergency_stop(self, reason: str = "manual E-stop") -> None:
        with self._lock:
            if self._estopped and self._last_error:
                return
            self._policy_running = False
            self._estopped = True
            self._last_error = reason
            self._status_text = "E-stop"
        try:
            if self._client is not None and self._client.connected:
                self._client.disable()
        except Exception as exc:
            with self._lock:
                self._last_error = f"{reason}; disable failed: {exc}"

    def reset_sim(self) -> None:
        with self._lock:
            self._request_reset = True
            self._status_text = "reset requested"

    def shutdown(self) -> None:
        with self._lock:
            self._policy_running = False
            self._running = False
        try:
            if self._client is not None and self._client.connected:
                try:
                    self._client.hold()
                except Exception:
                    pass
                try:
                    self._client.disable()
                except Exception:
                    pass
        finally:
            if self._sim_thread is not None:
                self._sim_thread.join(timeout=2.0)
            if self._client is not None:
                self._client.close()
            with self._lock:
                self._connected = False
                self._initialized = False
                self._status_text = "shutdown"

    def update_command(self, cmd_x: float, cmd_y: float, cmd_yaw: float) -> None:
        with self._lock:
            self._cmd[:] = [float(cmd_x), float(cmd_y), float(cmd_yaw)]

    def update_safety_settings(self, divergence_threshold_rad: float, auto_stop_on_divergence: bool) -> None:
        with self._lock:
            if self._cfg is not None:
                self._cfg.divergence_threshold_rad = float(divergence_threshold_rad)
                self._cfg.auto_stop_on_divergence = bool(auto_stop_on_divergence)

    def get_snapshot(self) -> EngineSnapshot:
        with self._lock:
            real = self._latest_real_state
            sim_only = self._cfg.sim_only if self._cfg else False
            if sim_only:
                real_joint = [0.0] * 12
                motors_online = 12
                feedback_age_ms = 0
                state_stream_age_ms = 0.0
            else:
                real_joint = list(real.joint_positions) if real is not None else [0.0] * 12
                motors_online = real.motors_online if real is not None else 0
                feedback_age_ms = real.feedback_age_ms if real is not None else -1
                state_stream_age_ms = float("inf")
                if self._client is not None:
                    state_stream_age_ms = self._client.latest_state_age_ms()
            sim_joint = list(self._sim_joint_policy)
            diff = [r - s for r, s in zip(real_joint, sim_joint)]
            return EngineSnapshot(
                connected=self._connected,
                initialized=self._initialized,
                policy_running=self._policy_running,
                sim_thread_running=self._running,
                estopped=self._estopped,
                viewer_enabled=bool(self._cfg.viewer_enabled) if self._cfg else False,
                last_error=self._last_error,
                last_warning=self._last_warning,
                status_text=self._status_text,
                motors_online=motors_online,
                feedback_age_ms=feedback_age_ms,
                state_stream_age_ms=state_stream_age_ms,
                sim_fps=self._sim_fps,
                policy_fps=self._policy_fps,
                real_joint_positions=real_joint,
                sim_joint_positions=sim_joint,
                scaled_action=list(self._scaled_action_policy),
                raw_action=list(self._raw_action_policy),
                diff_real_minus_sim=diff,
                cmd=list(self._cmd),
            )

    def _require_client(self) -> TwinAgentClient:
        if self._client is None or not self._connected:
            raise RuntimeError("twin_agent is not connected")
        return self._client

    def _reset_obs_history_locked(self) -> None:
        assert self._cfg is not None
        self._hist_obs = deque(maxlen=self._cfg.frame_stack)
        for _ in range(self._cfg.frame_stack):
            self._hist_obs.append(np.zeros(self._cfg.single_obs_dim, dtype=np.float32))

    def _sim_loop(self) -> None:
        assert self._cfg is not None
        assert self._model is not None and self._data is not None and self._session is not None
        cfg = self._cfg
        model = self._model
        data = self._data
        policy_ticks = 0
        sim_ticks = 0
        lowlevel_count = 0
        fps_t0 = time.perf_counter()

        if cfg.viewer_enabled:
            self._run_with_viewer(model, data, cfg, policy_ticks, sim_ticks, lowlevel_count, fps_t0)
        else:
            self._run_no_viewer(model, data, cfg, policy_ticks, sim_ticks, lowlevel_count, fps_t0)

    def _run_with_viewer(self, model, data, cfg, policy_ticks, sim_ticks, lowlevel_count, fps_t0):
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.distance = 3.0
            viewer.cam.azimuth = 90
            viewer.cam.elevation = -45
            viewer.cam.lookat[:] = np.array([0.0, -0.25, 0.824])
            while self._running and viewer.is_running():
                policy_ticks, sim_ticks, lowlevel_count, fps_t0 = self._run_one_iteration(
                    model, data, cfg, policy_ticks, sim_ticks, lowlevel_count, fps_t0
                )
                viewer.sync()

    def _run_no_viewer(self, model, data, cfg, policy_ticks, sim_ticks, lowlevel_count, fps_t0):
        while self._running:
            policy_ticks, sim_ticks, lowlevel_count, fps_t0 = self._run_one_iteration(
                model, data, cfg, policy_ticks, sim_ticks, lowlevel_count, fps_t0
            )

    def _run_one_iteration(self, model, data, cfg, policy_ticks, sim_ticks, lowlevel_count, fps_t0):
        t_start = time.perf_counter()
        if self._request_reset:
            self._apply_reset(data)

        with self._lock:
            client = self._client
            policy_running = self._policy_running and not self._estopped
            cmd = self._cmd.copy()

        if client is not None:
            real_state = client.latest_state()
            with self._lock:
                self._latest_real_state = real_state

        if policy_running:
            self._step_policy_and_sim(data, cmd, lowlevel_count)
            sim_ticks += 1
            if lowlevel_count % cfg.decimation == 0:
                policy_ticks += 1
        else:
            # Keep simulation frozen and lightweight when policy is paused.
            time.sleep(cfg.sim_dt)

        lowlevel_count += 1
        now = time.perf_counter()
        elapsed = now - fps_t0
        if elapsed >= 1.0:
            with self._lock:
                self._sim_fps = sim_ticks / elapsed
                self._policy_fps = policy_ticks / elapsed
            sim_ticks = 0
            policy_ticks = 0
            fps_t0 = now

        spent = time.perf_counter() - t_start
        remaining = cfg.sim_dt - spent
        if remaining > 0.0:
            time.sleep(remaining)
        return policy_ticks, sim_ticks, lowlevel_count, fps_t0

    def _apply_reset(self, data: mujoco.MjData) -> None:
        assert self._default_qpos is not None and self._default_qvel is not None
        data.qpos[:] = self._default_qpos
        data.qvel[:] = self._default_qvel
        if data.ctrl is not None and data.ctrl.shape[0] >= 12:
            data.ctrl[:12] = 0.0
        mujoco.mj_forward(self._model, data)
        with self._lock:
            self._target_q_sim = self._default_joint_sim.copy()
            self._raw_action_policy[:] = 0.0
            self._scaled_action_policy[:] = 0.0
            self._sim_joint_policy[:] = 0.0
            self._reset_obs_history_locked()
            self._request_reset = False
            self._status_text = "simulation reset"

    def _step_policy_and_sim(self, data: mujoco.MjData, cmd: np.ndarray, lowlevel_count: int) -> None:
        assert self._cfg is not None and self._session is not None and self._input_name is not None
        cfg = self._cfg

        q_sim_abs = np.array(data.qpos[7:19], dtype=np.float64)
        dq_sim = np.array(data.qvel[6:18], dtype=np.float64)

        q_policy_abs = q_sim_abs[SIM_TO_POLICY]
        dq_policy = dq_sim[SIM_TO_POLICY]
        q_policy_rel = q_policy_abs - self._default_joint_policy

        with self._lock:
            self._sim_joint_policy = q_policy_rel.copy()

        if lowlevel_count % cfg.decimation == 0:
            mj_quat = np.array(data.qpos[3:7], dtype=np.float64)
            quat_xyzw = np.array([mj_quat[1], mj_quat[2], mj_quat[3], mj_quat[0]], dtype=np.float64)
            omega = self._extract_angular_velocity(data)
            gravity = self._projected_gravity(quat_xyzw)

            obs_parts = [
                omega,
                gravity,
                cmd,
                q_policy_rel,
                dq_policy,
                self._raw_action_policy,
            ]
            current_obs = np.concatenate(obs_parts).astype(np.float32)
            current_obs = np.clip(current_obs, -cfg.clip_observations, cfg.clip_observations)
            with self._lock:
                self._hist_obs.append(current_obs)
                policy_input = np.concatenate(list(self._hist_obs), axis=0)[None, :]

            raw_action = self._session.run(None, {self._input_name: policy_input})[0][0]
            raw_action = np.asarray(raw_action, dtype=np.float64).reshape(-1)
            if raw_action.shape[0] != 12:
                self.emergency_stop(f"policy output shape mismatch: got {raw_action.shape}")
                return
            raw_action = np.clip(raw_action, -cfg.clip_actions, cfg.clip_actions)
            scaled_action = raw_action * cfg.action_scale

            with self._lock:
                self._raw_action_policy = raw_action.copy()
                self._scaled_action_policy = scaled_action.copy()

            target_q_sim = scaled_action[POLICY_TO_SIM] + self._default_joint_sim
            with self._lock:
                self._target_q_sim = target_q_sim

            if not self._safe_to_send(scaled_action):
                return
            if self._client is not None:
                try:
                    self._client.set_joint(scaled_action.tolist())
                except Exception as exc:
                    self.emergency_stop(f"set_joint failed: {exc}")
                    return

        tau = self._pd_control(self._target_q_sim, q_sim_abs, dq_sim)
        data.ctrl[:12] = tau
        mujoco.mj_step(self._model, data)

    def _pd_control(self, target_q_sim: np.ndarray, q_sim_abs: np.ndarray, dq_sim: np.ndarray) -> np.ndarray:
        assert self._cfg is not None
        kp = np.asarray(self._cfg.kp, dtype=np.float64)
        kd = np.asarray(self._cfg.kd, dtype=np.float64)
        tau_limit = np.asarray(self._cfg.tau_limit, dtype=np.float64)
        tau = kp * (target_q_sim - q_sim_abs) + kd * (0.0 - dq_sim)
        return np.clip(tau, -tau_limit, tau_limit)

    def _extract_angular_velocity(self, data: mujoco.MjData) -> np.ndarray:
        sensor_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SENSOR, "angular-velocity")
        if sensor_id != -1:
            adr = self._model.sensor_adr[sensor_id]
            dim = self._model.sensor_dim[sensor_id]
            if dim >= 3:
                return np.array(data.sensordata[adr:adr + 3], dtype=np.float64)
        return np.array(data.qvel[3:6], dtype=np.float64)

    @staticmethod
    def _projected_gravity(quat_xyzw: np.ndarray) -> np.ndarray:
        r = R.from_quat(quat_xyzw)
        gravity = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        return r.apply(gravity, inverse=True)

    def _safe_to_send(self, scaled_action: np.ndarray) -> bool:
        assert self._cfg is not None
        if self._cfg.sim_only:
            return True
        if self._client is None:
            self.emergency_stop("missing twin_agent client")
            return False
        state = self._client.latest_state()
        stream_age_ms = self._client.latest_state_age_ms()
        if state is None:
            self.emergency_stop("no real-robot state available")
            return False
        if state.motors_online != 12:
            self.emergency_stop(f"motors_online={state.motors_online}, expected 12")
            return False
        if state.feedback_age_ms < 0 or state.feedback_age_ms >= self._cfg.max_feedback_age_ms:
            self.emergency_stop(f"feedback_age_ms={state.feedback_age_ms} exceeds limit")
            return False
        if stream_age_ms >= self._cfg.max_state_stream_age_ms:
            self.emergency_stop(f"state stream stale: {stream_age_ms:.1f} ms")
            return False

        sim_joint = self._sim_joint_policy.copy()
        real_joint = np.array(state.joint_positions, dtype=np.float64)
        max_div = float(np.max(np.abs(real_joint - sim_joint)))
        if max_div >= self._cfg.divergence_threshold_rad:
            msg = f"joint divergence {max_div:.3f} rad exceeds threshold {self._cfg.divergence_threshold_rad:.3f}"
            with self._lock:
                self._last_warning = msg
            if self._cfg.auto_stop_on_divergence:
                with self._lock:
                    self._policy_running = False
                    self._status_text = "auto-stopped on divergence"
                return False
        else:
            with self._lock:
                self._last_warning = ""
        return True
