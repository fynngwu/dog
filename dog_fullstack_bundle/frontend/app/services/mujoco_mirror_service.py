"""MuJoCo mirror service for real-time joint control visualization.

This service manages a local MuJoCo simulation that mirrors commands sent to the
real robot, providing visual feedback without hardware dependency.
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import List, Optional

import mujoco
import mujoco.viewer
import numpy as np
from PySide6.QtCore import QObject, Signal

# Joint mapping: policy order -> sim order
# Policy: LF_HipA, LR_HipA, RF_HipA, RR_HipA, LF_HipF, LR_HipF, RF_HipF, RR_HipF, LF_Knee, LR_Knee, RF_Knee, RR_Knee
# Sim: LF_HipA, LF_HipF, LF_Knee, LR_HipA, LR_HipF, LR_Knee, RF_HipA, RF_HipF, RF_Knee, RR_HipA, RR_HipF, RR_Knee
POLICY_TO_SIM = np.array([0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11], dtype=np.int64)
SIM_TO_POLICY = np.array([0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11], dtype=np.int64)

# PD control gains
KP = 25.0
KD = 0.5
TAU_LIMIT = np.array([17.0, 17.0, 25.0] * 4, dtype=np.float64)


class MujocoMirrorService(QObject):
    """Qt-integrated MuJoCo simulation service.

    Provides a mirrored view of joint commands for debugging and visualization.
    Runs in a background thread with ~200Hz simulation rate.
    """

    log_msg = Signal(str)
    viewer_running = Signal(bool)
    state_changed = Signal(dict)

    def __init__(self, bundle_root: Optional[Path] = None):
        super().__init__()
        self.bundle_root = bundle_root or Path(__file__).parent.parent.parent.parent
        self.model: Optional[mujoco.MjModel] = None
        self.data: Optional[mujoco.MjData] = None
        self.default_joint_sim: np.ndarray = np.zeros(12, dtype=np.float64)
        self.target_q_sim: np.ndarray = np.zeros(12, dtype=np.float64)

        self._viewer_thread: Optional[threading.Thread] = None
        self._sim_thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()
        self._viewer_active = False
        self._sim_running = False
        self._lock = threading.Lock()

    def _resolve_xml_path(self, xml_path: str) -> Path:
        """Resolve XML path relative to bundle root if not absolute."""
        p = Path(xml_path)
        if p.is_absolute():
            return p
        return self.bundle_root / xml_path

    def load_model(self, xml_path: str) -> bool:
        """Load MuJoCo model from XML file."""
        try:
            resolved = self._resolve_xml_path(xml_path)
            self.model = mujoco.MjModel.from_xml_path(str(resolved))
            self.model.opt.timestep = 0.002  # 500Hz physics
            self.data = mujoco.MjData(self.model)
            self.default_joint_sim = np.array(self.model.qpos0[7:19], dtype=np.float64)
            self.target_q_sim = self.default_joint_sim.copy()
            # Reset to initial state
            self.data.qpos[:] = self.model.qpos0
            self.data.qvel[:] = 0.0
            mujoco.mj_step(self.model, self.data)
            self.log_msg.emit(f"[MuJoCo] Model loaded from {resolved}")
            return True
        except Exception as e:
            self.log_msg.emit(f"[MuJoCo] Failed to load model: {e}")
            return False

    def start_viewer(self) -> bool:
        """Start MuJoCo passive viewer in background thread."""
        if self._viewer_active:
            return True
        if self.model is None:
            self.log_msg.emit("[MuJoCo] Load model first")
            return False

        self._stop_flag.clear()
        self._viewer_active = True
        self._sim_running = True

        # Start simulation thread
        self._sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._sim_thread.start()

        # Start viewer thread
        self._viewer_thread = threading.Thread(target=self._viewer_loop, daemon=True)
        self._viewer_thread.start()

        self.viewer_running.emit(True)
        self.log_msg.emit("[MuJoCo] Viewer started")
        return True

    def stop_viewer(self) -> None:
        """Stop viewer and simulation threads."""
        self._stop_flag.set()
        self._viewer_active = False
        self._sim_running = False

        if self._viewer_thread and self._viewer_thread.is_alive():
            self._viewer_thread.join(timeout=2.0)
        if self._sim_thread and self._sim_thread.is_alive():
            self._sim_thread.join(timeout=2.0)

        self._viewer_thread = None
        self._sim_thread = None
        self.viewer_running.emit(False)
        self.log_msg.emit("[MuJoCo] Viewer stopped")

    def _sim_loop(self) -> None:
        """Background simulation loop at ~500Hz."""
        if self.model is None or self.data is None:
            return

        dt = self.model.opt.timestep
        while not self._stop_flag.is_set() and self._sim_running:
            with self._lock:
                # PD control
                q_sim = np.array(self.data.qpos[7:19], dtype=np.float64)
                dq_sim = np.array(self.data.qvel[6:18], dtype=np.float64)
                tau = (self.target_q_sim - q_sim) * KP - dq_sim * KD
                tau = np.clip(tau, -TAU_LIMIT, TAU_LIMIT)
                self.data.ctrl[:] = tau
                mujoco.mj_step(self.model, self.data)

                # Emit state at ~50Hz
                joint_pos_policy = q_sim[SIM_TO_POLICY] - self.default_joint_sim[SIM_TO_POLICY]
                self.state_changed.emit({
                    "joint_positions": joint_pos_policy.tolist(),
                    "joint_velocities": dq_sim[SIM_TO_POLICY].tolist(),
                })
            time.sleep(dt)

    def _viewer_loop(self) -> None:
        """Passive viewer loop."""
        if self.model is None or self.data is None:
            return

        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                viewer.cam.distance = 3.0
                viewer.cam.azimuth = 90.0
                viewer.cam.elevation = -45.0

                while viewer.is_running() and not self._stop_flag.is_set():
                    with self._lock:
                        viewer.sync()
                    time.sleep(0.01)  # ~100Hz render
        except Exception as e:
            self.log_msg.emit(f"[MuJoCo] Viewer error: {e}")
        finally:
            self._viewer_active = False
            self.viewer_running.emit(False)

    def set_joint_targets(self, targets: List[float]) -> None:
        """Set joint targets from policy-order list (relative to offset)."""
        if self.model is None:
            return
        targets_arr = np.array(targets, dtype=np.float64)
        if len(targets_arr) != 12:
            return
        with self._lock:
            # targets are relative to offset, convert to sim absolute
            self.target_q_sim = targets_arr[POLICY_TO_SIM] + self.default_joint_sim

    def joint_test(self, indices: List[int], target: float) -> None:
        """Apply target to specific joints (policy indices, relative to offset)."""
        if self.model is None:
            return
        with self._lock:
            current_policy = (self.target_q_sim - self.default_joint_sim)[SIM_TO_POLICY]
            for idx in indices:
                if 0 <= idx < 12:
                    current_policy[idx] = target
            self.target_q_sim = current_policy[POLICY_TO_SIM] + self.default_joint_sim

    def joint_sine(self, indices: List[int], amp: float, freq: float, duration: float) -> None:
        """Run sinusoidal motion on specific joints in background."""
        if self.model is None:
            return

        def sine_worker():
            start = time.time()
            base_targets = (self.target_q_sim - self.default_joint_sim).copy()
            while time.time() - start < duration and self._sim_running:
                t = time.time() - start
                value = amp * math.sin(2 * math.pi * freq * t)
                with self._lock:
                    current = base_targets.copy()
                    for idx in indices:
                        if 0 <= idx < 12:
                            current[idx] = value
                    self.target_q_sim = current[POLICY_TO_SIM] + self.default_joint_sim
                time.sleep(0.01)
            self.log_msg.emit("[MuJoCo] Sine motion completed")

        threading.Thread(target=sine_worker, daemon=True).start()
        self.log_msg.emit("[MuJoCo] Sine motion started")

    @property
    def is_running(self) -> bool:
        return self._viewer_active