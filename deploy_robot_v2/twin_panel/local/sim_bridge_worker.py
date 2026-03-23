from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from PyQt5 import QtCore

try:
    import mujoco
    import mujoco.viewer
    import numpy as np
except ImportError:
    mujoco = None
    np = None

from twin_topics import NUM_JOINTS


@dataclass
class SimConfig:
    xml_path: str
    kp: float = 30.0
    kd: float = 1.0
    inner_steps: int = 2
    action_scale: float = 0.25
    tau_limit: float = 17.0
    sim_default_dof_pos_policy: tuple = (0.0,) * NUM_JOINTS
    # policy 顺序 -> sim 顺序映射
    policy2sim_indices: tuple = (0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11)
    # sim 顺序 -> policy 顺序映射
    sim2policy_indices: tuple = (0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11)


class SimBridgeWorker(QtCore.QThread):
    status = QtCore.pyqtSignal(str)
    state_ready = QtCore.pyqtSignal(dict)

    def __init__(self, config: SimConfig, parent=None) -> None:
        super().__init__(parent)
        self._config = config
        self._lock = threading.Lock()
        self._latest_action = [0.0] * NUM_JOINTS
        self._action_dirty = True
        self._running = True
        self._viewer = None
        self._model = None
        self._data = None

    def stop(self) -> None:
        self._running = False

    def set_action(self, values: list) -> None:
        with self._lock:
            if len(values) >= NUM_JOINTS:
                self._latest_action = list(values[:NUM_JOINTS])
                self._action_dirty = True

    def run(self) -> None:
        if mujoco is None or np is None:
            self.status.emit('MuJoCo not available')
            return

        try:
            self._model = mujoco.MjModel.from_xml_path(self._config.xml_path)
            self._data = mujoco.MjData(self._model)
        except Exception as exc:
            self.status.emit(f'Failed to load MuJoCo: {exc}')
            return

        # 模型验证
        if self._model.nu < NUM_JOINTS:
            self.status.emit(f'ERROR: actuators={self._model.nu}, need >= {NUM_JOINTS}')
            return

        # 检测模型类型并验证
        if self._model.nq == NUM_JOINTS:
            # 固定基座模型
            is_floating = False
            qpos_offset = 0
            qvel_offset = 0
            self.status.emit(f'Fixed base model: nq={self._model.nq}, nv={self._model.nv}')
        elif self._model.nq == NUM_JOINTS + 7 and self._model.nv == NUM_JOINTS + 6:
            # 浮动基座模型 (7 pos quat + 12 joints, 6 vel + 12 joints)
            is_floating = True
            qpos_offset = 7
            qvel_offset = 6
            self.status.emit(f'Floating base model: nq={self._model.nq}, nv={self._model.nv}')
        else:
            self.status.emit(f'ERROR: Unexpected model dimensions: nq={self._model.nq}, nv={self._model.nv}')
            self.status.emit(f'  Expected: nq=12 (fixed) or nq=19,nv=18 (floating)')
            return

        # 打开 viewer（独立窗口）
        try:
            self._viewer = mujoco.viewer.launch_passive(self._model, self._data)
            self.status.emit('MuJoCo viewer opened (separate window)')
        except Exception as exc:
            self.status.emit(f'Viewer failed: {exc} (running headless)')

        q0_policy = np.array(self._config.sim_default_dof_pos_policy, dtype=float)
        p2s = np.array(self._config.policy2sim_indices, dtype=int)
        s2p = np.array(self._config.sim2policy_indices, dtype=int)
        self.status.emit('Sim ready')

        while self._running:
            with self._lock:
                raw_action = list(self._latest_action)
                dirty = self._action_dirty
                self._action_dirty = False

            if dirty:
                raw = np.array(raw_action, dtype=float)
                target_joint_rel = raw * self._config.action_scale
                target_q_policy = target_joint_rel + q0_policy
                target_q_sim = target_q_policy[p2s]

                # 读取当前关节状态
                q_sim = np.array(self._data.qpos[qpos_offset:qpos_offset + NUM_JOINTS])
                dq_sim = np.array(self._data.qvel[qvel_offset:qvel_offset + NUM_JOINTS])

                # PD 控制
                tau = self._config.kp * (target_q_sim - q_sim) - self._config.kd * dq_sim
                tau = np.clip(tau, -self._config.tau_limit, self._config.tau_limit)

                self._data.ctrl[:NUM_JOINTS] = tau
                for _ in range(self._config.inner_steps):
                    mujoco.mj_step(self._model, self._data)

                # 同步 viewer
                if self._viewer:
                    self._viewer.sync()

                # 发布状态
                q_policy = np.array(self._data.qpos[qpos_offset:qpos_offset + NUM_JOINTS])[s2p]
                sim_joint_rel = (q_policy - q0_policy).tolist()
                payload = {
                    'ok': True,
                    'mode': 'sim',
                    'raw_action': raw.tolist(),
                    'target_joint_rel': target_joint_rel.tolist(),
                    'sim_joint_rel': sim_joint_rel,
                }
                self.state_ready.emit(payload)

            time.sleep(0.01)

        # 清理
        if self._viewer:
            self._viewer.close()