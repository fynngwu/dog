"""Small orchestration layer for frontend-facing workflows."""

from __future__ import annotations

from typing import Callable, Dict, Iterable, Optional

from replay_controller import ReplayController
from robot_backend import RobotBackend


class ExperimentManager:
    def __init__(self, host: str, cmd_port: int = 47001, state_port: int = 47002, timeout_s: float = 3.0):
        self.backend = RobotBackend(host=host, cmd_port=cmd_port, state_port=state_port, timeout_s=timeout_s)
        self.replay = ReplayController(self.backend)

    def connect(self) -> bool:
        return self.backend.connect()

    def disconnect(self) -> None:
        self.backend.disconnect()

    def start_state_stream(self, callback: Optional[Callable[[Dict], None]] = None) -> None:
        self.backend.start_state_stream(callback)

    def stop_state_stream(self) -> None:
        self.backend.stop_state_stream()

    def init_robot(self, duration_s: float = 2.5) -> Dict:
        return self.backend.init(duration_s)

    def enable(self) -> Dict:
        return self.backend.enable()

    def disable(self) -> Dict:
        return self.backend.disable()

    def get_state(self) -> Dict:
        return self.backend.get_state()

    def set_mit_param(self, kp: float, kd: float, vel_limit: float, torque_limit: float) -> Dict:
        return self.backend.set_mit_param(kp, kd, vel_limit, torque_limit)

    def joint_test(self, joint_indices: Iterable[int], target_rad: float) -> Dict:
        return self.backend.joint_test(joint_indices, target_rad)

    def joint_sine(self, joint_indices: Iterable[int], amp_rad: float, freq_hz: float, duration_s: float) -> Dict:
        return self.backend.joint_sine(joint_indices, amp_rad, freq_hz, duration_s)
