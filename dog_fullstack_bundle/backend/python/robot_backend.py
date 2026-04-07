"""Thin backend facade intended for GUI / service layers."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, Dict, Iterable, Optional

from robot_client import RobotClient


class RobotBackend:
    def __init__(self, host: str, cmd_port: int = 47001, state_port: int = 47002, timeout_s: float = 3.0):
        self.client = RobotClient(host=host, cmd_port=cmd_port, state_port=state_port, timeout_s=timeout_s)

    def connect(self) -> None:
        self.client.connect()

    def disconnect(self) -> None:
        self.client.close()

    def start_state_stream(self, callback: Optional[Callable[[Dict[str, Any]], None]] = None) -> None:
        self.client.start_state_listener(callback)

    def stop_state_stream(self) -> None:
        self.client.stop_state_listener()

    def init(self, duration_s: float = 2.5) -> Dict[str, Any]:
        return self.client.init(duration_s)

    def enable(self) -> Dict[str, Any]:
        return self.client.enable()

    def disable(self) -> Dict[str, Any]:
        return self.client.disable()

    def get_state(self) -> Dict[str, Any]:
        return self.client.get_state_once()

    def set_joint_targets(self, targets_rad: Iterable[float]) -> Dict[str, Any]:
        return self.client.set_joint(list(targets_rad))

    def joint_test(self, joint_indices: Iterable[int], target_rad: float) -> Dict[str, Any]:
        return self.client.joint_test(list(joint_indices), target_rad)

    def joint_sine(self, joint_indices: Iterable[int], amp_rad: float, freq_hz: float, duration_s: float) -> Dict[str, Any]:
        return self.client.joint_sine(list(joint_indices), amp_rad, freq_hz, duration_s)

    def set_mit_param(self, kp: float, kd: float, vel_limit: float, torque_limit: float) -> Dict[str, Any]:
        return self.client.set_mit_param(kp, kd, vel_limit, torque_limit)

    def load_replay_csv(self, csv_path: str | Path) -> Dict[str, Any]:
        return self.client.load_replay_csv(str(csv_path))

    def replay_start(self) -> Dict[str, Any]:
        return self.client.replay_start(1.0)

    def replay_stop(self) -> Dict[str, Any]:
        return self.client.replay_stop()

    def replay_step(self) -> Dict[str, Any]:
        return self.client.replay_step()

    def replay_prev(self) -> Dict[str, Any]:
        return self.client.replay_prev()

    def replay_seek(self, frame_idx: int) -> Dict[str, Any]:
        return self.client.replay_seek(frame_idx)

    def replay_status(self) -> Dict[str, Any]:
        return self.client.replay_status()
