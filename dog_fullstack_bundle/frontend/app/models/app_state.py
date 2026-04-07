from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List


@dataclass
class RobotState:
    mode: str = "offline"
    seq: int = 0
    joint_positions: List[float] = field(default_factory=lambda: [0.0] * 12)
    joint_velocities: List[float] = field(default_factory=lambda: [0.0] * 12)
    joint_torques: List[float] = field(default_factory=lambda: [0.0] * 12)
    target_positions: List[float] = field(default_factory=lambda: [0.0] * 12)
    offline_motors: List[str] = field(default_factory=list)
    motion_active: bool = False
    motion_name: str = ""
    motion_last_error: str = ""
    replay_loaded: bool = False
    replay_status: str = "idle"
    replay_cursor: int = 0
    replay_total: int = 0
    fault_active: bool = False
    fault_code: str = ""
    fault_msg: str = ""
    fault_joint_name: str = ""
    fault_motor_index: int = -1

    def update_from_json(self, data: Dict[str, Any]) -> None:
        self.mode = data.get("mode", self.mode)
        self.seq = int(data.get("seq", self.seq))

        state_data = data.get("state", {})
        self.joint_positions = list(state_data.get("joint_positions", self.joint_positions))
        self.joint_velocities = list(state_data.get("joint_velocities", self.joint_velocities))
        self.joint_torques = list(state_data.get("joint_torques", self.joint_torques))
        self.target_positions = list(state_data.get("target_joint_positions", self.target_positions))
        self.offline_motors = [str(v) for v in state_data.get("offline_motors", self.offline_motors)]

        motion_data = data.get("motion", {})
        self.motion_active = bool(motion_data.get("active", self.motion_active))
        self.motion_name = str(motion_data.get("name", self.motion_name))
        self.motion_last_error = str(motion_data.get("last_error", self.motion_last_error))

        replay_data = data.get("replay", {})
        self.replay_loaded = bool(replay_data.get("loaded", self.replay_loaded))
        self.replay_status = str(replay_data.get("status", self.replay_status))
        self.replay_cursor = int(replay_data.get("cursor", self.replay_cursor))
        self.replay_total = int(replay_data.get("total_frames", self.replay_total))

        fault_data = data.get("fault") or {}
        self.fault_active = bool(fault_data)
        self.fault_code = str(fault_data.get("code", ""))
        self.fault_msg = str(fault_data.get("message", ""))
        self.fault_joint_name = str(fault_data.get("joint_name", ""))
        self.fault_motor_index = int(fault_data.get("motor_index", -1))
