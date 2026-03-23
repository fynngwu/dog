from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import List


@dataclass
class TwinState:
    ok: bool = False
    mode: str = ''
    seq: int = 0
    error: str = ''
    sample_fresh: bool = False
    max_feedback_age_ms: int = -1
    motors_online_count: int = 0
    raw_action: List[float] = field(default_factory=list)
    target_joint_rel: List[float] = field(default_factory=list)
    hw_joint_rel: List[float] = field(default_factory=list)
    sim_joint_rel: List[float] = field(default_factory=list)
    motor_abs: List[float] = field(default_factory=list)
    motor_vel: List[float] = field(default_factory=list)
    motor_tau: List[float] = field(default_factory=list)
    rtt_ms: int = 0

    @classmethod
    def from_json(cls, text: str) -> 'TwinState':
        data = json.loads(text)
        return cls(**{k: v for k, v in data.items() if k in cls.__dataclass_fields__})