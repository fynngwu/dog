"""Shared robot and simulation constants.

This module consolidates the joint ordering, limits, offsets, and policy/sim
index remaps extracted from the provided reference C++ and Python code.  The
joint order is the *policy order* used by the daemon TCP API and by sim_record
CSV files.
"""

from __future__ import annotations

from typing import Dict, List, Tuple

MOTOR_IDS: List[int] = [1, 5, 9, 13, 2, 6, 10, 14, 3, 7, 11, 15]
JOINT_NAMES: List[str] = [
    "LF_HipA",
    "LR_HipA",
    "RF_HipA",
    "RR_HipA",
    "LF_HipF",
    "LR_HipF",
    "RF_HipF",
    "RR_HipF",
    "LF_Knee",
    "LR_Knee",
    "RF_Knee",
    "RR_Knee",
]

KNEE_RATIO: float = 1.667
ACTION_SCALE: float = 0.25
MIT_KP: float = 40.0
MIT_KD: float = 0.5
MIT_VEL_LIMIT: float = 44.0
MIT_TORQUE_LIMIT: float = 17.0

SIM_TO_POLICY: List[int] = [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]
POLICY_TO_SIM: List[int] = [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]
DEFAULT_JOINT_SIM: List[float] = [0.0] * 12

_HIP_A_OFFSET = 0.37
_HIP_F_OFFSET = 0.13
_KNEE_OFFSET = 1.06 * KNEE_RATIO

JOINT_OFFSETS: List[float] = [
    _HIP_A_OFFSET,
    -_HIP_A_OFFSET,
    -_HIP_A_OFFSET,
    _HIP_A_OFFSET,
    _HIP_F_OFFSET,
    _HIP_F_OFFSET,
    -_HIP_F_OFFSET,
    -_HIP_F_OFFSET,
    _KNEE_OFFSET,
    _KNEE_OFFSET,
    -_KNEE_OFFSET,
    -_KNEE_OFFSET,
]

JOINT_DIRECTIONS: List[float] = [-1.0] * 8 + [1.0] * 4

_XML_MIN: List[float] = [
    -0.7853982,
    -0.7853982,
    -0.7853982,
    -0.7853982,
    -1.2217658,
    -1.2217305,
    -0.8726999,
    -0.8726999,
    -1.2217299 * KNEE_RATIO,
    -1.2217299 * KNEE_RATIO,
    -0.6,
    -0.6,
]

_XML_MAX: List[float] = [
    0.7853982,
    0.7853982,
    0.7853982,
    0.7853982,
    0.8726683,
    0.8726683,
    1.2217342,
    1.2217305,
    0.6,
    0.6,
    1.2217287 * KNEE_RATIO,
    1.2217287 * KNEE_RATIO,
]

JOINT_LIMITS: List[Tuple[float, float]] = []
for idx, (low, high) in enumerate(zip(_XML_MIN, _XML_MAX)):
    if idx >= 8:
        JOINT_LIMITS.append((low / KNEE_RATIO, high / KNEE_RATIO))
    else:
        JOINT_LIMITS.append((low, high))

JOINT_LIMITS_BY_NAME: Dict[str, Tuple[float, float]] = dict(zip(JOINT_NAMES, JOINT_LIMITS))
JOINT_INDEX_BY_NAME: Dict[str, int] = {name: i for i, name in enumerate(JOINT_NAMES)}


def clamp_relative_target(index: int, value: float) -> float:
    """Clamp one policy-order joint target to the configured relative limits.

    Args:
        index: Joint index in policy order.
        value: Relative target in radians, measured around the standing offset.

    Returns:
        The clamped relative target.
    """
    low, high = JOINT_LIMITS[index]
    return max(low, min(high, float(value)))
