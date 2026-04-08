"""Knee joint limit processor for CSV trajectory files.

Clamps right leg knee actions (RF_Knee, RR_Knee) within their hardware limits.
"""

from __future__ import annotations

import csv
from io import StringIO
from typing import Tuple

# Joint indices in policy order
RF_KNEE_IDX = 10  # Right Front Knee
RR_KNEE_IDX = 11  # Right Rear Knee

# Right leg knee limits from motor_config
RIGHT_KNEE_MIN = -0.36
RIGHT_KNEE_MAX = 1.22


def clamp_right_knee(value: float) -> float:
    """Clamp right knee action value to its limits."""
    return max(RIGHT_KNEE_MIN, min(RIGHT_KNEE_MAX, value))


def clamp_right_knee_csv(csv_text: str) -> Tuple[str, int]:
    """Process CSV text, clamping right knee actions.

    Args:
        csv_text: CSV content as string

    Returns:
        Tuple of (processed_csv_text, clamped_count)
    """
    clamped_count = 0
    output_lines = []

    reader = csv.DictReader(StringIO(csv_text))
    fieldnames = reader.fieldnames or []
    output_lines.append(",".join(fieldnames))

    for row in reader:
        for idx in [RF_KNEE_IDX, RR_KNEE_IDX]:
            # Clamp scaled_action columns
            col_name = f"scaled_action_{idx}"
            if col_name in row:
                original = float(row[col_name])
                clamped = clamp_right_knee(original)
                if abs(clamped - original) > 1e-6:
                    clamped_count += 1
                row[col_name] = f"{clamped:.6f}"

            # Clamp target_q columns
            target_col = f"target_q_{idx}"
            if target_col in row:
                original = float(row[target_col])
                clamped = clamp_right_knee(original)
                row[target_col] = f"{clamped:.6f}"

        output_lines.append(",".join(row.get(f, "") for f in fieldnames))

    return "\n".join(output_lines), clamped_count