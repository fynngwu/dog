"""Plot sim_record CSV files for joint debugging and limit inspection."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np

from motor_config import JOINT_LIMITS, JOINT_NAMES, clamp_relative_target

LEG_ORDER = ["LF", "LR", "RF", "RR"]
JOINT_TYPE_ORDER = ["HipA", "HipF", "Knee"]
PHASE_COLOR_FALLBACK = {
    "Stand": "gray",
    "Forward": "green",
    "Backward": "red",
    "Left": "blue",
    "Right": "cyan",
    "TurnL": "orange",
    "TurnR": "purple",
}


def read_csv(path: Path) -> Dict[str, np.ndarray]:
    """Load a sim_record CSV into numpy arrays keyed by column name.

    Args:
        path: CSV file path.

    Returns:
        Dictionary mapping column name to a numpy array.  String columns are
        stored as object arrays.
    """
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise ValueError(f"CSV is empty: {path}")
    columns: Dict[str, List[object]] = {name: [] for name in reader.fieldnames or []}
    for row in rows:
        for key, value in row.items():
            if key == "phase_name":
                columns[key].append(value)
            else:
                columns[key].append(float(value))
    out: Dict[str, np.ndarray] = {}
    for key, values in columns.items():
        if key == "phase_name":
            out[key] = np.asarray(values, dtype=object)
        else:
            out[key] = np.asarray(values, dtype=float)
    return out


def compute_phase_segments(data: Dict[str, np.ndarray]) -> List[Tuple[float, float, str]]:
    """Extract contiguous phase ranges from one CSV dataset.

    Args:
        data: Loaded CSV dictionary.

    Returns:
        List of ``(start_time, end_time, phase_name)`` tuples.
    """
    t = data["sim_time"]
    phase = data["phase"]
    phase_name = data.get("phase_name")
    segments: List[Tuple[float, float, str]] = []
    start = 0
    for idx in range(1, len(t)):
        if phase[idx] != phase[idx - 1]:
            name = str(phase_name[start]) if phase_name is not None else f"phase_{int(phase[start])}"
            segments.append((float(t[start]), float(t[idx]), name))
            start = idx
    name = str(phase_name[start]) if phase_name is not None else f"phase_{int(phase[start])}"
    segments.append((float(t[start]), float(t[-1]), name))
    return segments


def add_phase_background(ax: plt.Axes, segments: Sequence[Tuple[float, float, str]]) -> None:
    """Draw phase-colored background spans on one axis.

    Args:
        ax: Target matplotlib axis.
        segments: Contiguous phase segments.
    """
    for start, end, name in segments:
        ax.axvspan(start, end, alpha=0.10, color=PHASE_COLOR_FALLBACK.get(name, "lightgray"))


def joint_subplot_index(joint_name: str) -> Tuple[int, int]:
    """Map a joint name like ``LF_HipA`` to a ``(row, column)`` pair.

    Args:
        joint_name: Policy-order joint name.

    Returns:
        Tuple ``(row, column)`` for a 3x4 layout.
    """
    leg, joint_type = joint_name.split("_")
    return JOINT_TYPE_ORDER.index(joint_type), LEG_ORDER.index(leg)


def summarize_bound_hits(data: Dict[str, np.ndarray], threshold_ratio: float = 0.05) -> List[str]:
    """Create one textual summary per joint about limit proximity/violations.

    Args:
        data: Loaded CSV dictionary.
        threshold_ratio: Relative margin used for the near-limit check.

    Returns:
        Human-readable summary lines.
    """
    lines: List[str] = []
    for index, name in enumerate(JOINT_NAMES):
        pos = data[f"joint_pos_{index}"]
        low, high = JOINT_LIMITS[index]
        margin = max((high - low) * threshold_ratio, 1e-6)
        near = int(np.count_nonzero((pos - low <= margin) | (high - pos <= margin)))
        vio = int(np.count_nonzero((pos < low) | (pos > high)))
        lines.append(f"{name:8s}: near_limit={near:4d}, violations={vio:4d}, min={pos.min():+.3f}, max={pos.max():+.3f}")
    return lines


def plot_dataset(
    datasets: Sequence[Tuple[Path, Dict[str, np.ndarray]]],
    highlight_bounds: bool,
    output: Optional[Path],
) -> None:
    """Create the 3x4 joint plot for one or more CSV files.

    Args:
        datasets: Sequence of ``(path, data)`` pairs.
        highlight_bounds: Whether to shade violations and mark near-limit points.
        output: Optional output image path.
    """
    fig, axes = plt.subplots(3, 4, figsize=(20, 11), sharex=True)
    axes = np.asarray(axes)

    if datasets:
        segments = compute_phase_segments(datasets[0][1])
        for ax in axes.flat:
            add_phase_background(ax, segments)

    for dataset_idx, (path, data) in enumerate(datasets):
        label_prefix = path.stem if len(datasets) > 1 else ""
        t = data["sim_time"]
        for joint_idx, joint_name in enumerate(JOINT_NAMES):
            row, col = joint_subplot_index(joint_name)
            ax = axes[row, col]
            pos = data[f"joint_pos_{joint_idx}"]
            target = data.get(f"target_q_{joint_idx}", data.get(f"scaled_action_{joint_idx}"))
            low, high = JOINT_LIMITS[joint_idx]
            line_label = f"{label_prefix} position".strip()
            target_label = f"{label_prefix} target".strip()
            ax.plot(t, pos, label=line_label, linewidth=1.4)
            ax.plot(t, target, linestyle="--", label=target_label, linewidth=1.2)
            ax.axhline(low, linestyle=":", linewidth=1.0)
            ax.axhline(high, linestyle=":", linewidth=1.0)
            if highlight_bounds:
                outside = (pos < low) | (pos > high)
                near = ((pos - low) <= (high - low) * 0.05) | ((high - pos) <= (high - low) * 0.05)
                if np.any(outside):
                    ax.fill_between(t, low, high, where=outside, alpha=0.15)
                if np.any(near):
                    ax.scatter(t[near], pos[near], s=10)
            ax.set_title(joint_name)
            ax.grid(True, alpha=0.25)
            if col == 0:
                ax.set_ylabel("rad")
            if row == 2:
                ax.set_xlabel("sim_time (s)")

    handles, labels = axes[0, 0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="upper center", ncol=max(2, len(handles)))
    fig.suptitle("Joint tracking overview (3 rows × 4 columns)", y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    if output is not None:
        fig.savefig(output, dpi=160)
    else:
        plt.show()


def build_argparser() -> argparse.ArgumentParser:
    """Create the CLI argument parser."""
    ap = argparse.ArgumentParser(description="Plot sim_record CSV files")
    ap.add_argument("csv", nargs="+", help="One or more CSV files")
    ap.add_argument("--overlay", action="store_true", help="Overlay multiple CSVs on the same axes")
    ap.add_argument("--highlight-bounds", action="store_true", help="Shade violations and mark near-limit points")
    ap.add_argument("--output", type=Path, help="Optional output image path")
    return ap


def main(argv: Optional[Iterable[str]] = None) -> int:
    """Program entry point.

    Args:
        argv: Optional custom argv sequence.

    Returns:
        Exit code.
    """
    args = build_argparser().parse_args(list(argv) if argv is not None else None)
    paths = [Path(p) for p in args.csv]
    datasets = [(path, read_csv(path)) for path in paths]

    if args.highlight_bounds:
        for path, data in datasets:
            print(f"== {path} ==")
            for line in summarize_bound_hits(data):
                print(line)

    if len(datasets) > 1 and not args.overlay:
        raise SystemExit("Multiple CSV files require --overlay")

    plot_dataset(datasets, args.highlight_bounds, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
