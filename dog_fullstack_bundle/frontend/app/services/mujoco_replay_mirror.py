"""CSV replay driver for MuJoCo mirror.

Handles loading and playback of recorded CSV trajectories in the MuJoCo mirror.
"""

from __future__ import annotations

import csv
import threading
import time
from io import StringIO
from typing import List, Optional

import numpy as np
from PySide6.QtCore import QObject, Signal

from app.services.mujoco_mirror_service import MujocoMirrorService, POLICY_TO_SIM


class ReplaySample:
    """Single replay sample with time and joint targets."""

    def __init__(self, time_sec: float, targets: List[float]):
        self.time_sec = time_sec
        self.targets = targets  # Policy order, relative to offset


class MujocoReplayMirror(QObject):
    """CSV replay controller for MuJoCo mirror.

    Parses CSV text and drives the mirror through the recorded sequence.
    """

    cursor_changed = Signal(int)
    log_msg = Signal(str)

    def __init__(self, mirror_service: MujocoMirrorService):
        super().__init__()
        self.mirror = mirror_service
        self.samples: List[ReplaySample] = []
        self.cursor: int = 0
        self._playing = False
        self._play_thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()
        self._speed_factor = 1.0

    def load_csv(self, csv_text: str) -> bool:
        """Load CSV text into sample list."""
        self.samples = []
        self.cursor = 0

        try:
            reader = csv.DictReader(StringIO(csv_text))
            for row in reader:
                # Determine time
                time_sec = 0.0
                if "timestamp_ms" in row:
                    time_sec = float(row["timestamp_ms"]) / 1000.0
                elif "sim_time" in row:
                    time_sec = float(row["sim_time"])

                # Extract scaled_action_* columns
                targets = []
                for i in range(12):
                    key = f"scaled_action_{i}"
                    if key not in row:
                        self.log_msg.emit(f"[Replay] Missing column: {key}")
                        return False
                    targets.append(float(row[key]))

                self.samples.append(ReplaySample(time_sec, targets))

            if not self.samples:
                self.log_msg.emit("[Replay] CSV contains no samples")
                return False

            self.log_msg.emit(f"[Replay] Loaded {len(self.samples)} samples")
            self.cursor_changed.emit(0)
            return True

        except Exception as e:
            self.log_msg.emit(f"[Replay] CSV parse error: {e}")
            return False

    @property
    def total_frames(self) -> int:
        return len(self.samples)

    @property
    def is_loaded(self) -> bool:
        return len(self.samples) > 0

    @property
    def is_playing(self) -> bool:
        return self._playing

    def start(self, speed_factor: float = 1.0) -> bool:
        """Start playback from current cursor position."""
        if not self.is_loaded:
            self.log_msg.emit("[Replay] No CSV loaded")
            return False
        if self._playing:
            return True

        self._speed_factor = max(0.1, speed_factor)
        self._stop_flag.clear()
        self._playing = True

        self._play_thread = threading.Thread(target=self._play_loop, daemon=True)
        self._play_thread.start()
        self.log_msg.emit(f"[Replay] Started (speed={self._speed_factor:.1f}x)")
        return True

    def stop(self) -> None:
        """Stop playback."""
        self._stop_flag.set()
        self._playing = False
        if self._play_thread and self._play_thread.is_alive():
            self._play_thread.join(timeout=1.0)
        self._play_thread = None
        self.log_msg.emit("[Replay] Stopped")

    def _play_loop(self) -> None:
        """Playback loop."""
        if not self.samples:
            return

        start_idx = self.cursor
        if start_idx >= len(self.samples):
            start_idx = 0

        start_time = time.time()
        start_sim_time = self.samples[start_idx].time_sec

        while not self._stop_flag.is_set() and self.cursor < len(self.samples):
            elapsed = time.time() - start_time
            target_sim_time = start_sim_time + elapsed * self._speed_factor

            # Find matching sample
            while self.cursor < len(self.samples) and self.samples[self.cursor].time_sec <= target_sim_time:
                sample = self.samples[self.cursor]
                self.mirror.set_joint_targets(sample.targets)
                self.cursor_changed.emit(self.cursor)
                self.cursor += 1

            if self.cursor >= len(self.samples):
                break

            time.sleep(0.005)

        self._playing = False
        if self.cursor >= len(self.samples):
            self.cursor = len(self.samples) - 1
        self.log_msg.emit("[Replay] Playback finished")

    def step(self) -> None:
        """Advance one frame."""
        if not self.is_loaded:
            return
        if self.cursor < len(self.samples) - 1:
            self.cursor += 1
            sample = self.samples[self.cursor]
            self.mirror.set_joint_targets(sample.targets)
            self.cursor_changed.emit(self.cursor)

    def prev(self) -> None:
        """Go back one frame."""
        if not self.is_loaded:
            return
        if self.cursor > 0:
            self.cursor -= 1
            sample = self.samples[self.cursor]
            self.mirror.set_joint_targets(sample.targets)
            self.cursor_changed.emit(self.cursor)

    def seek(self, frame_idx: int) -> None:
        """Jump to specific frame."""
        if not self.is_loaded:
            return
        frame_idx = max(0, min(frame_idx, len(self.samples) - 1))
        self.cursor = frame_idx
        sample = self.samples[self.cursor]
        self.mirror.set_joint_targets(sample.targets)
        self.cursor_changed.emit(self.cursor)