"""Replay-oriented workflow helper for future GUI / API services."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional

from recorder import ReplayCsvStore
from robot_backend import RobotBackend


class ReplayController:
    def __init__(self, backend: RobotBackend, store_dir: str = "/tmp/dog_replay_csv"):
        self.backend = backend
        self.store = ReplayCsvStore(store_dir)

    def stage_and_load_csv(self, filename: str, csv_text: str) -> Dict:
        path = self.store.save_text(filename, csv_text)
        return self.backend.load_replay_csv(path)

    def start(self) -> Dict:
        return self.backend.replay_start()

    def stop(self) -> Dict:
        return self.backend.replay_stop()

    def step(self) -> Dict:
        return self.backend.replay_step()

    def prev(self) -> Dict:
        return self.backend.replay_prev()

    def seek(self, frame_idx: int) -> Dict:
        return self.backend.replay_seek(frame_idx)

    def status(self) -> Dict:
        return self.backend.replay_status()
