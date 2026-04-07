"""Helpers for staging replay CSV files on the Jetson side.

This module is intentionally filesystem-only so it can be embedded inside a
future frontend service without introducing network transfer assumptions.
"""

from __future__ import annotations

from pathlib import Path
from typing import Union


class ReplayCsvStore:
    def __init__(self, base_dir: Union[str, Path] = "/tmp/dog_replay_csv"):
        self.base_dir = Path(base_dir)
        self.base_dir.mkdir(parents=True, exist_ok=True)

    def save_text(self, filename: str, text: str) -> Path:
        path = self.base_dir / Path(filename).name
        path.write_text(text, encoding="utf-8")
        return path

    def save_bytes(self, filename: str, data: bytes) -> Path:
        path = self.base_dir / Path(filename).name
        path.write_bytes(data)
        return path
