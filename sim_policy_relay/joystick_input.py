from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Tuple

try:
    from inputs import get_gamepad  # type: ignore
    INPUTS_AVAILABLE = True
except Exception:
    INPUTS_AVAILABLE = False


@dataclass
class CommandState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class OptionalGamepad:
    """Best-effort gamepad reader.

    Mapping:
    - Left stick vertical  -> cmd_x  (forward/back)
    - Left stick horizontal -> cmd_y (left/right)
    - Right stick horizontal -> cmd_yaw

    This module is optional on purpose. The main app works without it.
    """

    def __init__(self, deadzone: float = 0.08):
        self.deadzone = float(deadzone)
        self.available = INPUTS_AVAILABLE
        self.enabled = False
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        self._state = CommandState()
        self._raw = {"ABS_X": 0, "ABS_Y": 0, "ABS_RX": 0}

    def start(self) -> None:
        if not self.available or self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="optional-gamepad", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = bool(enabled)

    def get_command(self) -> Tuple[float, float, float]:
        with self._lock:
            return self._state.x, self._state.y, self._state.yaw

    def _loop(self) -> None:
        if not self.available:
            return
        while self._running:
            try:
                events = get_gamepad()
                changed = False
                for event in events:
                    if event.code in self._raw:
                        self._raw[event.code] = int(event.state)
                        changed = True
                if changed:
                    self._update_state()
            except Exception:
                time.sleep(0.2)

    def _normalize(self, value: int) -> float:
        norm = max(-1.0, min(1.0, float(value) / 32767.0))
        if abs(norm) < self.deadzone:
            return 0.0
        return norm

    def _update_state(self) -> None:
        x = -self._normalize(self._raw["ABS_Y"])
        y = self._normalize(self._raw["ABS_X"])
        yaw = self._normalize(self._raw["ABS_RX"])
        with self._lock:
            self._state = CommandState(x=x, y=y, yaw=yaw)
