"""TCP client for the dog debug daemon.

The protocol is newline-delimited UTF-8 text on a single command socket
(request/reply JSON).
"""

from __future__ import annotations

import json
import socket
import threading
from typing import Any, Dict, List, Optional


class RobotClient:
    """Persistent command client for the robot debug daemon."""

    def __init__(self, host: str, cmd_port: int = 47001, timeout_s: float = 3.0):
        self.host = host
        self.cmd_port = int(cmd_port)
        self.timeout_s = float(timeout_s)

        self._cmd_sock: Optional[socket.socket] = None
        self._cmd_lock = threading.Lock()
        self._last_error = ""

    @property
    def connected(self) -> bool:
        return self._cmd_sock is not None

    @property
    def last_error(self) -> str:
        return self._last_error

    def connect(self) -> None:
        if self.connected:
            return
        self.close()
        self._cmd_sock = socket.create_connection((self.host, self.cmd_port), timeout=self.timeout_s)
        self._cmd_sock.settimeout(self.timeout_s)

    def close(self) -> None:
        if self._cmd_sock is None:
            return
        try:
            self._cmd_sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self._cmd_sock.close()
        except OSError:
            pass
        self._cmd_sock = None

    def send_command(self, command: str, timeout: Optional[float] = None) -> Dict[str, Any]:
        """Send one command line and parse the JSON reply."""
        if self._cmd_sock is None:
            raise RuntimeError("not connected")
        payload = (command.strip() + "\n").encode("utf-8")
        with self._cmd_lock:
            old_timeout = self._cmd_sock.gettimeout()
            self._cmd_sock.settimeout(self.timeout_s if timeout is None else timeout)
            try:
                self._cmd_sock.sendall(payload)
                buffer = b""
                while b"\n" not in buffer:
                    chunk = self._cmd_sock.recv(4096)
                    if not chunk:
                        raise RuntimeError("command socket closed")
                    buffer += chunk
            finally:
                self._cmd_sock.settimeout(old_timeout)
        reply = json.loads(buffer.split(b"\n", 1)[0].decode("utf-8"))
        if not reply.get("ok", False):
            self._last_error = str(reply.get("error", "unknown daemon error"))
        return reply

    def ping(self) -> bool:
        return bool(self.send_command("ping").get("ok", False))

    def get_state_once(self) -> Dict[str, Any]:
        return self.send_command("get_state")

    def enable(self) -> Dict[str, Any]:
        return self.send_command("enable")

    def disable(self) -> Dict[str, Any]:
        return self.send_command("disable")

    def set_joint(self, targets_rad: List[float]) -> Dict[str, Any]:
        if len(targets_rad) != 12:
            raise ValueError("set_joint expects exactly 12 values")
        cmd = "set_joint " + " ".join(f"{float(v):.6f}" for v in targets_rad)
        return self.send_command(cmd)

    def replay(self, csv_path: str, speed_factor: float = 1.0, skip_feedback_check: bool = False) -> Dict[str, Any]:
        cmd = f"replay {csv_path} {float(speed_factor):.3f}"
        if skip_feedback_check:
            cmd += " --no-feedback-check"
        return self.send_command(cmd)

    def set_mit_param(self, kp: float, kd: float, vel_limit: float, torque_limit: float) -> Dict[str, Any]:
        return self.send_command(f"set_mit_param {kp:.4f} {kd:.4f} {vel_limit:.4f} {torque_limit:.4f}")
