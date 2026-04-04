from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class RobotState:
    ok: bool = False
    mode: str = ""
    seq: int = 0
    motors_online: int = 0
    feedback_age_ms: int = -1
    joint_positions: List[float] = field(default_factory=lambda: [0.0] * 12)
    joint_velocities: List[float] = field(default_factory=lambda: [0.0] * 12)
    motor_positions: List[float] = field(default_factory=lambda: [0.0] * 12)
    motor_velocities: List[float] = field(default_factory=lambda: [0.0] * 12)
    motor_torques: List[float] = field(default_factory=lambda: [0.0] * 12)
    target_joint_positions: List[float] = field(default_factory=lambda: [0.0] * 12)
    raw: dict = field(default_factory=dict)

    @classmethod
    def from_json(cls, text: str) -> "RobotState":
        data = json.loads(text)
        return cls(
            ok=bool(data.get("ok", False)),
            mode=str(data.get("mode", "")),
            seq=int(data.get("seq", 0)),
            motors_online=int(data.get("motors_online", 0)),
            feedback_age_ms=int(data.get("feedback_age_ms", -1)),
            joint_positions=list(data.get("joint_positions", [0.0] * 12)),
            joint_velocities=list(data.get("joint_velocities", [0.0] * 12)),
            motor_positions=list(data.get("motor_positions", [0.0] * 12)),
            motor_velocities=list(data.get("motor_velocities", [0.0] * 12)),
            motor_torques=list(data.get("motor_torques", [0.0] * 12)),
            target_joint_positions=list(data.get("target_joint_positions", [0.0] * 12)),
            raw=data,
        )


class TwinAgentClient:
    """Persistent TCP client for twin_agent command + state channels."""

    def __init__(self, host: str, cmd_port: int = 47001, state_port: int = 47002, timeout_s: float = 3.0):
        self.host = host
        self.cmd_port = int(cmd_port)
        self.state_port = int(state_port)
        self.timeout_s = float(timeout_s)

        self._cmd_sock: Optional[socket.socket] = None
        self._state_sock: Optional[socket.socket] = None
        self._cmd_lock = threading.Lock()
        self._state_lock = threading.Lock()

        self._running = False
        self._state_thread: Optional[threading.Thread] = None
        self._latest_state: Optional[RobotState] = None
        self._latest_state_walltime = 0.0
        self._last_error = ""

    @property
    def connected(self) -> bool:
        return self._running and self._cmd_sock is not None and self._state_sock is not None

    @property
    def last_error(self) -> str:
        return self._last_error

    def connect(self) -> None:
        if self.connected:
            return
        self.close()
        self._cmd_sock = socket.create_connection((self.host, self.cmd_port), timeout=self.timeout_s)
        self._cmd_sock.settimeout(self.timeout_s)
        self._state_sock = socket.create_connection((self.host, self.state_port), timeout=self.timeout_s)
        self._state_sock.settimeout(1.0)
        self._running = True
        self._state_thread = threading.Thread(target=self._state_loop, name="twin-state", daemon=True)
        self._state_thread.start()

    def close(self) -> None:
        self._running = False
        for sock in (self._cmd_sock, self._state_sock):
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    sock.close()
                except OSError:
                    pass
        self._cmd_sock = None
        self._state_sock = None

    def _state_loop(self) -> None:
        buffer = b""
        assert self._state_sock is not None
        while self._running:
            try:
                chunk = self._state_sock.recv(4096)
                if not chunk:
                    self._last_error = "state stream closed"
                    break
                buffer += chunk
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    if not line.strip():
                        continue
                    try:
                        state = RobotState.from_json(line.decode("utf-8"))
                    except Exception as exc:
                        self._last_error = f"state parse failed: {exc}"
                        continue
                    with self._state_lock:
                        self._latest_state = state
                        self._latest_state_walltime = time.time()
            except socket.timeout:
                continue
            except OSError as exc:
                if self._running:
                    self._last_error = f"state socket error: {exc}"
                break
            except Exception as exc:
                if self._running:
                    self._last_error = f"state loop error: {exc}"
                break
        self._running = False

    def send_command(self, command: str) -> dict:
        if self._cmd_sock is None:
            raise RuntimeError("command socket is not connected")
        payload = (command.strip() + "\n").encode("utf-8")
        with self._cmd_lock:
            self._cmd_sock.sendall(payload)
            buffer = b""
            while b"\n" not in buffer:
                chunk = self._cmd_sock.recv(4096)
                if not chunk:
                    raise RuntimeError("command socket closed")
                buffer += chunk
        reply = json.loads(buffer.split(b"\n", 1)[0].decode("utf-8"))
        if not reply.get("ok", False):
            self._last_error = reply.get("error", "unknown twin-agent error")
        return reply

    def ping(self) -> bool:
        reply = self.send_command("ping")
        return bool(reply.get("ok", False))

    def init_robot(self, seconds: float = 2.5) -> dict:
        return self.send_command(f"init {float(seconds):.3f}")

    def enable(self) -> dict:
        return self.send_command("enable")

    def disable(self) -> dict:
        return self.send_command("disable")

    def hold(self) -> dict:
        return self.send_command("hold")

    def get_state_once(self) -> dict:
        return self.send_command("get_state")

    def set_joint(self, targets_rad: List[float]) -> dict:
        if len(targets_rad) != 12:
            raise ValueError("set_joint expects 12 targets")
        cmd = "set_joint " + " ".join(f"{float(v):.6f}" for v in targets_rad)
        return self.send_command(cmd)

    def latest_state(self) -> Optional[RobotState]:
        with self._state_lock:
            return self._latest_state

    def latest_state_age_ms(self) -> float:
        with self._state_lock:
            if self._latest_state_walltime <= 0.0:
                return float("inf")
            return max(0.0, (time.time() - self._latest_state_walltime) * 1000.0)
