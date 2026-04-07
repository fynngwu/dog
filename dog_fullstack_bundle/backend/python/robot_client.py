"""TCP client for the dog debug daemon.

Command channel uses newline-delimited UTF-8 request/reply JSON.
State channel is a newline-delimited JSON push stream.
"""

from __future__ import annotations

import json
import socket
import threading
from typing import Any, Callable, Dict, List, Optional


class RobotClient:
    """Persistent client for the robot debug daemon."""

    def __init__(self, host: str, cmd_port: int = 47001, state_port: int = 47002, timeout_s: float = 3.0):
        self.host = host
        self.cmd_port = int(cmd_port)
        self.state_port = int(state_port)
        self.timeout_s = float(timeout_s)

        self._cmd_sock: Optional[socket.socket] = None
        self._state_sock: Optional[socket.socket] = None
        self._cmd_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._last_error = ""
        self._state_thread: Optional[threading.Thread] = None
        self._state_stop = threading.Event()
        self._latest_state: Optional[Dict[str, Any]] = None

    @property
    def connected(self) -> bool:
        return self._cmd_sock is not None

    @property
    def state_connected(self) -> bool:
        return self._state_sock is not None

    @property
    def last_error(self) -> str:
        return self._last_error

    @property
    def latest_state(self) -> Optional[Dict[str, Any]]:
        return self._latest_state

    def connect(self) -> None:
        if self.connected:
            return
        self.close()
        self._cmd_sock = socket.create_connection((self.host, self.cmd_port), timeout=self.timeout_s)
        self._cmd_sock.settimeout(self.timeout_s)

    def connect_state_stream(self) -> None:
        if self.state_connected:
            return
        self._state_sock = socket.create_connection((self.host, self.state_port), timeout=self.timeout_s)
        self._state_sock.settimeout(self.timeout_s)

    def close(self) -> None:
        self.stop_state_listener()
        for sock_name in ("_cmd_sock", "_state_sock"):
            sock = getattr(self, sock_name)
            if sock is None:
                continue
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
            setattr(self, sock_name, None)

    def _extract_error(self, reply: Dict[str, Any]) -> str:
        error = reply.get("error")
        if isinstance(error, dict):
            return str(error.get("message") or error.get("detail") or "unknown daemon error")
        return str(reply.get("msg") or "unknown daemon error")

    def send_command(self, command: str, timeout: Optional[float] = None) -> Dict[str, Any]:
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
            self._last_error = self._extract_error(reply)
        return reply

    def start_state_listener(self, callback: Optional[Callable[[Dict[str, Any]], None]] = None) -> None:
        self.connect_state_stream()
        if self._state_thread is not None and self._state_thread.is_alive():
            return
        self._state_stop.clear()

        def _worker() -> None:
            assert self._state_sock is not None
            buffer = b""
            while not self._state_stop.is_set():
                try:
                    chunk = self._state_sock.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buffer += chunk
                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    if not raw.strip():
                        continue
                    state = json.loads(raw.decode("utf-8"))
                    with self._state_lock:
                        self._latest_state = state
                    if callback is not None:
                        callback(state)

        self._state_thread = threading.Thread(target=_worker, daemon=True)
        self._state_thread.start()

    def stop_state_listener(self) -> None:
        self._state_stop.set()
        if self._state_thread is not None and self._state_thread.is_alive():
            self._state_thread.join(timeout=1.0)
        self._state_thread = None

    def ping(self) -> bool:
        return bool(self.send_command("ping").get("ok", False))

    def get_state_once(self) -> Dict[str, Any]:
        return self.send_command("get_state")

    def enable(self) -> Dict[str, Any]:
        return self.send_command("enable")

    def disable(self) -> Dict[str, Any]:
        return self.send_command("disable")

    def init(self, duration_s: float = 2.5) -> Dict[str, Any]:
        return self.send_command(f"init {float(duration_s):.3f}")

    def set_joint(self, targets_rad: List[float]) -> Dict[str, Any]:
        if len(targets_rad) != 12:
            raise ValueError("set_joint expects exactly 12 values")
        cmd = "set_joint " + " ".join(f"{float(v):.6f}" for v in targets_rad)
        return self.send_command(cmd)

    def joint_test(self, joint_indices: List[int], target_rad: float) -> Dict[str, Any]:
        token = ",".join(str(int(i)) for i in joint_indices)
        return self.send_command(f"joint_test {token} {float(target_rad):.6f}")

    def joint_sine(self, joint_indices: List[int], amp_rad: float, freq_hz: float, duration_s: float) -> Dict[str, Any]:
        token = ",".join(str(int(i)) for i in joint_indices)
        return self.send_command(
            f"joint_sine {token} {float(amp_rad):.6f} {float(freq_hz):.6f} {float(duration_s):.6f}"
        )

    def load_replay_csv(self, csv_path: str) -> Dict[str, Any]:
        return self.send_command(f"load_replay_csv {csv_path}")

    def replay_start(self, speed_factor: float = 1.0) -> Dict[str, Any]:
        return self.send_command(f"replay_start {float(speed_factor):.3f}")

    def replay_stop(self) -> Dict[str, Any]:
        return self.send_command("replay_stop")

    def replay_step(self) -> Dict[str, Any]:
        return self.send_command("replay_step")

    def replay_prev(self) -> Dict[str, Any]:
        return self.send_command("replay_prev")

    def replay_seek(self, frame_idx: int) -> Dict[str, Any]:
        return self.send_command(f"replay_seek {int(frame_idx)}")

    def replay_status(self) -> Dict[str, Any]:
        return self.send_command("replay_status")

    def replay(self, csv_path: str, speed_factor: float = 1.0) -> Dict[str, Any]:
        return self.send_command(f"replay {csv_path} {float(speed_factor):.3f}")

    def set_mit_param(self, kp: float, kd: float, vel_limit: float, torque_limit: float) -> Dict[str, Any]:
        return self.send_command(f"set_mit_param {kp:.4f} {kd:.4f} {vel_limit:.4f} {torque_limit:.4f}")
