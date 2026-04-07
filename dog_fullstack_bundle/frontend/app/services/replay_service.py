from __future__ import annotations

from typing import Any, Dict, Optional

from PySide6.QtCore import QObject, QThreadPool, Signal

from app.services.command_worker import CommandWorker
from robot_backend import RobotBackend
from replay_controller import ReplayController


class ReplayService(QObject):
    log_msg = Signal(str)
    cmd_error = Signal(str)
    cmd_success = Signal(str)

    def __init__(self):
        super().__init__()
        self.thread_pool = QThreadPool.globalInstance()
        self.backend: Optional[RobotBackend] = None
        self.controller: Optional[ReplayController] = None

    def connect_backend(self, host: str, cmd_port: int, state_port: int = 47002) -> None:
        self.backend = RobotBackend(host=host, cmd_port=cmd_port, state_port=state_port)
        self.backend.connect()
        self.controller = ReplayController(self.backend)
        self.log_msg.emit(f"Replay controller connected to {host}:{cmd_port}")

    def disconnect_backend(self) -> None:
        if self.backend is not None:
            self.backend.disconnect()
        self.backend = None
        self.controller = None
        self.log_msg.emit("Replay controller disconnected")

    def _normalize_reply(self, result: Any) -> Dict[str, Any]:
        if isinstance(result, dict):
            return result
        return {"ok": False, "error": {"message": f"unexpected backend reply type: {type(result)!r}"}}

    def _execute_async(self, method_name: str, msg: str, *args, **kwargs) -> None:
        if not self.controller:
            self.log_msg.emit("[ERROR] Replay controller not connected.")
            self.cmd_error.emit("Replay controller not connected.")
            return
        func = getattr(self.controller, method_name)
        worker = CommandWorker(func, *args, **kwargs)
        worker.signals.finished.connect(lambda res: self._handle_reply(self._normalize_reply(res), msg))
        worker.signals.error.connect(self._on_error)
        self.thread_pool.start(worker)

    def _handle_reply(self, reply: Dict[str, Any], fallback_msg: str) -> None:
        if reply.get("ok"):
            msg = str(reply.get("msg") or fallback_msg)
            self.log_msg.emit(f"[OK] {msg}")
            self.cmd_success.emit(msg)
            return
        error = reply.get("error") or {}
        message = error.get("message") or reply.get("msg") or "unknown replay error"
        detail = error.get("detail")
        if detail:
            message = f"{message} | {detail}"
        self.log_msg.emit(f"[ERROR] {message}")
        self.cmd_error.emit(str(message))

    def _on_error(self, err: str) -> None:
        self.log_msg.emit(f"[EXCEPTION] {err}")
        self.cmd_error.emit(err)

    def load_csv(self, filename: str, content: str) -> None:
        self._execute_async("stage_and_load_csv", f"Loaded {filename}", filename, content)

    def start(self) -> None:
        self._execute_async("start", "Started playback")

    def stop(self) -> None:
        self._execute_async("stop", "Stopped playback")

    def step(self) -> None:
        self._execute_async("step", "Stepped forward")

    def prev(self) -> None:
        self._execute_async("prev", "Stepped backward")

    def seek(self, frame_idx: int) -> None:
        self._execute_async("seek", f"Seeked to frame {frame_idx}", frame_idx)
