from __future__ import annotations

from typing import Any, Dict, Optional

from PySide6.QtCore import QObject, QThreadPool, Signal

from app.services.command_worker import CommandWorker
from experiment_manager import ExperimentManager


class BackendService(QObject):
    log_msg = Signal(str)
    cmd_success = Signal(str)
    cmd_error = Signal(str)
    connected_changed = Signal(bool)

    def __init__(self):
        super().__init__()
        self.thread_pool = QThreadPool.globalInstance()
        self.manager: Optional[ExperimentManager] = None
        self.host = "127.0.0.1"
        self.cmd_port = 47001
        self.state_port = 47002

    def connect_backend(self, host: str, cmd_port: int, state_port: int = 47002) -> None:
        self.host = host
        self.cmd_port = int(cmd_port)
        self.state_port = int(state_port)
        self.manager = ExperimentManager(host=self.host, cmd_port=self.cmd_port, state_port=self.state_port)
        self.manager.connect()
        self.log_msg.emit(f"Connected command channel to {host}:{cmd_port}")
        self.connected_changed.emit(True)

    def disconnect_backend(self) -> None:
        if self.manager is not None:
            self.manager.disconnect()
        self.connected_changed.emit(False)
        self.log_msg.emit("Disconnected command channel")

    def _normalize_reply(self, result: Any) -> Dict[str, Any]:
        if isinstance(result, dict):
            return result
        return {"ok": False, "error": {"message": f"unexpected backend reply type: {type(result)!r}"}}

    def _execute_async(self, func, success_msg: str, *args, **kwargs) -> None:
        if not self.manager:
            self.cmd_error.emit("Backend not connected.")
            return
        worker = CommandWorker(func, *args, **kwargs)
        worker.signals.finished.connect(lambda res: self._handle_reply(self._normalize_reply(res), success_msg))
        worker.signals.error.connect(self._on_error)
        self.thread_pool.start(worker)

    def _handle_reply(self, reply: Dict[str, Any], success_msg: str) -> None:
        if reply.get("ok"):
            msg = reply.get("msg") or success_msg
            self.log_msg.emit(f"[OK] {msg}")
            self.cmd_success.emit(str(msg))
            return
        error = reply.get("error") or {}
        message = error.get("message") or reply.get("msg") or "unknown backend error"
        detail = error.get("detail")
        if detail:
            message = f"{message} | {detail}"
        self.log_msg.emit(f"[ERROR] {message}")
        self.cmd_error.emit(str(message))

    def _on_error(self, err: str) -> None:
        self.log_msg.emit(f"[EXCEPTION] {err}")
        self.cmd_error.emit(err)

    def init_robot(self, duration_s: float = 2.5) -> None:
        self._execute_async(self.manager.init_robot, "Robot initialized", duration_s)

    def enable(self) -> None:
        self._execute_async(self.manager.enable, "Robot enabled")

    def disable(self) -> None:
        self._execute_async(self.manager.disable, "Robot disabled")

    def joint_test(self, indices, target_rad: float) -> None:
        self._execute_async(self.manager.joint_test, f"joint_test sent to {indices}", indices, target_rad)

    def joint_sine(self, indices, amp: float, freq: float, duration: float) -> None:
        self._execute_async(self.manager.joint_sine, f"joint_sine sent to {indices}", indices, amp, freq, duration)
