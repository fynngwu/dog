from __future__ import annotations

import json
import socket
import time
from typing import Optional

from PySide6.QtCore import QThread, Signal


class StateStreamWorker(QThread):
    state_received = Signal(dict)
    connection_status = Signal(bool)
    stream_error = Signal(str)

    def __init__(self, host: str = "127.0.0.1", port: int = 47002):
        super().__init__()
        self.host = host
        self.port = int(port)
        self.running = False
        self._sock: Optional[socket.socket] = None

    def set_target(self, host: str, port: int) -> None:
        self.host = host
        self.port = int(port)

    def run(self) -> None:
        self.running = True
        while self.running:
            try:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._sock.settimeout(2.0)
                self._sock.connect((self.host, self.port))
                self.connection_status.emit(True)
                with self._sock.makefile("r", encoding="utf-8") as fp:
                    while self.running:
                        line = fp.readline()
                        if not line:
                            break
                        try:
                            self.state_received.emit(json.loads(line))
                        except json.JSONDecodeError:
                            continue
            except (socket.error, socket.timeout) as exc:
                self.connection_status.emit(False)
                self.stream_error.emit(str(exc))
                if self.running:
                    time.sleep(1.0)
            finally:
                if self._sock is not None:
                    try:
                        self._sock.close()
                    except OSError:
                        pass
                    self._sock = None

    def stop(self) -> None:
        self.running = False
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
