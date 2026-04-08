from __future__ import annotations

import traceback
from pathlib import Path

from PySide6.QtCore import Slot
from PySide6.QtWidgets import QLabel, QMainWindow, QTabWidget, QVBoxLayout, QWidget

from app.models.app_state import RobotState
from app.services.backend_service import BackendService
from app.services.mujoco_mirror_service import MujocoMirrorService
from app.services.mujoco_replay_mirror import MujocoReplayMirror
from app.services.replay_service import ReplayService
from app.services.state_stream_worker import StateStreamWorker
from app.styles import MAIN_STYLE
from app.views.dashboard_page import DashboardPage
from app.views.diagnostics_page import DiagnosticsPage
from app.views.joint_debug_page import JointDebugPage
from app.views.replay_page import ReplayPage


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dog Qt Frontend - Robot Debugger")
        self.resize(1180, 820)
        self.setStyleSheet(MAIN_STYLE)
        self.app_state = RobotState()
        self.backend_service = BackendService()
        self.replay_service = ReplayService()

        # MuJoCo mirror services
        bundle_root = Path(__file__).parent.parent.parent
        self.mujoco_service = MujocoMirrorService(bundle_root=bundle_root)
        self.mujoco_replay = MujocoReplayMirror(self.mujoco_service)

        # Connect MuJoCo signals
        self.mujoco_service.log_msg.connect(self.on_log_message)
        self.mujoco_replay.log_msg.connect(self.on_log_message)
        self.mujoco_replay.cursor_changed.connect(self._on_mujoco_cursor)

        self.stream_worker = StateStreamWorker()
        self.stream_worker.state_received.connect(self.on_state_received)
        self.stream_worker.connection_status.connect(self.on_stream_connection)
        self.stream_worker.stream_error.connect(self.on_stream_error)
        self.backend_service.log_msg.connect(self.on_log_message)
        self.backend_service.cmd_error.connect(self.on_cmd_error)
        self.replay_service.log_msg.connect(self.on_log_message)
        self.replay_service.cmd_error.connect(self.on_cmd_error)

        # Link replay service with MuJoCo mirror
        self.replay_service.set_mujoco_mirror(self.mujoco_replay)

        self.setup_ui()

    def setup_ui(self) -> None:
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        self.alert_bar = QLabel("")
        self.alert_bar.setStyleSheet("background-color: #e84118; color: white; padding: 10px; font-weight: bold;")
        self.alert_bar.hide()
        main_layout.addWidget(self.alert_bar)
        self.tabs = QTabWidget()
        self.dashboard_page = DashboardPage(self.backend_service, self.replay_service, self.stream_worker)
        self.joint_page = JointDebugPage(self.backend_service, self.mujoco_service)
        self.replay_page = ReplayPage(self.replay_service, self.mujoco_replay)
        self.diag_page = DiagnosticsPage()
        self.tabs.addTab(self.dashboard_page, "Dashboard")
        self.tabs.addTab(self.joint_page, "Joint Debug")
        self.tabs.addTab(self.replay_page, "Replay Controller")
        self.tabs.addTab(self.diag_page, "Diagnostics")
        main_layout.addWidget(self.tabs)
        self.statusBar().showMessage("Ready | Disconnected")

    @Slot(int)
    def _on_mujoco_cursor(self, cursor: int) -> None:
        """Sync MuJoCo replay cursor with UI if needed."""
        pass  # Cursor sync handled by replay page

    @Slot(dict)
    def on_state_received(self, data: dict) -> None:
        try:
            self.app_state.update_from_json(data)
            self.dashboard_page.update_state(self.app_state)
            self.joint_page.update_state(self.app_state)
            self.replay_page.update_state(self.app_state)
            self.diag_page.update_state(self.app_state, data)
            if self.app_state.fault_active:
                text = self.app_state.fault_msg or self.app_state.fault_code or "fault"
                self.alert_bar.setText(f"CRITICAL FAULT: {text}")
                self.alert_bar.show()
            else:
                self.alert_bar.hide()
        except Exception:
            traceback.print_exc()

    @Slot(bool)
    def on_stream_connection(self, connected: bool) -> None:
        self.dashboard_page.update_stream_status(connected)
        if connected:
            self.statusBar().showMessage("State Stream: Connected (20Hz)", 2000)
        else:
            self.statusBar().showMessage("State Stream: Disconnected / Reconnecting...")

    @Slot(str)
    def on_stream_error(self, msg: str) -> None:
        self.statusBar().showMessage(f"State stream error: {msg}")

    @Slot(str)
    def on_log_message(self, msg: str) -> None:
        self.statusBar().showMessage(msg, 5000)

    @Slot(str)
    def on_cmd_error(self, msg: str) -> None:
        self.alert_bar.setText(msg)
        self.alert_bar.show()

    def closeEvent(self, event) -> None:
        # Stop MuJoCo viewer
        self.mujoco_service.stop_viewer()
        self.stream_worker.stop()
        self.stream_worker.wait()
        self.backend_service.disconnect_backend()
        self.replay_service.disconnect_backend()
        super().closeEvent(event)