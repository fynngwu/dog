from __future__ import annotations

from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class DashboardPage(QWidget):
    def __init__(self, backend_service, replay_service, stream_worker):
        super().__init__()
        self.backend = backend_service
        self.replay = replay_service
        self.stream_worker = stream_worker
        self._intentional_disconnect = False
        self.backend.connected_changed.connect(self._on_connected_changed)
        self.setup_ui()

    def setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        conn_group = QGroupBox("Connection")
        conn_layout = QHBoxLayout()
        self.host_input = QLineEdit("127.0.0.1")
        self.cmd_port_input = QLineEdit("47001")
        self.state_port_input = QLineEdit("47002")
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        self.connect_btn.clicked.connect(self.on_connect)
        self.disconnect_btn.clicked.connect(self.on_disconnect)
        conn_layout.addWidget(QLabel("Host:"))
        conn_layout.addWidget(self.host_input)
        conn_layout.addWidget(QLabel("Cmd:"))
        conn_layout.addWidget(self.cmd_port_input)
        conn_layout.addWidget(QLabel("State:"))
        conn_layout.addWidget(self.state_port_input)
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.disconnect_btn)
        conn_group.setLayout(conn_layout)
        layout.addWidget(conn_group)

        ctrl_group = QGroupBox("Robot Control")
        ctrl_layout = QHBoxLayout()
        self.init_btn = QPushButton("Init Robot")
        self.enable_btn = QPushButton("Enable")
        self.disable_btn = QPushButton("Disable")
        self.disable_btn.setObjectName("danger")
        self.init_btn.clicked.connect(lambda: self.backend.init_robot(2.5))
        self.enable_btn.clicked.connect(self.backend.enable)
        self.disable_btn.clicked.connect(self.backend.disable)
        ctrl_layout.addWidget(self.init_btn)
        ctrl_layout.addWidget(self.enable_btn)
        ctrl_layout.addWidget(self.disable_btn)
        ctrl_group.setLayout(ctrl_layout)
        layout.addWidget(ctrl_group)

        status_group = QGroupBox("Status")
        status_layout = QGridLayout()
        self.lbl_conn = QLabel("Cmd: offline")
        self.lbl_mode = QLabel("Mode: offline")
        self.lbl_motion = QLabel("Motion: -")
        self.lbl_stream = QLabel("Stream: offline")
        self.lbl_fault = QLabel("Fault: -")
        status_layout.addWidget(self.lbl_conn, 0, 0)
        status_layout.addWidget(self.lbl_stream, 0, 1)
        status_layout.addWidget(self.lbl_mode, 1, 0)
        status_layout.addWidget(self.lbl_motion, 1, 1)
        status_layout.addWidget(self.lbl_fault, 2, 0, 1, 2)
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        layout.addStretch()

    def _apply_offline_style(self) -> None:
        self._intentional_disconnect = False
        self.lbl_conn.setText("Cmd: offline")
        self.lbl_conn.setStyleSheet("color: #636e72; font-weight: bold;")
        self.lbl_stream.setText("Stream: offline")
        self.lbl_stream.setStyleSheet("")
        self.lbl_mode.setText("Mode: offline")
        self.lbl_motion.setText("Motion: -")
        self.lbl_motion.setStyleSheet("")
        self.lbl_fault.setText("Fault: -")
        self.lbl_fault.setStyleSheet("")
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)

    def on_connect(self) -> None:
        host = self.host_input.text().strip()
        cmd_port = int(self.cmd_port_input.text())
        state_port = int(self.state_port_input.text())
        self._intentional_disconnect = False
        self.backend.connect_backend(host, cmd_port, state_port)
        self.replay.connect_backend(host, cmd_port, state_port)
        self.stream_worker.set_target(host, state_port)
        if not self.stream_worker.isRunning():
            self.stream_worker.start()

    def _on_connected_changed(self, connected: bool) -> None:
        if connected:
            self.lbl_conn.setText("Cmd: connected")
            self.lbl_conn.setStyleSheet("color: #27ae60; font-weight: bold;")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
        else:
            if not self._intentional_disconnect:
                QMessageBox.warning(
                    self, "Connection Failed",
                    f"Cannot connect to {self.host_input.text()}:{self.cmd_port_input.text()}",
                )
            self.stream_worker.stop()
            self._apply_offline_style()

    def on_disconnect(self) -> None:
        self._intentional_disconnect = True
        self.stream_worker.stop()
        self.backend.disconnect_backend()
        self.replay.disconnect_backend()

    def update_stream_status(self, connected: bool) -> None:
        if connected:
            self.lbl_stream.setText("Stream: connected")
            self.lbl_stream.setStyleSheet("color: #27ae60;")
        else:
            self.lbl_stream.setText("Stream: disconnected")
            self.lbl_stream.setStyleSheet("color: #e74c3c;")

    def update_state(self, state) -> None:
        self.lbl_mode.setText(f"Mode: {state.mode}")
        if state.motion_active:
            self.lbl_motion.setText(f"Motion: {state.motion_name}")
            self.lbl_motion.setStyleSheet("color: #2980b9;")
        else:
            self.lbl_motion.setText("Motion: idle")
            self.lbl_motion.setStyleSheet("")
        if state.fault_active:
            self.lbl_fault.setText(f"FAULT: {state.fault_msg or state.fault_code}")
            self.lbl_fault.setStyleSheet("color: #e74c3c; font-weight: bold;")
        else:
            self.lbl_fault.setText("Fault: none")
            self.lbl_fault.setStyleSheet("color: #27ae60;")
