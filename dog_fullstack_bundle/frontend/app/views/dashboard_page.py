from __future__ import annotations

from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
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
        self.setup_ui()

    def setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        conn_group = QGroupBox("Connection Settings")
        conn_layout = QHBoxLayout()
        self.host_input = QLineEdit("127.0.0.1")
        self.cmd_port_input = QLineEdit("47001")
        self.state_port_input = QLineEdit("47002")
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.connect_btn.clicked.connect(self.on_connect)
        self.disconnect_btn.clicked.connect(self.on_disconnect)
        conn_layout.addWidget(QLabel("Host:"))
        conn_layout.addWidget(self.host_input)
        conn_layout.addWidget(QLabel("Cmd Port:"))
        conn_layout.addWidget(self.cmd_port_input)
        conn_layout.addWidget(QLabel("State Port:"))
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

        status_group = QGroupBox("System Overview")
        status_layout = QGridLayout()
        self.lbl_mode = QLabel("Mode: N/A")
        self.lbl_motion = QLabel("Motion Active: N/A")
        self.lbl_stream = QLabel("State Stream: disconnected")
        self.lbl_fault = QLabel("Fault: None")
        self.lbl_fault.setStyleSheet("color: red; font-weight: bold;")
        status_layout.addWidget(self.lbl_mode, 0, 0)
        status_layout.addWidget(self.lbl_motion, 0, 1)
        status_layout.addWidget(self.lbl_stream, 1, 0)
        status_layout.addWidget(self.lbl_fault, 1, 1)
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        layout.addStretch()

    def on_connect(self) -> None:
        host = self.host_input.text().strip()
        cmd_port = int(self.cmd_port_input.text())
        state_port = int(self.state_port_input.text())
        self.backend.connect_backend(host, cmd_port, state_port)
        self.replay.connect_backend(host, cmd_port, state_port)
        self.stream_worker.set_target(host, state_port)
        if not self.stream_worker.isRunning():
            self.stream_worker.start()

    def on_disconnect(self) -> None:
        self.stream_worker.stop()
        self.backend.disconnect_backend()
        self.replay.disconnect_backend()
        self.lbl_stream.setText("State Stream: disconnected")

    def update_stream_status(self, connected: bool) -> None:
        self.lbl_stream.setText(f"State Stream: {'connected' if connected else 'disconnected'}")

    def update_state(self, state) -> None:
        self.lbl_mode.setText(f"Mode: {state.mode}")
        self.lbl_motion.setText(f"Motion Active: {state.motion_active} ({state.motion_name or '-'})")
        if state.fault_active:
            self.lbl_fault.setText(f"Fault: {state.fault_code or 'fault'} | {state.fault_msg}")
        else:
            self.lbl_fault.setText("Fault: None")
