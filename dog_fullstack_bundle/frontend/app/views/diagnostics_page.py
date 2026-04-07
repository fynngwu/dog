from __future__ import annotations

import json

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QGroupBox, QLabel, QTextEdit, QSplitter, QVBoxLayout, QWidget


class DiagnosticsPage(QWidget):
    def __init__(self):
        super().__init__()
        self.setup_ui()

    def setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        splitter = QSplitter(Qt.Vertical)
        top_widget = QWidget()
        top_layout = QVBoxLayout(top_widget)
        top_layout.setContentsMargins(0, 0, 0, 0)
        self.lbl_offline = QLabel("Offline Motors: None")
        self.lbl_offline.setStyleSheet("color: #e84118; font-weight: bold;")
        self.lbl_motion_err = QLabel("Last Motion Error: None")
        top_layout.addWidget(self.lbl_offline)
        top_layout.addWidget(self.lbl_motion_err)
        splitter.addWidget(top_widget)
        json_group = QGroupBox("Raw State JSON (20Hz Stream)")
        json_layout = QVBoxLayout()
        self.json_viewer = QTextEdit()
        self.json_viewer.setReadOnly(True)
        self.json_viewer.setStyleSheet("font-family: monospace; background: #2f3640; color: #f5f6fa;")
        json_layout.addWidget(self.json_viewer)
        json_group.setLayout(json_layout)
        splitter.addWidget(json_group)
        layout.addWidget(splitter)

    def update_state(self, state, raw_dict) -> None:
        if state.offline_motors:
            self.lbl_offline.setText(f"Offline Motors: {', '.join(state.offline_motors)}")
        else:
            self.lbl_offline.setText("Offline Motors: None")
        self.lbl_motion_err.setText(f"Last Motion Error: {state.motion_last_error or 'None'}")
        if state.seq % 10 == 0:
            self.json_viewer.setText(json.dumps(raw_dict, indent=2, ensure_ascii=False))
