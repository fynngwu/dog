from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


class ReplayPage(QWidget):
    def __init__(self, replay_service):
        super().__init__()
        self.replay = replay_service
        self._pending_seek = False
        self.setup_ui()

    def setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        file_group = QGroupBox("CSV Selection")
        file_layout = QHBoxLayout()
        self.lbl_file = QLabel("No file loaded")
        self.btn_load = QPushButton("Select & Load CSV")
        self.btn_load.clicked.connect(self.on_load_csv)
        file_layout.addWidget(self.lbl_file)
        file_layout.addWidget(self.btn_load)
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)

        ctrl_group = QGroupBox("Transport")
        ctrl_layout = QHBoxLayout()
        self.btn_start = QPushButton("Start")
        self.btn_stop = QPushButton("Stop")
        self.btn_step = QPushButton("Step")
        self.btn_prev = QPushButton("Prev")
        self.btn_start.clicked.connect(self.replay.start)
        self.btn_stop.clicked.connect(self.replay.stop)
        self.btn_step.clicked.connect(self.replay.step)
        self.btn_prev.clicked.connect(self.replay.prev)
        for w in [self.btn_start, self.btn_stop, self.btn_prev, self.btn_step]:
            ctrl_layout.addWidget(w)
        ctrl_group.setLayout(ctrl_layout)
        layout.addWidget(ctrl_group)

        seek_group = QGroupBox("Seek")
        seek_layout = QHBoxLayout()
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setTracking(False)
        self.spinbox = QSpinBox()
        self.lbl_total = QLabel("/ 0")
        self.slider.valueChanged.connect(self._sync_slider_to_spinbox)
        self.slider.sliderReleased.connect(self._emit_seek_from_slider)
        self.spinbox.editingFinished.connect(self.on_spinbox_changed)
        seek_layout.addWidget(self.slider)
        seek_layout.addWidget(self.spinbox)
        seek_layout.addWidget(self.lbl_total)
        seek_group.setLayout(seek_layout)
        layout.addWidget(seek_group)

        self.lbl_status = QLabel("Status: idle")
        layout.addWidget(self.lbl_status)
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        layout.addWidget(self.log_area)
        self.replay.log_msg.connect(self.append_log)

    def on_load_csv(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open CSV", "", "CSV Files (*.csv)")
        if not path:
            return
        try:
            content = Path(path).read_text(encoding="utf-8")
            self.lbl_file.setText(path)
            self.replay.load_csv(Path(path).name, content)
        except Exception as exc:
            self.append_log(f"Failed to read file: {exc}")

    def _sync_slider_to_spinbox(self, val: int) -> None:
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(val)
        self.spinbox.blockSignals(False)
        self._pending_seek = True

    def _emit_seek_from_slider(self) -> None:
        if self._pending_seek:
            self.replay.seek(self.slider.value())
            self._pending_seek = False

    def on_spinbox_changed(self) -> None:
        val = self.spinbox.value()
        self.slider.blockSignals(True)
        self.slider.setValue(val)
        self.slider.blockSignals(False)
        self.replay.seek(val)

    def append_log(self, msg: str) -> None:
        self.log_area.append(msg)

    def update_state(self, state) -> None:
        loaded = state.replay_loaded
        for widget in [self.btn_start, self.btn_step, self.btn_prev, self.slider, self.spinbox]:
            widget.setEnabled(loaded)
        self.btn_stop.setEnabled(loaded or state.replay_status == "playing")
        self.lbl_status.setText(f"Status: {state.replay_status}")
        if loaded and not self.slider.isSliderDown():
            max_idx = max(0, state.replay_total - 1)
            self.slider.blockSignals(True)
            self.spinbox.blockSignals(True)
            self.slider.setMaximum(max_idx)
            self.spinbox.setMaximum(max_idx)
            self.slider.setValue(min(state.replay_cursor, max_idx))
            self.spinbox.setValue(min(state.replay_cursor, max_idx))
            self.lbl_total.setText(f"/ {max_idx}")
            self.slider.blockSignals(False)
            self.spinbox.blockSignals(False)
