from __future__ import annotations

from pathlib import Path
from typing import Optional

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSlider,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from knee_processor import clamp_right_knee_csv


class RecordWorker(QThread):
    """Background worker for running SimRecorder headless."""
    log_msg = Signal(str)
    finished_csv = Signal(str)  # csv_text on success
    finished_error = Signal(str)  # error message on failure

    def __init__(self, xml_path: str, onnx_path: str, schedule_path: str, cmd_x: float, cmd_y: float, cmd_yaw: float, duration: float, output_path: str, knee_clamp: bool = True):
        super().__init__()
        self.xml_path = xml_path
        self.onnx_path = onnx_path
        self.schedule_path = schedule_path
        self.cmd_x = cmd_x
        self.cmd_y = cmd_y
        self.cmd_yaw = cmd_yaw
        self.duration = duration
        self.output_path = output_path
        self.knee_clamp = knee_clamp

    def run(self) -> None:
        try:
            import sys
            import os
            # Add backend/rec/sim_record to path
            bundle_root = Path(__file__).parent.parent.parent.parent
            sim_record_path = str(bundle_root / "backend" / "rec" / "sim_record")
            if sim_record_path not in sys.path:
                sys.path.insert(0, sim_record_path)

            from sim_record import SimRecorder, RecorderConfig, build_inline_schedule

            self.log_msg.emit("[Record] Starting policy recording...")

            schedule = build_inline_schedule([self.cmd_x, self.cmd_y, self.cmd_yaw], self.duration)
            cfg = RecorderConfig(
                xml_path=Path(self.xml_path),
                onnx_path=Path(self.onnx_path),
                output_csv=Path(self.output_path),
                plot_output=None,
                schedule=schedule,
                headless=True,
            )

            recorder = SimRecorder(cfg)
            recorder.run()

            # Read generated CSV
            csv_text = Path(self.output_path).read_text(encoding="utf-8")

            # Apply knee clamping if requested
            if self.knee_clamp:
                csv_text, clamped = clamp_right_knee_csv(csv_text)
                self.log_msg.emit(f"[Record] Knee clamping applied: {clamped} values")

            self.log_msg.emit(f"[Record] Recording complete: {len(recorder.records)} samples")
            self.finished_csv.emit(csv_text)

        except Exception as e:
            import traceback
            self.log_msg.emit(f"[Record] Error: {e}")
            self.finished_error.emit(str(e))


class ReplayPage(QWidget):
    def __init__(self, replay_service, mujoco_mirror=None):
        super().__init__()
        self.replay = replay_service
        self.mujoco_mirror = mujoco_mirror
        self._pending_seek = False
        self._csv_text: Optional[str] = None
        self._record_worker: Optional[RecordWorker] = None
        self.setup_ui()

    def set_mujoco_mirror(self, mirror) -> None:
        self.mujoco_mirror = mirror

    def setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        # Section A: Record Policy Trajectory
        layout.addWidget(self._build_record_section())

        # Section B: Load CSV
        layout.addWidget(self._build_load_section())

        # Section C: Transport
        layout.addWidget(self._build_transport_section())

        # Status and Log
        self.lbl_status = QLabel("Status: idle")
        layout.addWidget(self.lbl_status)
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        layout.addWidget(self.log_area)
        self.replay.log_msg.connect(self.append_log)

    def _build_record_section(self) -> QGroupBox:
        group = QGroupBox("Record Policy Trajectory")
        layout = QVBoxLayout(group)

        # Paths row
        paths_grid = QGridLayout()
        paths_grid.addWidget(QLabel("XML:"), 0, 0)
        self.xml_edit = QLineEdit("assets/leggedrobot_flat.xml")
        paths_grid.addWidget(self.xml_edit, 0, 1)

        paths_grid.addWidget(QLabel("ONNX:"), 1, 0)
        self.onnx_edit = QLineEdit()
        self.onnx_edit.setPlaceholderText("Path to policy.onnx")
        paths_grid.addWidget(self.onnx_edit, 1, 1)

        paths_grid.addWidget(QLabel("Schedule:"), 2, 0)
        self.schedule_edit = QLineEdit()
        self.schedule_edit.setPlaceholderText("Optional: path to schedule.yaml")
        paths_grid.addWidget(self.schedule_edit, 2, 1)

        btn_browse_onnx = QPushButton("Browse")
        btn_browse_onnx.clicked.connect(self._browse_onnx)
        paths_grid.addWidget(btn_browse_onnx, 1, 2)

        btn_browse_schedule = QPushButton("Browse")
        btn_browse_schedule.clicked.connect(self._browse_schedule)
        paths_grid.addWidget(btn_browse_schedule, 2, 2)

        layout.addLayout(paths_grid)

        # Inline command inputs
        cmd_label = QLabel("Or use inline command (cmd_x, cmd_y, cmd_yaw, duration):")
        layout.addWidget(cmd_label)

        cmd_row = QHBoxLayout()
        self.cmd_x_spin = self._make_spin(-2.0, 2.0, 0.1, 0.5)
        self.cmd_y_spin = self._make_spin(-2.0, 2.0, 0.1, 0.0)
        self.cmd_yaw_spin = self._make_spin(-3.14, 3.14, 0.1, 0.0)
        self.duration_spin = self._make_spin(0.5, 60.0, 0.5, 3.0)

        cmd_row.addWidget(QLabel("cmd_x:"))
        cmd_row.addWidget(self.cmd_x_spin)
        cmd_row.addWidget(QLabel("cmd_y:"))
        cmd_row.addWidget(self.cmd_y_spin)
        cmd_row.addWidget(QLabel("cmd_yaw:"))
        cmd_row.addWidget(self.cmd_yaw_spin)
        cmd_row.addWidget(QLabel("dur:"))
        cmd_row.addWidget(self.duration_spin)
        layout.addLayout(cmd_row)

        # Knee clamping
        self.chk_knee_clamp = QCheckBox("Apply right knee clamping")
        self.chk_knee_clamp.setChecked(True)
        layout.addWidget(self.chk_knee_clamp)

        # Record button
        self.btn_record = QPushButton("Record")
        self.btn_record.clicked.connect(self._on_record)
        layout.addWidget(self.btn_record)

        self.lbl_record_status = QLabel("")
        layout.addWidget(self.lbl_record_status)

        return group

    def _build_load_section(self) -> QGroupBox:
        group = QGroupBox("Load CSV")
        layout = QHBoxLayout(group)

        self.lbl_file = QLabel("No file loaded")
        layout.addWidget(self.lbl_file, 1)

        self.btn_select_csv = QPushButton("Select CSV")
        self.btn_select_csv.clicked.connect(self._on_select_csv)
        layout.addWidget(self.btn_select_csv)

        self.btn_load_robot = QPushButton("Load to Robot")
        self.btn_load_robot.clicked.connect(self._on_load_to_robot)
        self.btn_load_robot.setEnabled(False)
        layout.addWidget(self.btn_load_robot)

        self.btn_load_mujoco = QPushButton("Load to MuJoCo")
        self.btn_load_mujoco.clicked.connect(self._on_load_to_mujoco)
        self.btn_load_mujoco.setEnabled(False)
        layout.addWidget(self.btn_load_mujoco)

        self.lbl_load_status = QLabel("")
        layout.addWidget(self.lbl_load_status)

        return group

    def _build_transport_section(self) -> QGroupBox:
        group = QGroupBox("Transport")
        layout = QVBoxLayout(group)

        # Main controls row
        ctrl_row = QHBoxLayout()
        self.btn_start_both = QPushButton("Start Robot + MuJoCo")
        self.btn_stop_both = QPushButton("Stop Both")
        self.btn_start = QPushButton("Start Robot")
        self.btn_stop = QPushButton("Stop Robot")
        self.btn_step = QPushButton("Step")
        self.btn_prev = QPushButton("Prev")

        self.btn_start_both.clicked.connect(self._on_start_both)
        self.btn_stop_both.clicked.connect(self._on_stop_both)
        self.btn_start.clicked.connect(self.replay.start)
        self.btn_stop.clicked.connect(self.replay.stop)
        self.btn_step.clicked.connect(self._on_step_both)
        self.btn_prev.clicked.connect(self._on_prev_both)

        for w in [self.btn_start_both, self.btn_stop_both, self.btn_start, self.btn_stop, self.btn_prev, self.btn_step]:
            ctrl_row.addWidget(w)
        layout.addLayout(ctrl_row)

        # Seek row
        seek_row = QHBoxLayout()
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setTracking(False)
        self.spinbox = QSpinBox()
        self.lbl_total = QLabel("/ 0")
        self.slider.valueChanged.connect(self._sync_slider_to_spinbox)
        self.slider.sliderReleased.connect(self._emit_seek_from_slider)
        self.spinbox.editingFinished.connect(self._on_spinbox_changed)
        seek_row.addWidget(self.slider)
        seek_row.addWidget(self.spinbox)
        seek_row.addWidget(self.lbl_total)
        layout.addLayout(seek_row)

        return group

    @staticmethod
    def _make_spin(min_val: float, max_val: float, step: float, value: float):
        from PySide6.QtWidgets import QDoubleSpinBox
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setDecimals(3)
        spin.setSingleStep(step)
        spin.setValue(value)
        return spin

    def _browse_onnx(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Select ONNX", "", "ONNX Files (*.onnx)")
        if path:
            self.onnx_edit.setText(path)

    def _browse_schedule(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Select Schedule", "", "YAML Files (*.yaml *.yml)")
        if path:
            self.schedule_edit.setText(path)

    def _on_record(self) -> None:
        onnx_path = self.onnx_edit.text().strip()
        if not onnx_path:
            self.lbl_record_status.setText("Error: ONNX path required")
            return

        xml_path = self.xml_edit.text().strip() or "assets/leggedrobot_flat.xml"
        output_path = "/tmp/dog_recorded_trajectory.csv"

        self._record_worker = RecordWorker(
            xml_path=xml_path,
            onnx_path=onnx_path,
            schedule_path=self.schedule_edit.text().strip(),
            cmd_x=self.cmd_x_spin.value(),
            cmd_y=self.cmd_y_spin.value(),
            cmd_yaw=self.cmd_yaw_spin.value(),
            duration=self.duration_spin.value(),
            output_path=output_path,
            knee_clamp=self.chk_knee_clamp.isChecked(),
        )
        self._record_worker.log_msg.connect(self.append_log)
        self._record_worker.finished_csv.connect(self._on_record_finished)
        self._record_worker.finished_error.connect(lambda e: self.lbl_record_status.setText(f"Error: {e}"))
        self._record_worker.start()
        self.btn_record.setEnabled(False)
        self.lbl_record_status.setText("Recording...")

    def _on_record_finished(self, csv_text: str) -> None:
        self._csv_text = csv_text
        self.btn_record.setEnabled(True)
        self.lbl_record_status.setText("Recording complete!")
        self.btn_load_robot.setEnabled(True)
        self.btn_load_mujoco.setEnabled(True)
        self.lbl_file.setText("Generated: recorded_trajectory.csv")

    def _on_select_csv(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open CSV", "", "CSV Files (*.csv)")
        if not path:
            return
        try:
            self._csv_text = Path(path).read_text(encoding="utf-8")
            self.lbl_file.setText(path)
            self.btn_load_robot.setEnabled(True)
            self.btn_load_mujoco.setEnabled(True)
        except Exception as exc:
            self.append_log(f"Failed to read file: {exc}")

    def _on_load_to_robot(self) -> None:
        if self._csv_text:
            filename = Path(self.lbl_file.text()).name or "trajectory.csv"
            self.replay.load_csv_text(filename, self._csv_text)
            self.lbl_load_status.setText("Loaded to robot")

    def _on_load_to_mujoco(self) -> None:
        if self._csv_text and self.mujoco_mirror:
            filename = Path(self.lbl_file.text()).name or "trajectory.csv"
            self.replay.load_csv_to_mujoco(filename, self._csv_text)
            self.lbl_load_status.setText("Loaded to MuJoCo")

    def _on_start_both(self) -> None:
        self.replay.start()
        if self.mujoco_mirror:
            self.mujoco_mirror.start()

    def _on_stop_both(self) -> None:
        self.replay.stop()
        if self.mujoco_mirror:
            self.mujoco_mirror.stop()

    def _on_step_both(self) -> None:
        self.replay.step()
        if self.mujoco_mirror:
            self.mujoco_mirror.step()

    def _on_prev_both(self) -> None:
        self.replay.prev()
        if self.mujoco_mirror:
            self.mujoco_mirror.prev()

    def _sync_slider_to_spinbox(self, val: int) -> None:
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(val)
        self.spinbox.blockSignals(False)
        self._pending_seek = True

    def _emit_seek_from_slider(self) -> None:
        if self._pending_seek:
            idx = self.slider.value()
            self.replay.seek(idx)
            if self.mujoco_mirror:
                self.mujoco_mirror.seek(idx)
            self._pending_seek = False

    def _on_spinbox_changed(self) -> None:
        val = self.spinbox.value()
        self.slider.blockSignals(True)
        self.slider.setValue(val)
        self.slider.blockSignals(False)
        self.replay.seek(val)
        if self.mujoco_mirror:
            self.mujoco_mirror.seek(val)

    def append_log(self, msg: str) -> None:
        self.log_area.append(msg)

    def update_state(self, state) -> None:
        loaded = state.replay_loaded
        for widget in [self.btn_start, self.btn_start_both, self.btn_step, self.btn_prev, self.slider, self.spinbox]:
            widget.setEnabled(loaded)
        self.btn_stop.setEnabled(loaded or state.replay_status == "playing")
        self.btn_stop_both.setEnabled(loaded or state.replay_status == "playing")
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