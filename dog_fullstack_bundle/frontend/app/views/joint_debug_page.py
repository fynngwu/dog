from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QAbstractItemView,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
import pyqtgraph as pg

from app.models.joint_mapping import JOINT_INDEX_MAP


class JointDebugPage(QWidget):
    def __init__(self, backend_service):
        super().__init__()
        self.backend = backend_service
        self.setup_ui()

    def setup_ui(self) -> None:
        main_layout = QHBoxLayout(self)
        ctrl_panel = QWidget()
        ctrl_layout = QVBoxLayout(ctrl_panel)

        self.joint_list = QListWidget()
        self.joint_list.setSelectionMode(QAbstractItemView.MultiSelection)
        for idx, name in JOINT_INDEX_MAP.items():
            item = QListWidgetItem(f"[{idx}] {name}")
            item.setData(Qt.UserRole, idx)
            self.joint_list.addItem(item)
        list_group = QGroupBox("Select Joints")
        list_layout = QVBoxLayout()
        list_layout.addWidget(self.joint_list)
        list_group.setLayout(list_layout)
        ctrl_layout.addWidget(list_group)

        param_group = QGroupBox("Test Parameters")
        form = QFormLayout()
        self.tgt_spin = QDoubleSpinBox()
        self.tgt_spin.setRange(-3.14, 3.14)
        self.tgt_spin.setSingleStep(0.05)
        self.amp_spin = QDoubleSpinBox()
        self.amp_spin.setRange(0.0, 2.0)
        self.amp_spin.setSingleStep(0.05)
        self.freq_spin = QDoubleSpinBox()
        self.freq_spin.setRange(0.1, 10.0)
        self.freq_spin.setValue(1.0)
        self.dur_spin = QDoubleSpinBox()
        self.dur_spin.setRange(0.5, 60.0)
        self.dur_spin.setValue(5.0)
        self.btn_test = QPushButton("Apply Joint Test")
        self.btn_sine = QPushButton("Start Sine")
        self.btn_test.clicked.connect(self.on_joint_test)
        self.btn_sine.clicked.connect(self.on_joint_sine)
        form.addRow("Target (rad):", self.tgt_spin)
        form.addRow(self.btn_test)
        form.addRow("Sine Amp (rad):", self.amp_spin)
        form.addRow("Sine Freq (Hz):", self.freq_spin)
        form.addRow("Duration (s):", self.dur_spin)
        form.addRow(self.btn_sine)
        param_group.setLayout(form)
        ctrl_layout.addWidget(param_group)
        ctrl_layout.addStretch()
        main_layout.addWidget(ctrl_panel, 1)

        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground('w')
        self.pos_plot = self.plot_widget.addPlot(title="Position (rad)")
        self.pos_curve = self.pos_plot.plot(pen=pg.mkPen(width=2), name="Current")
        self.tgt_curve = self.pos_plot.plot(pen=pg.mkPen(width=2, style=Qt.DashLine), name="Target")
        self.plot_widget.nextRow()
        self.vel_plot = self.plot_widget.addPlot(title="Velocity (rad/s)")
        self.vel_curve = self.vel_plot.plot(pen=pg.mkPen(width=2))
        self.plot_widget.nextRow()
        self.err_plot = self.plot_widget.addPlot(title="Error (rad)")
        self.err_curve = self.err_plot.plot(pen=pg.mkPen(width=2))
        main_layout.addWidget(self.plot_widget, 3)

        self.history_len = 120
        self.pos_data = [0.0] * self.history_len
        self.tgt_data = [0.0] * self.history_len
        self.vel_data = [0.0] * self.history_len
        self.err_data = [0.0] * self.history_len

    def get_selected_indices(self):
        return [item.data(Qt.UserRole) for item in self.joint_list.selectedItems()]

    def on_joint_test(self) -> None:
        indices = self.get_selected_indices()
        if indices:
            self.backend.joint_test(indices, self.tgt_spin.value())

    def on_joint_sine(self) -> None:
        indices = self.get_selected_indices()
        if indices:
            self.backend.joint_sine(indices, self.amp_spin.value(), self.freq_spin.value(), self.dur_spin.value())

    def update_state(self, state) -> None:
        conflict = state.motion_active or state.mode == "disabled"
        self.btn_test.setDisabled(conflict)
        self.btn_sine.setDisabled(conflict)
        indices = self.get_selected_indices()
        if not indices:
            return
        idx = indices[0]
        pos = state.joint_positions[idx]
        tgt = state.target_positions[idx]
        vel = state.joint_velocities[idx]
        err = tgt - pos
        self.pos_data = self.pos_data[1:] + [pos]
        self.tgt_data = self.tgt_data[1:] + [tgt]
        self.vel_data = self.vel_data[1:] + [vel]
        self.err_data = self.err_data[1:] + [err]
        self.pos_curve.setData(self.pos_data)
        self.tgt_curve.setData(self.tgt_data)
        self.vel_curve.setData(self.vel_data)
        self.err_curve.setData(self.err_data)
