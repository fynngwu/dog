from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, TYPE_CHECKING

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from app.models.joint_mapping import JOINT_INDEX_MAP

if TYPE_CHECKING:
    from app.services.mujoco_mirror_service import MujocoMirrorService


@dataclass
class _JointCard:
    title: str
    value_label: QLabel


class JointDebugPage(QWidget):
    """Single-joint / multi-joint debug page.

    Design goals:
    - 12 joints always visible; no dropdown/list selection.
    - Common actions are buttons, not text-input-first workflows.
    - First selected joint drives the live cards and plots.
    - Controls are not disabled merely because mode == disabled; that made the page
      look dead before the user had a meaningful state update.
    """

    SLIDER_SCALE = 1000

    def __init__(self, backend_service, mujoco_service: Optional["MujocoMirrorService"] = None):
        super().__init__()
        self.backend = backend_service
        self.mujoco_service = mujoco_service
        self.joint_buttons: Dict[int, QPushButton] = {}
        self._cards: Dict[str, _JointCard] = {}
        self._selected_plot_joint: Optional[int] = None
        self._btn_style_keys: Dict[int, str] = {}
        self._mirror_enabled = False
        self.history_len = 160
        self.pos_data: deque = deque([0.0] * self.history_len, maxlen=self.history_len)
        self.tgt_data: deque = deque([0.0] * self.history_len, maxlen=self.history_len)
        self.vel_data: deque = deque([0.0] * self.history_len, maxlen=self.history_len)
        self.err_data: deque = deque([0.0] * self.history_len, maxlen=self.history_len)
        self.setup_ui()

    def setup_ui(self) -> None:
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(12)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setSpacing(10)

        left_layout.addWidget(self._build_joint_selector_group())
        left_layout.addWidget(self._build_quick_action_group())
        left_layout.addWidget(self._build_sine_group())
        left_layout.addWidget(self._build_mujoco_group())
        left_layout.addStretch()
        main_layout.addWidget(left_panel, 2)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setSpacing(10)
        right_layout.addWidget(self._build_status_cards_group())
        right_layout.addWidget(self._build_plot_group(), 1)
        main_layout.addWidget(right_panel, 5)

        self._refresh_action_enabled_state()

    def _build_joint_selector_group(self) -> QGroupBox:
        group = QGroupBox("Joint Selection")
        layout = QVBoxLayout(group)

        hint = QLabel(
            "Select one or more joints below. The first selected joint is used for the live cards and plots."
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        for visual_idx, (idx, name) in enumerate(sorted(JOINT_INDEX_MAP.items())):
            btn = QPushButton(name)
            btn.setCheckable(True)
            btn.setMinimumHeight(42)
            btn.clicked.connect(self._on_joint_selection_changed)
            btn.setToolTip(f"Joint index {idx}")
            self.joint_buttons[idx] = btn
            row = visual_idx // 2
            col = visual_idx % 2
            grid.addWidget(btn, row, col)
        layout.addLayout(grid)

        tool_row = QHBoxLayout()
        self.btn_select_all = QPushButton("Select All")
        self.btn_clear_sel = QPushButton("Clear")
        self.btn_select_all.clicked.connect(self._select_all)
        self.btn_clear_sel.clicked.connect(self._clear_selection)
        tool_row.addWidget(self.btn_select_all)
        tool_row.addWidget(self.btn_clear_sel)
        tool_row.addStretch()
        layout.addLayout(tool_row)

        self.lbl_selection = QLabel("Selected: none")
        self.lbl_plot_joint = QLabel("Plot Joint: -")
        layout.addWidget(self.lbl_selection)
        layout.addWidget(self.lbl_plot_joint)
        return group

    def _build_quick_action_group(self) -> QGroupBox:
        group = QGroupBox("Quick Actions")
        layout = QVBoxLayout(group)

        note = QLabel(
            "Target values are relative to offset (rad). Offset button sends 0.0. "
            "Mid / negative / positive buttons use the editable values below."
        )
        note.setWordWrap(True)
        layout.addWidget(note)

        preset_grid = QGridLayout()
        preset_grid.setHorizontalSpacing(8)
        preset_grid.setVerticalSpacing(8)

        self.neg_spin = self._make_spin(-math.pi, math.pi, 0.05, -0.5)
        self.mid_spin = self._make_spin(-math.pi, math.pi, 0.05, 0.0)
        self.pos_spin = self._make_spin(-math.pi, math.pi, 0.05, 0.5)

        self.btn_go_neg = QPushButton("Go Negative")
        self.btn_go_mid = QPushButton("Go Mid")
        self.btn_go_pos = QPushButton("Go Positive")
        self.btn_go_offset = QPushButton("Go Offset")

        self.btn_go_neg.clicked.connect(lambda: self._send_preset(self.neg_spin.value()))
        self.btn_go_mid.clicked.connect(lambda: self._send_preset(self.mid_spin.value()))
        self.btn_go_pos.clicked.connect(lambda: self._send_preset(self.pos_spin.value()))
        self.btn_go_offset.clicked.connect(self.on_go_offset)

        preset_grid.addWidget(QLabel("Negative:"), 0, 0)
        preset_grid.addWidget(self.neg_spin, 0, 1)
        preset_grid.addWidget(self.btn_go_neg, 0, 2)
        preset_grid.addWidget(QLabel("Mid:"), 1, 0)
        preset_grid.addWidget(self.mid_spin, 1, 1)
        preset_grid.addWidget(self.btn_go_mid, 1, 2)
        preset_grid.addWidget(QLabel("Positive:"), 2, 0)
        preset_grid.addWidget(self.pos_spin, 2, 1)
        preset_grid.addWidget(self.btn_go_pos, 2, 2)
        preset_grid.addWidget(self.btn_go_offset, 3, 0, 1, 3)
        layout.addLayout(preset_grid)

        target_group = QGroupBox("Manual Target")
        target_layout = QVBoxLayout(target_group)
        slider_row = QHBoxLayout()
        self.target_slider = QSlider(Qt.Horizontal)
        self.target_slider.setRange(int(-math.pi * self.SLIDER_SCALE), int(math.pi * self.SLIDER_SCALE))
        self.target_slider.setSingleStep(int(0.05 * self.SLIDER_SCALE))
        self.target_slider.valueChanged.connect(self._sync_slider_to_spin)
        self.target_spin = self._make_spin(-math.pi, math.pi, 0.05, 0.0)
        self.target_spin.valueChanged.connect(self._sync_spin_to_slider)
        slider_row.addWidget(self.target_slider, 1)
        slider_row.addWidget(self.target_spin)
        target_layout.addLayout(slider_row)

        btn_row = QHBoxLayout()
        self.btn_apply_target = QPushButton("Send Target")
        self.btn_zero_target = QPushButton("Zero")
        self.btn_apply_target.clicked.connect(self.on_joint_test)
        self.btn_zero_target.clicked.connect(lambda: self._set_manual_target_and_send(0.0))
        btn_row.addWidget(self.btn_apply_target)
        btn_row.addWidget(self.btn_zero_target)
        target_layout.addLayout(btn_row)
        layout.addWidget(target_group)
        return group

    def _build_sine_group(self) -> QGroupBox:
        group = QGroupBox("Sine Test")
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(8)

        self.amp_spin = self._make_spin(0.0, 2.0, 0.05, 0.2)
        self.freq_spin = self._make_spin(0.1, 10.0, 0.1, 0.5)
        self.dur_spin = self._make_spin(0.5, 60.0, 0.5, 5.0)
        self.btn_sine = QPushButton("Start Sine Around Offset")
        self.btn_sine.clicked.connect(self.on_joint_sine)

        layout.addWidget(QLabel("Amplitude (rad):"), 0, 0)
        layout.addWidget(self.amp_spin, 0, 1)
        layout.addWidget(QLabel("Frequency (Hz):"), 1, 0)
        layout.addWidget(self.freq_spin, 1, 1)
        layout.addWidget(QLabel("Duration (s):"), 2, 0)
        layout.addWidget(self.dur_spin, 2, 1)
        layout.addWidget(self.btn_sine, 3, 0, 1, 2)
        return group

    def _build_mujoco_group(self) -> QGroupBox:
        group = QGroupBox("MuJoCo Mirror")
        layout = QVBoxLayout(group)

        # XML path input
        xml_row = QHBoxLayout()
        xml_row.addWidget(QLabel("XML:"))
        self.xml_path_edit = QLineEdit("assets/leggedrobot_flat.xml")
        xml_row.addWidget(self.xml_path_edit, 1)
        layout.addLayout(xml_row)

        # Buttons
        btn_row = QHBoxLayout()
        self.btn_mujoco_start = QPushButton("Start Viewer")
        self.btn_mujoco_stop = QPushButton("Stop Viewer")
        self.btn_mujoco_start.clicked.connect(self._on_mujoco_start)
        self.btn_mujoco_stop.clicked.connect(self._on_mujoco_stop)
        self.btn_mujoco_stop.setEnabled(False)
        btn_row.addWidget(self.btn_mujoco_start)
        btn_row.addWidget(self.btn_mujoco_stop)
        layout.addLayout(btn_row)

        # Mirror checkbox
        self.chk_mirror = QCheckBox("Mirror commands to MuJoCo")
        self.chk_mirror.setChecked(True)
        self.chk_mirror.stateChanged.connect(self._on_mirror_changed)
        layout.addWidget(self.chk_mirror)

        # Status label
        self.lbl_mujoco_status = QLabel("Status: not running")
        layout.addWidget(self.lbl_mujoco_status)

        return group

    def _on_mujoco_start(self) -> None:
        if not self.mujoco_service:
            self.lbl_mujoco_status.setText("Status: service not available")
            return
        xml_path = self.xml_path_edit.text().strip()
        if not xml_path:
            self.lbl_mujoco_status.setText("Status: XML path required")
            return
        if self.mujoco_service.load_model(xml_path):
            if self.mujoco_service.start_viewer():
                self.btn_mujoco_start.setEnabled(False)
                self.btn_mujoco_stop.setEnabled(True)
                self._mirror_enabled = True
                self.chk_mirror.setChecked(True)
                self.lbl_mujoco_status.setText("Status: running")

    def _on_mujoco_stop(self) -> None:
        if self.mujoco_service:
            self.mujoco_service.stop_viewer()
        self.btn_mujoco_start.setEnabled(True)
        self.btn_mujoco_stop.setEnabled(False)
        self._mirror_enabled = False
        self.lbl_mujoco_status.setText("Status: stopped")

    def _on_mirror_changed(self, state: int) -> None:
        self._mirror_enabled = state == Qt.Checked and self.mujoco_service is not None and self.mujoco_service.is_running

    def _build_status_cards_group(self) -> QGroupBox:
        group = QGroupBox("Selected Joint Status")
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(10)

        cards = [
            ("Joint", "-"),
            ("Current (rad)", "0.000"),
            ("Target (rad)", "0.000"),
            ("Error (rad)", "0.000"),
            ("Velocity (rad/s)", "0.000"),
            ("Torque", "0.000"),
        ]

        for i, (title, initial) in enumerate(cards):
            frame = QFrame()
            frame.setFrameShape(QFrame.StyledPanel)
            frame.setStyleSheet("QFrame { border: 1px solid #dcdde1; border-radius: 8px; background: white; }")
            frame_layout = QVBoxLayout(frame)
            title_label = QLabel(title)
            title_label.setStyleSheet("font-size: 12px; color: #636e72;")
            value_label = QLabel(initial)
            value_label.setStyleSheet("font-size: 20px; font-weight: 600;")
            frame_layout.addWidget(title_label)
            frame_layout.addWidget(value_label)
            layout.addWidget(frame, i // 3, i % 3)
            self._cards[title] = _JointCard(title=title, value_label=value_label)
        return group

    def _build_plot_group(self) -> QGroupBox:
        group = QGroupBox("Live Curves")
        layout = QVBoxLayout(group)

        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground("w")
        self.pos_plot = self.plot_widget.addPlot(title="Position (rad)")
        self.pos_plot.showGrid(x=True, y=True, alpha=0.2)
        self.pos_curve = self.pos_plot.plot(pen=pg.mkPen(width=2), name="Current")
        self.tgt_curve = self.pos_plot.plot(pen=pg.mkPen(width=2, style=Qt.DashLine), name="Target")
        self.plot_widget.nextRow()
        self.vel_plot = self.plot_widget.addPlot(title="Velocity (rad/s)")
        self.vel_plot.showGrid(x=True, y=True, alpha=0.2)
        self.vel_curve = self.vel_plot.plot(pen=pg.mkPen(width=2))
        self.plot_widget.nextRow()
        self.err_plot = self.plot_widget.addPlot(title="Error (rad)")
        self.err_plot.showGrid(x=True, y=True, alpha=0.2)
        self.err_curve = self.err_plot.plot(pen=pg.mkPen(width=2))
        layout.addWidget(self.plot_widget)
        return group

    @staticmethod
    def _make_spin(min_val: float, max_val: float, step: float, value: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setDecimals(3)
        spin.setSingleStep(step)
        spin.setValue(value)
        return spin

    def _selected_indices(self) -> List[int]:
        return [idx for idx, btn in sorted(self.joint_buttons.items()) if btn.isChecked()]

    def _first_selected_index(self) -> Optional[int]:
        indices = self._selected_indices()
        return indices[0] if indices else None

    def _on_joint_selection_changed(self) -> None:
        indices = self._selected_indices()
        names = [JOINT_INDEX_MAP[i] for i in indices]
        self.lbl_selection.setText(f"Selected: {', '.join(names) if names else 'none'}")
        first = self._first_selected_index()
        self._selected_plot_joint = first
        self.lbl_plot_joint.setText(f"Plot Joint: {JOINT_INDEX_MAP[first]}" if first is not None else "Plot Joint: -")
        self._refresh_action_enabled_state()
        if first is None:
            self._set_card("Joint", "-")

    def _select_all(self) -> None:
        for btn in self.joint_buttons.values():
            btn.setChecked(True)
        self._on_joint_selection_changed()

    def _clear_selection(self) -> None:
        for btn in self.joint_buttons.values():
            btn.setChecked(False)
        self._on_joint_selection_changed()

    def _refresh_action_enabled_state(self, motion_active: bool = False) -> None:
        has_selection = bool(self._selected_indices())
        command_enabled = has_selection and not motion_active
        for widget in [
            self.btn_go_neg,
            self.btn_go_mid,
            self.btn_go_pos,
            self.btn_go_offset,
            self.btn_apply_target,
            self.btn_zero_target,
            self.btn_sine,
            self.target_slider,
            self.target_spin,
            self.neg_spin,
            self.mid_spin,
            self.pos_spin,
            self.amp_spin,
            self.freq_spin,
            self.dur_spin,
        ]:
            widget.setEnabled(command_enabled)

    def _set_card(self, key: str, text: str) -> None:
        card = self._cards.get(key)
        if card:
            card.value_label.setText(text)

    def _sync_slider_to_spin(self, slider_val: int) -> None:
        target = slider_val / self.SLIDER_SCALE
        self.target_spin.blockSignals(True)
        self.target_spin.setValue(target)
        self.target_spin.blockSignals(False)

    def _sync_spin_to_slider(self, value: float) -> None:
        self.target_slider.blockSignals(True)
        self.target_slider.setValue(int(round(value * self.SLIDER_SCALE)))
        self.target_slider.blockSignals(False)

    def _send_preset(self, value: float) -> None:
        indices = self._selected_indices()
        if indices:
            self.backend.joint_test(indices, value)
            if self._mirror_enabled and self.mujoco_service:
                self.mujoco_service.joint_test(indices, value)

    def _set_manual_target_and_send(self, value: float) -> None:
        self.target_spin.setValue(value)
        self.on_joint_test()

    def on_go_offset(self) -> None:
        self._send_preset(0.0)

    def on_joint_test(self) -> None:
        indices = self._selected_indices()
        if indices:
            target = self.target_spin.value()
            self.backend.joint_test(indices, target)
            if self._mirror_enabled and self.mujoco_service:
                self.mujoco_service.joint_test(indices, target)

    def on_joint_sine(self) -> None:
        indices = self._selected_indices()
        if indices:
            self.backend.joint_sine(indices, self.amp_spin.value(), self.freq_spin.value(), self.dur_spin.value())
            if self._mirror_enabled and self.mujoco_service:
                self.mujoco_service.joint_sine(indices, self.amp_spin.value(), self.freq_spin.value(), self.dur_spin.value())

    def _history_push(self, pos: float, tgt: float, vel: float, err: float) -> None:
        self.pos_data.append(pos)
        self.tgt_data.append(tgt)
        self.vel_data.append(vel)
        self.err_data.append(err)
        self.pos_curve.setData(list(self.pos_data))
        self.tgt_curve.setData(list(self.tgt_data))
        self.vel_curve.setData(list(self.vel_data))
        self.err_curve.setData(list(self.err_data))

    def _value_at(self, maybe_seq, idx: int, default: float = 0.0) -> float:
        try:
            return float(maybe_seq[idx])
        except Exception:
            return default

    _STYLE_OFFLINE = (
        "QPushButton { background: #f8d7da; border: 1px solid #e74c3c; border-radius: 8px; }"
        "QPushButton:checked { background: #f5b7b1; }"
    )
    _STYLE_CHECKED = (
        "QPushButton { background: #d6ecff; border: 1px solid #3498db; border-radius: 8px; font-weight: 600; }"
    )
    _STYLE_DEFAULT = ""

    def update_state(self, state) -> None:
        self._refresh_action_enabled_state(motion_active=bool(getattr(state, "motion_active", False)))

        offline_motors = set(getattr(state, "offline_motors", []) or [])
        for idx, btn in self.joint_buttons.items():
            name = JOINT_INDEX_MAP[idx]
            if name in offline_motors:
                key, style = "offline", self._STYLE_OFFLINE
            elif btn.isChecked():
                key, style = "checked", self._STYLE_CHECKED
            else:
                key, style = "default", self._STYLE_DEFAULT
            if self._btn_style_keys.get(idx) != key:
                btn.setStyleSheet(style)
                self._btn_style_keys[idx] = key

        idx = self._selected_plot_joint
        if idx is None:
            return

        target_positions = getattr(state, "target_positions", getattr(state, "target_joint_positions", []))
        joint_positions = getattr(state, "joint_positions", [])
        joint_velocities = getattr(state, "joint_velocities", [])
        joint_torques = getattr(state, "joint_torques", [])

        pos = self._value_at(joint_positions, idx)
        tgt = self._value_at(target_positions, idx)
        vel = self._value_at(joint_velocities, idx)
        torque = self._value_at(joint_torques, idx)
        err = tgt - pos

        self._set_card("Joint", JOINT_INDEX_MAP[idx])
        self._set_card("Current (rad)", f"{pos:.3f}")
        self._set_card("Target (rad)", f"{tgt:.3f}")
        self._set_card("Error (rad)", f"{err:.3f}")
        self._set_card("Velocity (rad/s)", f"{vel:.3f}")
        self._set_card("Torque", f"{torque:.3f}")
        self._history_push(pos, tgt, vel, err)
