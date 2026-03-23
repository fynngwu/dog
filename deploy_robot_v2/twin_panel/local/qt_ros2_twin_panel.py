#!/usr/bin/env python3
"""
Qt 数字孪生面板 - 现代化 UI 升级版

架构：三线程直连模式
- 主线程：Qt UI
- Sim worker：MuJoCo 仿真
- SSH worker：远端 CLI 调用
- ROS2：仅做观测转发（可选）
"""
from __future__ import annotations

import os
# 自动检测 Wayland 并设置 Qt 平台
if 'WAYLAND_DISPLAY' in os.environ and 'QT_QPA_PLATFORM' not in os.environ:
    os.environ['QT_QPA_PLATFORM'] = 'wayland'

import json
import sys
from typing import List

from PyQt5 import QtCore, QtWidgets, QtGui

try:
    import rclpy
except ImportError:
    rclpy = None

from hw_ssh_bridge_worker import HwSshBridgeWorker, HwSshConfig
from ros_qt_node import TwinRosNode
from sim_bridge_worker import SimBridgeWorker, SimConfig
from twin_topics import ACTION_SLIDER_MAX, ACTION_SLIDER_MIN, ACTION_VALUE_SCALE, NUM_JOINTS, JOINT_NAMES

# ----------------- 现代化 QSS 样式 -----------------
MODERN_QSS = """
QWidget {
    background-color: #1E1E1E;
    color: #D4D4D4;
    font-family: "Segoe UI", -apple-system, BlinkMacSystemFont, Roboto, Arial, sans-serif;
    font-size: 14px;
}

/* 按钮样式 */
QPushButton {
    background-color: #2D2D30;
    border: 1px solid #3E3E42;
    border-radius: 6px;
    padding: 8px 16px;
    font-weight: bold;
}
QPushButton:hover {
    background-color: #3E3E42;
    border: 1px solid #007ACC;
}
QPushButton:pressed {
    background-color: #007ACC;
    color: white;
}

/* 卡片容器 */
QFrame#Card {
    background-color: #252526;
    border: 1px solid #333337;
    border-radius: 8px;
}

/* 状态栏 */
QLabel#StatusLabel {
    font-size: 16px;
    font-weight: bold;
    padding: 10px;
    border-radius: 6px;
    background-color: #2D2D30;
}

/* 表头 */
QLabel#Header {
    font-weight: bold;
    color: #9CDCFE;
    padding-bottom: 5px;
    border-bottom: 1px solid #3E3E42;
}

/* 数值标签：使用等宽字体对齐 */
QLabel.ValueLabel {
    font-family: "Consolas", "Courier New", monospace;
    font-size: 15px;
    min-width: 70px;
}

/* 滑块样式 */
QSlider::groove:horizontal {
    border: 1px solid #3E3E42;
    height: 6px;
    background: #1E1E1E;
    border-radius: 3px;
}
QSlider::sub-page:horizontal {
    background: #007ACC;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #CCCCCC;
    border: 1px solid #555555;
    width: 16px;
    height: 16px;
    margin-top: -5px;
    margin-bottom: -5px;
    border-radius: 8px;
}
QSlider::handle:horizontal:hover {
    background: #FFFFFF;
    border: 1px solid #007ACC;
}

/* 日志框 */
QPlainTextEdit {
    background-color: #1E1E1E;
    border: 1px solid #333337;
    border-radius: 6px;
    padding: 8px;
    font-family: "Consolas", "Courier New", monospace;
    color: #CE9178;
}
"""

class TwinPanel(QtWidgets.QWidget):
    def __init__(self, ssh_target: str, mujoco_xml: str, use_ros2: bool = True) -> None:
        super().__init__()
        self.setWindowTitle('Digital Twin Control Center')
        self.resize(1100, 850)
        self.setStyleSheet(MODERN_QSS) # 应用全局样式

        self._use_ros2 = use_ros2 and rclpy is not None

        # ROS2（仅做观测转发，不参与控制链路）
        if self._use_ros2:
            self.ros = TwinRosNode(self)
            self.ros.hw_state_received.connect(self._on_hw_state)
            self.ros.sim_state_received.connect(self._on_sim_state)

        # Workers（直连，不走 ROS2）
        self.sim_worker = SimBridgeWorker(SimConfig(xml_path=mujoco_xml), self)
        self.sim_worker.status.connect(self._log)
        self.sim_worker.state_ready.connect(self._on_sim_payload)

        self.hw_worker = HwSshBridgeWorker(HwSshConfig(ssh_target=ssh_target), self)
        self.hw_worker.status.connect(self._log)
        self.hw_worker.state_ready.connect(self._on_hw_payload)

        # 状态
        self._sim_state = {}
        self._hw_state = {}
        self._sliders: List[QtWidgets.QSlider] = []
        self._value_labels: List[QtWidgets.QLabel] = []
        self._target_labels: List[QtWidgets.QLabel] = []
        self._sim_labels: List[QtWidgets.QLabel] = []
        self._hw_labels: List[QtWidgets.QLabel] = []
        self._diff_labels: List[QtWidgets.QLabel] = []

        self._build_ui()

        # 定时发布（5 Hz）
        self._publish_timer = QtCore.QTimer(self)
        self._publish_timer.timeout.connect(self._publish_action)
        self._publish_timer.start(200)

        # 启动
        if self._use_ros2:
            self.ros.start()
        self.sim_worker.start()
        self.hw_worker.start()

    def closeEvent(self, event) -> None:
        self._publish_timer.stop()
        self.hw_worker.stop()
        self.sim_worker.stop()
        self.hw_worker.wait(5000)
        self.sim_worker.wait(2000)
        if self._use_ros2:
            self.ros.shutdown()
        super().closeEvent(event)

    def _build_ui(self) -> None:
        root = QtWidgets.QVBoxLayout(self)
        root.setSpacing(15)
        root.setContentsMargins(20, 20, 20, 20)

        # 1. 顶部控制栏 (Card)
        control_frame = QtWidgets.QFrame()
        control_frame.setObjectName("Card")
        control_layout = QtWidgets.QHBoxLayout(control_frame)
        control_layout.setContentsMargins(15, 15, 15, 15)
        
        for text, cb in [
            ('🚀 Enable', lambda: self._send_hw_cmd('enable')),
            ('🎯 Go Offset', lambda: self._send_hw_cmd('goto_offset 2.0')),
            ('🛑 Disable', lambda: self._send_hw_cmd('disable')),
            ('⏸ Hold', lambda: self._send_hw_cmd('hold')),
            ('📡 Get State', lambda: self._send_hw_cmd('get_state')),
            ('🔄 Auto Report', lambda: self._send_hw_cmd('enable_auto_report')),
            ('🔕 Stop Report', lambda: self._send_hw_cmd('disable_auto_report')),
            ('⚡ Zero All', self._zero_all),
        ]:
            btn = QtWidgets.QPushButton(text)
            btn.clicked.connect(cb)
            # 给 Zero All 加个特殊颜色
            if "Zero" in text:
                btn.setStyleSheet("color: #E51400; border-color: #E51400;")
            control_layout.addWidget(btn)
        root.addWidget(control_frame)

        # 2. 状态栏
        self._status_label = QtWidgets.QLabel('⏳ System Initializing...')
        self._status_label.setObjectName("StatusLabel")
        self._status_label.setAlignment(QtCore.Qt.AlignCenter)
        root.addWidget(self._status_label)

        # 3. 数据与滑块区 (Card)
        data_frame = QtWidgets.QFrame()
        data_frame.setObjectName("Card")
        data_layout = QtWidgets.QVBoxLayout(data_frame)
        data_layout.setContentsMargins(20, 20, 20, 20)

        grid = QtWidgets.QGridLayout()
        grid.setVerticalSpacing(12)
        grid.setHorizontalSpacing(20)

        headers = ['Joint', 'Action Slider', 'Raw Value', 'Target Command', 'Sim (Virtual)', 'HW (Real)', 'Diff (HW-Sim)']
        for c, h in enumerate(headers):
            lbl = QtWidgets.QLabel(h)
            lbl.setObjectName("Header")
            lbl.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
            grid.addWidget(lbl, 0, c)

        for i, name in enumerate(JOINT_NAMES):
            # 关节名
            name_lbl = QtWidgets.QLabel(f'<b>{name}</b>')
            name_lbl.setStyleSheet("color: #4EC9B0;")
            grid.addWidget(name_lbl, i + 1, 0)

            # 滑块
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(ACTION_SLIDER_MIN, ACTION_SLIDER_MAX)
            slider.setValue(0)
            slider.setMinimumWidth(250)
            slider.valueChanged.connect(self._refresh_labels)
            self._sliders.append(slider)
            grid.addWidget(slider, i + 1, 1)

            # 数值标签初始化
            labels_group = [self._value_labels, self._target_labels, self._sim_labels, self._hw_labels, self._diff_labels]
            for labels in labels_group:
                lbl = QtWidgets.QLabel('0.0000')
                lbl.setProperty("class", "ValueLabel") # 应用等宽字体样式
                labels.append(lbl)

            grid.addWidget(self._value_labels[i], i + 1, 2)
            grid.addWidget(self._target_labels[i], i + 1, 3)
            grid.addWidget(self._sim_labels[i], i + 1, 4)
            grid.addWidget(self._hw_labels[i], i + 1, 5)
            grid.addWidget(self._diff_labels[i], i + 1, 6)

        data_layout.addLayout(grid)
        root.addWidget(data_frame)

        # 4. 日志区
        self.log_box = QtWidgets.QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumHeight(120)
        self.log_box.setPlaceholderText("System logs will appear here...")
        root.addWidget(self.log_box)

    def _log(self, text: str) -> None:
        self.log_box.appendPlainText(f"[LOG] {text}")
        self.log_box.verticalScrollBar().setValue(self.log_box.verticalScrollBar().maximum())

    def _current_action(self) -> list:
        return [s.value() / ACTION_VALUE_SCALE for s in self._sliders]

    def _publish_action(self) -> None:
        action = self._current_action()
        self.sim_worker.set_action(action)
        self.hw_worker.set_action(action)
        # ROS2 仅做观测转发，不参与控制链路
        self._render_table()

    def _send_hw_cmd(self, cmd: str) -> None:
        self._log(f'> {cmd}')
        self.hw_worker.send_command(cmd)
        # ROS2 仅做观测转发，不参与控制链路

    def _zero_all(self) -> None:
        for s in self._sliders:
            s.blockSignals(True)
            s.setValue(0)
            s.blockSignals(False)
        self._refresh_labels()
        self._publish_action()

    def _refresh_labels(self) -> None:
        for s, lbl in zip(self._sliders, self._value_labels):
            val = s.value() / ACTION_VALUE_SCALE
            lbl.setText(f'{val:+.4f}')
            # 当前拉动的值给个高亮反馈
            lbl.setStyleSheet("color: #DCDCAA;" if val != 0 else "color: #D4D4D4;")

    def _on_sim_payload(self, payload: dict) -> None:
        self._sim_state = payload
        self._render_table()

    def _on_hw_payload(self, payload: dict) -> None:
        self._hw_state = payload
        self._update_status(payload)
        self._render_table()

    def _on_hw_state(self, text: str) -> None:
        try:
            self._hw_state = json.loads(text)
        except: pass
        self._render_table()

    def _on_sim_state(self, text: str) -> None:
        try:
            self._sim_state = json.loads(text)
        except: pass
        self._render_table()

    def _update_status(self, payload: dict) -> None:
        ok = payload.get('ok', False)
        mode = payload.get('mode', '')
        rtt = payload.get('rtt_ms', 0)
        error = payload.get('error', '')
        fresh = payload.get('sample_fresh', True)
        online = payload.get('motors_online_count', 0)
        
        if ok and fresh:
            status = f'🟢 {mode.upper()} | RTT: {rtt}ms | Online: {online}/{NUM_JOINTS}'
            self._status_label.setStyleSheet('background-color: #1B2A1E; color: #4EC9B0; border: 1px solid #4EC9B0;')
        elif ok and not fresh:
            status = f'🟡 {mode.upper()} | RTT: {rtt}ms | Stale (age: {payload.get("max_feedback_age_ms", "?")}ms)'
            self._status_label.setStyleSheet('background-color: #2D2816; color: #DCDCAA; border: 1px solid #DCDCAA;')
        else:
            status = f'🔴 {mode.upper()} | Error: {error}'
            self._status_label.setStyleSheet('background-color: #331A1A; color: #F44336; border: 1px solid #F44336;')
        
        self._status_label.setText(status)

    def _arr(self, state: dict, key: str) -> list:
        v = state.get(key, [])
        return v[:NUM_JOINTS] if isinstance(v, list) and len(v) >= NUM_JOINTS else [0.0] * NUM_JOINTS

    def _render_table(self) -> None:
        target = self._arr(self._sim_state or self._hw_state, 'target_joint_rel')
        sim = self._arr(self._sim_state, 'sim_joint_rel')
        hw = self._arr(self._hw_state, 'hw_joint_rel')
        
        for i in range(NUM_JOINTS):
            self._target_labels[i].setText(f'{target[i]:+.4f}')
            self._sim_labels[i].setText(f'{sim[i]:+.4f}')
            self._hw_labels[i].setText(f'{hw[i]:+.4f}')
            
            # 动态误差高亮
            diff = hw[i] - sim[i]
            self._diff_labels[i].setText(f'{diff:+.4f}')
            
            # 误差判断：如果偏差绝对值大于 0.05 亮红色警告，小于0.01亮绿色，其余默认
            if abs(diff) > 0.05:
                self._diff_labels[i].setStyleSheet("color: #F44336;") # 红
            elif abs(diff) < 0.01 and diff != 0.0:
                self._diff_labels[i].setStyleSheet("color: #4EC9B0;") # 绿
            else:
                self._diff_labels[i].setStyleSheet("color: #D4D4D4;") # 默认灰

def main() -> int:
    import argparse
    parser = argparse.ArgumentParser(description='Digital Twin Panel')
    parser.add_argument('--ssh', default='robot@192.168.123.10', help='SSH target')
    parser.add_argument('--xml', default='/home/wufy/git_resp/dog/leggedrobot_flat.xml', help='MuJoCo XML path')
    parser.add_argument('--no-ros2', action='store_true', help='Disable ROS2')
    args = parser.parse_args()

    # 高 DPI 支持必须在 QApplication 创建之前设置
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)

    app = QtWidgets.QApplication(sys.argv)

    win = TwinPanel(ssh_target=args.ssh, mujoco_xml=args.xml, use_ros2=not args.no_ros2)
    win.show()
    return app.exec_()


if __name__ == '__main__':
    raise SystemExit(main())