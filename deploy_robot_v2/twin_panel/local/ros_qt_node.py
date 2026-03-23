from __future__ import annotations

from PyQt5 import QtCore

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, String
except ImportError:
    rclpy = None
    Node = object
    Float32MultiArray = None
    String = None

from twin_topics import RAW_ACTION_TOPIC, HW_CONTROL_TOPIC, HW_STATE_TOPIC, SIM_STATE_TOPIC


class TwinRosNode(QtCore.QObject):
    hw_state_received = QtCore.pyqtSignal(str)
    sim_state_received = QtCore.pyqtSignal(str)
    ros_ready = QtCore.pyqtSignal(bool, str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._spin_once)
        self._node = None
        self._raw_pub = None
        self._hw_control_pub = None

    def start(self) -> None:
        if rclpy is None:
            self.ros_ready.emit(False, 'ROS2 not available')
            return
        rclpy.init(args=None)
        self._node = Node('twin_panel')
        self._raw_pub = self._node.create_publisher(Float32MultiArray, RAW_ACTION_TOPIC, 10)
        self._hw_control_pub = self._node.create_publisher(String, HW_CONTROL_TOPIC, 10)
        self._node.create_subscription(String, HW_STATE_TOPIC, self._on_hw_state, 10)
        self._node.create_subscription(String, SIM_STATE_TOPIC, self._on_sim_state, 10)
        self._timer.start(10)
        self.ros_ready.emit(True, 'ROS2 ready')

    def shutdown(self) -> None:
        self._timer.stop()
        if self._node:
            self._node.destroy_node()
        if rclpy and rclpy.ok():
            rclpy.shutdown()

    def publish_raw_action(self, values: list) -> None:
        if self._raw_pub and rclpy and rclpy.ok():
            try:
                msg = Float32MultiArray()
                msg.data = list(values)
                self._raw_pub.publish(msg)
            except Exception:
                pass

    def send_hw_command(self, cmd: str) -> None:
        if self._hw_control_pub and rclpy and rclpy.ok():
            try:
                msg = String()
                msg.data = cmd
                self._hw_control_pub.publish(msg)
            except Exception:
                pass

    def _on_hw_state(self, msg) -> None:
        self.hw_state_received.emit(msg.data)

    def _on_sim_state(self, msg) -> None:
        self.sim_state_received.emit(msg.data)

    def _spin_once(self) -> None:
        if self._node and rclpy and rclpy.ok():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.0)
            except Exception:
                pass