"""
ROS2 Interface for Legged Robot Control
Subscribes to geometry_msgs/Twist on cmd_vel topic.
"""

import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class ROS2Interface:
    """
    ROS2 interface that subscribes to cmd_vel topic.

    Maps Twist message to velocity commands:
    - linear.x: forward velocity
    - linear.y: lateral velocity
    - angular.z: yaw rate
    """

    def __init__(self, max_v_x=2.0, max_v_y=1.0, max_omega=1.5):
        """
        Initialize ROS2 interface.

        Args:
            max_v_x: Maximum forward velocity (m/s)
            max_v_y: Maximum lateral velocity (m/s)
            max_omega: Maximum yaw rate (rad/s)
        """
        self.max_v_x = max_v_x
        self.max_v_y = max_v_y
        self.max_omega = max_omega

        # Command values
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_yaw = 0.0

        # Lock for thread safety
        self.lock = threading.Lock()

        # ROS2 node
        self.node = None
        self.spin_thread = None
        self.running = False

        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS2 not available. Please install rclpy and geometry_msgs.")

        self._start()

    def _start(self):
        """Start ROS2 node and subscriber."""
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()

        # Create node
        self.node = Node('dog_cmd_vel_subscriber')

        # Subscribe to cmd_vel
        self.subscription = self.node.create_subscription(
            Twist,
            'cmd_vel',
            self._cmd_vel_callback,
            10
        )

        self.running = True

        # Start spinning in a separate thread
        self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.spin_thread.start()

        print(f"[ROS2] Subscribed to /cmd_vel topic")
        print(f"[ROS2] Max velocity: x={self.max_v_x}, y={self.max_v_y}, yaw={self.max_omega}")

    def _spin_loop(self):
        """Spin ROS2 node in a loop."""
        while self.running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _cmd_vel_callback(self, msg: Twist):
        """Callback for cmd_vel messages."""
        with self.lock:
            # Apply limits
            self.cmd_x = max(-self.max_v_x, min(self.max_v_x, msg.linear.x))
            self.cmd_y = max(-self.max_v_y, min(self.max_v_y, msg.linear.y))
            self.cmd_yaw = max(-self.max_omega, min(self.max_omega, msg.angular.z))

    def get_command(self):
        """
        Get current velocity command.

        Returns:
            tuple: (cmd_x, cmd_y, cmd_yaw)
                - cmd_x: Forward velocity (m/s)
                - cmd_y: Lateral velocity (m/s)
                - cmd_yaw: Yaw rate (rad/s)
        """
        with self.lock:
            return self.cmd_x, self.cmd_y, self.cmd_yaw

    def stop(self):
        """Stop the ROS2 interface."""
        self.running = False
        if self.node:
            self.node.destroy_node()
        print("[ROS2] Stopped")


# Simple test
if __name__ == "__main__":
    if not ROS2_AVAILABLE:
        print("ROS2 not available. Please install rclpy and geometry_msgs.")
        exit(1)

    ros2_interface = ROS2Interface()

    print("Testing ROS2 interface...")
    print("Publish Twist messages to /cmd_vel topic")
    print("Press Ctrl+C to exit")

    try:
        while True:
            cmd_x, cmd_y, cmd_yaw = ros2_interface.get_command()
            print(f"\rcmd_x: {cmd_x:6.2f}  cmd_y: {cmd_y:6.2f}  cmd_yaw: {cmd_yaw:6.2f}", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ros2_interface.stop()
        print()