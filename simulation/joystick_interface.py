"""
Joystick Interface for Legged Robot Control
Reads from Linux joystick device (/dev/input/jsX) and converts to velocity commands.
"""

import struct
import threading
import time
import os


class JoystickInterface:
    """
    Linux joystick interface that reads from /dev/input/jsX device.
    
    Maps joystick axes to velocity commands:
    - Left stick X: lateral velocity (y)
    - Left stick Y: forward velocity (x)
    - Right stick X: yaw rate (omega)
    """
    
    # Linux joystick event format: timestamp (uint32), value (int16), type (uint8), number (uint8)
    JS_EVENT_FORMAT = "IhBB"
    JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
    
    # Event types
    JS_EVENT_BUTTON = 0x01
    JS_EVENT_AXIS = 0x02
    JS_EVENT_INIT = 0x80
    
    def __init__(self, device_path="/dev/input/js0", max_v_x=2.0, max_v_y=1.0, max_omega=1.5):
        """
        Initialize joystick interface.
        
        Args:
            device_path: Path to joystick device (default: /dev/input/js0)
            max_v_x: Maximum forward velocity (m/s)
            max_v_y: Maximum lateral velocity (m/s)
            max_omega: Maximum yaw rate (rad/s)
        """
        self.device_path = device_path
        self.max_v_x = max_v_x
        self.max_v_y = max_v_y
        self.max_omega = max_omega
        
        # Command values
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_yaw = 0.0
        
        # Joystick state (axes: -32768 to 32767)
        self.axis_state = [0] * 8  # Support up to 8 axes
        self.button_state = [0] * 12  # Support up to 12 buttons
        
        # Running flag
        self.running = False
        self.joystick_file = None
        self.thread = None
        
        # Deadzone for joystick (ignore small values)
        self.deadzone = 3000  # Out of 32767
        
        # Start reading thread
        self._start()
    
    def _start(self):
        """Start the joystick reading thread."""
        if not os.path.exists(self.device_path):
            print(f"[Warning] Joystick device not found: {self.device_path}")
            print("[Warning] Running in keyboard simulation mode (WASD + Q/E)")
            self.running = True
            self._start_keyboard_thread()
            return
        
        try:
            self.joystick_file = open(self.device_path, "rb")
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print(f"[Joystick] Connected to {self.device_path}")
        except PermissionError:
            print(f"[Error] Permission denied for {self.device_path}")
            print("[Warning] Running in keyboard simulation mode (WASD + Q/E)")
            self.running = True
            self._start_keyboard_thread()
        except Exception as e:
            print(f"[Error] Failed to open joystick: {e}")
            print("[Warning] Running in keyboard simulation mode (WASD + Q/E)")
            self.running = True
            self._start_keyboard_thread()
    
    def _start_keyboard_thread(self):
        """Start keyboard input thread as fallback using stdin."""
        self.keys_pressed = set()
        print("[Info] Keyboard mode: Use terminal to input commands")
        print("[Info] Commands will default to zero (no input)")
    
    def _read_loop(self):
        """Main loop to read joystick events."""
        while self.running:
            try:
                # Read event
                event_data = self.joystick_file.read(self.JS_EVENT_SIZE)
                if len(event_data) < self.JS_EVENT_SIZE:
                    continue
                
                # Parse event
                timestamp, value, event_type, number = struct.unpack(self.JS_EVENT_FORMAT, event_data)
                
                # Ignore init events
                if event_type & self.JS_EVENT_INIT:
                    continue
                
                # Handle axis motion
                if event_type == self.JS_EVENT_AXIS and number < len(self.axis_state):
                    self.axis_state[number] = value
                
                # Handle button press
                elif event_type == self.JS_EVENT_BUTTON and number < len(self.button_state):
                    self.button_state[number] = value
                
                # Update commands based on axis state
                self._update_commands()
                
            except Exception as e:
                if self.running:
                    print(f"[Joystick] Read error: {e}")
                break
    
    def _update_commands(self):
        """Convert joystick axis values to velocity commands."""
        # Standard Xbox/PS controller mapping:
        # Axis 0: Left stick X (negative = left)
        # Axis 1: Left stick Y (negative = forward)
        # Axis 3: Right stick X (negative = left)
        
        # Left stick Y -> forward velocity (inverted)
        raw_x = -self.axis_state[1]
        # Left stick X -> lateral velocity
        raw_y = self.axis_state[0]
        # Right stick X -> yaw rate
        raw_yaw = -self.axis_state[3]
        
        # Apply deadzone
        if abs(raw_x) < self.deadzone:
            raw_x = 0
        if abs(raw_y) < self.deadzone:
            raw_y = 0
        if abs(raw_yaw) < self.deadzone:
            raw_yaw = 0
        
        # Normalize to [-1, 1] and scale
        self.cmd_x = (raw_x / 32767.0) * self.max_v_x
        self.cmd_y = (raw_y / 32767.0) * self.max_v_y
        self.cmd_yaw = (raw_yaw / 32767.0) * self.max_omega
    
    def _update_keyboard_commands(self):
        """Update commands from keyboard input."""
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_yaw = 0.0
        
        if hasattr(self, 'keys_pressed'):
            if 'w' in self.keys_pressed:
                self.cmd_x = self.max_v_x
            if 's' in self.keys_pressed:
                self.cmd_x = -self.max_v_x
            if 'a' in self.keys_pressed:
                self.cmd_y = self.max_v_y
            if 'd' in self.keys_pressed:
                self.cmd_y = -self.max_v_y
            if 'q' in self.keys_pressed:
                self.cmd_yaw = self.max_omega
            if 'e' in self.keys_pressed:
                self.cmd_yaw = -self.max_omega
    
    def get_command(self):
        """
        Get current velocity command.
        
        Returns:
            tuple: (cmd_x, cmd_y, cmd_yaw)
                - cmd_x: Forward velocity (m/s)
                - cmd_y: Lateral velocity (m/s)
                - cmd_yaw: Yaw rate (rad/s)
        """
        # If using keyboard fallback
        if self.joystick_file is None:
            self._update_keyboard_commands()
        
        return self.cmd_x, self.cmd_y, self.cmd_yaw
    
    def stop(self):
        """Stop the joystick interface."""
        self.running = False
        if self.joystick_file:
            self.joystick_file.close()
        if hasattr(self, 'keyboard_listener'):
            self.keyboard_listener.stop()
        print("[Joystick] Stopped")


# Simple test
if __name__ == "__main__":
    joy = JoystickInterface()
    
    print("Testing joystick interface...")
    print("Press Ctrl+C to exit")
    
    try:
        while True:
            cmd_x, cmd_y, cmd_yaw = joy.get_command()
            print(f"\rcmd_x: {cmd_x:6.2f}  cmd_y: {cmd_y:6.2f}  cmd_yaw: {cmd_yaw:6.2f}", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        joy.stop()
        print()