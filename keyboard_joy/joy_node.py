#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from pynput import keyboard
import threading

class KeyboardJoy(Node):
    def __init__(self):
        super().__init__('keyboard_joy')

        # Print key usage instructions
        self.print_usage_instructions()

        # Create a publisher for the Joy message
        self.joy_publisher = self.create_publisher(Joy, 'joy', 10)

        # Initialize Joy message with 8 axes and 12 buttons (common setup)
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8   
        self.joy_msg.buttons = [0] * 12 

        # Create a lock for thread-safe updates
        self.lock = threading.Lock()

        # Start a thread to listen to keyboard inputs
        self.listener_thread = threading.Thread(target=self.start_keyboard_listener)
        self.listener_thread.start()

        # Create a timer to publish Joy messages at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_joy)

    def print_usage_instructions(self):
        """Print instructions on how to use the keyboard for controlling the joy node."""
        self.get_logger().info("KeyboardJoy Node Started")
        self.get_logger().info("Use the following keys to simulate joystick input:")
        self.get_logger().info("  w/s: Increase/Decrease axis 1 value")
        self.get_logger().info("  d/a: Increase/Decrease axis 2 value")
        self.get_logger().info("  Arrow Up/Down: Increase/Decrease axis 3 value")
        self.get_logger().info("  Arrow Left/Right: Increase/Decrease axis 4 value")
        self.get_logger().info("  0-9: Press number keys to toggle corresponding button")

    def start_keyboard_listener(self):
        """Start listening to keyboard inputs in a separate thread."""
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

    def on_press(self, key):
        """Callback for keyboard key press events."""
        with self.lock:
            try:
                # Adjust axes based on key presses
                if key.char == 'w':
                    self.joy_msg.axes[0] = 1.0  # Increase axis 1
                elif key.char == 's':
                    self.joy_msg.axes[0] = -1.0  # Decrease axis 1
                elif key.char == 'd':
                    self.joy_msg.axes[1] = 1.0  # Increase axis 2
                elif key.char == 'a':
                    self.joy_msg.axes[1] = -1.0  # Decrease axis 2
                elif key.char in '0123456789':
                    self.joy_msg.buttons[int(key.char)] = 1  # Toggle button state
            except AttributeError:
                # Use special keys for more axes
                if key == keyboard.Key.up:
                    self.joy_msg.axes[2] = 1.0  # Increase axis 3
                elif key == keyboard.Key.down:
                    self.joy_msg.axes[2] = -1.0  # Decrease axis 3
                elif key == keyboard.Key.left:
                    self.joy_msg.axes[3] = -1.0  # Decrease axis 4
                elif key == keyboard.Key.right:
                    self.joy_msg.axes[3] = 1.0  # Increase axis 4

    def on_release(self, key):
        """Callback for keyboard key release events."""
        with self.lock:
            try:
                # Reset axes when keys are released
                if key.char in ['w', 's']:
                    self.joy_msg.axes[0] = 0.0
                elif key.char in ['a', 'd']:
                    self.joy_msg.axes[1] = 0.0
                elif key.char in '0123456789':
                    self.joy_msg.buttons[int(key.char)] = 0
            except AttributeError:
                # Reset axes for special keys
                if key in [keyboard.Key.up, keyboard.Key.down]:
                    self.joy_msg.axes[2] = 0.0
                elif key in [keyboard.Key.left, keyboard.Key.right]:
                    self.joy_msg.axes[3] = 0.0

    def publish_joy(self):
        """Publish the Joy message based on current keyboard state."""
        with self.lock:
            self.joy_msg.header.stamp = self.get_clock().now().to_msg()
            self.joy_publisher.publish(self.joy_msg)

    def destroy_node(self):
        """Ensure the listener thread is properly stopped."""
        self.listener_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    keyboard_joy = KeyboardJoy()

    try:
        rclpy.spin(keyboard_joy)
    except KeyboardInterrupt:
        pass

    keyboard_joy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
