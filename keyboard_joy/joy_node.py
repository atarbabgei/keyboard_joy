#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from pynput import keyboard
import threading
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class KeyboardJoy(Node):
    def __init__(self):
        super().__init__('keyboard_joy')

        # Declare a parameter for the configuration file path
        self.declare_parameter('config', '')

        # Load key mappings from YAML file
        self.load_key_mappings()

        # Print a message to indicate that the node has started
        self.get_logger().info("KeyboardJoy Node Started")

        # Print the loaded key mappings
        self.get_logger().info(f"Loaded axis mappings: {self.axis_mappings}")
        self.get_logger().info(f"Loaded button mappings: {self.button_mappings}")
        
        # Create a publisher for the Joy message
        self.joy_publisher = self.create_publisher(Joy, 'joy', 10)

        # Initialize Joy message with the correct number of axes and buttons
        max_axis_index = max(self.axis_mappings.values(), key=lambda x: x[0])[0] if self.axis_mappings else 0
        max_button_index = max(self.button_mappings.values()) if self.button_mappings else 0
        
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * (max_axis_index + 1)
        self.joy_msg.buttons = [0] * (max_button_index + 1)

        # Create a lock for thread-safe updates
        self.lock = threading.Lock()

        # Start a thread to listen to keyboard inputs
        self.listener_thread = threading.Thread(target=self.start_keyboard_listener)
        self.listener_thread.start()

        # Create a timer to publish Joy messages at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_joy)

    def load_key_mappings(self):
        """Load key mappings from a YAML file."""
        # Get the config parameter
        config_file_path = self.get_parameter('config').get_parameter_value().string_value

        if not config_file_path:
            # Use default file path if no parameter provided
            config_file_path = os.path.join(get_package_share_directory('keyboard_joy'), 'config', 'key_mappings.yaml')

        # Load the YAML file
        try:
            with open(config_file_path, 'r') as file:
                key_mappings = yaml.safe_load(file)
        except FileNotFoundError:
            self.get_logger().error(f"Configuration file not found: {config_file_path}")
            key_mappings = {}

        # Extract axes and buttons mappings from the loaded YAML file
        self.axis_mappings = key_mappings.get('axes', {})
        self.button_mappings = key_mappings.get('buttons', {})

    def start_keyboard_listener(self):
        """Start listening to keyboard inputs in a separate thread."""
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

    def on_press(self, key):
        """Callback for keyboard key press events."""
        with self.lock:
            key_str = self.key_to_string(key)
            if key_str in self.axis_mappings:
                axis, value = self.axis_mappings[key_str]
                self.joy_msg.axes[axis] = value
            elif key_str in self.button_mappings:
                button_index = self.button_mappings[key_str]
                self.joy_msg.buttons[button_index] = 1

    def on_release(self, key):
        """Callback for keyboard key release events."""
        with self.lock:
            key_str = self.key_to_string(key)
            if key_str in self.axis_mappings:
                axis, _ = self.axis_mappings[key_str]
                self.joy_msg.axes[axis] = 0.0
            elif key_str in self.button_mappings:
                button_index = self.button_mappings[key_str]
                self.joy_msg.buttons[button_index] = 0

    def key_to_string(self, key):
        """Convert keyboard key to string for parameter lookup."""
        if hasattr(key, 'char') and key.char is not None:
            return key.char
        elif hasattr(key, 'name') and key.name is not None:
            return f'Key.{key.name}'
        else:
            # Fallback to the string representation if all else fails
            return str(key)

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
