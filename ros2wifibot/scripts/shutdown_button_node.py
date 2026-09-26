#!/usr/bin/env python3
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class ShutdownButtonNode(Node):
    def __init__(self):
        super().__init__('shutdown_button_node')

        self.declare_parameter('button_index', 8)      # bouton "Connect" par exemple
        self.declare_parameter('hold_duration_sec', 3.0)

        self.button_index = self.get_parameter('button_index').value
        self.hold_duration = self.get_parameter('hold_duration_sec').value

        self.press_start_time = None
        self.triggered = False

        self.sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.get_logger().info(
            f'Shutdown button node: button={self.button_index}, hold={self.hold_duration}s')

    def joy_cb(self, msg: Joy):
        if self.triggered:
            return

        pressed = (len(msg.buttons) > self.button_index and
                   msg.buttons[self.button_index] == 1)

        now = self.get_clock().now().nanoseconds / 1e9

        if pressed:
            if self.press_start_time is None:
                self.press_start_time = now
            elif (now - self.press_start_time) >= self.hold_duration:
                self.triggered = True
                self.get_logger().warn('Shutdown triggered by button hold!')
                subprocess.run(['sudo', 'shutdown', '-h', 'now'])
        else:
            self.press_start_time = None


def main():
    rclpy.init()
    node = ShutdownButtonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()