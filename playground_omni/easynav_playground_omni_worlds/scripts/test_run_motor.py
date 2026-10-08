#!/usr/bin/env python3
# Copyright (c) 2025 Yohan Prakoso
# SPDX-License-Identifier: MIT

import signal
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class WheelCommandPublisher(Node):

    def __init__(self):
        super().__init__('wheel_command_publisher')
        self.shutdown_flag = False

        # Create publishers for each wheel
        self.wheel1_pub = self.create_publisher(
            Float64MultiArray, 'wheel1_controller/commands', 10)
        self.wheel2_pub = self.create_publisher(
            Float64MultiArray, 'wheel2_controller/commands', 10)
        self.wheel3_pub = self.create_publisher(
            Float64MultiArray, 'wheel3_controller/commands', 10)

        signal.signal(signal.SIGINT, self.handle_shutdown)

    def publish_commands(self, w1, w2, w3):
        if self.shutdown_flag:
            return

        msg1 = Float64MultiArray()
        msg2 = Float64MultiArray()
        msg3 = Float64MultiArray()

        # Example commands (adjust as needed)
        msg1.data = [float(w1)]  # Wheel 1 speed
        msg2.data = [float(w2)]  # Wheel 2 speed
        msg3.data = [float(w3)]  # Wheel 3 speed

        self.wheel1_pub.publish(msg1)
        self.wheel2_pub.publish(msg2)
        self.wheel3_pub.publish(msg3)

        self.get_logger().info(f'Published: W1={msg1.data}, W2={msg2.data}, W3={msg3.data}')

    def handle_shutdown(self, sig, frame):
        self.publish_commands(0, 0, 0)
        self.shutdown_flag = True
        time.sleep(1)

        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WheelCommandPublisher()
    speed = 5.0
    while rclpy.ok():
        node.publish_commands(speed, speed, speed)
        time.sleep(2)
        node.publish_commands(-speed, -speed, -speed)
        time.sleep(2)

    print('End...')


if __name__ == '__main__':
    main()
