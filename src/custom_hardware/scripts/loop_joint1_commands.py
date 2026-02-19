#!/usr/bin/env python3
"""
Publish a simple oscillation on joint 1 to /esp32/joint_commands.

Usage:
  python3 loop_joint1_commands.py            # defaults
  python3 loop_joint1_commands.py --rate 1.0 --low 60 --high 120

:author: Jamal
:date: February 2026
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class Joint1Looper(Node):
    def __init__(self, low: float, high: float, rate_hz: float):
        super().__init__('joint1_looper')
        self.publisher = self.create_publisher(
            Float64MultiArray, '/esp32/joint_commands', 10
        )
        self.low = low
        self.high = high
        self.rate_hz = rate_hz
        self.state = False
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_cmd)

    def publish_cmd(self):
        angle = self.high if self.state else self.low
        self.state = not self.state
        msg = Float64MultiArray()
        # 6 joints expected; keep others at 90 (neutral)
        msg.data = [angle, 90.0, 90.0, 90.0, 90.0, 90.0]
        self.publisher.publish(msg)
        self.get_logger().info(f'Published joint1={angle}')


def parse_args():
    parser = argparse.ArgumentParser(description='Loop joint 1 commands')
    parser.add_argument('--low', type=float, default=60.0,
                        help='Low angle for joint 1 (degrees)')
    parser.add_argument('--high', type=float, default=120.0,
                        help='High angle for joint 1 (degrees)')
    parser.add_argument('--rate', type=float, default=1.0,
                        help='Publish rate in Hz')
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Joint1Looper(args.low, args.high, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
