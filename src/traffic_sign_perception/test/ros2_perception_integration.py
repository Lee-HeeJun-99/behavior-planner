#!/usr/bin/env python3
"""Weight-free ROS 2 smoke probe for raw zero detection and debug-image output."""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16


class Probe(Node):
    def __init__(self):
        super().__init__('speed_sign_perception_probe')
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.create_subscription(Int16, '/Perception/speed_limit', self._speed_callback, 10)
        self.create_subscription(
            Image, '/Perception/speed_sign/debug_image', self._debug_callback, 10)
        self.speed = None
        self.debug_encoding = None
        self.create_timer(0.1, self._publish_image)

    def _publish_image(self):
        message = Image()
        message.height = 2
        message.width = 2
        message.encoding = 'bgr8'
        message.step = 6
        message.data = [0] * 12
        self.image_publisher.publish(message)

    def _speed_callback(self, message):
        self.speed = message.data

    def _debug_callback(self, message):
        self.debug_encoding = message.encoding


def main():
    rclpy.init()
    probe = Probe()
    deadline = time.monotonic() + 5.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.1)
        if probe.speed is not None and probe.debug_encoding is not None:
            break
    print(f'speed_limit={probe.speed}')
    print(f'debug_encoding={probe.debug_encoding}')
    success = probe.speed == 0 and probe.debug_encoding == 'bgr8'
    probe.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
