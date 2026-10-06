#!/usr/bin/env python3
"""Repeated static-image input; no synthetic YOLO detections."""
import math
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class MockCamera(Node):
    def __init__(self):
        super().__init__('mock_camera_node')
        self.declare_parameter('image_path', '')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('publish_rate_hz', 5.0)
        filename = str(Path(self.get_parameter('image_path').value).expanduser())
        image = cv2.imread(filename)
        if image is None:
            raise ValueError(f'Cannot read image_path: {filename}')
        self.message = CvBridge().cv2_to_imgmsg(image, encoding='bgr8')
        self.message.header.frame_id = 'mock_camera'
        self.publisher = self.create_publisher(
            Image, self.get_parameter('image_topic').value, 2)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if not math.isfinite(rate) or not 0.0 < rate <= 50.0:
            raise ValueError('publish_rate_hz must be in (0, 50]')
        self.create_timer(1.0 / rate, self.publish_image)

    def publish_image(self):
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MockCamera()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
