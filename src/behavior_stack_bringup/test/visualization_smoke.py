#!/usr/bin/env python3
"""Mock-only smoke test for debug_visualizer_node; no vehicle interface is used."""
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray


class Probe(Node):
    def __init__(self):
        super().__init__('visualization_smoke_probe')
        self.position_pub = self.create_publisher(PointStamped, '/Local/utm', 10)
        self.heading_pub = self.create_publisher(Float64, '/Local/heading', 10)
        self.behavior_pub = self.create_publisher(String, '/Planning/behavior', 10)
        self.relative_pub = self.create_publisher(
            Float64MultiArray, '/LiDAR/object_cen', 10)
        self.small_pub = self.create_publisher(
            Float64MultiArray, '/Convert/small_object_UTM', 10)
        self.big_pub = self.create_publisher(
            Float64MultiArray, '/Convert/big_object_UTM', 10)
        self.ego_count = 0
        self.relative_count = 0
        self.utm_count = 0
        self.behavior = ''
        self.create_subscription(
            MarkerArray, '/Planning/debug/ego', self.ego_cb, 10)
        self.create_subscription(
            MarkerArray, '/Visualization/lidar_relative_objects',
            self.relative_cb, 10)
        self.create_subscription(
            MarkerArray, '/Visualization/obstacles_utm', self.utm_cb, 10)
        self.create_subscription(
            Marker, '/Visualization/behavior', self.behavior_cb, 10)
        self.create_timer(0.1, self.publish_mock)

    def publish_mock(self):
        position = PointStamped()
        position.header.frame_id = 'map'
        position.point.x = 300000.0
        position.point.y = 4100000.0
        self.position_pub.publish(position)
        self.heading_pub.publish(Float64(data=0.5))
        self.behavior_pub.publish(String(data='AVOID'))
        self.relative_pub.publish(Float64MultiArray(data=[4.0, 0.5, -1000.0]))
        self.small_pub.publish(Float64MultiArray(data=[300004.0, 4100000.5]))
        self.big_pub.publish(Float64MultiArray(data=[300008.0, 4100001.0]))

    def ego_cb(self, message):
        self.ego_count = len(message.markers)

    def relative_cb(self, message):
        self.relative_count = sum(marker.type == Marker.SPHERE for marker in message.markers)

    def utm_cb(self, message):
        self.utm_count = sum(marker.type == Marker.SPHERE for marker in message.markers)

    def behavior_cb(self, message):
        self.behavior = message.text


def main():
    rclpy.init()
    probe = Probe()
    deadline = time.monotonic() + 5.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.1)
        if (probe.ego_count >= 2 and probe.relative_count == 1 and
                probe.utm_count == 2 and probe.behavior == 'AVOID'):
            break
    success = (probe.ego_count >= 2 and probe.relative_count == 1 and
               probe.utm_count == 2 and probe.behavior == 'AVOID')
    print(f'ego_markers={probe.ego_count}')
    print(f'relative_obstacles={probe.relative_count}')
    print(f'utm_obstacles={probe.utm_count}')
    print(f'behavior_marker={probe.behavior}')
    probe.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
