#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float64, Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray


class DebugVisualizer(Node):
    def __init__(self):
        super().__init__('debug_visualizer_node')
        self.declare_parameter('vehicle_half_width', 0.6)
        self.declare_parameter('vehicle_front', 1.6)
        self.declare_parameter('vehicle_rear', 0.7)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.half_width = float(self.get_parameter('vehicle_half_width').value)
        self.front = float(self.get_parameter('vehicle_front').value)
        self.rear = float(self.get_parameter('vehicle_rear').value)
        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointStamped, '/Local/utm', self.position_cb, qos)
        self.create_subscription(Float64, '/Local/heading', self.heading_cb, qos)
        self.create_subscription(String, '/Planning/behavior', self.behavior_cb, qos)
        self.create_subscription(
            Float64MultiArray, '/LiDAR/object_cen', self.relative_obstacles_cb, qos)
        self.create_subscription(
            Float64MultiArray, '/Convert/small_object_UTM',
            lambda msg: self.utm_obstacles_cb(msg, 'small', (1.0, 0.4, 0.0)), qos)
        self.create_subscription(
            Float64MultiArray, '/Convert/big_object_UTM',
            lambda msg: self.utm_obstacles_cb(msg, 'big', (0.8, 0.0, 1.0)), qos)

        self.ego_pub = self.create_publisher(MarkerArray, '/Planning/debug/ego', 2)
        self.behavior_pub = self.create_publisher(Marker, '/Visualization/behavior', 2)
        self.relative_pub = self.create_publisher(
            MarkerArray, '/Visualization/lidar_relative_objects', 2)
        self.utm_pub = self.create_publisher(MarkerArray, '/Visualization/obstacles_utm', 2)
        self.position = None
        self.heading = None
        self.behavior = 'UNKNOWN'
        self.utm_groups = {'small': [], 'big': []}
        self.create_timer(1.0 / rate, self.publish_vehicle_debug)

    @staticmethod
    def pairs(data):
        points = []
        for index in range(0, len(data) - 1, 2):
            if data[index] <= -999.0:
                break
            points.append((float(data[index]), float(data[index + 1])))
        return points

    def position_cb(self, message):
        self.position = (message.point.x, message.point.y)

    def heading_cb(self, message):
        self.heading = message.data

    def behavior_cb(self, message):
        self.behavior = message.data

    def relative_obstacles_cb(self, message):
        markers = self.obstacle_markers(
            self.pairs(message.data), 'velodyne', 'relative_small', (1.0, 0.3, 0.0))
        self.relative_pub.publish(markers)

    def utm_obstacles_cb(self, message, namespace, color):
        self.utm_groups[namespace] = self.pairs(message.data)
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        marker_id = 0
        for group, points in self.utm_groups.items():
            group_color = color if group == namespace else ((1.0, 0.4, 0.0) if group == 'small' else (0.8, 0.0, 1.0))
            generated = self.obstacle_markers(
                points, 'map', 'utm_' + group, group_color, marker_id, clear_first=False)
            markers.markers.extend(generated.markers)
            marker_id += len(generated.markers)
        self.utm_pub.publish(markers)

    def obstacle_markers(self, points, frame, namespace, color, start_id=0,
                         clear_first=True):
        result = MarkerArray()
        if clear_first:
            clear = Marker()
            clear.header.frame_id = frame
            clear.action = Marker.DELETEALL
            result.markers.append(clear)
        for index, (x, y) in enumerate(points):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = frame
            marker.ns = namespace
            marker.id = start_id + index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.45
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 0.9
            result.markers.append(marker)
        return result

    def publish_vehicle_debug(self):
        if self.position is None or self.heading is None:
            return
        stamp = self.get_clock().now().to_msg()
        x, y = self.position
        yaw = self.heading
        array = MarkerArray()

        footprint = Marker()
        footprint.header.stamp = stamp
        footprint.header.frame_id = 'map'
        footprint.ns = 'ego'
        footprint.id = 0
        footprint.type = Marker.CUBE
        footprint.action = Marker.ADD
        centre_offset = (self.front - self.rear) * 0.5
        footprint.pose.position.x = x + centre_offset * math.cos(yaw)
        footprint.pose.position.y = y + centre_offset * math.sin(yaw)
        footprint.pose.position.z = 0.15
        footprint.pose.orientation.z = math.sin(yaw * 0.5)
        footprint.pose.orientation.w = math.cos(yaw * 0.5)
        footprint.scale.x = self.front + self.rear
        footprint.scale.y = self.half_width * 2.0
        footprint.scale.z = 0.3
        footprint.color.b = 1.0
        footprint.color.a = 0.45
        array.markers.append(footprint)

        arrow = Marker()
        arrow.header = footprint.header
        arrow.ns = 'ego'
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x = 0.12
        arrow.scale.y = 0.22
        arrow.scale.z = 0.25
        arrow.color.g = 1.0
        arrow.color.a = 1.0
        start = Point(x=x, y=y, z=0.4)
        end = Point(x=x + 2.0 * math.cos(yaw), y=y + 2.0 * math.sin(yaw), z=0.4)
        arrow.points = [start, end]
        array.markers.append(arrow)
        self.ego_pub.publish(array)

        text = Marker()
        text.header = footprint.header
        text.ns = 'behavior'
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = 1.5
        text.pose.orientation.w = 1.0
        text.scale.z = 0.55
        text.color.r = 1.0 if self.behavior == 'EMERGENCY_STOP' else 0.1
        text.color.g = 0.2 if self.behavior == 'EMERGENCY_STOP' else 1.0
        text.color.b = 0.1
        text.color.a = 1.0
        text.text = self.behavior
        self.behavior_pub.publish(text)


def main(args=None):
    rclpy.init(args=args)
    node = DebugVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
