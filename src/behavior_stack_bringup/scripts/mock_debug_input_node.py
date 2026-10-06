#!/usr/bin/env python3
"""Map-based visualization inputs. Never starts sensor/control/vehicle nodes."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import Bool, Float64, Float64MultiArray, Int16
from tf2_ros import StaticTransformBroadcaster
import yaml
from planning_test_scenarios import load_path, offset_obstacles


SCENARIOS = ('cruise', 'avoid_center', 'avoid_left', 'avoid_right',
             'blocked', 'emergency', 'speed_20', 'speed_50')


class MockDebugInput(Node):
    def __init__(self):
        super().__init__('mock_debug_input_node')
        self.declare_parameter('scenario', 'cruise')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('base_index', 0)
        self.declare_parameter('obstacle_distance', 8.0)
        self.declare_parameter('global_path_file', '')
        self.declare_parameter('planning_config', '')
        self.declare_parameter('publish_mock_tf', True)
        share = Path(get_package_share_directory('planning_pkg_2025'))
        config_path = self.get_parameter('planning_config').value or str(
            share / 'config/planning.yaml')
        with open(config_path, encoding='utf-8') as stream:
            config = yaml.safe_load(stream)['planning_node']['ros__parameters']
        filename = self.get_parameter('global_path_file').value or config['global_path_file']
        path_file = Path(filename)
        if not path_file.is_absolute():
            path_file = share / path_file
        self.path = load_path(path_file)
        self.base_index = int(self.get_parameter('base_index').value)
        if not 0 <= self.base_index < len(self.path):
            raise ValueError('base_index is outside the loaded global path')
        self.vehicle = self.path[self.base_index][:3]
        self.obstacle_index = self.base_index
        distance = 0.0
        desired = float(self.get_parameter('obstacle_distance').value)
        if not math.isfinite(desired) or desired <= 0.0:
            raise ValueError('obstacle_distance must be positive and finite')
        while self.obstacle_index + 1 < len(self.path) and distance < desired:
            first, second = self.path[self.obstacle_index:self.obstacle_index + 2]
            distance += math.hypot(second[0] - first[0], second[1] - first[1])
            self.obstacle_index += 1
        self.blocked_offsets = [value * 0.5 for value in range(
            -int(math.ceil(float(config['road_right_bound']) * 2)),
            int(math.ceil(float(config['road_left_bound']) * 2)) + 1)]
        self.last_scenario = None
        self.output_pubs = {
            'position': self.create_publisher(PointStamped, '/Local/utm', 10),
            'heading': self.create_publisher(Float64, '/Local/heading', 10),
            'relative': self.create_publisher(Float64MultiArray, '/LiDAR/object_cen', 10),
            'small': self.create_publisher(Float64MultiArray, '/Convert/small_object_UTM', 10),
            'big': self.create_publisher(Float64MultiArray, '/Convert/big_object_UTM', 10),
            'emergency': self.create_publisher(Bool, '/LiDAR/dynamic_stop', 10),
            'speed': self.create_publisher(Int16, '/Perception/speed_limit', 10),
        }
        # Synthetic TF only in this isolated mock graph: sensor origin equals ego.
        # Allows relative detections and map obstacles to be viewed together.
        self.tf_broadcaster = None
        if self.get_parameter('publish_mock_tf').value:
            self.tf_broadcaster = StaticTransformBroadcaster(self)
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'velodyne'
            transform.transform.translation.x = self.vehicle[0]
            transform.transform.translation.y = self.vehicle[1]
            transform.transform.rotation.z = math.sin(self.vehicle[2] * 0.5)
            transform.transform.rotation.w = math.cos(self.vehicle[2] * 0.5)
            self.tf_broadcaster.sendTransform(transform)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if not math.isfinite(rate) or not 0.0 < rate <= 50.0:
            raise ValueError('publish_rate_hz must be in (0, 50]')
        self.create_timer(1.0 / rate, self.publish_inputs)
        self.get_logger().info(
            f'MOCK ONLY map={path_file}, base={self.base_index}, '
            f'obstacle_index={self.obstacle_index}, vehicle={self.vehicle}')

    def publish_inputs(self):
        scenario = self.get_parameter('scenario').value
        if scenario not in SCENARIOS:
            if scenario != self.last_scenario:
                self.get_logger().warning(f'Unknown scenario {scenario}; using cruise')
            effective = 'cruise'
        else:
            effective = scenario
        if scenario != self.last_scenario:
            self.get_logger().info(f'Mock scenario: {effective}')
            self.last_scenario = scenario
        offsets = {'avoid_center': [0.0], 'avoid_left': [0.75],
                   'avoid_right': [-0.75], 'blocked': self.blocked_offsets}.get(effective, [])
        obstacles = offset_obstacles(self.path, self.obstacle_index, offsets)
        x, y, yaw = self.vehicle
        relative = []
        for ox, oy in obstacles:
            dx, dy = ox - x, oy - y
            relative.extend([dx * math.cos(yaw) + dy * math.sin(yaw),
                             -dx * math.sin(yaw) + dy * math.cos(yaw)])
        relative.append(-1000.0)  # Existing LiDAR sentinel convention.
        position = PointStamped()
        position.header.stamp = self.get_clock().now().to_msg()
        position.header.frame_id = 'map'
        position.point.x, position.point.y = x, y
        self.output_pubs['position'].publish(position)
        self.output_pubs['heading'].publish(Float64(data=yaw))
        self.output_pubs['relative'].publish(Float64MultiArray(data=relative))
        self.output_pubs['small'].publish(Float64MultiArray(
            data=[coordinate for point in obstacles for coordinate in point]))
        self.output_pubs['big'].publish(Float64MultiArray(data=[]))
        self.output_pubs['emergency'].publish(Bool(data=effective == 'emergency'))
        self.output_pubs['speed'].publish(Int16(data=50 if effective == 'speed_50' else 20))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MockDebugInput()
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
