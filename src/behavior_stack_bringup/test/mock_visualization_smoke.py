#!/usr/bin/env python3
"""Run isolated real Planning/visualizer with mock inputs, without hardware."""
import argparse
import json
import math
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Path as PathMessage
from std_msgs.msg import Float64, Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray


class Probe(Node):
    def __init__(self):
        super().__init__('mock_visualization_probe')
        self.messages = {}
        self.subscriptions_kept = []
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        global_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        topics = {
            '/Planning/debug/global_path': PathMessage,
            '/Planning/debug/local_path': PathMessage,
            '/Planning/debug/candidate_paths': MarkerArray,
            '/Planning/debug/ego': MarkerArray,
            '/Visualization/obstacles_utm': MarkerArray,
            '/Visualization/lidar_relative_objects': MarkerArray,
            '/Visualization/behavior': Marker,
            '/Planning/behavior': String,
            '/Planning/target_velocity': Float64,
            '/Planning/debug/candidates': Float64MultiArray,
        }
        for topic, message_type in topics.items():
            self.subscriptions_kept.append(self.create_subscription(
                message_type, topic,
                lambda msg, key=topic: self.messages.__setitem__(key, msg),
                global_qos if topic.endswith('global_path') else qos))
        self.required = set(topics)

    def verify(self, scenario):
        assert self.required <= self.messages.keys(), 'Missing topics: ' + str(
            self.required - self.messages.keys())
        get = lambda suffix: self.messages['/Planning/' + suffix]
        behavior = get('behavior').data
        expected = ('EMERGENCY_STOP' if scenario in ('blocked', 'emergency') else
                    'AVOID' if scenario.startswith('avoid_') else 'CRUISE')
        assert behavior == expected, (behavior, expected)
        velocity = get('target_velocity').data
        assert math.isclose(velocity, 0.0 if scenario in ('blocked', 'emergency') else
                            4.5 if scenario == 'speed_50' else 2.5, abs_tol=1e-6)
        global_path = get('debug/global_path')
        local_path = get('debug/local_path')
        assert len(global_path.poses) > 0 and global_path.header.frame_id == 'map'
        assert scenario == 'blocked' or len(local_path.poses) > 0
        if local_path.poses:
            assert local_path.header.frame_id == 'map'
        candidate_markers = get('debug/candidate_paths').markers
        lines = [m for m in candidate_markers if m.type == Marker.LINE_STRIP and m.points]
        assert lines and all(len(m.points) > 1 for m in lines)
        metadata = get('debug/candidates').data
        valid = sum(metadata[i + 1] >= 0.0 for i in range(0, len(metadata), 3))
        invalid = len(metadata) // 3 - valid
        valid_metadata = [metadata[i:i + 3] for i in range(0, len(metadata), 3)
                          if metadata[i + 1] >= 0.0]
        selected = min(valid_metadata, key=lambda row: row[1]) if valid_metadata else None
        if scenario == 'blocked':
            assert valid == 0 and invalid > 0 and not local_path.poses
        else:
            assert valid > 0
        if scenario.startswith('avoid_'):
            assert invalid > 0
            assert any('selected' in m.text for m in candidate_markers)
            assert selected[2] >= 0.0
        ego = get('debug/ego').markers
        assert any(m.type == Marker.ARROW for m in ego)
        assert any(m.type == Marker.CUBE for m in ego)
        assert self.messages['/Visualization/behavior'].text == expected
        relative = [m for m in self.messages['/Visualization/lidar_relative_objects'].markers
                    if m.action == Marker.ADD]
        utm = [m for m in self.messages['/Visualization/obstacles_utm'].markers
               if m.action == Marker.ADD]
        obstacle_scenario = scenario.startswith('avoid_') or scenario == 'blocked'
        assert len(relative) == len(utm)
        assert bool(utm) == obstacle_scenario
        assert all(m.header.frame_id == 'map' for m in utm)
        assert all(m.header.frame_id == 'velodyne' for m in relative)
        arrow = next(m for m in ego if m.type == Marker.ARROW)
        x, y = arrow.points[0].x, arrow.points[0].y
        yaw = math.atan2(arrow.points[1].y - y, arrow.points[1].x - x)
        for relative_marker, utm_marker in zip(relative, utm):
            rx, ry = relative_marker.pose.position.x, relative_marker.pose.position.y
            assert math.isclose(x + rx * math.cos(yaw) - ry * math.sin(yaw),
                                utm_marker.pose.position.x, abs_tol=1e-6)
            assert math.isclose(y + rx * math.sin(yaw) + ry * math.cos(yaw),
                                utm_marker.pose.position.y, abs_tol=1e-6)
        if local_path.poses:
            first = local_path.poses[0].pose.position
            assert math.hypot(first.x - x, first.y - y) < 0.1
        nodes = set(self.get_node_names())
        assert nodes <= {'mock_debug_input_node', 'planning_node',
                         'debug_visualizer_node', 'mock_visualization_probe'}, nodes
        command_topics = [t for t, _ in self.get_topic_names_and_types()
                          if t in ('/Control/vehicle_cmd', '/erp42pro/cmd_debug')]
        assert not command_topics, command_topics
        return {'scenario': scenario, 'behavior': behavior, 'velocity': velocity,
                'global_points': len(global_path.poses), 'local_points': len(local_path.poses),
                'geometry_candidates': len(lines), 'valid': valid, 'invalid': invalid,
                'minimum_cost_offset': selected[0] if selected else None,
                'minimum_clearance': selected[2] if selected else None,
                'obstacles': len(utm), 'nodes': sorted(nodes), 'result': 'PASS'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenarios', nargs='+', default=[
        'cruise', 'avoid_center', 'avoid_left', 'avoid_right', 'blocked',
        'emergency', 'speed_20', 'speed_50'])
    args = parser.parse_args()
    failures = []
    for index, scenario in enumerate(args.scenarios):
        # Unique domain prevents existing publishers from satisfying this test.
        os.environ['ROS_DOMAIN_ID'] = str(110 + index)
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        with tempfile.TemporaryFile(mode='w+') as output:
            process = subprocess.Popen([
                'ros2', 'launch', 'behavior_stack_bringup', 'mock_visualization.launch.py',
                'scenario:=' + scenario, 'enable_rviz:=false'],
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            probe = None
            try:
                rclpy.init()
                probe = Probe()
                deadline = time.monotonic() + 10.0
                last_error = None
                while time.monotonic() < deadline and process.poll() is None:
                    rclpy.spin_once(probe, timeout_sec=0.1)
                    try:
                        result = probe.verify(scenario)
                        print(json.dumps(result), flush=True)
                        break
                    except AssertionError as error:
                        last_error = str(error)
                else:
                    raise AssertionError(last_error or 'Launch exited unexpectedly')
            except Exception as error:
                failures.append(scenario)
                output.seek(0)
                print(f'{scenario}: FAIL: {error}\n{output.read()}', flush=True)
            finally:
                if probe is not None:
                    probe.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=5)
    raise SystemExit(1 if failures else 0)


if __name__ == '__main__':
    main()
