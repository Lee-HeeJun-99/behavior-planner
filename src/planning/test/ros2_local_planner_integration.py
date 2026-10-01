#!/usr/bin/env python3
"""Black-box ROS 2 integration probe for planning_node. Never imported by production code."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float64, Float64MultiArray, Int16, String


def load_path(filename):
    result = []
    with open(filename, encoding="utf-8") as stream:
        for line in stream:
            fields = [float(value) for value in line.strip().split(",")]
            if len(fields) >= 4:
                result.append(tuple(fields[:4]))
    return result


class Probe(Node):
    def __init__(self, global_path):
        super().__init__("local_planner_integration_probe")
        self.global_path = global_path
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.position_pub = self.create_publisher(PointStamped, "/Local/utm", qos)
        self.heading_pub = self.create_publisher(Float64, "/Local/heading", qos)
        self.obstacle_pub = self.create_publisher(Float64MultiArray, "/Convert/small_object_UTM", qos)
        self.speed_limit_pub = self.create_publisher(Int16, "/Perception/speed_limit", qos)
        self.emergency_pub = self.create_publisher(Bool, "/LiDAR/dynamic_stop", qos)
        self.create_subscription(String, "/Planning/behavior", self.behavior_callback, qos)
        self.create_subscription(Float64, "/Planning/target_velocity", self.velocity_callback, qos)
        self.create_subscription(Float64MultiArray, "/Planning/local_path", self.path_callback, qos)
        self.create_subscription(Float64MultiArray, "/Planning/path_yaw", self.yaw_callback, qos)
        self.create_subscription(Float64MultiArray, "/Planning/curvature", self.curvature_callback, qos)
        self.create_subscription(Float64MultiArray, "/Planning/debug/candidates", self.candidate_callback, qos)
        self.behavior = None
        self.transitions = []
        self.velocity = None
        self.local_path = []
        self.yaws = []
        self.curvatures = []
        self.candidates = []
        self.obstacles = []
        self.speed_events = []
        self.emergency_stop = False
        self.vehicle = list(global_path[0][:3])
        self.timer = self.create_timer(0.05, self.publish_inputs)

    def behavior_callback(self, message):
        if message.data != self.behavior:
            self.transitions.append((time.monotonic(), self.behavior, message.data))
            self.behavior = message.data

    def velocity_callback(self, message):
        self.velocity = message.data

    def path_callback(self, message):
        self.local_path = list(zip(message.data[0::2], message.data[1::2]))

    def yaw_callback(self, message):
        self.yaws = list(message.data)

    def curvature_callback(self, message):
        self.curvatures = list(message.data)

    def candidate_callback(self, message):
        self.candidates = [tuple(message.data[i:i + 3]) for i in range(0, len(message.data), 3)]

    def publish_inputs(self):
        x, y, yaw = self.vehicle
        position = PointStamped()
        position.header.stamp = self.get_clock().now().to_msg()
        position.header.frame_id = "map"
        position.point.x, position.point.y = x, y
        self.position_pub.publish(position)
        self.heading_pub.publish(Float64(data=yaw))
        flat = [coordinate for point in self.obstacles for coordinate in point]
        self.obstacle_pub.publish(Float64MultiArray(data=flat))
        self.emergency_pub.publish(Bool(data=self.emergency_stop))
        if self.speed_events:
            self.speed_limit_pub.publish(Int16(data=self.speed_events.pop(0)))

    def detect_speed(self, *limits):
        self.speed_events.extend(limits)

    def metrics(self):
        valid = [candidate for candidate in self.candidates if candidate[1] >= 0.0]
        invalid = [candidate for candidate in self.candidates if candidate[1] < 0.0]
        selected = min(valid, key=lambda candidate: candidate[1]) if valid else None
        start_distance = float("inf")
        maximum_curvature = max((abs(value) for value in self.curvatures), default=float("nan"))
        maximum_deviation = 0.0
        rms_deviation_accumulator = 0.0
        maximum_segment = 0.0
        maximum_yaw_step = 0.0
        if self.local_path:
            vehicle = self.vehicle
            start_distance = math.hypot(self.local_path[0][0] - vehicle[0], self.local_path[0][1] - vehicle[1])
            deviations = []
            for index, point in enumerate(self.local_path):
                deviation = min(math.hypot(point[0] - ref[0], point[1] - ref[1]) for ref in self.global_path)
                deviations.append(deviation)
                maximum_deviation = max(maximum_deviation, deviation)
                rms_deviation_accumulator += deviation * deviation
                if index:
                    maximum_segment = max(maximum_segment, math.hypot(
                        point[0] - self.local_path[index - 1][0], point[1] - self.local_path[index - 1][1]))
            rms_deviation = math.sqrt(rms_deviation_accumulator / len(self.local_path))
            initial_deviation = deviations[0]
            final_deviation = deviations[-1]
        else:
            rms_deviation = float("nan")
            initial_deviation = float("nan")
            final_deviation = float("nan")
        for index in range(1, len(self.yaws)):
            maximum_yaw_step = max(maximum_yaw_step, abs(math.remainder(self.yaws[index] - self.yaws[index - 1], 2 * math.pi)))
        return {
            "behavior": self.behavior,
            "target_velocity": self.velocity,
            "path_points": len(self.local_path),
            "selected_offset": selected[0] if selected else None,
            "minimum_clearance": selected[2] if selected else None,
            "valid_candidates": len(valid),
            "invalid_candidates": len(invalid),
            "maximum_curvature": maximum_curvature,
            "start_distance": start_distance,
            "maximum_global_deviation": maximum_deviation,
            "rms_global_deviation": rms_deviation,
            "initial_global_deviation": initial_deviation,
            "final_global_deviation": final_deviation,
            "maximum_segment": maximum_segment,
            "maximum_yaw_step": maximum_yaw_step,
            "candidate_table": self.candidates,
        }


def spin_for(node, seconds):
    deadline = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def offset_obstacles(path, index, offsets):
    x, y, yaw, _ = path[index]
    return [(x - math.sin(yaw) * offset, y + math.cos(yaw) * offset) for offset in offsets]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_file")
    parser.add_argument("--station-index", type=int, default=160)
    args = parser.parse_args()
    path = load_path(args.map_file)
    rclpy.init()
    probe = Probe(path)
    results = {}
    spin_for(probe, 1.5)
    results["CRUISE"] = probe.metrics()
    probe.detect_speed(50, 0)
    spin_for(probe, 0.5)
    results["SINGLE_FALSE_50"] = probe.metrics()
    probe.detect_speed(50, 50, 50)
    spin_for(probe, 0.5)
    results["CONFIRMED_50"] = probe.metrics()
    probe.detect_speed(0)
    spin_for(probe, 0.5)
    results["NO_DETECTION_AFTER_50"] = probe.metrics()
    probe.obstacles = offset_obstacles(path, args.station_index, [0.0])
    spin_for(probe, 1.5)
    results["SINGLE_OBSTACLE"] = probe.metrics()
    if len(probe.local_path) > 100 and len(probe.yaws) > 100:
        probe.vehicle = [probe.local_path[100][0], probe.local_path[100][1], probe.yaws[100]]
    probe.obstacles = []
    spin_for(probe, 1.5)
    results["RECOVERY"] = probe.metrics()
    probe.emergency_stop = True
    spin_for(probe, 0.5)
    results["EMERGENCY_STOP"] = probe.metrics()
    probe.emergency_stop = False
    spin_for(probe, 0.7)
    results["EMERGENCY_RELEASE"] = probe.metrics()
    probe.obstacles = offset_obstacles(path, args.station_index, [-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0])
    spin_for(probe, 1.5)
    results["BLOCKED"] = probe.metrics()
    print("TRANSITIONS")
    for _, previous, current in probe.transitions:
        print(f"  {previous or 'NONE'} -> {current}")
    for name, metrics in results.items():
        print(f"SCENARIO {name}")
        for key, value in metrics.items():
            print(f"  {key}: {value}")
    probe.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
