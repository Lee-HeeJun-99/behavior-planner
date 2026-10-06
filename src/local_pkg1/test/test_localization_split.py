import math
import sys
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
sys.modules.setdefault(
    'utm', SimpleNamespace(from_latlon=lambda latitude, longitude: (
        longitude * 10000.0, latitude * 10000.0, 52, 'S')))
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32MultiArray

from local_pkg.tae_localization import GpsImuHeading


def make_node(name, heading, position):
    return GpsImuHeading(
        node_name=name, publish_heading=heading, publish_position=position)


def feed_identical_input(node):
    erp = Float32MultiArray()
    erp.data = [1.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0]
    node.erp_serial_callback(erp)

    fix = NavSatFix()
    fix.latitude = 37.239255
    fix.longitude = 126.773019
    fix.altitude = 10.0
    node.gps_callback(fix)

    imu = Imu()
    yaw = math.radians(15.0)
    imu.orientation.z = math.sin(yaw * 0.5)
    imu.orientation.w = math.cos(yaw * 0.5)
    node.imu_callback(imu)


def test_split_nodes_preserve_core_localization_state():
    rclpy.init()
    nodes = [
        make_node('legacy_reference', True, True),
        make_node('heading_under_test', True, False),
        make_node('position_under_test', False, True),
    ]
    try:
        for node in nodes:
            feed_identical_input(node)
        reference = nodes[0]
        for node in nodes[1:]:
            assert node.current_x == pytest.approx(reference.current_x)
            assert node.current_y == pytest.approx(reference.current_y)
            assert node.center_velocity == pytest.approx(reference.center_velocity)
            assert node.steering == pytest.approx(reference.steering)
            assert node.imu_input_heading == pytest.approx(reference.imu_input_heading)
        assert nodes[1].heading_publisher is not None
        assert nodes[1].utm_publisher is None
        assert nodes[2].heading_publisher is None
        assert nodes[2].utm_publisher is not None
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
