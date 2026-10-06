from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    calibration = PathJoinSubstitution(
        [FindPackageShare('velodyne_pointcloud'), 'params', 'VLP16db.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('enable_gps', default_value='true'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('enable_lidar_sensor', default_value='true'),
        DeclareLaunchArgument('gps_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('velodyne_ip', default_value='192.168.1.201'),
        Node(package='nmea_navsat_driver', executable='nmea_serial_driver',
             name='nmea_serial_driver',
             parameters=[{'port': LaunchConfiguration('gps_port')}],
             condition=IfCondition(LaunchConfiguration('enable_gps'))),
        Node(package='wit_ros2_imu', executable='wit_ros2_imu',
             name='wit_ros2_imu', parameters=[{'port': LaunchConfiguration('imu_port')}],
             condition=IfCondition(LaunchConfiguration('enable_imu'))),
        Node(package='velodyne_driver', executable='velodyne_driver_node',
             name='velodyne_driver_node',
             parameters=[{'device_ip': LaunchConfiguration('velodyne_ip'),
                          'model': 'VLP16', 'port': 2368}],
             condition=IfCondition(LaunchConfiguration('enable_lidar_sensor'))),
        Node(package='velodyne_pointcloud', executable='velodyne_transform_node',
             name='velodyne_transform_node',
             parameters=[{'calibration': calibration, 'model': 'VLP16'}],
             condition=IfCondition(LaunchConfiguration('enable_lidar_sensor'))),
    ])
