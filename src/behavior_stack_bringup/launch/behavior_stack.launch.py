from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_gps = LaunchConfiguration('enable_gps')
    enable_imu = LaunchConfiguration('enable_imu')
    enable_localization = LaunchConfiguration('enable_localization')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_vehicle_interface = LaunchConfiguration('enable_vehicle_interface')

    gps_port = LaunchConfiguration('gps_port')
    imu_port = LaunchConfiguration('imu_port')
    erp_port = LaunchConfiguration('erp_port')
    velodyne_ip = LaunchConfiguration('velodyne_ip')

    calibration = PathJoinSubstitution(
        [FindPackageShare('velodyne_pointcloud'), 'params', 'VLP16db.yaml']
    )
    planning_config = PathJoinSubstitution(
        [FindPackageShare('planning_pkg_2025'), 'config', 'planning.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_gps', default_value='true'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('enable_localization', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        # Safety default: a dry run never opens the ERP actuator serial port.
        DeclareLaunchArgument('enable_vehicle_interface', default_value='false'),
        DeclareLaunchArgument('gps_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('erp_port', default_value='/dev/ttyUSB2'),
        DeclareLaunchArgument('velodyne_ip', default_value='192.168.1.201'),

        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_serial_driver',
            parameters=[{'port': gps_port}],
            condition=IfCondition(enable_gps),
        ),
        Node(
            package='wit_ros2_imu',
            executable='wit_ros2_imu',
            name='wit_ros2_imu',
            parameters=[{'port': imu_port}],
            condition=IfCondition(enable_imu),
        ),
        Node(
            package='local_pkg1',
            executable='tae_localization',
            name='tae_localization',
            condition=IfCondition(enable_localization),
        ),

        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            parameters=[{'device_ip': velodyne_ip, 'model': 'VLP16', 'port': 2368}],
            condition=IfCondition(enable_lidar),
        ),
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform_node',
            parameters=[{'calibration': calibration, 'model': 'VLP16'}],
            condition=IfCondition(enable_lidar),
        ),
        Node(
            package='lidar',
            executable='start',
            name='lidar_small_static',
            condition=IfCondition(enable_lidar),
        ),
        Node(
            package='kroad_planning_utm_pkg',
            executable='relative_2_UTM',
            name='relative_2_utm',
            condition=IfCondition(enable_lidar),
        ),

        Node(
            package='planning_pkg_2025',
            executable='planning_node',
            name='planning_node',
            parameters=[planning_config],
        ),
        Node(
            package='control',
            executable='car_control',
            name='erp_control',
        ),
        Node(
            package='erp_ros2_bridge',
            executable='erp_ros2_bridge',
            name='erp_ros2_bridge',
            parameters=[{'port': erp_port}],
            condition=IfCondition(enable_vehicle_interface),
        ),
    ])
