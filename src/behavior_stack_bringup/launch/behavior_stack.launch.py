from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def subsystem(name, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('behavior_stack_bringup'), 'launch', name])),
        launch_arguments=(arguments or {}).items(), condition=condition)


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('enable_gps', default_value='true'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('enable_lidar_sensor', default_value='true'),
        DeclareLaunchArgument('enable_localization', default_value='true'),
        DeclareLaunchArgument('enable_lidar_perception', default_value='true'),
        DeclareLaunchArgument('enable_camera_sign', default_value='false'),
        DeclareLaunchArgument('enable_planning', default_value='true'),
        DeclareLaunchArgument('enable_control', default_value='true'),
        DeclareLaunchArgument('enable_vehicle_interface', default_value='false'),
        DeclareLaunchArgument('enable_visualization', default_value='false'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('gps_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('velodyne_ip', default_value='192.168.1.201'),
        DeclareLaunchArgument('camera_image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('speed_sign_weights', default_value=''),
        DeclareLaunchArgument('speed_sign_roi_enabled', default_value='true'),
        DeclareLaunchArgument('speed_sign_publish_debug', default_value='true'),
        DeclareLaunchArgument('can_channel', default_value='can0'),
        DeclareLaunchArgument('can_receive_only', default_value='true'),
        DeclareLaunchArgument('can_max_speed_kph', default_value='10.0'),
        DeclareLaunchArgument('can_bringup', default_value='false'),
        DeclareLaunchArgument('can_dry_run', default_value='false'),
    ]
    includes = [
        subsystem('sensors.launch.py', {
            'enable_gps': LaunchConfiguration('enable_gps'),
            'enable_imu': LaunchConfiguration('enable_imu'),
            'enable_lidar_sensor': LaunchConfiguration('enable_lidar_sensor'),
            'gps_port': LaunchConfiguration('gps_port'),
            'imu_port': LaunchConfiguration('imu_port'),
            'velodyne_ip': LaunchConfiguration('velodyne_ip')}),
        subsystem('localization.launch.py', {
            'enable_localization': LaunchConfiguration('enable_localization')}),
        subsystem('perception.launch.py', {
            'enable_lidar_perception': LaunchConfiguration('enable_lidar_perception'),
            'enable_speed_sign': LaunchConfiguration('enable_camera_sign'),
            'camera_image_topic': LaunchConfiguration('camera_image_topic'),
            'speed_sign_weights': LaunchConfiguration('speed_sign_weights'),
            'speed_sign_roi_enabled': LaunchConfiguration('speed_sign_roi_enabled'),
            'speed_sign_publish_debug': LaunchConfiguration('speed_sign_publish_debug')}),
        subsystem('planning.launch.py', {
            'enable_planning': LaunchConfiguration('enable_planning')}),
        subsystem('control.launch.py', {
            'enable_control': LaunchConfiguration('enable_control')}),
        subsystem('vehicle_interface.launch.py', {
            'enable_vehicle_interface': LaunchConfiguration('enable_vehicle_interface'),
            'can_channel': LaunchConfiguration('can_channel'),
            'can_receive_only': LaunchConfiguration('can_receive_only'),
            'can_max_speed_kph': LaunchConfiguration('can_max_speed_kph'),
            'can_bringup': LaunchConfiguration('can_bringup'),
            'can_dry_run': LaunchConfiguration('can_dry_run')}),
        subsystem('visualization.launch.py', {
            'enable_rviz': LaunchConfiguration('enable_rviz')},
            IfCondition(LaunchConfiguration('enable_visualization'))),
    ]
    return LaunchDescription(arguments + includes)
