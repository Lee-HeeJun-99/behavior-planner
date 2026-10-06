from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    speed_config = PathJoinSubstitution(
        [FindPackageShare('traffic_sign_perception'), 'config', 'speed_sign.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('enable_lidar_perception', default_value='true'),
        DeclareLaunchArgument('enable_speed_sign', default_value='false'),
        DeclareLaunchArgument('camera_image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('speed_sign_weights', default_value=''),
        DeclareLaunchArgument('speed_sign_roi_enabled', default_value='true'),
        DeclareLaunchArgument('speed_sign_publish_debug', default_value='true'),
        Node(package='lidar', executable='start', name='lidar_small_static',
             condition=IfCondition(LaunchConfiguration('enable_lidar_perception'))),
        Node(package='kroad_planning_utm_pkg', executable='relative_2_UTM',
             name='relative_2_utm',
             condition=IfCondition(LaunchConfiguration('enable_lidar_perception'))),
        Node(package='traffic_sign_perception', executable='speed_sign_node',
             name='speed_sign_node', parameters=[speed_config, {
                 'image_topic': LaunchConfiguration('camera_image_topic'),
                 'weights_path': LaunchConfiguration('speed_sign_weights'),
                 'roi_enabled': ParameterValue(
                     LaunchConfiguration('speed_sign_roi_enabled'), value_type=bool),
                 'publish_debug_image': ParameterValue(
                     LaunchConfiguration('speed_sign_publish_debug'), value_type=bool),
             }], condition=IfCondition(LaunchConfiguration('enable_speed_sign'))),
    ])
