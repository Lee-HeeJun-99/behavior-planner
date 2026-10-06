from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('traffic_sign_perception'), 'config', 'speed_sign.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('image_path'),
        DeclareLaunchArgument('weights_path', default_value=''),
        DeclareLaunchArgument('camera_image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('enable_debug_image', default_value='true'),
        DeclareLaunchArgument('publish_rate_hz', default_value='5.0'),
        Node(package='behavior_stack_bringup', executable='mock_camera_node', parameters=[{
            'image_path': LaunchConfiguration('image_path'),
            'image_topic': LaunchConfiguration('camera_image_topic'),
            'publish_rate_hz': ParameterValue(
                LaunchConfiguration('publish_rate_hz'), value_type=float),
        }]),
        Node(package='traffic_sign_perception', executable='speed_sign_node',
             parameters=[config, {
                 'weights_path': LaunchConfiguration('weights_path'),
                 'image_topic': LaunchConfiguration('camera_image_topic'),
                 'publish_debug_image': ParameterValue(
                     LaunchConfiguration('enable_debug_image'), value_type=bool),
             }]),
    ])
