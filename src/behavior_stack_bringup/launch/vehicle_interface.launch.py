from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('erp42pro_interface'), 'config', 'erp42pro_bridge.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('enable_vehicle_interface', default_value='false'),
        DeclareLaunchArgument('can_channel', default_value='can0'),
        DeclareLaunchArgument('can_receive_only', default_value='true'),
        DeclareLaunchArgument('can_max_speed_kph', default_value='10.0'),
        DeclareLaunchArgument('can_bringup', default_value='false'),
        DeclareLaunchArgument('can_dry_run', default_value='false'),
        Node(package='erp42pro_interface', executable='can_bridge_node',
             name='erp42pro_can_bridge', parameters=[config, {
                 'can_channel': LaunchConfiguration('can_channel'),
                 'receive_only': ParameterValue(
                     LaunchConfiguration('can_receive_only'), value_type=bool),
                 'max_speed_kph': ParameterValue(
                     LaunchConfiguration('can_max_speed_kph'), value_type=float),
                 'can_bringup': ParameterValue(
                     LaunchConfiguration('can_bringup'), value_type=bool),
                 'dry_run': ParameterValue(
                     LaunchConfiguration('can_dry_run'), value_type=bool),
             }], condition=IfCondition(LaunchConfiguration('enable_vehicle_interface'))),
    ])
