from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('planning_pkg_2025'), 'config', 'planning.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('planning_config', default_value=config),
        DeclareLaunchArgument('enable_planning', default_value='true'),
        Node(package='planning_pkg_2025', executable='planning_node',
             name='planning_node', parameters=[LaunchConfiguration('planning_config')],
             condition=IfCondition(LaunchConfiguration('enable_planning'))),
    ])
