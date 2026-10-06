"""Isolated mock graph: never includes sensors, controller or CAN interface."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('behavior_stack_bringup')
    config = PathJoinSubstitution([share, 'config', 'mock_planning.yaml'])
    rviz = PathJoinSubstitution([share, 'config', 'mock_behavior_debug.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='cruise'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=rviz),
        DeclareLaunchArgument('publish_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('base_index', default_value='0'),
        DeclareLaunchArgument('obstacle_distance', default_value='8.0'),
        DeclareLaunchArgument('planning_config', default_value=config),
        Node(package='behavior_stack_bringup', executable='mock_debug_input_node',
             name='mock_debug_input_node', parameters=[{
                 'scenario': LaunchConfiguration('scenario'),
                 'planning_config': LaunchConfiguration('planning_config'),
                 'publish_rate_hz': ParameterValue(
                     LaunchConfiguration('publish_rate_hz'), value_type=float),
                 'base_index': ParameterValue(LaunchConfiguration('base_index'), value_type=int),
                 'obstacle_distance': ParameterValue(
                     LaunchConfiguration('obstacle_distance'), value_type=float),
             }]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, 'launch', 'planning.launch.py'])),
            launch_arguments={'planning_config': LaunchConfiguration('planning_config')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, 'launch', 'visualization.launch.py'])),
            launch_arguments={
                'enable_rviz': LaunchConfiguration('enable_rviz'),
                'rviz_config': LaunchConfiguration('rviz_config'),
            }.items()),
    ])
