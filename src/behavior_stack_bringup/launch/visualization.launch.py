from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def generate_launch_description():
    default_rviz = PathJoinSubstitution(
        [FindPackageShare('behavior_stack_bringup'), 'config', 'behavior_debug.rviz'])
    planning_config = os.path.join(
        get_package_share_directory('planning_pkg_2025'), 'config', 'planning.yaml')
    with open(planning_config, encoding='utf-8') as stream:
        planning_parameters = yaml.safe_load(stream)['planning_node']['ros__parameters']
    vehicle_parameters = {
        key: planning_parameters[key]
        for key in ('vehicle_half_width', 'vehicle_front', 'vehicle_rear')
    }
    return LaunchDescription([
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        Node(package='behavior_stack_bringup', executable='debug_visualizer_node',
             name='debug_visualizer_node', parameters=[vehicle_parameters]),
        Node(package='rviz2', executable='rviz2', name='behavior_debug_rviz',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             condition=IfCondition(LaunchConfiguration('enable_rviz'))),
    ])
