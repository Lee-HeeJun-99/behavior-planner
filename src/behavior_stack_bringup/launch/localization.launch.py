from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enabled = LaunchConfiguration('enable_localization')
    return LaunchDescription([
        DeclareLaunchArgument('enable_localization', default_value='true'),
        Node(package='local_pkg1', executable='heading_estimator',
             name='heading_estimator', condition=IfCondition(enabled)),
        Node(package='local_pkg1', executable='position_estimator',
             name='position_estimator', condition=IfCondition(enabled)),
    ])
