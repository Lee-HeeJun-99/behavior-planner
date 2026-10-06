from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_control', default_value='true'),
        Node(package='control', executable='car_control', name='erp_control',
             condition=IfCondition(LaunchConfiguration('enable_control'))),
    ])
