from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('traffic_sign_perception'), 'config', 'speed_sign.yaml')
    return LaunchDescription([
        Node(
            package='traffic_sign_perception',
            executable='speed_sign_node',
            name='speed_sign_node',
            output='screen',
            parameters=[config],
        )
    ])
