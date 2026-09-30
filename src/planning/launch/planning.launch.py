from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    share = get_package_share_directory('planning_pkg_2025')
    return LaunchDescription([Node(package='planning_pkg_2025', executable='planning_node',
        name='planning_node', output='screen',
        parameters=[os.path.join(share, 'config', 'planning.yaml')])])
