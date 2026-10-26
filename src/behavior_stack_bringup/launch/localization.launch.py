from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# wego_ws 측위 코드(local_pkg_2025)로 교체됨.
# 예전 heading_estimator / position_estimator 두 노드 대신
# tae_localization 한 노드가 /Local/utm, /Local/heading, /beta 를 모두 내보낸다.
# (GPS/IMU 드라이버는 sensors.launch.py 에서 켠다)
def generate_launch_description():
    enabled = LaunchConfiguration('enable_localization')
    return LaunchDescription([
        DeclareLaunchArgument('enable_localization', default_value='true'),
        Node(package='local_pkg1', executable='tae_localization',
             name='tae_localization', output='screen', emulate_tty=True,
             condition=IfCondition(enabled)),
    ])
