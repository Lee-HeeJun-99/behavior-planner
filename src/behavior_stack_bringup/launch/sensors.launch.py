from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ------------------------------------------------------------------------------
# GPS / IMU / LiDAR 센서 실행.
#
# GPS 는 wego_ws 와 같은 RTK 구성 (국토지리정보원 NGII VRS):
#   ntrip_client  : NGII 서버 -> /rtcm            (서버로 보낼 위치는 /gga)
#   nmea_serial_driver : /rtcm -> 수신기(USB) 에 write, 수신기 -> /fix, /nmea(GGA)
#   gga_relay     : /fix -> GGA -> /gga -> ntrip_client
#                   (VRS 는 수신기 위치(GGA)를 받아야 보정정보를 보내기 시작한다)
#
# RTK 를 끄려면 enable_rtk:=false (드라이버는 단독 측위로 계속 /fix 를 낸다).
# 장치 경로는 `ls /dev/serial/by-id/` 로 확인한다.
# ------------------------------------------------------------------------------
GPS_PORT = '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00'  # FST-UEF9P (u-blox ZED-F9P)
GPS_BAUD = 115200  # USB(CDC) 라서 실제로는 속도와 무관
IMU_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'  # Hiwonder IM10A

# NTRIP 보정 서버 : 국토지리정보원(NGII) VRS  (wego_ws local_pkg_2025/launch/local.launch.py 와 동일)
NTRIP_PARAMS = {
    'host': 'RTS1.ngii.go.kr',     # 안 되면 RTS2.ngii.go.kr / RTS3.ngii.go.kr
    'port': 2101,
    'mountpoint': 'VRS-RTCM34',
    'authenticate': True,
    'username': 'jh011027',        # geodesy.ngii.go.kr 에서 신청한 계정
    'password': 'ngii',
    'ssl': False,
    'rtcm_frame_id': 'odom',
    'rtcm_message_package': 'rtcm_msgs',
    'nmea_max_length': 128,   # 고정밀(High precision) GGA 는 82자를 넘을 수 있음
    'nmea_min_length': 3,
    'reconnect_attempt_max': 100,
    'reconnect_attempt_wait_seconds': 5,
    'rtcm_timeout_seconds': 10,
}


def generate_launch_description():
    calibration = PathJoinSubstitution(
        [FindPackageShare('velodyne_pointcloud'), 'params', 'VLP16db.yaml'])
    rtk_enabled = IfCondition(LaunchConfiguration('enable_rtk'))
    return LaunchDescription([
        DeclareLaunchArgument('enable_gps', default_value='true'),
        DeclareLaunchArgument('enable_rtk', default_value='true'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('enable_lidar_sensor', default_value='true'),
        DeclareLaunchArgument('gps_port', default_value=GPS_PORT),
        DeclareLaunchArgument('imu_port', default_value=IMU_PORT),
        DeclareLaunchArgument('velodyne_ip', default_value='192.168.1.201'),

        # GPS 드라이버: 수신기 -> /fix, /nmea  /  /rtcm -> 수신기
        Node(package='nmea_navsat_driver', executable='nmea_serial_driver',
             name='nmea_serial_driver', output='screen', emulate_tty=True,
             parameters=[{'port': LaunchConfiguration('gps_port'), 'baud': GPS_BAUD}],
             condition=IfCondition(LaunchConfiguration('enable_gps'))),

        # NTRIP 클라이언트: NGII 서버 -> /rtcm, 서버로 보낼 GGA 는 /gga
        Node(package='ntrip_client', executable='ntrip_ros.py',
             name='ntrip_client', output='screen', emulate_tty=True,
             parameters=[NTRIP_PARAMS],
             remappings=[('nmea', '/gga')],
             condition=rtk_enabled),

        # GGA 중계: /fix -> /gga (10초마다 GGA 수, RTCM 수신량, RTK 상태 출력)
        Node(package='local_pkg1', executable='gga_relay',
             name='gga_relay', output='screen', emulate_tty=True,
             condition=rtk_enabled),

        Node(package='wit_ros2_imu', executable='wit_ros2_imu',
             name='wit_ros2_imu', parameters=[{'port': LaunchConfiguration('imu_port')}],
             condition=IfCondition(LaunchConfiguration('enable_imu'))),
        Node(package='velodyne_driver', executable='velodyne_driver_node',
             name='velodyne_driver_node',
             parameters=[{'device_ip': LaunchConfiguration('velodyne_ip'),
                          'model': 'VLP16', 'port': 2368}],
             condition=IfCondition(LaunchConfiguration('enable_lidar_sensor'))),
        Node(package='velodyne_pointcloud', executable='velodyne_transform_node',
             name='velodyne_transform_node',
             parameters=[{'calibration': calibration, 'model': 'VLP16'}],
             condition=IfCondition(LaunchConfiguration('enable_lidar_sensor'))),
    ])
