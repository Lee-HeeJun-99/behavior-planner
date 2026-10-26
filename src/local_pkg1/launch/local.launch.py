from launch import LaunchDescription
from launch_ros.actions import Node

# ------------------------------------------------------------------------------
# K-ROAD (ERP42 Pro) 로컬라이제이션 실행 파일
#   GPS 드라이버 + 보정 프로그램(NTRIP) + IMU 드라이버 + 로컬라이제이션을 한 번에 켠다.
#
#   ros2 launch local_pkg1 local.launch.py
#
#   네 프로그램의 출력은 모두 이 화면에 나온다 (줄 앞의 [이름-번호] 로 구분).
#   설정은 프로그램을 하나씩 "ros2 run" 으로 켤 때와 같다 (Control_Manual.txt 3장).
#   이 파일을 고친 뒤에는 다시 빌드하지 않고 다시 켜기만 하면 된다 (--symlink-install 로 빌드한 경우).
#
#   장치 경로는 `ls /dev/serial/by-id/` 로 확인한다.
# ------------------------------------------------------------------------------
GPS_PORT = '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00'  # FST-UEF9P (u-blox ZED-F9P)
GPS_BAUD = 115200  # USB(CDC) 라서 실제로는 속도와 무관
IMU_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'  # Hiwonder IM10A

# NTRIP 보정 서버 : 국토지리정보원(NGII) VRS
#   VRS 는 수신기 위치(GGA)를 받아야 보정정보를 보내기 시작한다.
#   -> gga_relay 가 /fix 로 GGA 를 만들어 /gga 로 내보내고, ntrip_client 가 그걸 서버로 보낸다.
#   -> 수신기가 측위를 하기 전(실내 등)에는 GGA 가 비어 있어서 보정정보도 오지 않는다.
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

# 예전 서울시 서버 (단일 기준국, GGA 없이도 보정정보가 옴). 위 NGII 대신 쓰려면 아래 줄의 # 을 지운다.
# NTRIP_PARAMS.update({'host': 'gnss.eseoul.go.kr', 'mountpoint': 'YONS-RTCM32',
#                      'username': 'seoul', 'password': 'seoul'})


def generate_launch_description():
    # GPS 드라이버: 수신기 -> /fix, /nmea
    gps_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='nmea_serial_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'port': GPS_PORT, 'baud': GPS_BAUD}],
    )

    # NTRIP 클라이언트: 보정 서버 -> /rtcm -> GPS 드라이버 -> 수신기, GGA(/nmea) -> 서버
    ntrip_node = Node(
        package='ntrip_client',
        executable='ntrip_ros.py',
        name='ntrip_client',
        output='screen',
        emulate_tty=True,
        parameters=[NTRIP_PARAMS],
        # 서버로 보낼 GGA 는 gga_relay 가 만든 /gga 를 쓴다 (드라이버가 내보내는 /nmea 와 상관없이)
        remappings=[('nmea', '/gga')],
    )

    # GGA 중계: /fix -> GGA -> /gga -> ntrip_client -> NGII 서버 (VRS 는 이게 있어야 보정정보를 보냄)
    #   10초마다 GGA 전송 수, RTCM 수신량, GPS 상태(RTK 여부)를 출력한다.
    gga_relay_node = Node(
        package='local_pkg1',
        executable='gga_relay',
        name='gga_relay',
        output='screen',
        emulate_tty=True,
    )

    # IMU 드라이버: -> /imu
    imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        output='screen',
        emulate_tty=True,
        parameters=[{'port': IMU_PORT}],
    )

    # 로컬라이제이션: /fix, /imu, /ERP/serial_data -> /Local/utm, /Local/heading, /beta
    tae_localization_node = Node(
        package='local_pkg1',
        executable='tae_localization',
        name='tae_localization_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        gps_node,
        ntrip_node,
        gga_relay_node,
        imu_node,
        tae_localization_node,
    ])
