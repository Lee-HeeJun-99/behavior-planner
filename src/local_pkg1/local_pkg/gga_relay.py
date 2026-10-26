# 국토지리정보원(NGII) VRS 용 GGA 중계 노드
#
# VRS(가상기준점) 방식은 차량의 현재 위치(GGA 문장)를 서버로 보내야
# 그 위치 근처에 가상 기준국을 만들어 RTK 보정정보(RTCM)를 보내 준다.
# GGA 가 서버에 가지 않으면 보정정보가 오지 않아 RTK Fix 가 되지 않는다.
#
# 받는 것
#   /fix   (sensor_msgs/NavSatFix)  GPS 드라이버 출력
#   /rtcm  (rtcm_msgs/Message)      ntrip_client 가 받은 보정정보 (수신량 확인용)
# 내보내는 것
#   /gga   (nmea_msgs/Sentence)     GGA 문장, 1 Hz -> ntrip_client 가 서버로 보냄
#                                   (런치에서 ntrip_client 의 nmea 입력을 /gga 로 연결)
#
# 10초마다 "GGA 보낸 개수 / RTCM 받은 양 / GPS 상태" 를 출력한다.
#   RTCM 이 0 이면: 서버가 보정정보를 안 보내는 것 (계정/마운트포인트/GGA 확인)
#   RTCM 이 들어오는데 status 가 2 가 아니면: 수신기 쪽 문제 (RTCM 입력 설정, 안테나, 하늘 시야)

from datetime import datetime, timezone
from math import isnan

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Sentence
from rtcm_msgs.msg import Message as Rtcm


STATUS_NAME = {-1: '측위 없음', 0: '단독측위', 1: 'DGPS/SBAS', 2: 'RTK(Float/Fix)'}


class GgaRelay(Node):

    def __init__(self):
        super().__init__('gga_relay')
        self.gga_period = self.declare_parameter('gga_period', 1.0).value  # [s]
        fix_topic = self.declare_parameter('fix_topic', '/fix').value
        nmea_topic = self.declare_parameter('nmea_topic', '/gga').value
        rtcm_topic = self.declare_parameter('rtcm_topic', '/rtcm').value

        sensor_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                                reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(NavSatFix, fix_topic, self.fix_callback, sensor_qos)
        self.create_subscription(Rtcm, rtcm_topic, self.rtcm_callback, sensor_qos)
        self.nmea_pub = self.create_publisher(Sentence, nmea_topic, 10)

        self.last_fix = None
        self.gga_count = 0
        self.rtcm_bytes = 0
        self.status_count = {}

        self.create_timer(self.gga_period, self.send_gga)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'{fix_topic} -> GGA -> {nmea_topic} ({1.0 / self.gga_period:.1f} Hz)')

    def fix_callback(self, msg):
        self.status_count[msg.status.status] = self.status_count.get(msg.status.status, 0) + 1
        if isnan(msg.latitude) or isnan(msg.longitude) or msg.status.status < 0:
            return
        self.last_fix = msg

    def rtcm_callback(self, msg):
        self.rtcm_bytes += len(msg.message)

    def send_gga(self):
        if self.last_fix is None:
            return
        out = Sentence()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'gps'
        out.sentence = self.make_gga(self.last_fix)
        self.nmea_pub.publish(out)
        self.gga_count += 1

    @staticmethod
    def make_gga(fix):
        # VRS 서버는 대략적인 위치만 필요하다 (가상 기준국을 만들 위치).
        now = datetime.now(timezone.utc)
        hhmmss = now.strftime('%H%M%S') + '.%02d' % (now.microsecond // 10000)

        lat, lon = fix.latitude, fix.longitude
        alat, alon = abs(lat), abs(lon)
        lat_d, lon_d = int(alat), int(alon)
        lat_s = '%02d%08.5f' % (lat_d, (alat - lat_d) * 60.0)
        lon_s = '%03d%08.5f' % (lon_d, (alon - lon_d) * 60.0)

        quality = {0: 1, 1: 2, 2: 5}.get(fix.status.status, 1)
        alt = 0.0 if isnan(fix.altitude) else fix.altitude

        body = 'GPGGA,%s,%s,%s,%s,%s,%d,12,1.0,%.3f,M,0.000,M,,' % (
            hhmmss, lat_s, 'N' if lat >= 0 else 'S', lon_s, 'E' if lon >= 0 else 'W', quality, alt)
        checksum = 0
        for c in body:
            checksum ^= ord(c)
        return '$%s*%02X' % (body, checksum)

    def report(self):
        status = ', '.join(f'{STATUS_NAME.get(k, k)} {v}' for k, v in sorted(self.status_count.items()))
        msg = (f'최근 10초: GGA {self.gga_count}개 전송, RTCM {self.rtcm_bytes} bytes 수신, '
               f'GPS 상태 [{status or "없음"}]')
        if self.gga_count == 0:
            self.get_logger().warning(msg + ' -> 측위 전이라 GGA 를 못 보냄 (VRS 는 GGA 가 있어야 보정정보를 보냄)')
        elif self.rtcm_bytes == 0:
            self.get_logger().warning(msg + ' -> 보정정보가 안 옴 (ntrip_client 접속/계정/마운트포인트 확인)')
        elif self.status_count.get(2, 0) == 0:
            self.get_logger().warning(msg + ' -> 보정정보는 오는데 RTK 가 아님 (수신기 RTCM 입력/안테나 확인)')
        else:
            self.get_logger().info(msg)
        self.gga_count = 0
        self.rtcm_bytes = 0
        self.status_count = {}


def main(args=None):
    rclpy.init(args=args)
    node = GgaRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
