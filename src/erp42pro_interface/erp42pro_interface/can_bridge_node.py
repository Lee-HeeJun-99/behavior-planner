"""ERP42 Pro CAN bridge node.

control 패키지 <-> ERP42 Pro 차량 사이의 데이터 변환만 담당한다.

  구독  /Control/vehicle_cmd (std_msgs/Float32MultiArray)
        [valid, e_stop, gear(0 P / 1 D / 2 N / 3 R), speed(m/s, 실제), steer(rad), brake(%)]
  발행  /ERP/serial_data     (std_msgs/Float32MultiArray)
        [control_mode, e_stop, gear(0 전진 / 1 중립 / 2 후진), speed(m/s, 실제), steer(rad), brake(MPa), 0]
  발행  /erp42pro/cmd_debug  (std_msgs/Float32MultiArray)
        [valid, speed_kph(실제 기준), steer_deg, brake_pct, pro_gear, speed_kph_can(차량에 보낸 값)]

  속도는 speed_scale (= 실제 속도 / 차량이 재는 속도) 로 양방향 보정한다.
  속도 상한은 두지 않는다. 얼마로 달릴지는 플래닝의 목표 속도가 정한다.

  CAN  송신 0x501/0x502/0x503/0x504 (send_rate_hz 주기), 수신 0x303/0x304
       시작할 때 can0 이 꺼져 있으면 직접 켠다 (can_setup.py, PC 마다 한 번 tools/setup_can_sudo.sh 필요).
"""

import threading

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

from erp42pro_interface import protocol as P
from erp42pro_interface.can_setup import CanSetupError, ensure_can_up

try:
    import can
except ImportError:  # python-can 미설치 시 dry_run 으로만 동작
    can = None


class Erp42ProCanBridge(Node):

    def __init__(self):
        super().__init__('erp42pro_can_bridge')

        # ---------------------------------------------------- parameters
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('receive_only', True)
        self.declare_parameter('max_speed_kph', 10.0)
        self.declare_parameter('can_bringup', False)       # True: 시작할 때 can0 이 꺼져 있으면 직접 켠다
        self.declare_parameter('dry_run', False)          # True: CAN 송수신 없이 변환만 확인
        self.declare_parameter('send_rate_hz', 50.0)      # datasheet cycle 20 ms
        self.declare_parameter('cmd_topic', '/Control/vehicle_cmd')
        self.declare_parameter('feedback_topic', '/ERP/serial_data')
        self.declare_parameter('debug_topic', '/erp42pro/cmd_debug')
        self.declare_parameter('steer_sign', 1.0)         # 조향 부호 (차량에서 확인 후 +1 / -1)
        self.declare_parameter('max_steer_deg', 28.0)     # <= 30
        self.declare_parameter('speed_scale', 1.146)      # 실제 속도 / 차량이 재는 속도 (GPS 로 측정)
        self.declare_parameter('estop_brake_pct', 100.0)
        self.declare_parameter('cmd_timeout_sec', 0.3)    # 명령 끊기면 정지
        self.declare_parameter('timeout_brake_pct', 30.0)
        self.declare_parameter('rolling_counter', True)   # heartbeat 0~15 증가 / False: 15 고정
        self.declare_parameter('acc_raw', 0)              # 0x504 byte4 (withus 드라이버와 동일 기본값)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.channel = gp('can_channel')
        self.dry_run = bool(gp('dry_run'))
        self.receive_only = bool(gp('receive_only'))
        self.max_speed_kph = float(gp('max_speed_kph'))
        self.steer_sign = float(gp('steer_sign'))
        self.max_steer_deg = min(float(gp('max_steer_deg')), 30.0)
        self.speed_scale = float(gp('speed_scale'))
        if not self.speed_scale > 0.0:
            self.get_logger().error('speed_scale 은 0 보다 커야 합니다 -> 1.0 사용')
            self.speed_scale = 1.0
        self.estop_brake_pct = float(gp('estop_brake_pct'))
        self.cmd_timeout = float(gp('cmd_timeout_sec'))
        self.timeout_brake_pct = float(gp('timeout_brake_pct'))
        self.rolling_counter = bool(gp('rolling_counter'))
        self.acc_raw = int(gp('acc_raw'))

        # ---------------------------------------------------------- state
        self.rx_stop = threading.Event()
        self.lock = threading.Lock()
        self.last_cmd = None
        self.last_cmd_time = None
        self.counter = 0
        self.status1 = None
        self.timeout_warned = False

        # ------------------------------------------------------------ CAN
        self.bus = None
        if not self.dry_run:
            if can is None:
                raise RuntimeError('python-can is required for CAN feedback')
            else:
                if bool(gp('can_bringup')):
                    # 꺼져 있으면 켠다. 켤 수 없으면 CanSetupError (main 에서 이유를 알리고 끝낸다).
                    self.get_logger().info(ensure_can_up(self.channel))
                self.bus = can.interface.Bus(
                    channel=self.channel, bustype='socketcan',
                    can_filters=[
                        {'can_id': P.ID_VCU_STATUS_1, 'can_mask': 0x7FF},
                        {'can_id': P.ID_VCU_STATUS_2, 'can_mask': 0x7FF},
                    ])

        # ------------------------------------------------------------ ROS
        # 구독: BEST_EFFORT -> control 쪽 publisher가 best_effort(2025) / reliable(2024) 어느 쪽이든 연결됨
        # 발행: RELIABLE     -> control 쪽 subscriber가 best_effort / reliable 어느 쪽이든 연결됨
        sub_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Float32MultiArray, gp('cmd_topic'), self.cmd_cb, sub_qos)
        self.fb_pub = self.create_publisher(Float32MultiArray, gp('feedback_topic'), pub_qos)
        self.dbg_pub = self.create_publisher(Float32MultiArray, gp('debug_topic'), pub_qos)
        self.create_timer(1.0 / float(gp('send_rate_hz')), self.send_timer_cb)

        if self.bus is not None:
            self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
            self.rx_thread.start()

        self.get_logger().info(
            f'ERP42 Pro bridge start (channel={self.channel}, dry_run={self.dry_run}, '
            f'receive_only={self.receive_only}, steer_sign={self.steer_sign}, '
            f'speed_scale={self.speed_scale})')

    # ---------------------------------------------------------- callbacks
    def cmd_cb(self, msg):
        cmd = list(msg.data)
        if not P.cmd_is_wellformed(cmd):
            # 형식이 다른 명령(칸 수 부족, NaN)은 버린다 -> 계속되면 아래 timeout 으로 정지
            self.get_logger().error(
                f'잘못된 형식의 명령을 무시합니다 (칸 {len(cmd)}개, {P.CMD_LEN}개 필요)',
                throttle_duration_sec=1.0)
            return
        with self.lock:
            self.last_cmd = cmd
            self.last_cmd_time = self.get_clock().now()
        self.timeout_warned = False

    def send_timer_cb(self):
        if self.receive_only:
            return
        with self.lock:
            cmd = self.last_cmd
            t = self.last_cmd_time

        timed_out = (t is None or
                     (self.get_clock().now() - t).nanoseconds * 1e-9 > self.cmd_timeout)

        if cmd is None:
            return  # 아직 control 명령을 받은 적 없음 -> 아무것도 보내지 않음

        out = P.vehicle_cmd_to_pro(
            cmd, self.steer_sign, self.max_steer_deg,
            self.speed_scale, self.estop_brake_pct)

        out['speed_kph'] = min(out['speed_kph'], self.max_speed_kph)
        out['speed_kph_can'] = out['speed_kph'] / self.speed_scale

        if timed_out:
            if not self.timeout_warned:
                self.get_logger().warn('control 명령 timeout -> 정지 명령 송신')
                self.timeout_warned = True
            out['speed_kph'] = 0.0
            out['speed_kph_can'] = 0.0
            out['brake_pct'] = max(out['brake_pct'], self.timeout_brake_pct)

        cnt = self.counter if self.rolling_counter else 0x0F
        self.counter = (self.counter + 1) & 0x0F

        frames = [
            (P.ID_AD_CONTROL_FLAG, P.encode_control_flag(out['valid'], cnt)),
            (P.ID_AD_CONTROL_STEERING, P.encode_steering(out['steer_deg'], out['valid'], cnt)),
            (P.ID_AD_CONTROL_ACCEL, P.encode_accel(out['speed_kph_can'], out['pro_gear'],
                                                   out['valid'], cnt, acc_raw=self.acc_raw)),
            (P.ID_AD_CONTROL_BRAKE, P.encode_brake(out['brake_pct'], out['valid'], cnt)),
        ]
        if self.bus is not None:
            for can_id, data in frames:
                try:
                    self.bus.send(can.Message(arbitration_id=can_id, data=data,
                                              is_extended_id=False))
                except can.CanError as e:
                    self.get_logger().error(f'CAN send fail 0x{can_id:03X}: {e}',
                                            throttle_duration_sec=1.0)

        dbg = Float32MultiArray()
        dbg.data = [float(out['valid']), float(out['speed_kph']), float(out['steer_deg']),
                    float(out['brake_pct']), float(out['pro_gear']), float(out['speed_kph_can'])]
        self.dbg_pub.publish(dbg)

    def rx_loop(self):
        while not self.rx_stop.is_set() and rclpy.ok():
            try:
                m = self.bus.recv(0.1)
            except can.CanError:
                continue
            if m is None or len(m.data) < 8:
                continue
            if m.arbitration_id == P.ID_VCU_STATUS_1:
                self.status1 = P.decode_status_1(m.data)
            elif m.arbitration_id == P.ID_VCU_STATUS_2:
                s2 = P.decode_status_2(m.data)
                s1 = self.status1
                control_mode = 1.0 if (s1 is not None and s1['drive_mode'] == 1) else 0.0
                fb = Float32MultiArray()
                fb.data = P.pro_status_to_feedback(s2, s1, self.steer_sign, control_mode,
                                                   speed_scale=self.speed_scale)
                self.fb_pub.publish(fb)

    def stop_vehicle(self):
        """노드 종료 시 정지 + 브레이크 명령."""
        if self.bus is None or self.receive_only:
            return
        for _ in range(5):
            self.bus.send(can.Message(arbitration_id=P.ID_AD_CONTROL_ACCEL,
                                      data=P.encode_accel(0.0, P.PRO_GEAR_N, 1, 0x0F),
                                      is_extended_id=False))
            self.bus.send(can.Message(arbitration_id=P.ID_AD_CONTROL_BRAKE,
                                      data=P.encode_brake(self.estop_brake_pct, 1, 0x0F),
                                      is_extended_id=False))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Erp42ProCanBridge()
    except CanSetupError as e:
        rclpy.logging.get_logger('erp42pro_can_bridge').error('브릿지를 시작하지 못했습니다.\n' + str(e))
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.rx_stop.set()
        if hasattr(node, "rx_thread"):
            node.rx_thread.join(timeout=1.0)
        node.stop_vehicle()
        if node.bus is not None:
            node.bus.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
