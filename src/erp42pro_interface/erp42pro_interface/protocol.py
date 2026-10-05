"""ERP42 Pro (WITH:US-S1) CAN protocol encode/decode helpers.

ROS에 의존하지 않는 순수 함수만 모아둔 모듈 (단위 테스트 가능: test/test_protocol.py).
근거: withus_CAN_datasheet.pdf (CAN 1 ID)

  송신 (PC -> 차량), 20 ms 주기
    0x501 AD_Control_Flag       byte0 [3:0]=Request_Flag, [7:4]=MsgCntr
    0x502 AD_Control_Steering   byte0 [3:0]=Valid, [7:4]=MsgCntr
                                byte4-5 Steering_Angle_Cmd  0.1 deg, offset -30  (-30 ~ +30 deg)
    0x503 AD_Control_Brake      byte0 [3:0]=Valid, [7:4]=MsgCntr
                                byte1   BrakePressure_Cmd   1 %, (0 ~ 100 %)
    0x504 AD_Control_Accelerate byte0 [3:0]=Valid, [7:4]=MsgCntr
                                byte2   Work_Mode (0 torque / 1 speed)
                                byte3   Gear (0 P / 1 D / 2 N / 3 R)
                                byte4   Acc_De  0.1 m/s^2, offset -5
                                byte5   Torque  1 %
                                byte6-7 Speed_Control 0.1 km/h (0 ~ 80)

  수신 (차량 -> PC)
    0x303 VCU_Vehicle_Status_1  byte0 [1:0]=Vehicle_Gear, byte1 [3:0]=Drive_Mode_State
    0x304 VCU_Vehicle_Status_2  byte0-1 Vehicle_Speed   0.1 km/h, offset -80
                                byte2-3 Brake_Pressure  0.01 MPa
                                byte4-5 [9:0] Steering_Angle 0.1 deg, offset -35
"""

import math

# ---------------------------------------------------------------- CAN IDs
ID_AD_CONTROL_FLAG = 0x501
ID_AD_CONTROL_STEERING = 0x502
ID_AD_CONTROL_BRAKE = 0x503
ID_AD_CONTROL_ACCEL = 0x504
ID_VCU_STATUS_1 = 0x303
ID_VCU_STATUS_2 = 0x304

# ------------------------------------------------------- gear definitions
# ERP42 Pro gear code (명령 /Control/vehicle_cmd 와 CAN 에서 그대로 사용)
PRO_GEAR_P = 0
PRO_GEAR_D = 1
PRO_GEAR_N = 2
PRO_GEAR_R = 3
_PRO_GEARS = (PRO_GEAR_P, PRO_GEAR_D, PRO_GEAR_N, PRO_GEAR_R)

# /ERP/serial_data 의 gear 칸은 플래닝·로컬이 쓰는 기존 코드를 유지한다: 0 전진, 1 중립, 2 후진
FB_GEAR_FORWARD = 0
FB_GEAR_NEUTRAL = 1
FB_GEAR_REVERSE = 2
_PRO_TO_FB_GEAR = {
    PRO_GEAR_P: FB_GEAR_NEUTRAL,
    PRO_GEAR_D: FB_GEAR_FORWARD,
    PRO_GEAR_N: FB_GEAR_NEUTRAL,
    PRO_GEAR_R: FB_GEAR_REVERSE,
}

# /Control/vehicle_cmd 칸 번호
CMD_VALID, CMD_ESTOP, CMD_GEAR, CMD_SPEED, CMD_STEER, CMD_BRAKE = range(6)
CMD_LEN = 6


def clip(x, lo, hi):
    return max(lo, min(hi, x))


def pro_gear_to_feedback(pro_gear):
    return _PRO_TO_FB_GEAR.get(int(pro_gear), FB_GEAR_NEUTRAL)


def _flag_byte(valid, counter):
    """byte0: 하위 4bit = Valid/Request flag, 상위 4bit = heartbeat counter."""
    return ((int(counter) & 0x0F) << 4) | (int(valid) & 0x0F)


def _u16_le(raw):
    raw = int(raw) & 0xFFFF
    return raw & 0xFF, (raw >> 8) & 0xFF


# ------------------------------------------------------------ encoders
def encode_control_flag(valid, counter):
    return [_flag_byte(valid, counter), 0, 0, 0, 0, 0, 0, 0]


def encode_steering(steer_deg, valid, counter):
    """steer_deg: -30 ~ +30 deg."""
    steer_deg = clip(steer_deg, -30.0, 30.0)
    raw = int(round((steer_deg + 30.0) / 0.1))
    lo, hi = _u16_le(raw)
    return [_flag_byte(valid, counter), 0, 0, 0, lo, hi, 0, 0]


def encode_brake(brake_pct, valid, counter):
    """brake_pct: 0 ~ 100 %."""
    raw = int(round(clip(brake_pct, 0.0, 100.0)))
    return [_flag_byte(valid, counter), raw, 0, 0, 0, 0, 0, 0]


def encode_accel(speed_kph, pro_gear, valid, counter,
                 work_mode=1, acc_raw=0, torque_pct=0):
    """speed_kph: 0 ~ 80 km/h (크기만), pro_gear: 0 P / 1 D / 2 N / 3 R."""
    speed_raw = int(round(clip(abs(speed_kph), 0.0, 80.0) / 0.1))
    lo, hi = _u16_le(speed_raw)
    return [
        _flag_byte(valid, counter),
        0,
        int(work_mode) & 0xFF,
        int(pro_gear) & 0xFF,
        int(acc_raw) & 0xFF,
        int(clip(torque_pct, 0, 100)) & 0xFF,
        lo,
        hi,
    ]


# ------------------------------------------------------------ decoders
def decode_status_2(data):
    """0x304 -> dict(speed_kph, brake_mpa, steer_deg)."""
    speed_raw = data[0] | (data[1] << 8)
    brake_raw = data[2] | (data[3] << 8)
    steer_raw = (data[4] | (data[5] << 8)) & 0x03FF  # 10 bit
    return {
        'speed_kph': speed_raw * 0.1 - 80.0,
        'brake_mpa': brake_raw * 0.01,
        'steer_deg': steer_raw * 0.1 - 35.0,
    }


def decode_status_1(data):
    """0x303 -> dict(pro_gear, drive_mode)."""
    return {
        'pro_gear': data[0] & 0x03,
        'drive_mode': data[1] & 0x0F,
    }


# ------------------------------------------------ control <-> ERP42 Pro
def cmd_is_wellformed(cmd):
    """/Control/vehicle_cmd 로 쓸 수 있는 메시지인지 (길이, 숫자) 확인."""
    if cmd is None or len(cmd) < CMD_LEN:
        return False
    return all(math.isfinite(float(x)) for x in cmd[:CMD_LEN])


def vehicle_cmd_to_pro(cmd, steer_sign=1.0, max_steer_deg=28.0,
                       speed_scale=1.0, estop_brake_pct=100.0):
    """control 패키지 출력(/Control/vehicle_cmd) -> ERP42 Pro 명령.

    cmd = [valid, e_stop, gear(Pro: 0 P / 1 D / 2 N / 3 R), speed(m/s, 실제 속도), steer(rad), brake(%)]

    speed_scale = 실제 속도 / 차량이 재는 속도.  차량은 자기가 잰 속도를 명령에 맞추므로
    실제로 v 로 달리게 하려면 v / speed_scale 을 보내야 한다.

    속도는 여기서 제한하지 않는다. 얼마로 달릴지는 플래닝의 목표 속도가 정하고,
    control 이 그 값을 차량 한계(vehicle_param.hpp 의 MAX_SPEED, 6.9 m/s) 안에서 내보낸다.

    반환: dict(valid, speed_kph(실제 기준), speed_kph_can(차량에 보내는 값), steer_deg, brake_pct, pro_gear)
    """
    if not cmd_is_wellformed(cmd):
        raise ValueError('malformed vehicle_cmd')
    valid = 1 if cmd[CMD_VALID] >= 0.5 else 0
    e_stop = cmd[CMD_ESTOP] >= 0.5
    gear = int(round(cmd[CMD_GEAR]))
    pro_gear = gear if gear in _PRO_GEARS else PRO_GEAR_N

    speed_kph = abs(cmd[CMD_SPEED]) * 3.6
    steer_deg = clip(math.degrees(cmd[CMD_STEER]) * steer_sign, -max_steer_deg, max_steer_deg)
    brake_pct = clip(cmd[CMD_BRAKE], 0.0, 100.0)

    if e_stop:
        speed_kph = 0.0
        brake_pct = estop_brake_pct

    scale = speed_scale if speed_scale > 0.0 else 1.0
    return {
        'valid': valid,
        'speed_kph': speed_kph,
        'speed_kph_can': speed_kph / scale,
        'steer_deg': steer_deg,
        'brake_pct': brake_pct,
        'pro_gear': pro_gear,
    }


def pro_status_to_feedback(status2, status1, steer_sign=1.0, control_mode=1.0,
                           e_stop=0.0, speed_scale=1.0):
    """ERP42 Pro 상태 -> /ERP/serial_data (플래닝, 로컬, 컨트롤 입력).

    [control_mode, e_stop, gear(0 전진 / 1 중립 / 2 후진), speed(m/s, 실제 속도, 크기), steer(rad), brake(MPa), 0]
    speed 는 차량이 잰 값에 speed_scale 을 곱한 실제 차량 속도이다 (바퀴 한쪽 속도가 아님).
    """
    scale = speed_scale if speed_scale > 0.0 else 1.0
    speed_mps = abs(status2['speed_kph']) * scale / 3.6
    steer_rad = math.radians(status2['steer_deg']) * steer_sign
    gear = pro_gear_to_feedback(status1['pro_gear']) if status1 else FB_GEAR_FORWARD
    return [
        float(control_mode),
        float(e_stop),
        float(gear),
        float(speed_mps),
        float(steer_rad),
        float(status2['brake_mpa']),
        0.0,
    ]
