"""protocol.py 단위 시험.  실행: python3 -m pytest test/ (ROS 불필요)"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from erp42pro_interface import protocol as P  # noqa: E402

SCALE = 1.146


def cmd(valid=1, e_stop=0, gear=P.PRO_GEAR_D, speed=0.0, steer=0.0, brake=0.0):
    return [valid, e_stop, gear, speed, steer, brake]


def test_steering_encode_matches_datasheet():
    assert P.encode_steering(0.0, 1, 0)[4:6] == [0x2C, 0x01]       # 0 deg -> raw 300
    assert P.encode_steering(40.0, 1, 0) == P.encode_steering(30.0, 1, 0)   # +-30 deg 로 제한


def test_brake_and_accel_encode():
    assert P.encode_brake(30.0, 1, 5) == [0x51, 30, 0, 0, 0, 0, 0, 0]
    assert P.encode_brake(150.0, 1, 0)[1] == 100
    f = P.encode_accel(10.0, P.PRO_GEAR_D, 1, 0)
    assert f[3] == P.PRO_GEAR_D and f[6] | (f[7] << 8) == 100       # 0.1 km/h 단위


def test_status_decode():
    s2 = P.decode_status_2([0x4C, 0x03, 0x00, 0x00, 0x68, 0x01, 0, 0])
    assert s2['speed_kph'] == pytest.approx(4.4)
    assert s2['brake_mpa'] == pytest.approx(0.0)
    assert s2['steer_deg'] == pytest.approx(1.0)
    assert P.decode_status_1([0x01, 0x01, 0, 0, 0, 0, 0, 0]) == {'pro_gear': 1, 'drive_mode': 1}


def test_cmd_units_pass_through():
    out = P.vehicle_cmd_to_pro(cmd(speed=2.0, steer=math.radians(10.0), brake=37.5))
    assert out['valid'] == 1 and out['pro_gear'] == P.PRO_GEAR_D
    assert out['speed_kph'] == pytest.approx(7.2)
    assert out['steer_deg'] == pytest.approx(10.0)
    assert out['brake_pct'] == pytest.approx(37.5)                  # % 그대로, 영점 보정 없음


def test_speed_scale_both_ways():
    # 실제 10 km/h 로 달리게 하려면 차량에는 10 / 1.146 을 보낸다
    out = P.vehicle_cmd_to_pro(cmd(speed=10 / 3.6), speed_scale=SCALE)
    assert out['speed_kph'] == pytest.approx(10.0)
    assert out['speed_kph_can'] == pytest.approx(10.0 / SCALE)
    # 차량이 10 km/h 라고 알려 주면 실제는 11.46 km/h
    fb = P.pro_status_to_feedback({'speed_kph': 10.0, 'brake_mpa': 0.0, 'steer_deg': 0.0},
                                  {'pro_gear': 1, 'drive_mode': 1}, speed_scale=SCALE)
    assert fb[3] == pytest.approx(10.0 * SCALE / 3.6)
    # 명령 -> 차량 -> 되돌아온 값이 같아야 한다
    fb2 = P.pro_status_to_feedback({'speed_kph': out['speed_kph_can'], 'brake_mpa': 0.0, 'steer_deg': 0.0},
                                   {'pro_gear': 1, 'drive_mode': 1}, speed_scale=SCALE)
    assert fb2[3] == pytest.approx(10 / 3.6)


def test_speed_is_not_capped_by_bridge():
    # 브릿지는 속도를 제한하지 않는다. control 이 낸 속도(플래닝의 목표 속도)가 그대로 나간다.
    for mps in (2.0, 4.0, 5.0, 6.0, 6.9):
        out = P.vehicle_cmd_to_pro(cmd(speed=mps), speed_scale=SCALE)
        assert out['speed_kph'] == pytest.approx(mps * 3.6)
        assert out['speed_kph_can'] == pytest.approx(mps * 3.6 / SCALE)
    # CAN 의 속도 칸은 0 ~ 80 km/h 까지만 담을 수 있다 (프로토콜의 범위)
    frame = P.encode_accel(500.0, P.PRO_GEAR_D, 1, 0)
    assert frame[6] | (frame[7] << 8) == 800


def test_limits():
    out = P.vehicle_cmd_to_pro(cmd(steer=1.0, brake=250.0), max_steer_deg=28.0)
    assert out['steer_deg'] == pytest.approx(28.0) and out['brake_pct'] == 100.0
    out = P.vehicle_cmd_to_pro(cmd(steer=-1.0, brake=-5.0), max_steer_deg=28.0, steer_sign=-1.0)
    assert out['steer_deg'] == pytest.approx(28.0) and out['brake_pct'] == 0.0


def test_estop_and_invalid():
    out = P.vehicle_cmd_to_pro(cmd(e_stop=1, speed=3.0, brake=0.0), estop_brake_pct=100.0)
    assert out['speed_kph'] == 0.0 and out['speed_kph_can'] == 0.0 and out['brake_pct'] == 100.0
    assert P.vehicle_cmd_to_pro(cmd(valid=0))['valid'] == 0


def test_gear():
    assert P.vehicle_cmd_to_pro(cmd(gear=P.PRO_GEAR_R, speed=1.5))['pro_gear'] == P.PRO_GEAR_R
    assert P.vehicle_cmd_to_pro(cmd(gear=7))['pro_gear'] == P.PRO_GEAR_N        # 모르는 값은 중립
    assert P.vehicle_cmd_to_pro(cmd(gear=P.PRO_GEAR_R, speed=-1.5))['speed_kph'] == pytest.approx(5.4)
    fb = P.pro_status_to_feedback({'speed_kph': -3.6, 'brake_mpa': 0.0, 'steer_deg': 0.0},
                                  {'pro_gear': P.PRO_GEAR_R, 'drive_mode': 1})
    assert fb[2] == P.FB_GEAR_REVERSE and fb[3] == pytest.approx(1.0)   # 플래닝·로컬용 기존 코드, 속도는 크기


def test_malformed_command_rejected():
    assert not P.cmd_is_wellformed([1, 0, 1, 2.0, 0.0])                 # 칸 부족
    assert not P.cmd_is_wellformed([1, 0, 1, float('nan'), 0.0, 0.0])
    assert not P.cmd_is_wellformed(None)
    assert P.cmd_is_wellformed(cmd())
    with pytest.raises(ValueError):
        P.vehicle_cmd_to_pro([1, 0, 1, 2.0, 0.0])


def test_feedback_format():
    fb = P.pro_status_to_feedback({'speed_kph': 7.2, 'brake_mpa': 0.9, 'steer_deg': -5.0},
                                  {'pro_gear': P.PRO_GEAR_D, 'drive_mode': 1}, control_mode=1.0)
    assert len(fb) == 7
    assert fb[0] == 1.0 and fb[1] == 0.0 and fb[2] == P.FB_GEAR_FORWARD
    assert fb[3] == pytest.approx(2.0) and fb[4] == pytest.approx(math.radians(-5.0)) and fb[5] == pytest.approx(0.9)
