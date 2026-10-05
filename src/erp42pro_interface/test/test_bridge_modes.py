"""Exercise bridge transmission guards without ROS discovery or a vehicle."""
import threading
from types import SimpleNamespace
from unittest.mock import Mock
from rclpy.time import Time
from erp42pro_interface.can_bridge_node import Erp42ProCanBridge


def bridge(receive_only=False, age=0.0):
    obj = SimpleNamespace(
        receive_only=receive_only, bus=Mock(), lock=threading.Lock(),
        last_cmd=[1, 0, 1, 5.0, 0.0, 0.0],
        last_cmd_time=Time(seconds=1.0),
        cmd_timeout=0.3, steer_sign=1.0, max_steer_deg=28.0,
        max_speed_kph=10.0, speed_scale=1.146, brake_input_max=200.0, brake_input_min=20.0,
        estop_brake_pct=100.0, timeout_brake_pct=30.0,
        timeout_warned=False, counter=0, rolling_counter=True, acc_raw=0,
        dbg_pub=Mock(), get_logger=lambda: Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=1.0 + age)),
    )
    return obj


def test_receive_only_never_transmits_even_on_shutdown():
    obj = bridge(receive_only=True)
    Erp42ProCanBridge.send_timer_cb(obj)
    Erp42ProCanBridge.stop_vehicle(obj)
    obj.bus.send.assert_not_called()
    obj.dbg_pub.publish.assert_not_called()


def test_control_conversion_caps_speed_and_maps_brake_zero():
    obj = bridge()
    Erp42ProCanBridge.send_timer_cb(obj)
    assert obj.bus.send.call_count == 4
    assert {call.args[0].arbitration_id for call in obj.bus.send.call_args_list} == {0x501, 0x502, 0x503, 0x504}
    debug = obj.dbg_pub.publish.call_args.args[0].data
    assert list(debug[:5]) == [1.0, 10.0, 0.0, 0.0, 1.0]
    assert abs(debug[5] - 10.0 / 1.146) < 1e-5


def test_command_timeout_stops_with_brake():
    obj = bridge(age=0.5)
    Erp42ProCanBridge.send_timer_cb(obj)
    debug = obj.dbg_pub.publish.call_args.args[0].data
    assert debug[1] == 0.0 and debug[3] == 30.0


def test_no_command_does_not_transmit():
    obj = bridge()
    obj.last_cmd = None
    obj.last_cmd_time = None
    Erp42ProCanBridge.send_timer_cb(obj)
    obj.bus.send.assert_not_called()
