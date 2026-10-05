#ifndef VEHICLE_PARAM_HPP
#define VEHICLE_PARAM_HPP

/*
ERP42 Pro (WITH:US-S1) 차량 상수.
컨트롤 패키지 안에서 차량에 따라 달라지는 값은 모두 여기에서만 정의한다.
게인(튜닝 값)은 여기가 아닌 json 파일에 둔다.
*/

namespace vehicle
{
// 축거 [m]. 예전 ERP42 값 그대로이다. 실측하면 이 값만 바꾼다.
// (10/3 주행 기록에서는 조향각 대비 실제 회전이 이 값으로 계산한 것의 약 0.8배였다.)
constexpr double WHEEL_BASE = 1.04;

// 조향 명령 한계 [rad] (28.17 deg). ERP42 Pro 의 명령 한계는 +-30 deg 이다.
constexpr double MAX_STEER = 0.491642;

// 조향 명령부터 실제 조향까지의 지연 [s].
// 10/3 주행 기록에서 측정: 순수 지연 0.12 s + 1차 지연 0.08 s (예전 ERP42 는 약 0.6 s).
constexpr double STEER_DELAY = 0.2;

// 차량 상태(/ERP/serial_data)가 들어오는 주기 [s].
// ERP42 Pro 브릿지는 50 Hz 로 보낸다 (10/3 주행 기록에서 50.0 Hz). 예전 ERP42 는 20 Hz 였다.
constexpr double FEEDBACK_PERIOD = 0.02;

// 속도 명령 상한 [m/s] (24.8 km/h)
constexpr double MAX_SPEED = 6.9;

// 기어 코드. /Control/vehicle_cmd 와 ERP42 Pro CAN 에서 같은 값을 쓴다.
constexpr int GEAR_P = 0;
constexpr int GEAR_D = 1;
constexpr int GEAR_N = 2;
constexpr int GEAR_R = 3;

// 브레이크 명령 [%]. 예전 코드의 0~200 눈금(20 = 제동 없음)을 ERP42 Pro 의 % 로 옮긴 값이다.
//   % = (예전 값 - 20) / 180 * 100
constexpr double BRAKE_STOP = (180.0 - 20.0) / 1.8;        // 정지 명령 (예전 180)  = 88.9 %
constexpr double BRAKE_BASE = (30.0 - 20.0) / 1.8;         // 감속 시작 값 (예전 30) =  5.6 %
constexpr double BRAKE_SCALE = 200.0 / 1.8;                // 감속 PD 출력 1 당 %  (예전 200)
constexpr double BRAKE_MAX = (199.0 - 20.0) / 1.8;         // 상한 (예전 199)       = 99.4 %
constexpr double BRAKE_NARROW_MIN = (30.0 - 20.0) / 1.8;   // 협로(998) 하한 (예전 30)
constexpr double BRAKE_NARROW_MAX = (120.0 - 20.0) / 1.8;  // 협로(998) 상한 (예전 120) = 55.6 %
} // namespace vehicle

#endif // VEHICLE_PARAM_HPP
