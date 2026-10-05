# Behavior Planner 실차 검증 절차

## 1. 목적

본 문서는 Ubuntu 20.04 + ROS 2 Foxy 환경에서 `feature/Sign_Detection` 브랜치의
20/50 km/h 속도표지판 기반 Behavior Planning 구조를 검증하기 위한 절차다.

현재 완료된 범위:

- 기존 30/40/50 속도표지판 구조를 20/50 구조로 변경
- Perception → ContextManager → Planning 연동 수정
- Planning / Perception unit test 완료
- ROS2 offline integration test 완료
- Controller, ERP bridge, Localization, LiDAR, Local Planner 알고리즘은 수정하지 않음

현재 미완료 범위:

- 실제 YOLO `.pt` 파일 적용
- 실제 `model.names` 확인
- 실제 카메라 기반 20/50 detection 검증
- ROI / confidence / bbox threshold 튜닝
- ERP42 실제 target velocity 검증
- Controller → ERP42 실제 command/feedback 검증
- 조종기 / Autopilot / E-stop 검증
- 저속 실차 검증

---

# 2. 현재 코드 기준 구조

```text
Camera
  ↓
ROI
  ↓
YOLO
  ↓
20 / 50 supported class filtering
  ↓
/Perception/speed_limit
  ↓
ContextManager
  ↓
20 / 50 confirmation
  ↓
active speed limit
  ↓
BehaviorPlanner
  ↓
LocalPlanner
  ↓
/Planning/local_path
/Planning/target_velocity
  ↓
Controller
  ↓
/Control/serial_data
  ↓
ERP Bridge
  ↓
ERP42
```

속도표지판 의미:

```text
0  = 지원 표지판 미검출
20 = 20 km/h 표지판
50 = 50 km/h 표지판
```

현재 Planning 설정:

```text
default_speed_limit_kph = 20
speed_sign_confirmation_count = 3

speed_limit_20_target = 2.5   # provisional
speed_limit_50_target = 4.5   # provisional

avoid_max_velocity = 2.5
```

`2.5`, `4.5`는 구조 검증용 임시값이며 실제 ERP42 calibration 값이 아니다.

---

# 3. 완료된 Offline 검증

## 3.1 Build

```text
planning_pkg_2025 build              PASS
traffic_sign_perception build        PASS
```

## 3.2 Unit Test

Planning GTest:

```text
17 PASS
```

Perception helper test:

```text
6 PASS
```

`colcon test-result`:

```text
errors   : 0
failures : 0
skipped  : 0
```

검증 완료 항목:

- default active limit = 20
- 20 연속 3회 → active 20
- 50 연속 3회 → active 50
- `50, 0, 50` → active 변경 없음
- `50, 50, 20, 50` → 잘못된 50 confirmation 없음
- 30 / 40 / 70 → unsupported
- 0 반복 → 기존 active speed 유지
- AVOID velocity cap
- EMERGENCY_STOP → target velocity 0
- emergency release 후 active target 복귀
- 기존 obstacle collision / boundary / local path 회귀
- NO_VALID_PATH 안전 처리

## 3.3 ROS2 Planning Integration

```text
CRUISE                  → 2.5
SINGLE_FALSE_50         → 2.5
CONFIRMED_50            → 4.5
NO_DETECTION_AFTER_50   → 4.5
CONFIRMED_20            → 2.5
SINGLE_OBSTACLE / AVOID → 2.5
RECOVERY / CRUISE       → 2.5
EMERGENCY_STOP          → 0.0
EMERGENCY_RELEASE       → 2.5
BLOCKED                 → 0.0
```

현재 상태에서는 구조 변경 자체는 완료로 본다.

---

# 4. STEP 1 — 실제 PT 파일 적용

현재 repository에는 `.pt` 또는 `.onnx` 파일이 없다.

예상 배치 위치:

```text
src/traffic_sign_perception/weights/<model>.pt
```

파일을 넣은 뒤 실제 class 이름을 확인한다.

```bash
python3 - <<'PY'
from ultralytics import YOLO

model = YOLO(
    "src/traffic_sign_perception/weights/<model>.pt"
)

print(model.names)
PY
```

## PASS 기준

실제 `model.names`에서 20 / 50 class를 확인할 수 있어야 한다.

현재 코드:

```python
CLASS_TO_LIMIT = {
    "speed_20": 20,
    "speed_50": 50,
}
```

실제 class 이름이 다르면 `model.names`에 맞춰 mapping만 수정한다.

class ID나 이름을 임의로 추정하지 않는다.

---

# 5. STEP 2 — PT 단독 Inference 확인

실제 PT 파일이 정상적으로 load되는지 확인한다.

확인 항목:

- model load 성공
- 20 class 존재
- 50 class 존재
- inference error 없음
- unsupported class가 Planning 값으로 전달되지 않음

기록:

```text
Weight:
Ultralytics version:
Torch version:
model.names:

20 image result:
50 image result:
```

---

# 6. STEP 3 — Perception Fail-safe

## 6.1 Weight 없음

```bash
ros2 run traffic_sign_perception speed_sign_node --ros-args   -p weights_path:=""
```

예상:

```text
/Perception/speed_limit = 0
```

Node가 종료되면 FAIL.

## 6.2 Invalid ROI

```bash
ros2 run traffic_sign_perception speed_sign_node --ros-args   -p weights_path:="<model>.pt"   -p roi_enabled:=true   -p roi_x_min:=0.9   -p roi_x_max:=0.1
```

예상:

```text
Invalid ROI
→ full image fallback
→ node 유지
```

## PASS 기준

- node alive
- speed_limit publish 유지
- unsupported / invalid 상황에서 0 publish
- debug image publish 가능
- exception으로 node 종료되지 않음

---

# 7. STEP 4 — 실제 카메라 연결

```bash
ros2 topic list | grep camera
ros2 topic info /camera/image_raw
ros2 topic hz /camera/image_raw
```

Perception 실행:

```bash
ros2 launch behavior_stack_bringup behavior_stack.launch.py   enable_gps:=false   enable_imu:=false   enable_localization:=false   enable_lidar:=false   enable_camera_sign:=true   enable_vehicle_interface:=false   camera_image_topic:=/camera/image_raw   speed_sign_weights:=<model>.pt   speed_sign_roi_enabled:=true   speed_sign_publish_debug:=true
```

확인:

```bash
ros2 topic echo /Perception/speed_limit
ros2 topic hz /Perception/speed_limit
rqt_image_view /Perception/speed_sign/debug_image
```

---

# 8. STEP 5 — 20 / 50 실제 Detection

각 표지판에 대해:

```text
근거리
중거리
원거리
```

가능하면:

```text
정면
약간 좌측
약간 우측
역광
그늘
```

도 기록한다.

기록값:

```text
distance
class
confidence
bbox size
bbox area ratio
detection success/fail
FPS
```

## PASS 기준

- 20 → `/Perception/speed_limit = 20`
- 50 → `/Perception/speed_limit = 50`
- unsupported class → 0
- 반복적으로 20/50이 뒤바뀌지 않음
- 원거리에서 의미 있는 detection 가능

---

# 9. STEP 6 — ROI / Confidence / bbox 튜닝

먼저:

```text
roi_enabled = false
```

상태에서 실제 표지판 위치를 확인한다.

그 다음:

```text
roi_enabled = true
```

로 변경한다.

검증 순서:

1. 같은 차선의 20 표지판 포함
2. 같은 차선의 50 표지판 포함
3. 반대 차선 표지판 최대한 제외
4. 원거리 표지판 ROI 내부 유지
5. 근거리 bbox clipping 없음
6. confidence threshold 조정
7. `min_bbox_area_ratio` 조정

주의:

```text
min_bbox_area_ratio ↑
→ 작은 bbox를 더 많이 제거
```

따라서 원거리 표지판을 살리려면 지나치게 높이지 않는다.

---

# 10. STEP 7 — Planning + 실제 Perception 연동

차량 구동은 비활성화한다.

```text
enable_vehicle_interface = false
```

확인:

```bash
ros2 topic echo /Perception/speed_limit
ros2 topic echo /Planning/behavior
ros2 topic echo /Planning/target_velocity
```

Scenario:

```text
기본 상태
→ active 20
→ target = speed_limit_20_target

50 × 3
→ active 50
→ target = speed_limit_50_target

50 confirmed 후 0 반복
→ active 50 유지

20 × 3
→ active 20
→ target = speed_limit_20_target
```

---

# 11. STEP 8 — Full Stack Dry Run

```bash
ros2 launch behavior_stack_bringup behavior_stack.launch.py   enable_gps:=false   enable_imu:=false   enable_localization:=false   enable_lidar:=false   enable_camera_sign:=false   enable_vehicle_interface:=false
```

확인:

```bash
ros2 node list
ros2 topic list

ros2 topic echo /Planning/target_velocity
ros2 topic echo /Planning/behavior
ros2 topic echo /Control/serial_data
```

PASS 기준:

- Planning node 정상
- Controller node 정상
- localization invalid 상황에서 unsafe speed command 없음
- `enable_vehicle_interface=false` 상태에서 ERP actuation 없음

---

# 12. STEP 9 — Sensor별 연결 확인

순서:

```text
1. GPS
2. IMU
3. Localization
4. LiDAR
5. LiDAR clustering
6. Relative → UTM
7. Camera
8. Perception
9. Planning
10. Controller
11. ERP feedback
```

확인 topic:

```bash
ros2 topic hz /fix
ros2 topic hz /imu
ros2 topic hz /Local/utm
ros2 topic hz /Local/heading
ros2 topic hz /velodyne_points
ros2 topic hz /LiDAR/object_cen
ros2 topic hz /Convert/small_object_UTM
ros2 topic hz /Perception/speed_limit
ros2 topic hz /Planning/local_path
```

기록:

```text
Expected Hz:
Measured Hz:
PASS/FAIL:
```

---

# 13. STEP 10 — LiDAR → UTM 좌표 정합

차량 정차 상태에서 장애물을 다음 위치에 둔다.

```text
A. 전방 5 m / center
B. 전방 5 m / left 1 m
C. 전방 5 m / right 1 m
```

확인:

```bash
ros2 topic echo /Local/utm
ros2 topic echo /LiDAR/object_cen
ros2 topic echo /Convert/small_object_UTM
```

PASS 기준:

- 전후 방향 일치
- 좌우 방향 일치
- 좌/우 부호 반전 없음
- sensor offset 반영 정상
- heading 변화 시 transform 방향 정상

---

# 14. STEP 11 — Controller Dry Run

아직:

```text
enable_vehicle_interface = false
```

상태를 유지한다.

확인:

```bash
ros2 topic echo /Planning/behavior
ros2 topic echo /Planning/target_velocity
ros2 topic echo /Planning/local_path
ros2 topic echo /Control/serial_data
```

검증:

| Planning 상태 | Controller 기대값 |
|---|---|
| target=0 | speed command=0 |
| target>0 | positive speed command |
| left path | steering 한 방향 |
| right path | steering 반대 방향 |
| EMERGENCY_STOP | speed=0 |
| NO_VALID_PATH | speed=0 |

---

# 15. STEP 12 — 차량 / 조종기 사전 확인

조종기 기준:

```text
E 레버
위   = 전진
아래 = 후진

C 다이얼
맨 아래 = 중립 ON
주행 시 중립 해제

왼쪽 조이스틱
스로틀 / 브레이크

오른쪽 조이스틱
조향

A 버튼
Autopilot mode

B 버튼
E-stop
```

주의:

```text
B E-stop ≠ 일반 Brake
```

차량 Panel:

```text
Mode LED:
주황 = Manual
초록 = Autopilot

Controller LED:
초록 = Connected
빨강 = Disconnected
```

실차 actuation 전 확인 순서:

1. Controller 연결 LED 확인
2. Manual mode 확인
3. E lever 방향 확인
4. C dial neutral 상태 확인
5. throttle / brake 확인
6. steering 좌우 확인
7. A 버튼 Autopilot 전환 확인
8. B E-stop 확인

---

# 16. STEP 13 — ERP Stationary / Wheels-Off-Ground Test

차량이 실제 이동하지 않는 안전상태에서 수행한다.

이 단계에서 처음:

```text
enable_vehicle_interface = true
```

를 허용한다.

순서:

```text
1. speed 0
2. 최소 positive speed
3. speed 0 복귀
4. steering left
5. steering right
6. E-stop
7. E-stop release
8. command / feedback 비교
```

확인:

```bash
ros2 topic echo /Control/serial_data
ros2 topic echo /ERP/serial_data
```

PASS 기준:

- speed direction 정상
- steering sign 정상
- command / feedback 일치
- brake 정상
- E-stop 정상
- command 제거 후 0 복귀

FAIL 시 실차 주행 금지.

---

# 17. STEP 14 — 20 / 50 Target 실제 속도 Calibration

현재 provisional:

```text
speed_limit_20_target = 2.5
speed_limit_50_target = 4.5
```

실제 차량에서 확인:

```text
Planning target
→ Controller command
→ ERP feedback
→ 실제 vehicle speed
```

측정:

```text
target = 2.5
Actual speed = ?

target = 4.5
Actual speed = ?
```

필요한 경우 `planning.yaml`만 수정한다.

---

# 18. STEP 15 — 제한된 저속 실차

순서:

## 18.1 CRUISE

```text
기본 active = 20
```

확인:

- global path tracking
- steering oscillation
- target / actual speed

## 18.2 50 Sign

```text
50 × confirmation
→ active 50
```

확인:

- raw detection
- confirmation
- target 변경
- 실제 속도 응답

## 18.3 20 Sign

```text
20 × confirmation
→ active 20
```

확인:

- target 복귀
- 실제 감속

## 18.4 Static Obstacle

```text
CRUISE
→ AVOID
→ CRUISE
```

확인:

- obstacle detection
- Behavior transition
- candidate 선택
- clearance
- recovery

## 18.5 NO_VALID_PATH

위험한 실제 blocking 상황을 만들지 말고 통제된 환경 또는 mock 입력으로 검증한다.

기대:

```text
valid candidates = 0
behavior = EMERGENCY_STOP
target_velocity = 0
control speed = 0
```

## 18.6 E-stop

마지막으로 물리 E-stop을 확인한다.

---

# 19. 기록할 Topic

```text
/camera/image_raw
/Perception/speed_limit
/Perception/speed_sign/debug_image

/Local/utm
/Local/heading

/LiDAR/object_cen
/Convert/small_object_UTM

/Planning/behavior
/Planning/target_velocity
/Planning/local_path
/Planning/path_yaw
/Planning/curvature
/Planning/debug/candidates

/Control/serial_data
/ERP/serial_data
```

가능하면 rosbag으로 동시에 저장한다.

---

# 20. 최종 PASS / FAIL

| 항목 | PASS/FAIL | 측정값 / 로그 | 비고 |
|---|---|---|---|
| Foxy clean build | PASS |  | 완료 |
| Planning unit test | PASS | 17 PASS | 완료 |
| Perception helper test | PASS | 6 PASS | 완료 |
| ROS2 offline integration | PASS | errors 0 / failures 0 | 완료 |
| 실제 PT load |  |  |  |
| 실제 model.names |  |  |  |
| 20 class mapping |  |  |  |
| 50 class mapping |  |  |  |
| 20 detection |  |  |  |
| 50 detection |  |  |  |
| Unsupported class rejection |  |  |  |
| ROI |  |  |  |
| Confidence threshold |  |  |  |
| min bbox area |  |  |  |
| 20 confirmation |  |  |  |
| 50 confirmation |  |  |  |
| Active limit persistence |  |  |  |
| LiDAR → UTM 정합 |  |  |  |
| Controller speed mapping |  |  |  |
| Controller steering sign |  |  |  |
| Localization invalid → speed 0 |  |  |  |
| ERP feedback |  |  |  |
| Manual throttle / brake |  |  |  |
| Manual steering |  |  |  |
| Manual forward / reverse |  |  |  |
| Autopilot mode switch |  |  |  |
| Physical E-stop |  |  |  |
| Wheels-off-ground actuation |  |  |  |
| 20 target actual speed |  |  |  |
| 50 target actual speed |  |  |  |
| CRUISE |  |  |  |
| AVOID |  |  |  |
| AVOID → CRUISE |  |  |  |
| NO_VALID_PATH → EMERGENCY_STOP |  |  |  |
| Low-speed real vehicle test |  |  |  |

---

# 21. 최종 진행 순서

```text
[완료]
20/50 구조 변경
→ Build
→ Unit Test
→ ROS2 Offline Integration

[다음]
PT 파일 적용
→ model.names 확인
→ 20/50 이미지 inference
→ 실제 Camera
→ ROI / Confidence tuning
→ Planning + Perception
→ Full Stack Dry Run
→ Sensor별 연결
→ LiDAR → UTM 정합
→ Controller Dry Run
→ 조종기 Manual 확인
→ ERP Wheels-Off-Ground Test
→ 20/50 Target Calibration
→ 저속 실차
```

원칙:

```text
앞 단계 FAIL
→ 다음 단계 진행 금지
```
