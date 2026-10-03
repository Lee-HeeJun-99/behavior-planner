# Ubuntu 20.04 / ROS 2 Foxy 검증 절차

이 문서는 `lhj_behavior_stage`를 Ubuntu 20.04 + ROS 2 Foxy 차량 PC에서 검증하는 순서다.  
모든 차량 구동 시험 전까지 `enable_vehicle_interface:=false`를 유지한다.

---

## 1. 검증 범위

- Foxy clean build
- Planning 및 ROI helper unit test
- launch/config 로딩
- 카메라 → ROI → YOLO → `/Perception/speed_limit`
- Planning speed confirmation과 상태 유지
- `CRUISE → AVOID → CRUISE`
- `NO_VALID_PATH → EMERGENCY_STOP`
- LiDAR relative 좌표 → UTM 변환 정합
- Planning → Controller 연결
- Controller 출력값/부호/정지 명령 검증
- Controller → ERP42 연결 전 dry run
- ERP feedback 확인
- Wheels-off-ground 또는 제조사 권장 안전상태 actuation test
- 제한된 저속 실차 시험

실차에서 별도로 확정해야 하는 값:

- 카메라 topic, 해상도, FPS, 장착 위치
- YOLO weight와 class 이름 (`speed_30`, `speed_40`, `speed_50`)
- ROI와 `min_bbox_area_ratio`
- confidence threshold와 검출 거리
- 30/40/50 ERP target 및 `avoid_max_velocity`
- LiDAR ROI/extrinsic 및 sensor offset
- 실제 도로 좌/우 경계
- 차량 footprint (`vehicle_half_width`, `vehicle_front`, `vehicle_rear`)
- `safety_margin`
- lateral candidate offset 및 `maximum_curvature`
- Controller calibration 및 steering 부호/단위

---

## 2. 환경 확인

```bash
lsb_release -a
python3 --version
source /opt/ros/foxy/setup.bash
echo "$ROS_DISTRO"
ros2 pkg prefix rclpy
```

예상 결과:

```text
Ubuntu 20.04
Python 3.8.x
ROS_DISTRO=foxy
```

Foxy는 EOL이므로 검증 완료 후 apt/pip 패키지 버전을 기록하고 고정한다.

---

## 3. 시스템 의존성 설치

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  python3-serial \
  python3-opencv \
  python3-numpy \
  python3-scipy \
  libpcl-all-dev \
  nlohmann-json3-dev \
  ros-foxy-cv-bridge \
  ros-foxy-pcl-conversions \
  ros-foxy-velodyne-driver \
  ros-foxy-velodyne-pointcloud
```

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
cd /home/ubuntu/lhj_behavior_stage
rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy
python3 -m pip install --user utm
```

YOLO runtime은 Python 3.8과 weight 형식이 모두 맞는 버전을 사용한다. 예시:

```bash
python3 -m pip install --user "ultralytics==8.0.196"
python3 -c "import torch, ultralytics; print(torch.__version__, ultralytics.__version__)"
```

이 버전은 예시이며 실제 weight를 로드해 확인한 조합을 최종 고정한다.

---

## 4. Source 정적 확인

```bash
cd /home/ubuntu/lhj_behavior_stage
grep -RIn "/opt/ros/humble\|Ubuntu 22.04\|ROS 2 Humble" \
  README.md docs src --exclude-dir=__pycache__
grep -RIn "throttle_duration_sec" src
python3 -m py_compile \
  src/traffic_sign_perception/traffic_sign_perception/speed_sign_node.py \
  src/traffic_sign_perception/traffic_sign_perception/speed_sign_utils.py \
  src/traffic_sign_perception/launch/speed_sign.launch.py \
  src/behavior_stack_bringup/launch/behavior_stack.launch.py
```

예상 결과:

- Humble 전용 경로/문구 없음
- `throttle_duration_sec` 없음
- Python syntax error 없음

---

## 5. Foxy Clean Build

```bash
cd /home/ubuntu/lhj_behavior_stage
source /opt/ros/foxy/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

판정 기준:

- 모든 package build 성공
- compiler error 0
- linker error 0
- Controller의 기존 warning은 별도 기록

빌드 실패 시 기존 Humble workspace를 source하지 않았는지 확인한다.

```bash
env | grep -E "ROS_DISTRO|AMENT_PREFIX_PATH|COLCON_PREFIX_PATH"
```

---

## 6. Unit Test

```bash
cd /home/ubuntu/lhj_behavior_stage
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon test --packages-select planning_pkg_2025 traffic_sign_perception \
  --event-handlers console_direct+
colcon test-result --verbose
```

필수 PASS 항목:

- speed 50 연속 3회 → active 50
- `50, 0, 50` → active 변경 없음
- `50, 50, 40, 50` → candidate reset 후 잘못된 confirmation 없음
- 70 반복 검출 무시
- invalid default 70 → 30 fallback
- 0 반복 후 active limit 유지
- AVOID target 2.5 cap
- EMERGENCY_STOP target 0
- 해제 후 기존 active target 복귀
- ROI validation/pixel conversion
- ROI bbox → global bbox
- bbox area filtering와 best detection
- 기존 obstacle collision/boundary/local path 테스트
- NO_VALID_PATH 처리 관련 unit test가 존재하면 반드시 PASS

---

## 7. Launch 로딩 확인

```bash
source /opt/ros/foxy/setup.bash
source /home/ubuntu/lhj_behavior_stage/install/setup.bash
ros2 launch traffic_sign_perception speed_sign.launch.py --show-args
ros2 launch behavior_stack_bringup behavior_stack.launch.py --show-args
```

PASS 기준:

- Python import error 없음
- package-not-found 없음
- parameter type error 없음
- launch argument가 정상 출력됨

---

## 8. Perception 무게 파일 없음 / 잘못된 ROI 안전 시험

터미널 1:

```bash
source /opt/ros/foxy/setup.bash
source /home/ubuntu/lhj_behavior_stage/install/setup.bash
ros2 run traffic_sign_perception speed_sign_node --ros-args \
  -p weights_path:="" \
  -p roi_enabled:=true \
  -p roi_x_min:=0.9 \
  -p roi_x_max:=0.1
```

터미널 2:

```bash
source /opt/ros/foxy/setup.bash
source /home/ubuntu/lhj_behavior_stage/install/setup.bash
python3 /home/ubuntu/lhj_behavior_stage/src/traffic_sign_perception/test/ros2_perception_integration.py
```

예상 결과:

```text
Invalid normalized ROI ... falling back to the full image
YOLO weights_path is empty ...
speed_limit=0
debug_encoding=bgr8
```

PASS 기준:

- node가 종료되지 않음
- `/Perception/speed_limit`가 계속 publish됨
- publish 값이 `0`
- invalid ROI 시 full image fallback
- debug image가 정상 publish됨

Node가 종료되거나 예외 후 publish가 중단되면 FAIL이다.

---

## 9. 실제 카메라와 ROI 확인

먼저 카메라 driver를 별도로 실행한다. 이 workspace에는 특정 카메라 driver가 포함되어 있지 않다.

```bash
ros2 topic list | grep camera
ros2 topic info /camera/image_raw
ros2 topic hz /camera/image_raw
```

Perception 실행:

```bash
ros2 launch behavior_stack_bringup behavior_stack.launch.py \
  enable_gps:=false \
  enable_imu:=false \
  enable_localization:=false \
  enable_lidar:=false \
  enable_camera_sign:=true \
  enable_vehicle_interface:=false \
  camera_image_topic:=/camera/image_raw \
  speed_sign_weights:=speed_sign.pt \
  speed_sign_roi_enabled:=true \
  speed_sign_publish_debug:=true
```

확인:

```bash
ros2 topic echo /Perception/speed_limit
ros2 topic hz /Perception/speed_limit
rqt_image_view /Perception/speed_sign/debug_image
```

ROI 실험 순서:

1. 정차 상태에서 전체 이미지(`roi_enabled: false`)의 표지판 위치 확인
2. `roi_enabled: true`로 변경
3. 동일 차선 표지판이 충분히 포함되는지 확인
4. 반대 차선 표지판이 제외되는지 확인
5. 근거리/원거리에서 bbox가 유지되는지 확인
6. `min_bbox_area_ratio=0`에서 시작
7. false positive 제거를 위해 값을 조금씩 증가
8. 원거리 표지판 bbox가 제거되기 시작하는 임계값을 확인
9. 원거리 표지판이 유지되는 범위 내에서 최종값 결정
10. 역광, 그늘, 야간에서 confidence 기록

주의:

- `min_bbox_area_ratio`를 증가시키면 작은 bbox는 더 많이 제거된다.
- 원거리 표지판을 살리기 위해서는 값을 낮게 유지해야 한다.

ROI와 bbox threshold 변경 후에는 node를 재시작한다.

기록 항목:

- 카메라 FPS
- perception output Hz
- detection distance
- class
- confidence
- bbox 크기
- ROI 내부/외부 여부
- false positive 여부

---

## 10. Planning ROS2 Integration Test

터미널 1:

```bash
source /opt/ros/foxy/setup.bash
source /home/ubuntu/lhj_behavior_stage/install/setup.bash
ros2 run planning_pkg_2025 planning_node --ros-args \
  --params-file /home/ubuntu/lhj_behavior_stage/src/planning/test/integration_planning.yaml
```

터미널 2:

```bash
source /opt/ros/foxy/setup.bash
source /home/ubuntu/lhj_behavior_stage/install/setup.bash
python3 /home/ubuntu/lhj_behavior_stage/src/planning/test/ros2_local_planner_integration.py \
  /home/ubuntu/lhj_behavior_stage/src/planning/map/map_final_0921/0-0.txt
```

예상 핵심 결과:

```text
CRUISE
  behavior CRUISE
  target_velocity 2.5

SINGLE_FALSE_50
  target_velocity 2.5

CONFIRMED_50
  target_velocity 4.5

NO_DETECTION_AFTER_50
  target_velocity 4.5

SINGLE_OBSTACLE
  behavior AVOID
  target_velocity 2.5

RECOVERY
  behavior CRUISE
  target_velocity 4.5

EMERGENCY_STOP
  behavior EMERGENCY_STOP
  target_velocity 0.0

EMERGENCY_RELEASE
  behavior CRUISE
  target_velocity 4.5

BLOCKED
  valid_candidates 0
  behavior EMERGENCY_STOP
  target_velocity 0.0
```

필수 확인:

- `CRUISE → AVOID → CRUISE` 전이가 정상인지
- AVOID 시 valid candidate가 존재하는지
- selected offset이 obstacle을 실제로 피하는 방향인지
- NO_VALID_PATH 시 `valid_candidates=0`
- NO_VALID_PATH 시 behavior가 `EMERGENCY_STOP`
- NO_VALID_PATH 시 `target_velocity=0.0`
- emergency release 후 기존 active speed target으로 복귀하는지

---

## 11. 전체 Stack Dry Run

차량 구동을 비활성화한다.

```bash
ros2 launch behavior_stack_bringup behavior_stack.launch.py \
  enable_gps:=false \
  enable_imu:=false \
  enable_localization:=false \
  enable_lidar:=false \
  enable_camera_sign:=false \
  enable_vehicle_interface:=false
```

다른 터미널에서 확인:

```bash
ros2 node list
ros2 topic list
ros2 topic info /Planning/local_path
ros2 topic info /Planning/target_velocity
ros2 topic info /Control/serial_data
ros2 topic echo /Control/serial_data
```

Localization을 비활성화했으므로 Planning의 localization 대기 warning은 정상이다.

PASS 기준:

- planning node alive
- controller node alive
- `/Planning/target_velocity`가 안전 상태 값 유지
- localization invalid 상태에서 `/Control/serial_data`의 speed command가 0
- vehicle interface가 비활성화되어 ERP serial port를 열지 않음

---

## 12. Sensor별 단계 실행

다음 순서로 하나씩 활성화하고 각 단계에서 topic type, rate, 값의 정상 여부를 확인한다.

1. GPS: `/fix`
2. IMU: `/imu`
3. GPS + IMU 입력 안정성 확인
4. Localization: `/Local/utm`, `/Local/heading`
5. Velodyne: `/velodyne_points`
6. LiDAR clustering: `/LiDAR/object_cen`
7. 상대좌표 → UTM 변환: `/Convert/small_object_UTM`
8. Camera/YOLO: `/Perception/speed_limit`
9. Planning: `/Planning/local_path`, `/Planning/target_velocity`, `/Planning/behavior`
10. Controller: `/Control/serial_data`
11. ERP feedback: 차량 안전상태에서 `/ERP/serial_data`

예시:

```bash
ros2 topic info /Local/utm
ros2 topic hz /Local/utm

ros2 topic info /velodyne_points
ros2 topic hz /velodyne_points

ros2 topic info /Convert/small_object_UTM
ros2 topic hz /Convert/small_object_UTM

ros2 topic echo /Planning/behavior
ros2 topic echo /Planning/target_velocity

ros2 topic echo /Control/serial_data
```

Rate 판정 기준은 실제 설정값과 센서 spec을 기준으로 기록한다.

권장 기록 방식:

```text
Expected Hz:
Measured Hz:
Minimum acceptable Hz:
PASS/FAIL:
```

예:

```text
Planning expected: 20 Hz
Planning minimum acceptable: 18 Hz
```

정확한 허용범위는 최종 차량 환경에서 확정한다.

---

## 13. LiDAR Relative → UTM 좌표 정합 실험

Planning의 장애물 판단 전에 반드시 수행한다.

### 13.1 정차 상태 준비

차량을 평탄한 장소에 정차한다.

확인:

```bash
ros2 topic echo /Local/utm
ros2 topic echo /LiDAR/object_cen
ros2 topic echo /Convert/small_object_UTM
```

### 13.2 장애물 배치

최소 다음 위치에서 cone 또는 고정 물체를 배치한다.

```text
Case A: 차량 기준 전방 5 m, lateral 0 m
Case B: 차량 기준 전방 5 m, 좌측 1 m
Case C: 차량 기준 전방 5 m, 우측 1 m
```

가능하면 추가:

```text
전방 3 m
전방 10 m
```

### 13.3 검증

확인 항목:

- `/LiDAR/object_cen`의 relative 좌표가 실제 배치 방향과 일치
- `/Convert/small_object_UTM`의 위치가 `/Local/utm` 기준 실제 장애물 위치와 일치
- 좌/우 부호가 반전되지 않음
- heading 변화 시 회전변환 방향이 정상
- sensor longitudinal offset이 실제 장착 위치와 맞음

PASS 기준:

```text
Vehicle UTM + transformed relative obstacle
≈ measured obstacle UTM
```

허용오차는 실측 환경과 GPS 정확도를 고려하여 별도 기록한다.

반드시 기록할 값:

- 차량 UTM
- LiDAR relative x/y
- 변환된 obstacle UTM
- 실제 측정 거리/방향
- 오차

---

## 14. Planning Parameter 실차 정합 확인

실차 주행 전 다음 config가 실제 차량과 맞는지 확인한다.

### 차량 footprint

확인:

```text
vehicle_half_width
vehicle_front
vehicle_rear
safety_margin
```

실제 차량 치수와 비교한다.

### 도로 경계

Global Path 기준 실제 도로 좌/우 여유폭을 측정하고 다음과 비교한다.

```text
road_left_bound
road_right_bound
```

### lateral candidate

다음 offset 후보가 실제 도로 폭과 차량 조향 성능 안에 있는지 확인한다.

```text
-1.2
-0.9
-0.6
-0.3
0.0
0.3
0.6
0.9
1.2
```

### curvature

최대 offset candidate의:

```text
maximum_curvature
steering command
```

를 확인한다.

PASS 기준:

- 차량 footprint가 실제 차량보다 작게 설정되지 않음
- candidate가 도로 경계를 침범하지 않음
- 최대 candidate가 차량 조향 한계를 초과하지 않음
- safety margin이 실제 장애물 회피에 충분함

---

## 15. Controller Dry Run 검증

`enable_vehicle_interface:=false` 상태에서 Planning과 Controller만 연결한다.

확인할 mapping:

| Planning 상태/입력 | Controller 기대 결과 |
|---|---|
| `target_velocity=0` | speed command = 0 |
| `target_velocity>0` | 양의 speed command |
| 좌회전 path | steering 한쪽 부호 |
| 우회전 path | steering 반대 부호 |
| `EMERGENCY_STOP` | speed command = 0 |
| `NO_VALID_PATH` | speed command = 0 |

확인:

```bash
ros2 topic echo /Planning/target_velocity
ros2 topic echo /Planning/local_path
ros2 topic echo /Planning/behavior
ros2 topic echo /Control/serial_data
```

PASS 기준:

- speed 단위 변환 정상
- steering 부호 정상
- steering saturation 정상
- emergency 상태에서 speed 0
- invalid localization 또는 no valid path 상태에서 speed 0

---

## 16. End-to-End 반응시간 확인

실차 전 mock obstacle 또는 정적 장애물 입력을 사용해 다음 latency를 기록한다.

```text
t0: obstacle input/detection
t1: /Planning/behavior = AVOID
t2: /Planning/local_path 변경
t3: /Control/serial_data 변경
```

확인 항목:

- obstacle detection confirmation에 필요한 시간
- Planning 20 Hz 주기 영향
- Controller까지 전달되는 전체 latency
- clear confirmation 후 `AVOID → CRUISE` 복귀 시간

기록:

```text
Obstacle → AVOID:
AVOID → local_path change:
local_path → control output:
Total:
```

초기에는 절대 기준보다 반복 측정 시 일관성이 있는지를 우선 확인한다.

---

## 17. 차량 연결 전 안전 체크

- 차량 바퀴를 지면에서 분리하거나 제조사 권장 안전 상태 사용
- 물리 Emergency Stop 동작 확인
- 조향/속도 command 단위와 부호 확인
- ERP serial port 확인
- speed target을 가장 낮은 값부터 검증
- 주변 사람과 장애물을 모두 제거
- 한 명은 즉시 E-stop을 누를 수 있도록 대기
- command와 feedback topic을 동시에 모니터링
- `enable_vehicle_interface`는 이 단계 이전까지 false 유지

포트 확인:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
groups
```

필요 시 재로그인 전에:

```bash
sudo usermod -aG dialout "$USER"
```

---

## 18. ERP42 Stationary / Wheels-Off-Ground Actuation Test

차량이 실제 이동하지 않는 안전상태에서만 수행한다.

이 단계에서 처음으로:

```text
enable_vehicle_interface:=true
```

를 허용한다.

최소 시험 순서:

1. speed = 0 상태 확인
2. 최소 양의 speed command
3. command 제거 후 speed 0 복귀
4. steering 소량 좌측
5. steering 소량 우측
6. E-stop
7. E-stop 해제 후 speed 0 유지 확인
8. `/Control/serial_data`와 `/ERP/serial_data` 비교

확인:

```bash
ros2 topic echo /Control/serial_data
ros2 topic echo /ERP/serial_data
```

PASS 기준:

- command와 실제 feedback 방향 일치
- steering 좌/우 부호 일치
- speed 단위 정상
- brake 값 정상 범위
- E-stop 즉시 반영
- 통신 해제/재시작 후 비정상 command 발생 없음

FAIL 시 실차 주행 금지.

---

## 19. 제한된 저속 실차 실행

안전 책임자 승인 후에만 실행한다.

```bash
ros2 launch behavior_stack_bringup behavior_stack.launch.py \
  gps_port:=/dev/ttyUSB0 \
  imu_port:=/dev/ttyUSB1 \
  erp_port:=/dev/ttyUSB2 \
  velodyne_ip:=192.168.1.201 \
  enable_camera_sign:=true \
  camera_image_topic:=/camera/image_raw \
  speed_sign_weights:=speed_sign.pt \
  enable_vehicle_interface:=true
```

초기 실차 시험은 가장 낮은 target velocity에서 수행한다.

### 19.1 CRUISE

확인:

- global path 정상 추종
- steering oscillation 없음
- speed command/feedback 정상

### 19.2 Speed Sign

확인:

- raw detection
- 3-frame confirmation
- active speed state 유지
- target velocity 변경

### 19.3 Static Obstacle Avoidance

확인:

- obstacle detection
- `CRUISE → AVOID`
- candidate 선택
- 실제 장애물 clearance
- `AVOID → CRUISE` 복귀

### 19.4 NO_VALID_PATH

실제 차량을 위험하게 막지 말고, 안전한 mock 또는 통제된 환경에서 검증한다.

확인:

```text
valid_candidates = 0
behavior = EMERGENCY_STOP
target_velocity = 0
control speed command = 0
```

### 19.5 Emergency Stop

물리 E-stop과 software emergency 상태를 각각 확인한다.

처음에는 다음을 기록한다.

```bash
ros2 topic echo /Planning/behavior
ros2 topic echo /Perception/speed_limit
ros2 topic echo /Planning/target_velocity
ros2 topic echo /Control/serial_data
ros2 topic echo /ERP/serial_data
```

가능하면 rosbag으로 핵심 topic을 함께 기록한다.

---

## 20. 최종 PASS / FAIL 기록

| 항목 | PASS/FAIL | 측정값/로그 |
|---|---|---|
| Foxy clean build |  |  |
| Planning unit test |  |  |
| ROI helper test |  |  |
| Launch import |  |  |
| Missing weight 안전 처리 |  |  |
| Invalid ROI fallback |  |  |
| Debug image ROI/bbox |  |  |
| 30/40/50 raw detection |  |  |
| Planning 3-frame confirmation |  |  |
| CRUISE target mapping |  |  |
| AVOID velocity cap |  |  |
| EMERGENCY_STOP 0속도 |  |  |
| NO_VALID_PATH → EMERGENCY_STOP |  |  |
| 장애물 회피/복귀 |  |  |
| LiDAR relative → UTM 정합 |  |  |
| 차량 footprint 검증 |  |  |
| 도로 좌/우 경계 검증 |  |  |
| lateral candidate 실차 가능성 |  |  |
| Planning runtime frequency |  |  |
| Obstacle → AVOID latency |  |  |
| Controller input 연결 |  |  |
| Localization invalid → Control speed 0 |  |  |
| Steering command 부호/단위 |  |  |
| Controller speed 단위 |  |  |
| ERP feedback 연결 |  |  |
| E-stop command/feedback |  |  |
| Wheels-off-ground actuation |  |  |
| 제한된 차량 actuation |  |  |

FAIL이 발생하면 다음을 함께 보존한다.

- 사용한 git commit
- apt package 버전
- pip package 버전
- planning/config 값
- camera/YOLO 설정
- rosbag
- console log
- 실패 시점의 topic echo
- 실제 차량/장애물 배치 조건

---

## 21. 최종 실험 순서 요약

```text
1. 환경 확인
2. 의존성 설치
3. Source 정적 확인
4. Foxy Clean Build
5. Unit Test
6. Launch 확인
7. Perception fail-safe
8. Camera / ROI
9. Planning Integration
10. Full Stack Dry Run
11. Sensor별 입력 확인
12. LiDAR → UTM 좌표 정합
13. Planning parameter 실차 정합
14. Controller Dry Run
15. End-to-End latency
16. 차량 연결 전 안전 체크
17. Wheels-Off-Ground ERP actuation
18. 제한된 저속 실차
19. 최종 PASS / FAIL 정리
```

원칙:

```text
Offline
→ Sensor 단독
→ Planning Integration
→ Full Stack Dry Run
→ 좌표/파라미터 정합
→ Controller 출력 검증
→ ERP 안전 actuation
→ 저속 실차
```

앞 단계가 FAIL이면 다음 단계로 진행하지 않는다.
'''

path = Path("/mnt/data/Test_revised.md")
path.write_text(content, encoding="utf-8")
print(path)
