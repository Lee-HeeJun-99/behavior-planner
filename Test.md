# Ubuntu 20.04 / ROS 2 Foxy 검증 절차

이 문서는 `lhj_behavior_stage`를 Ubuntu 20.04 + ROS 2 Foxy 차량 PC에서 검증하는 순서다.
모든 차량 구동 시험 전까지 `enable_vehicle_interface:=false`를 유지한다.

## 1. 검증 범위

- Foxy clean build
- Planning 및 ROI helper unit test
- launch/config 로딩
- 카메라 → ROI → YOLO → `/Perception/speed_limit`
- Planning speed confirmation과 상태 유지
- `CRUISE → AVOID → CRUISE`
- `NO_VALID_PATH → EMERGENCY_STOP`
- Planning → Controller 연결
- Controller → ERP42 연결 전 dry run

실차에서 별도로 확정해야 하는 값:

- 카메라 topic, 해상도, FPS, 장착 위치
- YOLO weight와 class 이름 (`speed_30`, `speed_40`, `speed_50`)
- ROI와 `min_bbox_area_ratio`
- confidence threshold와 검출 거리
- 30/40/50 ERP target 및 `avoid_max_velocity`
- LiDAR ROI/extrinsic, 도로 경계, Controller calibration

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

## 7. Launch 로딩 확인

```bash
source /opt/ros/foxy/setup.bash
source /home/ubuntu/lhj_behavior_stage/install/setup.bash
ros2 launch traffic_sign_perception speed_sign.launch.py --show-args
ros2 launch behavior_stack_bringup behavior_stack.launch.py --show-args
```

Python import error, package-not-found, parameter type error가 없어야 한다.

## 8. Perception 무게 파일 없음/잘못된 ROI 안전 시험

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

Node가 종료되면 FAIL이다.

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
6. 작은 원거리 bbox가 필요하면 `min_bbox_area_ratio`를 0부터 조금씩 증가
7. 역광, 그늘, 야간에서 confidence 기록

ROI와 bbox threshold 변경 후에는 node를 재시작한다.

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
CRUISE                    target_velocity 2.5
SINGLE_FALSE_50           target_velocity 2.5
CONFIRMED_50              target_velocity 4.5
NO_DETECTION_AFTER_50     target_velocity 4.5
SINGLE_OBSTACLE / AVOID   target_velocity 2.5
RECOVERY / CRUISE         target_velocity 4.5
EMERGENCY_STOP            target_velocity 0.0
EMERGENCY_RELEASE         target_velocity 4.5
BLOCKED                    valid 0, target_velocity 0.0
```

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
```

Localization을 비활성화했으므로 Planning의 localization 대기 warning은 정상이다.

## 12. Sensor별 단계 실행

다음 순서로 하나씩 활성화하고 각 단계에서 topic rate/type을 확인한다.

1. GPS: `/fix`
2. IMU: `/imu`
3. ERP feedback: `/ERP/serial_data`
4. Localization: `/Local/utm`, `/Local/heading`
5. Velodyne: `/velodyne_points`
6. LiDAR clustering: `/LiDAR/object_cen`
7. 변환: `/Convert/small_object_UTM`
8. Camera/YOLO: `/Perception/speed_limit`
9. Planning: `/Planning/local_path`, `/Planning/target_velocity`
10. Controller: `/Control/serial_data`

예시:

```bash
ros2 topic info /Local/utm
ros2 topic hz /Local/utm
ros2 topic info /Convert/small_object_UTM
ros2 topic hz /Convert/small_object_UTM
ros2 topic echo /Planning/behavior
ros2 topic echo /Planning/target_velocity
```

## 13. 차량 연결 전 안전 체크

- 차량 바퀴를 지면에서 분리하거나 제조사 권장 안전 상태 사용
- 물리 Emergency Stop 동작 확인
- 조향/속도 command 단위와 부호 확인
- ERP serial port 확인
- speed target을 가장 낮은 값부터 검증
- `enable_vehicle_interface`는 마지막 단계에서만 true
- 주변 사람과 장애물을 모두 제거
- 한 명은 즉시 E-stop을 누를 수 있도록 대기

포트 확인:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
groups
```

필요 시 재로그인 전에:

```bash
sudo usermod -aG dialout "$USER"
```

## 14. 제한된 실차 실행

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

처음에는 다음을 기록한다.

```bash
ros2 topic echo /Planning/behavior
ros2 topic echo /Perception/speed_limit
ros2 topic echo /Planning/target_velocity
```

## 15. 최종 PASS / FAIL 기록

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
| 장애물 회피/복귀 |  |  |
| Controller input 연결 |  |  |
| ERP feedback 연결 |  |  |
| 제한된 차량 actuation |  |  |

FAIL이 발생하면 사용한 commit, apt/pip 버전, config, rosbag 또는 console log를 함께 보존한다.
