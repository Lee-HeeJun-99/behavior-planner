# Behavior Stack Debug Run Guide

이 문서는 Ubuntu 20.04 / ROS 2 Foxy 배포 환경을 기준으로 한다. 모든 기본 launch는 차량
actuation을 비활성화한다. 저장소의 `erp_ros2_bridge`는 legacy source이며 active launch에는
포함되지 않는다.

## 1. Build와 공통 환경

```bash
cd ~/lhj_behavior_stage
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

주요 추가 의존성은 `python3-can`, `python3-serial`, `python3-numpy`, `python3-scipy`,
Python `utm`, PCL, `cv_bridge`, `rviz2`, `rqt_image_view`이다. YOLO를 사용할 때만 배포
환경과 호환되는 PyTorch/Ultralytics가 필요하다.

## 2. Runtime node와 topic

| Node (`ros2 run`) | Input | Output | 역할 |
|---|---|---|---|
| `nmea_navsat_driver nmea_serial_driver` | GPS serial | `/fix` | NMEA GPS |
| `wit_ros2_imu wit_ros2_imu` | IMU serial | `/imu` | WIT IMU |
| `local_pkg1 heading_estimator` | `/fix`, `/imu`, `/ERP/serial_data` | `/Local/heading` | 기존 heading 계산 |
| `local_pkg1 position_estimator` | `/fix`, `/imu`, `/ERP/serial_data` | `/Local/utm`, `/beta` | 기존 position/DR 계산 |
| `lidar start` | `/velodyne_points` | `/LiDAR/object_cen`, `/Cluster/small_static` | 정적 장애물 clustering |
| `kroad_planning_utm_pkg relative_2_UTM` | `/Local/utm`, `/Local/heading`, `/LiDAR/object_cen` | `/Convert/small_object_UTM` | 상대좌표→UTM |
| `traffic_sign_perception speed_sign_node` | `/camera/image_raw` | `/Perception/speed_limit`, `/Perception/speed_sign/debug_image` | ROI/YOLO raw 20·50 표지판 |
| `planning_pkg_2025 planning_node` | localization, obstacle, speed sign | `/Planning/*` | Context→Behavior→Local path |
| `control car_control` | localization, planning, ERP feedback | `/Control/vehicle_cmd` | 기존 제어기 |
| `erp42pro_interface can_bridge_node` | `/Control/vehicle_cmd`, CAN RX | `/ERP/serial_data`, CAN TX | ERP42 Pro CAN bridge |
| `behavior_stack_bringup debug_visualizer_node` | debug/runtime topics | Marker topics | RViz용 표시 |

`src/convertcs`는 디렉터리 이름이고 실제 ROS package 이름은
`kroad_planning_utm_pkg`이다. `local_pkg1 tae_localization`은 compatibility executable로
유지한다. 분리 노드는 수치 회귀를 피하기 위해 같은 기존 계산 core를 사용한다.

## 3. 노드별 실행

```bash
ros2 run local_pkg1 heading_estimator
ros2 run local_pkg1 position_estimator
ros2 run lidar start
ros2 run kroad_planning_utm_pkg relative_2_UTM
ros2 run traffic_sign_perception speed_sign_node
ros2 run planning_pkg_2025 planning_node
ros2 run control car_control
ros2 run erp42pro_interface can_bridge_node --ros-args -p receive_only:=true -p dry_run:=true
```

마지막 명령은 CAN을 열지 않는 변환/기동 확인이다.

## 4. Subsystem launch

```bash
ros2 launch behavior_stack_bringup sensors.launch.py
ros2 launch behavior_stack_bringup localization.launch.py
ros2 launch behavior_stack_bringup perception.launch.py enable_speed_sign:=false
ros2 launch behavior_stack_bringup planning.launch.py
ros2 launch behavior_stack_bringup control.launch.py
ros2 launch behavior_stack_bringup vehicle_interface.launch.py
ros2 launch behavior_stack_bringup visualization.launch.py
```

전체 stack의 안전한 offline 기동 예시는 다음과 같다.

```bash
ros2 launch behavior_stack_bringup behavior_stack.launch.py \
  enable_gps:=false enable_imu:=false enable_lidar_sensor:=false \
  enable_localization:=false enable_lidar_perception:=false \
  enable_camera_sign:=false enable_vehicle_interface:=false \
  enable_visualization:=true
```

주요 통합 인자는 `enable_*`, `gps_port`, `imu_port`, `velodyne_ip`,
`camera_image_topic`, `speed_sign_weights`, `speed_sign_roi_enabled`,
`speed_sign_publish_debug`, `can_channel`, `can_receive_only`,
`can_max_speed_kph`, `can_bringup`, `can_dry_run`이다. 전체 목록은 다음으로 확인한다.

```bash
ros2 launch behavior_stack_bringup behavior_stack.launch.py --show-args
```

## 5. Logitech USB camera

Behavior Stack은 특정 Logitech 모델/드라이버를 실행하지 않는다. 드라이버가
`sensor_msgs/Image`를 제공하면 된다.

```bash
# 예 1: v4l2_camera가 설치된 경우
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 -r image_raw:=/camera/image_raw

# 예 2: usb_cam이 설치된 경우(배포판 package의 executable/parameter 확인 필요)
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 -r image_raw:=/camera/image_raw
```

드라이버 topic이 `/image_raw`이면 다음처럼 override한다.

```bash
ros2 launch behavior_stack_bringup perception.launch.py \
  enable_lidar_perception:=false enable_speed_sign:=true \
  camera_image_topic:=/image_raw speed_sign_weights:=<WEIGHT_PATH>
```

## 6. Camera/YOLO debug

```bash
ros2 topic hz /camera/image_raw
rqt_image_view /camera/image_raw
rqt_image_view /Perception/speed_sign/debug_image
# 또는
ros2 run rqt_image_view rqt_image_view
```

Debug image에는 ROI, 원본 좌표로 복원된 bbox, class, confidence, mapped limit(0/20/50)이
표시된다. weight가 비었거나 없으면 node는 종료하지 않고 매 frame `0`을 발행한다. ROI와
confidence/min bbox 값은 `traffic_sign_perception/config/speed_sign.yaml`에서 관리한다.

## 7. CAN 안전 확인

CAN device를 열지 않는 dry-run:

```bash
ros2 launch behavior_stack_bringup vehicle_interface.launch.py \
  enable_vehicle_interface:=true can_receive_only:=true can_dry_run:=true
```

실제 CAN receive-only(송신 없음):

```bash
ros2 launch behavior_stack_bringup vehicle_interface.launch.py \
  enable_vehicle_interface:=true can_channel:=can0 \
  can_receive_only:=true can_bringup:=false
```

송신 모드는 후속 실차 안전 절차에서만 사용한다. 명령 형식은
`[valid,e_stop,gear,speed_mps,steer_rad,brake_pct]`이며, 기본 safety 값은
`max_speed_kph=10`, `cmd_timeout_sec=0.3`, timeout brake 30%, steer limit 28 deg이다.
송신은 `enable_vehicle_interface:=true can_receive_only:=false`를 모두 명시해야 가능하다.
이 저장소 정리 단계에서는 실행하지 않는다.

## 8. RViz2

```bash
ros2 launch behavior_stack_bringup visualization.launch.py
```

Fixed Frame은 `map`이다. 확인 가능한 항목:

- `/velodyne_points`: raw PointCloud2
- `/Planning/debug/global_path`: transient-local global path
- `/Planning/debug/local_path`: selected local path
- `/Planning/debug/candidate_paths`: 실제 candidate geometry, valid/invalid/selected 색상과 label
- `/Planning/debug/ego`: footprint와 heading arrow
- `/Visualization/behavior`: current behavior text
- `/Visualization/lidar_relative_objects`: LiDAR 상대 검출
- `/Visualization/obstacles_utm`: Planning 입력 UTM 장애물

Raw LiDAR frame이 `map`과 다르고 기존 TF가 없다면 PointCloud display의 Fixed Frame을 센서
frame으로 바꿔 raw cloud만 별도 확인한다. 이번 작업에서는 새로운 TF 체계를 만들지 않았다.

## 9. 추천 디버깅 순서

1. `ros2 topic hz /fix`, `/imu`, `/velodyne_points`, 카메라 topic 확인
2. `/Local/heading`, `/Local/utm` 확인
3. `/LiDAR/object_cen`과 relative marker 비교
4. `/Convert/small_object_UTM`과 UTM marker 비교
5. raw/debug camera 영상을 나란히 확인
6. `/Perception/speed_limit`이 frame raw detection인지 확인
7. `/Planning/behavior`, local/global/candidate path 확인
8. `/Planning/target_velocity`와 `/Control/vehicle_cmd`는 echo만 수행
9. ERP는 dry-run 또는 receive-only로만 확인

```bash
ros2 topic list -t
ros2 topic info /Planning/debug/candidate_paths --verbose
ros2 topic echo /Planning/behavior
ros2 topic hz /Planning/debug/local_path
ros2 param list /planning_node
ros2 param get /planning_node avoid_max_velocity
ros2 param dump /planning_node
```

## 10. Offline test

```bash
colcon test --packages-select planning_pkg_2025 traffic_sign_perception \
  erp42pro_interface local_pkg1
colcon test-result --verbose
```

Planning ROS graph probe와 visualization mock probe:

```bash
python3 src/planning/test/ros2_local_planner_integration.py \
  src/planning/map/map_final_0921/0-0.txt
python3 src/behavior_stack_bringup/test/visualization_smoke.py
```

실차 후속 확인 항목은 camera 해상도/장착 위치/ROI, YOLO weight와 실제 `model.names`,
LiDAR extrinsic, steering sign, 20/50 target calibration, speed scale, CAN feedback 단위이다.

## 11. Offline / Mock Visualization Test

이 launch는 `mock_debug_input_node`, `planning_node`, `debug_visualizer_node`, 선택적
`rviz2`만 실행한다. GPS/IMU/LiDAR/camera driver, Controller와 CAN bridge는 실행하지 않는다.
다른 stack과 입력이 섞이지 않도록 전용 ROS domain을 사용한다. 명령은 저장소 root에서 실행한다.

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=105
export ROS_LOCALHOST_ONLY=1

ros2 launch behavior_stack_bringup mock_visualization.launch.py scenario:=cruise
# 이전 launch를 Ctrl-C로 종료한 뒤 하나씩 실행한다.
ros2 launch behavior_stack_bringup mock_visualization.launch.py scenario:=avoid_center
ros2 launch behavior_stack_bringup mock_visualization.launch.py scenario:=avoid_left
ros2 launch behavior_stack_bringup mock_visualization.launch.py scenario:=avoid_right
ros2 launch behavior_stack_bringup mock_visualization.launch.py scenario:=blocked
ros2 launch behavior_stack_bringup mock_visualization.launch.py scenario:=emergency
ros2 launch behavior_stack_bringup mock_visualization.launch.py scenario:=speed_20
ros2 launch behavior_stack_bringup mock_visualization.launch.py scenario:=speed_50
```

화면 없이 topic만 확인하려면 `enable_rviz:=false`를 추가한다.

| Scenario | 입력 | Expected Behavior | 현재 target |
|---|---|---|---|
| `cruise` | map 위 ego, 장애물 없음, sign 20 | CRUISE | 2.5 |
| `avoid_center` | 전방 map arc length 약 8 m, lateral 0 | AVOID | 2.5 |
| `avoid_left` | 같은 전방 위치, lateral +0.75 m(왼쪽) | AVOID | 2.5 |
| `avoid_right` | 같은 전방 위치, lateral -0.75 m(오른쪽) | AVOID | 2.5 |
| `blocked` | 같은 전방 위치, lateral -2.5~+2.5 m / 0.5 m 간격 | EMERGENCY_STOP, valid 0 | 0 |
| `emergency` | dynamic_stop=true, 장애물 없음 | EMERGENCY_STOP | 0 |
| `speed_20` | sign 20 반복 | CRUISE, active 20 | 2.5 |
| `speed_50` | sign 50 반복 | CRUISE, active 50 | 4.5 |

속도 값은 기존 config의 provisional target이며 이번 작업에서 조정하지 않았다.
기본 차량은 Global Path의 `base_index=0` 위치/heading을 사용한다. `obstacle_distance=8.0`은
단순 UTM X 이동이 아니라 Global Path arc length로 전방점을 찾는다. 장애물 lateral 방향은
기존 integration probe의 `offset_obstacles()`를 재사용한다. 두 모듈의 좌표계에서 +lateral은
heading 왼쪽이다. 해당 map의 첫 구간이 직선이므로 기본 테스트에서 heading 좌표 전방 8 m와
동일하다. `base_index`, `obstacle_distance`, `publish_rate_hz`를 launch에서 변경할 수 있다.

Mock launch의 `mock_planning.yaml`은 `planning/test/integration_planning.yaml`을 build 시
설치한 파일이다. 테스트용 경계 ±2.5 m와 11개 lateral 후보를 사용한다. Production 경계
±1.5 m와 Planning 알고리즘은 그대로 유지된다. Production config를 시험하려면:

```bash
ros2 launch behavior_stack_bringup mock_visualization.launch.py \
  scenario:=avoid_center planning_config:=<PRODUCTION_PLANNING_YAML>
```

이 경우 회피 공간 부족으로 EMERGENCY_STOP이 나올 수 있으며 기본 mock 회피 PASS 조건을
그대로 적용하면 안 된다.

RViz의 Fixed Frame은 `map`이다. mock 전용 config는 `velodyne` target frame을 따라 ego 근처로
view를 잡고, global path에는 transient-local QoS, local/candidate path에는 best-effort QoS를
사용한다. 기존 selected=green, valid=blue, invalid=red 색상과 실제 candidate path geometry를
표시한다. 차단에서는 selected local path가 비어도 invalid candidate 11개의 형상은 남는다.

Mock node만 합성 `map → velodyne` static TF를 발행한다. 센서 원점은 ego 원점과 같다고
가정한다. `/LiDAR/object_cen`은 local XY pairs + -1000 sentinel이고, `/Convert/small_object_UTM`은
동일 장애물의 map XY pairs이다. Big obstacle은 빈 배열로 clear한다. Relative marker와 UTM
marker가 겹치는지 확인할 수 있지만 **실제 converter/extrinsic 검증은 아니다**.

RViz display/topic:

| Display | Topic | 확인 |
|---|---|---|
| Global Path | `/Planning/debug/global_path` | 전체 preferred path |
| Selected Local Path | `/Planning/debug/local_path` | 최종 경로 |
| Candidate Paths | `/Planning/debug/candidate_paths` | 형상·valid/invalid·selected label |
| Ego | `/Planning/debug/ego` | footprint·heading arrow |
| Relative LiDAR Objects | `/Visualization/lidar_relative_objects` | 상대 장애물 |
| UTM Obstacles | `/Visualization/obstacles_utm` | Planning이 받는 장애물 |
| Behavior | `/Visualization/behavior` | 현재 Behavior text |

별도 terminal에서도 같은 domain/source를 설정한 뒤 확인한다.

```bash
ros2 node list
ros2 topic echo /Planning/behavior
ros2 topic echo /Planning/target_velocity
ros2 topic echo /Planning/debug/candidates
ros2 topic info /Planning/debug/candidate_paths --verbose
ros2 topic hz /Planning/debug/local_path
ros2 param set /mock_debug_input_node scenario blocked
ros2 param set /mock_debug_input_node scenario cruise
```

`scenario`는 runtime 변경 가능하다. Ego는 고정 위치이므로 AVOID 종료 후 실제 이동/경로 복귀
연속성 검증은 기존 `ros2_local_planner_integration.py`의 차량 위치 갱신 테스트를 사용한다.

자동 smoke test는 launch까지 실행/정리하며 8개 scenario별 독립 domain(110~117)을 사용한다.
관련 node가 이미 같은 domain에서 실행 중이면 종료하고 실행한다.

```bash
python3 src/behavior_stack_bringup/test/mock_visualization_smoke.py
python3 src/behavior_stack_bringup/test/mock_visualization_smoke.py \
  --scenarios cruise avoid_center blocked
```

실제 publish 여부, map frame, candidate geometry, valid/invalid 개수, ego arrow/footprint,
relative/UTM marker, behavior text, target, node graph와 vehicle command topic 부재를 assert한다.

## 12. Static Image로 Perception 확인

사용자가 제공한 실제 이미지 파일만 사용한다. 저장소에 이미지/YOLO weight는 포함되어 있지
않으며 임의 샘플을 생성하지 않는다.

```bash
ros2 run behavior_stack_bringup mock_camera_node --ros-args \
  -p image_path:=<IMAGE_PATH> -p publish_rate_hz:=5.0

ros2 launch behavior_stack_bringup mock_perception.launch.py \
  image_path:=<IMAGE_PATH> weights_path:=<MODEL.pt> \
  camera_image_topic:=/camera/image_raw enable_debug_image:=true

ros2 run rqt_image_view rqt_image_view
```

rqt에서 `/camera/image_raw`와 `/Perception/speed_sign/debug_image`를 비교한다.
PT 없이 image subscription, ROI crop, debug rendering, 미검출 0만 확인 가능하다.
PT가 있어야 실제 inference, class/confidence/bbox 및 20/50 mapping을 검증할 수 있다.
실제 센서 성능·검출거리·extrinsic·차량 제어 성능은 이 mock test의 검증 범위에 포함되지 않는다.

### 이번 검증 결과

2026-10-06, ROS 2 Humble 호스트에서 clean build 11개 package가 성공했다. Foxy에서는 별도
재검증이 필요하다. Mock smoke 8개 scenario 모두 PASS했고 global path 1,453점, 정상 local
path 300점, avoidance candidate geometry 11개가 실제 발행됐다.

| Scenario | Behavior | Valid / Invalid | 최저 cost offset | Clearance | Target |
|---|---|---|---|---|---|
| cruise | CRUISE | 1 / 0 | 0.0 | 장애물 없음 | 2.5 |
| avoid_center | AVOID | 2 / 9 | +1.5 | 0.30 m | 2.5 |
| avoid_left | AVOID | 3 / 8 | -0.9 | 0.45 m | 2.5 |
| avoid_right | AVOID | 3 / 8 | +0.9 | 0.45 m | 2.5 |
| blocked | EMERGENCY_STOP | 0 / 11 | 없음 | 없음 | 0 |
| emergency | EMERGENCY_STOP | 1 / 0 | 0.0 | 장애물 없음 | 0 |
| speed_20 | CRUISE | 1 / 0 | 0.0 | 장애물 없음 | 2.5 |
| speed_50 | CRUISE | 1 / 0 | 0.0 | 장애물 없음 | 4.5 |

좌우 대칭 후보의 미세 cost 차이로 중앙 회피 방향은 달라질 수 있다. 자동 test는 특정
offset을 요구하지 않는다. RViz mock config는 실제 X display에서 열렸으며 Global Status OK,
path/candidate/ego/obstacle/AVOID text 표시를 육안 확인했다. RViz 시작 때 parameter가 로드되기
전 임시 reliable 구독에서 QoS 경고가 나올 수 있지만, 최종 candidate 구독은 BEST_EFFORT,
global path 구독은 RELIABLE/TRANSIENT_LOCAL로 publisher와 일치했다.

Planning GTest 17개, Perception helper 6개, Localization split test 1개는 PASS했다.
전체 `colcon test-result --verbose`는 276 tests / 0 errors / 228 failures / 15 skipped로,
기존 control/lidar/convertcs/legacy ERP/IMU의 lint·copyright·XML schema 문제 때문에 FAIL이다.
이번 작업에서 해당 package들의 formatting이나 알고리즘을 변경하지 않았다.

Mock camera는 syntax/launch 검사를 통과했으나 사용자 이미지와 PT가 없어 실제 정적 이미지
발행 및 YOLO inference는 실행하지 않았다. CAN/Control/vehicle interface node는 mock launch와
smoke에서 실행하지 않았고 `/Control/vehicle_cmd`와 `/erp42pro/cmd_debug` 부재도 확인했다.
