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
