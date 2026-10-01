# K-ROAD Behavior Stack

## Project Overview

Minimal ROS 2 Humble source workspace for the K-ROAD low-speed fixed-route vehicle. The repository
contains the sensor interfaces, localization, static-obstacle perception, Behavior Planning, existing
controller, and ERP42 serial bridge that participate in the current runtime graph.

## System Architecture

```text
GPS + IMU + ERP feedback -> Localization --------------------+
                                                              |
Velodyne -> small-static clustering -> relative-to-UTM -------+-> Planning
                                                              |      -> Controller
Camera -> YOLO speed-sign classification ---------------------+      -> ERP serial bridge
                                                                     -> ERP42
```

See `docs/ARCHITECTURE.md` for the topic-level graph and `docs/PACKAGE_SELECTION.md` for the complete
source-workspace classification.

## Package Structure

| Package | Role |
|---|---|
| `nmea_navsat_driver` | GPS serial driver, publishes `/fix` |
| `wit_ros2_imu` | IMU serial driver, publishes `/imu` |
| `local_pkg1` | GPS/IMU/ERP localization |
| `lidar` | VLP-16 small static-obstacle clustering |
| `kroad_planning_utm_pkg` | LiDAR-relative XY to UTM conversion |
| `traffic_sign_perception` | Camera/YOLO `speed_30`, `speed_40`, `speed_50` classification |
| `planning_pkg_2025` | Context → Behavior → Local Planner |
| `control` | Existing path-following controller |
| `erp_ros2_bridge` | ERP42 serial input/output |
| `behavior_stack_bringup` | Integrated launch |

## ROS 2 Distribution

Ubuntu 22.04 with ROS 2 Humble.

## Dependencies

ROS dependencies are declared in each `package.xml`. Hardware runtime additionally requires:

- `velodyne_driver` and `velodyne_pointcloud` for VLP-16 input
- Python `pyserial`, `utm`, NumPy, and SciPy
- PCL and `pcl_conversions`
- `nlohmann-json3-dev`
- `cv_bridge` and, when camera recognition is enabled, Python `ultralytics`

Use `rosdep install --from-paths src --ignore-src -r -y` and install the Python `utm` module on the
vehicle PC. The current validation host does not have the Velodyne ROS packages, `python3-serial`, or
the Python `utm` module installed.

## Hardware

- Velodyne VLP-16
- NMEA GPS receiver
- WIT serial IMU
- ERP42-compatible serial vehicle interface

Serial device paths and the Velodyne IP are launch arguments; no PC-specific hardware path is embedded
in the repository.

## Build

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

Safe dry run (no hardware drivers and no ERP actuation):

```bash
ros2 launch behavior_stack_bringup behavior_stack.launch.py \
  enable_gps:=false enable_imu:=false enable_localization:=false \
  enable_lidar:=false enable_vehicle_interface:=false
```

Vehicle run after hardware verification:

```bash
ros2 launch behavior_stack_bringup behavior_stack.launch.py \
  gps_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1 erp_port:=/dev/ttyUSB2 \
  velodyne_ip:=192.168.1.201 enable_vehicle_interface:=true
```

Enable speed-sign recognition after installing Ultralytics and supplying trained weights:

```bash
ros2 launch behavior_stack_bringup behavior_stack.launch.py \
  enable_camera_sign:=true camera_image_topic:=/camera/image_raw \
  speed_sign_weights:=speed_sign.pt
```

Relative weight paths are resolved from `traffic_sign_perception/weights`; an empty or missing file
produces a warning and a zero (no-detection) result without terminating the node.

`enable_vehicle_interface` defaults to `false` so launching the stack cannot actuate the vehicle by
default.

## Topic Graph

The principal chain is `/fix`, `/imu`, `/ERP/serial_data` → `/Local/utm`, `/Local/heading` and
`/velodyne_points` → `/LiDAR/object_cen` → `/Convert/small_object_UTM`, and camera images →
`/Perception/speed_limit` → `/Planning/*` →
`/Control/serial_data` → ERP42.

## Planning Architecture

```text
ContextManager -> BehaviorPlanner -> LocalPlanner -> Existing Controller
```

Behaviors are `CRUISE`, `AVOID`, `STOP`, `WAIT`, and `EMERGENCY_STOP`. Planning remains spatial;
The active 30/40/50 km/h sign state selects a configurable ERP target value; velocity-profile
generation and controller redesign remain outside this repository.

## Configuration

- Planning parameters and active global path: `src/planning/config` and `src/planning/map`
- Controller calibration JSON: `src/control/include/control`
- LiDAR-to-vehicle longitudinal offset: `sensor_offset_x` on `relative_2_UTM`
- Hardware ports/IP: integrated launch arguments
- Speed-sign model/runtime: `src/traffic_sign_perception/config/speed_sign.yaml`

The production Planning road bounds remain ±1.5 m. Surveyed road boundaries still need vehicle-site
configuration. The target values for 30/40/50 signs are provisional vehicle commands, not km/h values.

## Test

```bash
colcon test --packages-select planning_pkg_2025
colcon test-result --verbose
```

The Planning ROS graph probe is `src/planning/test/ros2_local_planner_integration.py`.

## Current Scope

Fixed route, low speed, static-obstacle avoidance, speed-limit sign state, and the existing ERP controller.

## Known Limitations

- Hardware-in-the-loop validation was not possible on this host.
- Velodyne and Python serial/UTM dependencies must be installed on the vehicle PC.
- VLP-16 ROI, sensor extrinsic, controller calibration, speed targets, and road bounds require vehicle-site
  validation.
- Camera model, image topic, trained YOLO weights, and sign-class confidence require deployment setup.
- `/Planning/mission` remains only for controller compatibility; perception no longer depends on it.
