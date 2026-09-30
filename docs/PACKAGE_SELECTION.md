# Source Workspace Package Selection

Classification is based on the Behavior Planning runtime graph, not on historical use.

| Package | Existing role | Decision | Included | Reason |
|---|---|---|---|---|
| `planning_pkg_2025` | Context/Behavior/Local Planning | REQUIRED | Yes | Produces the controller path and stop contract |
| `local_pkg1` | GPS/IMU/ERP localization | REQUIRED | Yes | Publishes `/Local/utm`, `/Local/heading`, `/beta` |
| `nmea_navsat_driver` | GPS serial driver | REQUIRED | Yes | Publishes `/fix` consumed by localization |
| `wit_ros2_imu` | IMU serial driver | REQUIRED | Yes | Publishes `/imu` consumed by localization/control |
| `lidar` | Mission LiDAR processing | REQUIRED, reduced | Yes | Only small-static clustering participates in V1 |
| `kroad_planning_utm_pkg` | Relative obstacle coordinate conversion | REQUIRED, reduced | Yes | Publishes `/Convert/small_object_UTM` |
| `control` | Existing path-following controller | REQUIRED | Yes | Consumes Planning/localization and publishes ERP commands |
| `erp_ros2_bridge` | ERP42 serial bridge | REQUIRED | Yes | Vehicle feedback and optional actuation |
| `serial` | C++ serial library | UNUSED | No | Retained drivers use Python `pyserial`, not this library |
| `perception` | Historical aggregate launch package | OPTIONAL | No | Replaced by `behavior_stack_bringup`; source algorithms are elsewhere |
| `plotting_pkg` | Matplotlib runtime visualization | OPTIONAL | No | Debug only; not in the vehicle control chain |
| `amz_planning_pkg_2025` | Previous mission-number planner | LEGACY | No | Replaced by `planning_pkg_2025` |
| `delivery_yolo` | Delivery mission vision | LEGACY | No | Delivery is outside V1 scope |
| `amz_yolo` | Mission camera detector | LEGACY | No | Camera missions are outside V1 scope |
| `object_detection` | Delivery/traffic/mission detection | LEGACY | No | Does not produce a current Planning input |
| `lidar_lane` | Narrow-road/lane processing | LEGACY | No | Narrow-road mission is outside V1 scope |
| `fusion` | Camera/LiDAR mission fusion | LEGACY | No | Delivery/traffic/narrow fusion is outside V1 scope |
| `yolo_narrow_11` | Narrow-road detector | LEGACY | No | Narrow-road mission is outside V1 scope |
| `traffic_yolo` | Traffic-light detector | LEGACY | No | Traffic-light behavior is outside V1 scope |
| `yolo_traffic_11` | Traffic-light detector | LEGACY | No | Traffic-light behavior is outside V1 scope |
| `camera` | Multi-camera and traffic bridge nodes | LEGACY | No | No camera topic participates in the V1 graph |

No workspace custom-message package is required. The retained graph uses standard ROS messages.

## External/system dependencies

| Dependency | Kind | Status on validation host |
|---|---|---|
| ROS 2 Humble | System | Installed |
| PCL / `pcl_conversions` | System | Installed |
| `velodyne_driver` | Hardware driver | Missing |
| `velodyne_pointcloud` | Hardware driver | Missing |
| `python3-serial` | System Python dependency | Missing |
| Python `utm` | Python/pip dependency | Missing |
| NumPy / SciPy | Python dependency | Installed for system Python |
| nlohmann-json | External C++ library | Installed |

## Package-internal removals

- `lidar`: removed big-static, parking, U-turn, tunnel/dynamic, delivery, examples, and mission RViz files.
- `kroad_planning_utm_pkg`: removed narrow-road executable and mission-specific conversion branches;
  retained the existing small-static coordinate transform.
- `local_pkg1`: retained only `tae_localization` and its executable entry point.
- `control`: removed IDE and PlotJuggler artifacts; retained A/B/B_wynz vehicle calibration files.
- `planning_pkg_2025`: retained the verified V1 code/tests/config/docs and only the active 1,389-point map.
