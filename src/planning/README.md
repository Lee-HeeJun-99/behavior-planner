# Planning V1

## Overview

ROS 2 spatial-planning package for a low-speed vehicle on a fixed route. It replaces mission-number
planning decisions with an explicit Context → Behavior → Local Planner flow. The existing controller
interface is preserved; acceleration and velocity-profile planning are intentionally excluded.

## Architecture

```text
Localization + Map + Static Obstacles
                  ↓
            ContextManager
                  ↓
           BehaviorPlanner
                  ↓
             LocalPlanner
                  ↓
             Local Path
                  ↓
        Existing Controller
```

## Behaviors

- `CRUISE`
- `AVOID`
- `STOP`
- `WAIT`
- `EMERGENCY_STOP`

## Input

| Topic | Type |
|---|---|
| `/Local/utm` | `geometry_msgs/msg/PointStamped` |
| `/Local/heading` | `std_msgs/msg/Float64` |
| `/Convert/small_object_UTM` | `std_msgs/msg/Float64MultiArray` |
| `/Convert/big_object_UTM` | `std_msgs/msg/Float64MultiArray` |
| `/LiDAR/dynamic_stop` | `std_msgs/msg/Bool` |

## Output

| Topic | Type |
|---|---|
| `/Planning/local_path` | `std_msgs/msg/Float64MultiArray` |
| `/Planning/path_yaw` | `std_msgs/msg/Float64MultiArray` |
| `/Planning/curvature` | `std_msgs/msg/Float64MultiArray` |
| `/Planning/target_velocity` | `std_msgs/msg/Float64` |
| `/Planning/mission` | `std_msgs/msg/Int16` |
| `/Planning/behavior` | `std_msgs/msg/String` |

Debug output is available on `/Planning/debug/local_path` and `/Planning/debug/candidates`.

## Build

```bash
colcon build --packages-select planning_pkg_2025 --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch planning_pkg_2025 planning.launch.py
```

## Test

```bash
colcon test --packages-select planning_pkg_2025
colcon test-result --verbose
```

The ROS graph integration probe is kept in `test/ros2_local_planner_integration.py` with its isolated
`test/integration_planning.yaml` configuration.

## Current Scope

- Fixed route
- Low, constant cruise speed
- Static obstacles
- Map stop points
- Spatial local-path planning

## Not Included

- Dynamic-obstacle prediction
- FOLLOW, YIELD, or OVERTAKE
- Acceleration or velocity-profile planning
- Controller redesign

## Known Limitations

- Production road bounds are configured as ±1.5 m and are not a substitute for surveyed boundaries.
- Stop points must be populated from the deployed map survey.
- Vehicle footprint, safety margin, road bounds, and candidate offsets require vehicle-site validation.
- V1 consumes obstacle centers rather than an occupancy-grid boundary.

See `docs/ARCHITECTURE.md` for migration decisions, module responsibilities, and interface details.
