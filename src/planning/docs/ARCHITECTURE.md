# Planning architecture and migration report

## Previous structure and data flow

The former `planning_node.cpp` subscribed directly to localization, vehicle, LiDAR, and vision topics.
It populated `Local`, `Lidar`, `Vision`, and `Control` containers, then `Planning::chooseFunc()` selected
delivery, static-obstacle, parking, U-turn, tunnel, traffic-light, stop, bump, and speed logic with a
mission-number switch. Mission implementations changed both the local path and target velocity. The
same node published controller data and many mission-specific plotting topics.

The unchanged controller consumes interleaved XY coordinates on `/Planning/local_path`, path yaw on
`/Planning/path_yaw`, a scalar control switch/constant target on `/Planning/target_velocity`, and the
legacy integer `/Planning/mission`.

## Classification

| Previous file/class group | Previous role | Decision | Reason | Final location |
|---|---|---|---|---|
| `map/**/*.txt` | Surveyed global paths | REUSE | Proven map data and compatible XY/yaw/curvature format | `map/` |
| `Path`, KD-tree | Path storage and nearest lookup | REFACTOR | Useful behavior, but tightly coupled custom containers | `map/GlobalPath` with bounded nearest search |
| Frenet/polynomial planner | Lateral candidates, collision and curvature filtering | REFACTOR | Core spatial idea retained; old code included longitudinal time/acceleration terms | `local/PathGenerator`, `CollisionChecker`, `CostEvaluator` |
| `Lidar` callbacks | Decode absolute obstacle arrays | REFACTOR | Message contract retained; mission state and clustering removed | `PlanningNode::decodePoints`, `ContextManager` |
| Controller output publishers | Controller path/stop contract | REUSE | Controller must remain unchanged | `src/nodes/planning_node.cpp` |
| `Planning`, mission-number switch | Select all behavior and control | REPLACE | Mixed context, behavior, path, and speed responsibilities | `ContextManager` + `BehaviorPlanner` + `LocalPlanner` |
| Mission headers (delivery, parking, U-turn, tunnel, bump, traffic) | Competition mission state machines | REMOVE | Outside fixed-route V1 or coupled to mission number/speed planning | No replacement in V1 |
| Old `Control`, `MissionData`, `Vision` | Planning-owned speed/mission state | REMOVE | Velocity planning and traffic missions are outside scope | Existing controller + behavior output |
| DWA/RRT and unused utilities | Alternate planners | REMOVE | Not used by fixed-route static-obstacle V1 | No replacement |
| Mission plotting publishers | Mission-specific debug | REPLACE | Coupled to removed mission classes | `/Planning/debug/local_path`, `/Planning/debug/candidates` |

## New modules

| File/class | Responsibility | Input | Output |
|---|---|---|---|
| `common/planning_types.hpp` | Shared context/path/behavior types | — | Typed domain data |
| `map/GlobalPath` | CSV loading and nearest path index | map file, vehicle XY | preferred path/index |
| `context/ContextManager` | Normalize vehicle, obstacles, speed-sign state, free space | localization, map, LiDAR, speed limit | `BehaviorContext` |
| `behavior/BehaviorPlanner` | CRUISE/AVOID/STOP/WAIT/EMERGENCY_STOP with debounce | `BehaviorContext` | `Behavior` |
| `local/PathGenerator` | Smooth lateral SL-like candidates | preferred path, behavior | candidate paths |
| `local/CollisionChecker` | Boundary, footprint, clearance, curvature checks | candidates/context | valid or INVALID |
| `local/CostEvaluator` | Deviation, curvature, smoothness, clearance cost | valid candidates | scalar cost |
| `local/LocalPlanner` | Select lowest-cost feasible local path | behavior/context | local path |
| `nodes/PlanningNode` | ROS conversion and orchestration only | ROS topics/parameters | controller and debug topics |

## ROS 2 interfaces

| Node | Direction | Topic | Type | Purpose |
|---|---|---|---|---|
| planning_node | Subscribe | `/Local/utm` | `geometry_msgs/PointStamped` | vehicle position |
| planning_node | Subscribe | `/Local/heading` | `std_msgs/Float64` | vehicle yaw |
| planning_node | Subscribe | `/Convert/small_object_UTM`, `/Convert/big_object_UTM` | `std_msgs/Float64MultiArray` | static obstacle XY pairs |
| planning_node | Subscribe | `/LiDAR/dynamic_stop` | `std_msgs/Bool` | existing emergency-stop signal |
| planning_node | Subscribe | `/Perception/speed_limit` | `std_msgs/Int16` | filtered speed-sign value (`0`, `30`, `40`, `50`) |
| planning_node | Publish | `/Planning/local_path` | `std_msgs/Float64MultiArray` | controller-compatible interleaved XY path |
| planning_node | Publish | `/Planning/path_yaw` | `std_msgs/Float64MultiArray` | controller-compatible yaw array |
| planning_node | Publish | `/Planning/curvature` | `std_msgs/Float64MultiArray` | path debug/compatibility |
| planning_node | Publish | `/Planning/target_velocity` | `std_msgs/Float64` | active speed-limit target or zero-stop contract |
| planning_node | Publish | `/Planning/mission` | `std_msgs/Int16` | controller compatibility only, never planning selection |
| planning_node | Publish | `/Planning/behavior` | `std_msgs/String` | explicit behavior state |
| planning_node | Publish | `/Planning/debug/local_path` | `nav_msgs/Path` | RViz selected path |
| planning_node | Publish | `/Planning/debug/candidates` | `std_msgs/Float64MultiArray` | offset, cost/invalid flag, clearance triples |

## Scenario flow

- Normal: context is clear -> CRUISE -> zero-offset candidate -> preferred global path slice.
- Static obstacle: confirmed blockage -> AVOID -> lateral candidates -> hard feasibility rejection ->
  lowest valid spatial cost. After confirmed clearance, CRUISE regenerates the zero-offset path, so
  recovery starts at the vehicle's current signed lateral offset and converges smoothly to the preferred
  path without a RETURN_TO_PATH state.
- Speed sign: the same supported sign must meet the confirmation count before it replaces the active
  limit. A zero/no-detection message clears only the instantaneous detection, not the active limit.
- STOP and WAIT enum values remain available for future traffic-control inputs, but V1 no longer creates
  them from map stop points.
- No feasible candidate, missing localization, or emergency input -> EMERGENCY_STOP.

## Scope and deployment notes

There is no acceleration, deceleration, velocity-profile, dynamic prediction, or controller code here.
Static boundaries are V1 lateral bounds from configuration; occupancy-grid boundaries are not available
in the current workspace interface. Before vehicle deployment, tune the speed-target mapping and
vehicle footprint/bounds, replay representative rosbags, and inspect the debug `nav_msgs/Path` in RViz.
The production bounds are currently ±1.5 m; the wider bounds used by integration tests are isolated in
`test/integration_planning.yaml` and must not be treated as surveyed deployment boundaries.
