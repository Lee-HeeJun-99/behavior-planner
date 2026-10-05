# End-to-End Runtime Architecture

## Runtime graph

```text
NMEA GPS -- /fix -------------------------------+
WIT IMU -- /imu -----------------------------+ |
ERP bridge -- /ERP/serial_data --------------+| |
                                             vv v
                                      tae_localization
                                             |
                         /Local/utm, /Local/heading, /beta
                                             |
                                             +---------------------------+
                                                                         |
VLP-16 -> velodyne_driver -> velodyne_transform -> /velodyne_points      |
                                                     |                   |
                                             LiDAR_small_static          |
                                                     |                   |
                                             /LiDAR/object_cen           |
                                                     |                   |
                                             relative_2_UTM <------------+
                                                     |
                                      /Convert/small_object_UTM
                                                     |
Camera -> traffic_sign_perception -> /Perception/speed_limit ------------+
                                                     |
                                                     v
                                              planning_node
                         ContextManager -> BehaviorPlanner -> LocalPlanner
                                                     |
             /Planning/local_path, /Planning/path_yaw,
             /Planning/target_velocity, /Planning/mission
                                                     |
                                                     v
                                                erp_control
                                                     |
                                          /Control/serial_data
                                                     |
                                                     v
                                            erp_ros2_bridge
                                                     |
                                                     v
                                                   ERP42
```

The Planning-facing localization, perception, Planning, and control connections use compatible
depth-one QoS. Positions are UTM metres, headings and steering are radians, and controller speed values
are SI before the ERP bridge converts them to the device protocol.

## Perception integration decision

The former LiDAR executable created big-static, parking, tunnel, U-turn, and other mission-number gated
nodes. Behavior Planning needs only the fixed-route small-static obstacle stream, so the new workspace
retains `LiDAR_small_static` only. Its ROI and clustering algorithm are unchanged. Mission-number
subscription gating was removed so obstacle detection runs during `CRUISE`; otherwise Planning could not
observe an obstacle before deciding `AVOID`.

The coordinate converter retains the existing 0.35 m longitudinal sensor offset and rotation/translation
formula, but only the small-static topic path is built. This removes delivery, parking, U-turn, and narrow
road conversion branches from the runtime.

## Source versus external dependencies

Workspace source packages are under `src/`. ROS 2, PCL, Velodyne drivers, Python scientific/serial
modules, and nlohmann-json are system dependencies and are intentionally not copied into this repository.

## Speed-sign integration

`traffic_sign_perception` subscribes to a camera `sensor_msgs/Image`, crops a configurable normalized
ROI, and accepts only the direct YOLO classes `speed_20` and `speed_50`. It publishes the raw
semantic result for each frame (`0` means no supported detection); it owns no temporal state. Planning's
`ContextManager` is the single owner of consecutive-detection confirmation and treats a confirmed sign
as an event: subsequent zero detections do not erase the active limit. `ContextManager` maps the active
limit to the configured ERP target. AVOID caps that target with `avoid_max_velocity`; STOP, WAIT, and
EMERGENCY_STOP force zero output. The optional debug image shows the ROI and restores ROI-relative YOLO
boxes to original-image coordinates.

## Safety

The integrated launch defaults `enable_vehicle_interface` to `false`. A dry run starts Planning and the
controller without opening the ERP serial actuator. Enabling vehicle output requires an explicit launch
argument after hardware and emergency-stop checks.
