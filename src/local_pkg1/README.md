# local_pkg1 (wego_ws local_pkg_2025 에서 가져옴)

### UTM 경로 기록 (map)

로컬라이제이션이 `/Local/utm` (`geometry_msgs/PointStamped`)을 발행하는 상태에서 별도 터미널로 실행합니다.

```bash
source /opt/ros/foxy/setup.bash
source ~/behavior_ws/behavior-planner/install/setup.bash
ros2 run local_pkg1 map
```

- `P`: 현재 구간을 TXT로 저장하고 다음 구간 기록을 계속합니다.
- `S` 또는 `Ctrl+C`: 마지막 구간을 저장하고 종료합니다.
- 저장 위치: `(save_path 파라미터 경로)/record_날짜_시간/` 안의 `1.txt`, `2.txt`, … 및 경로 PNG.
- 기존 지도는 이동하거나 덮어쓰지 않습니다. 좌표가 없는 구간은 저장하지 않습니다.
- 저장 위치 변경: `ros2 run local_pkg1 map --ros-args -p save_path:=/원하는/경로`
