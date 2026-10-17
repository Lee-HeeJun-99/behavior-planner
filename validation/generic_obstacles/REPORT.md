# Generic obstacle 변경 보고

Perception은 주변 유효 장애물을 넓게 검출하고, Planning은 global/candidate path에
영향을 주는지 판단한다. 물체 종류 인식/동적 trajectory prediction은 추가하지 않았다.

## 변경 파일

- `src/lidar/src/LiDAR_small_static.cpp`: category gate 제거, 모든 centroid publish, 모든 중심점 marker, raw input 통계 유지.
- `src/lidar/include/data_header/generic_obstacle.hpp`: finite XYZ + 기존 voxel/ROI/Euclidean pipeline 공유.
- `src/lidar/include/mission_header/LiDAR_small_static.hpp`, `src/lidar/src/main.cpp`: 내부 class/node generic 명칭.
- `src/lidar/test/generic_obstacle_test.cpp`: synthetic 검증.
- `src/lidar/CMakeLists.txt`, `src/lidar/package.xml`: gtest 등록.
- `src/planning/include/planning_pkg_2025/common/planning_types.hpp`: generic static obstacle 의미와 footprint TODO 주석.
- `src/planning/test/planning_core_test.cpp`: off-path/side 충돌 및 7개 core scenario 회귀 검증.
- `src/behavior_stack_bringup/scripts/mock_debug_input_node.py`: 두 신규 mock scenario.
- `src/behavior_stack_bringup/test/mock_visualization_smoke.py`: 실제 PlanningNode smoke 확장.
- `src/behavior_stack_bringup/CMakeLists.txt`, `src/behavior_stack_bringup/package.xml`: smoke를 colcon test에 등록.
- `src/behavior_stack_bringup/scripts/debug_visualizer_node.py`: namespace generic화, legacy 입력 호환 유지.
- `src/behavior_stack_bringup/launch/perception.launch.py`: node 이름.
- `src/behavior_stack_bringup/config/mock_behavior_debug.rviz`: raw/cluster cloud 표시 추가.
- `DEBUG_RUN_GUIDE.md`: audit, 역할 분리, 실행 절차 및 한계.
- `validation/generic_obstacles/*`: 검증 결과와 본 보고.

## Audit / acceptance / 호환성

기존: `/velodyne_points` → 0.1m voxel → ROI → Euclidean → size category filter
→ `/LiDAR/object_cen` → relative_2_UTM → `/Convert/small_object_UTM` → Planning.

제거한 OR 조건:
- small_object: width > 0.5 && width < 1.5 && max_z < 0.3
- passenger_car: width >= 0.3 && width <= 5.5 && length <= 5.5 && max_z <= 1.8
  && (width >= 0.8 || length >= 0.8)

새 acceptance: finite XYZ → 0.1m voxel → x=[0,20], y=[-3.5,3.5], z=[-0.35,1.8]
→ tolerance 0.4m, 최소 3 voxel 점의 Euclidean cluster → 모든 centroid.
별도 semantic/치수 gate 없음. 기존 max_ok=false여서 20000점 상한은 적용되지 않았고
그대로다. ground는 기존 z crop에 의존하며, ROI 내 지면 평면을 별도 제거하지 않는다.

`/LiDAR/object_cen` Float64MultiArray `[x,y,...,-1000]`와 best-effort QoS는 불변.
convertcs/Planning의 기존 UTM topic도 유지. `start` executable과 소스 파일명 유지.
node=`lidar_obstacle_detector`, class=`LidarObstacleDetector`,
cloud=`/Cluster/obstacles`, marker=`/Marker/obstacles`로 변경.
기존 debug small_static topic 구독자는 새 이름으로 변경해야 한다.
로그는 raw input/ROI/cluster/accepted generic obstacles 수를 출력한다.

## Planning 확인

ContextManager는 20m relevance distance로 보존하고 global path로 선별하지 않는다.
pathBlocked는 nearest global index → 전방 검사 → 10m lookahead → corridor 0.8m +
radius 비교 및 기존 AVOID pass margin을 유지한다. path 밖 장애물은 CRUISE→AVOID
원인이 아니다. CollisionChecker는 모든 candidate point와 모든 context obstacle을
기존 차량 footprint로 비교한다. retained path도 같은 checker로 재검사한다.

PlanningNode에는 기존에 CRUISE local plan이 infeasible하면 AVOID plan을 시도하는
fallback도 있다. 이번에 변경하지 않았다. 따라서 behavior는 pathBlocked뿐 아니라
실제 local path 안전성에도 영향을 받는다. 신규 off-path 배치는 두 경로 모두 clear하다.

## 검증 결과

- 깨끗한 전체 `colcon build --symlink-install`: 11 packages 성공.
  삭제 명령은 자동 검토에서 거부되어 기존 build/install/log를 workspace의
  `generic_obstacle_build_backup_*` 디렉터리로 옮겨 clean build했다.
- 최종 전체 재빌드도 11 packages 성공. 마지막 planning header 주석 반영 후 planning_node/core/test 직접 빌드도 성공.
- LiDAR gtest: 3/3 PASS. 드럼/사람 폭/차량/작은 박스 크기 수용,
  sparse noise/ROI 밖/ground crop 밖/non-finite 제거.
- Planning gtest: 31/31 PASS. 신규 3개 테스트 포함.
- 요청한 3개 package 대상 colcon test 실행. LiDAR lint(copyright/cpplint/
  uncrustify/xmllint) 실패가 남아 있다. 기존 파일뿐 아니라 추가 파일의 style/license
  검사도 포함되므로 이를 전부 기존 문제라고 간주하지 않는다. 기능 gtest는 통과.
- `colcon test-result --verbose`: 129 tests, 1 error, 80 failures (기능 gtest 34개 통과; lint 및 ROS smoke 실패 상세는 동봉 log).
- 실제 PlanningNode + visualizer smoke는 ROS 초기화가 `getifaddrs: Operation not permitted`
  로 중단되어 검증 불가. GUI RViz도 직접 확인하지 않았다. 실제 노드 7개 시나리오를
  PASS로 간주하지 않는다. 권한이 있는 ROS 환경에서 DEBUG_RUN_GUIDE 절차로 재검증 필요.

| Scenario | Planning core 결과 | 실제 PlanningNode/RViz |
|---|---|---|
| cruise | CRUISE, feasible | 환경 제한으로 미검증 |
| avoid_center | AVOID, feasible | 환경 제한으로 미검증 |
| avoid_left | AVOID, feasible | 환경 제한으로 미검증 |
| avoid_right | AVOID, feasible | 환경 제한으로 미검증 |
| blocked | 유효 candidate 0, infeasible, 정지 target 계산 0 | EMERGENCY_STOP 출력은 미검증 |
| off_path_obstacles | context 4개, CRUISE, target 2.5, 중앙 경로 | 환경 제한으로 미검증 |
| avoid_with_side_obstacle | AVOID, 측면 충돌 candidate invalid, 선택 경로 전체 장애물 검사 통과 | 환경 제한으로 미검증 |

blocked의 실제 EMERGENCY_STOP 전이는 기존 PlanningNode의 infeasible 처리 코드를
확인했고 core 테스트에서는 정지 속도 계산을 검증했다. ROS 출력 검증을 대체하지 않는다.

## RViz / 변경 영향 / 실차 TODO

Raw LiDAR, generic cluster cloud, relative/UTM center, global/candidate/selected path,
behavior 표시가 설정되어 있다. 신규 mock은 중심점만 생성하므로 raw/cluster cloud는
실제 perception 실행에서 확인한다. 후보 label의 invalid reason 및 selected 색상으로
장애물과 경로의 관련성을 확인할 수 있다. 중심점 단일 검출/3개 이상도 모두 표시한다.

기존에는 크기 gate로 사라지던 cluster가 Planning에 전달된다. 물체 수, 불필요 검출 및
path 충돌 판단이 늘 수 있다. state machine/경로 생성/footprint/cost/control은 불변이다.

- 실제 도로 폭/candidate 영역보다 ROI가 충분히 넓은지 실차 확인 필요.
- z crop의 ground 억제, 경사/센서 자세, 최소 3점 및 붙은 cluster/노이즈를 실측 검증.
- center-only + 고정 기본 radius 0.35m는 큰 차량/팔레트의 실제 점유 공간을 놓칠 수 있다.
  bbox min/max는 이미 계산 중이므로 향후 width/length/radius 또는 (x,y,radius) 확장 TODO.
- 보행자/동적 물체 미래 경로 prediction 없음. path 밖 사람이 진입할 가능성을 모델링하지
  않는다. 실제 사람 주변 실차 주행은 별도 dynamic safety policy 필요.

## 후속 실제 bag 검증: 10_05_2

[전체 분석](bag_10_05_2/REPORT.md): 원본 1,008프레임을 동일한 production 필터로
오프라인 재처리했다. 평균 generic 5.72개/frame, 이전 소스 gate 재적용 3.72개/frame.
추가 수용 2,021 cluster 관측. 낮고 얇은 cluster가 많아 ground 잔여 검출 가능성이
확인됐으며, non-ground 품질을 보장하는 결과는 아니다. 원본 bag에는 UTM/heading 및
Planning 출력이 없어 실제 localization 기반 Planning 검증은 수행하지 못했다.
