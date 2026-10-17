# 10_05_2 실제 bag 검증

## 결론

원본 `/velodyne_points` **1,008프레임 전체(약 101.6초)**를 현재 production
`lidar::obstacleClusters()` 함수로 재처리했다. 특정 물체 크기 조건을 제거하면
실제 데이터에서도 검출이 늘어난다. 다만 **지면 잔여 성분이 장애물로 전달될 가능성**이
확인되어, 이 결과만으로 실차용 non-ground 검출 품질이 확보됐다고 판단할 수 없다.
이번 검증에서는 ROI/필터/Planning 설정을 추가로 변경하지 않았다.

ROS 노드 실행은 이 환경에서 `getifaddrs: Operation not permitted`로 실패했다.
따라서 SQLite를 읽기 전용으로 열어 CDR 메시지를 역직렬화한 후, 노드가 사용하는
동일한 PCL 필터 함수를 호출했다. ROS topic publish/subscribe, 시간/QoS, RViz 및
실제 PlanningNode end-to-end 재생을 검증한 것은 아니다.

## 실제 bag 입력

- 파일: `/home/wgr/behavior_ws/10_05_2/10_05_2_0.db3`
- 전체 기록 길이: metadata 기준 약 101.617초
- raw LiDAR: 1,008건, 첫~마지막 raw 프레임 간격 약 101.612초
- 기록된 `/LiDAR/object_cen`: 836건, 834건 빈 검출, 2건만 중심점 1개
- `/Local/utm`, `/Local/heading`, `/Convert/small_object_UTM`, `/Planning/behavior`:
  **모두 0건**. 기록된 실제 localization 기반 global/candidate path 검증은 불가능하다.
- IMU 20,396건은 존재하나 UTM 위치를 제공하지 않으므로 localization을 대체하지 않았다.
- 기록된 중심점 836건의 trailing `-1000` sentinel은 모두 정상.

토픽별 실제 DB count는 `topics.json`, 첫 PointCloud2 schema는 `cloud_schema.json`에 저장했다.

## 전체 재처리 결과

| 항목 | 결과 |
|---|---:|
| 완료 raw 프레임 | 1,008 / 1,008 |
| 현재 generic 검출 | 누적 5,770 cluster 관측 |
| 프레임당 generic 검출 | 평균 5.72, 중앙값 6, 최소 1, 최대 12 |
| 변경 전 소스의 size gate를 같은 입력에 재적용 | 누적 3,749, 프레임당 평균 3.72 |
| size gate 제거로 추가 수용 | 누적 2,021 cluster 관측 |
| 추가 수용이 있는 프레임 | 1,008 / 1,008 |
| 평균 raw / finite / ROI 점 수 | 29,181 / 14,835 / 161 |
| acceptance 최소 cluster 점 수 | 3 |
| center 좌표 finite / z ROI bounds | 전체 통과 |
| 필터 함수 처리 시간 | 평균 2.41ms, p95 2.50ms (본 환경 1회 측정) |

누적 수치는 **고유 물체 수가 아니라 프레임별 반복 관측 합계**다. 처리 시간은
역직렬화/CSV 기록/ROS 통신/전체 node callback 비용을 포함하지 않아 실시간 성능 보장이 아니다.

비교 대상인 “변경 전 소스 size gate 재적용”은 이전 턴에 audit한 small_object OR
passenger_car 조건이다. bag에 기록된 당시 검출 출력(2건)과는 별개의 값이다.
기록 당시 detector 코드/설정 및 raw 프레임과 검출 출력의 처리 구간 차이를 확정할 수 없어
두 결과를 동일한 baseline으로 취급하지 않았다.

## 품질상 확인된 문제

- 전체 cluster 관측 5,770건 중 **3,915건(67.9%)**은 최고 z가 -0.2m 이하이다.
- 새로 수용된 2,021건 중 **924건(45.7%)**도 최고 z가 -0.2m 이하이다.
- 전체 cluster 높이 중앙값은 약 **3.78cm**이다.
- 대표 프레임의 XY 점들은 원호 형태의 낮고 얇은 표면도 포함한다.
  이는 **ground 또는 낮은 표면의 잔여 검출 가능성을 시사**한다. 영상/라벨/정답 point
  segmentation이 없으므로 각 cluster를 ground/드럼/사람/차량으로 확정하지 않는다.
- 특히 frame 0의 큰 cluster는 length 약 2.71m, width 약 5.37m지만 높이는 약 4.33cm이고,
  z는 약 [-0.349,-0.306]m이다. 기존 passenger_car size gate도 이를 수용한다.
  즉 크기 class 조건 자체가 지면 제거를 보장했던 것은 아니다.
- 원래 요청대로 z crop [-0.35,1.8]m는 유지했다. 이번 bag 결과는 **z crop만으로
  non-ground를 보장할 수 없다는 실제 데이터 근거**다. 향후 지면 제거 방식을 검토하되
  단순히 자동차/드럼 크기 gate를 되살리는 것은 역할 분리 취지에 맞지 않는다.

## 확인 범위와 남은 항목

검증 완료: 실제 raw bag 전체의 generic acceptance 확대, sparse 최소 점 수,
finite center, 기존 z crop 적용, 기존 size gate와의 동일 입력 비교.

미검증: 실제 장애물별 재현율/오탐률, ROS 전달/sentinel publish, UTM conversion,
Global Path blocking, AVOID 전이, candidate 선택, RViz 화면. 원본에 UTM/heading이 없고
본 실행 환경은 ROS 네트워크 초기화도 차단된다. 임의 위치나 heading을 넣어 실제
주행 Planning 검증으로 보고하지 않았다.

실차 또는 localization이 함께 기록된 bag에서 ground 잔여 검출, 실제 obstacle 여부,
ROI 도로/candidate 범위, center-only 고정 radius의 큰 물체 충돌 표현을 확인해야 한다.
동적 물체 prediction은 현재 구조에 없다.

## 산출물 / 재현

- `frames.csv`: 매 raw 프레임 점 수, 검출 수, 이전 gate 수용 수, 처리 시간
- `clusters.csv`: 매 cluster centroid, 점 수, bbox 치수 및 z bounds, 이전 gate 여부
- `roi_0.csv`, `roi_250.csv`, `roi_500.csv`, `roi_750.csv`, `roi_1007.csv`: 대표 ROI 점
- `detection_timeline.png`: 전 구간 검출 수 및 ROI 점 수
- `roi_comparison.png`: 대표 3프레임 XY 비교 (파랑=양쪽 수용, 주황=추가 generic 검출)
- `summary.json`, `topics.json`, `cloud_schema.json`, `recorded_centers.json`
- `replay.log`, `build.log`, `ros_node_attempt.log`

workspace root(`/home/wgr/behavior_ws/behavior-planner`)에서:

```bash
source /opt/ros/foxy/setup.bash
cmake -S validation/generic_obstacles/bag_10_05_2 -B /tmp/bag_obstacle_audit_build
cmake --build /tmp/bag_obstacle_audit_build -j2
/tmp/bag_obstacle_audit_build/bag_obstacle_audit \
  /home/wgr/behavior_ws/10_05_2/10_05_2_0.db3 \
  "$PWD/validation/generic_obstacles/bag_10_05_2"
python3 validation/generic_obstacles/bag_10_05_2/inspect_bag.py
python3 validation/generic_obstacles/bag_10_05_2/summarize.py
```
