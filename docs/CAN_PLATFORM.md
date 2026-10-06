# ERP42 Pro CAN 플랫폼 실행

참고: `/home/wgr/Control_Back_UP/1003/wego_ws/src/Control_Manual.txt` 및 같은 백업의 `erp42pro_interface` 소스/테스트.
백업 프로토콜과 calibration을 사용하며 Controller와 Localization의 토픽 배열 형식은 유지한다.

- SocketCAN: can0, 500000 bit/s.
- 수신: 0x303 모드/기어, 0x304 속도/조향/브레이크.
- 송신: 0x501~0x504, 50 Hz. 기본 속도 상한 10 km/h.
- `/ERP/serial_data`: CAN feedback을 기존 ROS 배열로 변환. 이름의 serial은 호환 목적이다.
- `can_receive_only=true`: 실제 feedback 수신, CAN 제어 송신 및 종료 정지 송신 없음.
- `can_receive_only=false`: Controller 명령을 CAN으로 송신. 명령 timeout 0.3초 후 속도 0, 브레이크 최소 30%.
- 기존 serial 브리지 또는 다른 CAN 제어 송신 노드를 동시에 실행하지 않는다.
- GPS/IMU 기본 포트는 연결 확인된 by-id 경로다.

## 차량 PC 일반 터미널

기존 스택은 해당 터미널에서 Ctrl+C로 종료한다. CAN이 아직 설정되지 않았다면:

```bash
sudo ip link set can0 up type can bitrate 500000
ip -details -statistics link show can0
```

다른 bitrate로 이미 활성화된 경우 스택 종료 후 can0을 down하고 설정한다. 이미 올바르게 설정된 CAN은 그대로 사용한다.

```bash
cd /home/wgr/behavior_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export PYTHONNOUSERSITE=1
export PYTHONPATH="/home/wgr/behavior_ws/validation/live/python_deps:$PYTHONPATH"
ros2 launch behavior_stack_bringup behavior_stack.launch.py \
  enable_vehicle_interface:=true can_channel:=can0 can_receive_only:=true
```

로컬 NumPy와 시스템 SciPy 충돌을 피하려고 PYTHONNOUSERSITE를 사용한다. 이 PC에 이미 설치된 utm, transforms3d, python-can 및 import 의존성을 `validation/live/python_deps`에 따로 복사하여 import 확인했다. 다른 PC는 해당 의존성을 별도로 설치해야 한다.

다른 터미널에서 동일 ROS 환경을 source한 후:

```bash
python3 /home/wgr/behavior_ws/validation/live/check_topics.py
```

`/ERP/serial_data`가 수신되어야 한다. 수신 전용에서는 `/erp42pro/cmd_debug`를 발행하지 않는다.

매뉴얼과 현 Localization 코드는 초기 GPS/IMU/ERP 입력 외에 최초 GPS 기반 heading 보정도 필요하다. 매뉴얼은 수동 직진 7 km/h 이상에서 `corrections imu with gps` 확인을 요구한다. 정차 상태의 CAN feedback만으로 `/Local/heading`과 Planning 출력이 시작된다고 보장할 수 없다. 실제 차량 이동과 자율 모드 전환은 이번 변경 검증에서 수행하지 않았다.

차량 제어를 사용하는 실행은 위 launch에 `can_receive_only:=false can_max_speed_kph:=10.0`을 지정한다. 현재 테스트는 소프트웨어 변환/송신 모드 검증이며 실차 조향 부호, 물리 속도, 감속 성능 검증을 대체하지 않는다.

## 소프트웨어 검증

CAN protocol 6개 + 브리지 모드 4개 테스트 통과. 수신 전용 타이머/종료 송신 차단, CAN ID 네 개 송신, 속도 상한, brake 20→0%, timeout 정지, 명령 미수신 송신 차단을 확인했다. DDS 및 실제 SocketCAN 통신은 실행 환경 소켓 제한으로 미검증이다.

## 2026-10-05 제어 백업 적용

참고 백업: `/home/wgr/Control_Back_UP/1005/wego_ws`. Controller와 CAN 브리지를 함께 적용했다.

- 명령 토픽은 `/Control/vehicle_cmd`, 배열은 `[valid, e_stop, gear, speed_mps, steer_rad, brake_pct]`다.
- 명령 gear는 Pro 코드 0=P, 1=D, 2=N, 3=R. 피드백 gear는 기존 코드 0=전진, 1=중립, 2=후진이다.
- 속도 명령은 목표 속도를 직접 전달한다. 예전 가속용 PD 증폭은 제거했다.
- `speed_scale=1.146`을 적용하며, 송신 속도는 실제 기준 속도/scale, 피드백은 센서 속도*scale이다.
- Localization과 Controller에서 차량 중심 속도를 왼쪽 바퀴 속도로 간주하던 환산을 제거했다.
- 기존 `B_wynz.json` 게인을 유지하며 ament package share 경로로 로드한다. 백업의 A_KCITY 게인으로 바꾸지 않았다.
- 기존 수신 전용 기본값과 10 km/h 상한을 유지했다. 백업의 자동 sudo CAN bringup은 기본 false로 두고 CAN은 일반 터미널에서 설정한다.
- 종료 시 CAN 수신 스레드를 먼저 정리한다.
- debug 배열은 `[valid, 실제기준속도_kph, 조향_deg, 브레이크_pct, Pro_gear, CAN속도_kph]`다.
- 수신 전용에서는 debug 송신 명령 출력이 없다. 송신하려면 `can_receive_only:=false`.

### 카메라

설치된 RealSense 드라이버를 기준으로 RGB 640×480, 30 fps 구성을 추가했다. 카메라 종류는 아직 사용자 확인 전이다.

카메라만 실행:

```bash
bash /home/wgr/behavior_ws/validation/1005_reference/start_camera.sh
```

통합 실행에는 `enable_camera:=true`를 추가한다. 카메라를 따로 실행한 상태에서는 중복으로 켜지 않는다.
카메라는 core stack 밖에서 실행하며 기본 영상 계약은 `/camera/image_raw`이다. 실제 Logitech
driver topic이 다르면 `camera_image_topic` launch argument로 변경한다.
속도표지판 인식을 켜려면 `enable_camera_sign:=true speed_sign_weights:=traffic_sign_detector.pt`를 추가한다. 모델 클래스 매핑과 실영상 인식은 이번 검사에서 검증하지 않았다.

카메라 시작 시도는 sandbox의 `getifaddrs: Operation not permitted`로 실패했다. 실제 카메라 영상 수신 완료로 판정하지 않는다. 일반 터미널에서 시작 후 `validation/live/check_topics.py`로 확인한다.

검증: Controller/CAN/Localization/bringup 빌드, CAN protocol/bringup/bridge 모드 23개 테스트, 종방향 직접 목표 전달·브레이크·속도 한계 C++ probe. 로그는 `validation/1005_reference`에 저장한다. 실제 ROS/CAN/영상 동작은 일반 터미널 검증이 필요하다.
