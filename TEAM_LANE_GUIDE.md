# 팀원용 로봇 설치·차선 주행 가이드

**이 브랜치와 관제 저장소의 같은 이름 브랜치를 함께 사용하세요.**

| 항목 | 값 |
|---|---|
| 기준 | 2026-10-01 현장 적용 코드 |
| 로봇 저장소 / 브랜치 | `jsh0116/pinky-lane-driving` / `codex/lane-field-20261001` |
| 관제 저장소 / 브랜치 | `INYUP-BAEK/pinky-fleet-control` / `codex/lane-field-20261001` |
| 환경 | ROS 2 Jazzy, Python 3.12; 시험 로봇은 aarch64 Raspberry Pi |
| 이번 실행 모드 | 차선만 시험, Nav2 꺼짐, 횡단보도 정지 꺼짐 |
| 기본 식별자 | robot1 / 로봇 domain 21 / 관제 PC domain 22 |

관제 설치·실행은 [관제 가이드](https://github.com/INYUP-BAEK/pinky-fleet-control/blob/codex/lane-field-20261001/TEAM_LANE_GUIDE.md)를 따릅니다. 기존 README의 기본 설정보다 **이 브랜치 시험에는 이 문서를 우선**합니다.

## 1. 설치 전 준비

이 문서는 ROS Jazzy와 Pinky의 하드웨어 구성이 준비된 로봇에 적용합니다. 공장 초기 OS에 UART·카메라 드라이버·모터 펌웨어까지 설치하는 이미지는 포함하지 않습니다.

- 모터: `/dev/ttyAMA4`, 라이다 C1: `/dev/ttyAMA0`. 다른 하드웨어에서는 실제 포트와 전원·UART 설정을 먼저 맞춥니다. 접근 권한은 `ls -l /dev/ttyAMA{0,4}`와 `id`로 확인합니다.
- 현재 카메라: OV5647, 640×480, orientation 180. YOLO 입력은 **448**입니다.
- 카메라/차체 장착·차선 폭·차체 크기가 달라지면 아래 보정을 다시 합니다. 현재 차선 폭은 0.17m입니다.
- 같은 네트워크에 domain 21을 쓰는 기존 로봇이 동시에 있으면 이 예제 설정으로 시작하지 않습니다. 다른 기체 한 대에 같은 robot1 프로필을 재사용하는 절차입니다.
- 기존 bringup, UI, bridge를 종료한 뒤 진행합니다. 한 기체에 bringup을 중복 실행하지 않습니다.

### 로봇 저장소 설치

새 워크스페이스에서 실행합니다. 이미 같은 패키지가 `src`에 있으면 중복 clone하지 마세요.

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-rosdep \
  python3-venv python3-pip libcamera-dev
source /opt/ros/jazzy/setup.bash
mkdir -p ~/pinky_robot_ws/src
git clone --branch codex/lane-field-20261001 \
  https://github.com/jsh0116/pinky-lane-driving.git \
  ~/pinky_robot_ws/src/pinky-lane-driving
```

권한이 필요한 저장소는 GitHub 계정의 읽기 권한과 인증이 있어야 clone할 수 있습니다. SSH 개인키·토큰은 공유하지 않습니다.

### 카메라 드라이버 설치

기존 시험 로봇은 별도 `camera_ws`의 다음 소스를 사용했습니다.

```bash
mkdir -p ~/camera_ws/src
git clone https://github.com/christianrauch/camera_ros.git ~/camera_ws/src/camera_ros
git -C ~/camera_ws/src/camera_ros checkout 8f792e27a6dbc81e4943a75765fc1b7b7d37b301
source /opt/ros/jazzy/setup.bash
# rosdep 초기화가 안 된 장비에서만: sudo rosdep init
rosdep update
rosdep install --from-paths ~/camera_ws/src --ignore-src -r -y --rosdistro jazzy
cd ~/camera_ws
colcon build --symlink-install --packages-select camera_ros
source ~/camera_ws/install/setup.bash
```

시험 장비는 `/usr/local/lib/aarch64-linux-gnu`의 **libcamera 0.3.2**를 사용했습니다. 위 apt의 libcamera만으로 다른 이미지에서 OV5647이 인식되는지는 별도 확인해야 합니다. 이미 Pinky 카메라 구성이 작동하면 해당 구성을 유지하세요. `camera_ros` 빌드 성공만으로 카메라 동작이 검증되지는 않습니다.

### 추론 환경과 모델

모델은 저장소에 포함하지 않습니다. 팀 내 별도 전달받은 `best.pt`를 **빌드 전에** 아래 위치에 둡니다.

```bash
mkdir -p ~/pinky_robot_ws/src/pinky-lane-driving/pinky_lane_driving/models
# 전달받은 best.pt를 위 models/best.pt 위치로 복사한 후 확인
sha256sum ~/pinky_robot_ws/src/pinky-lane-driving/pinky_lane_driving/models/best.pt
```

기준 모델: 6,541,917 bytes, SHA256:

```text
5a01c38f6744f228b0d3e99c14989386185442de0efaf1235ad9a331950b7ede
```

현장 확인 버전은 `ultralytics 8.4.148`, `torch 2.14.0`, `numpy 1.26.4`, `opencv-python 4.10.0.84`입니다. 이미 이 조합이 설치되어 있다면 재설치하지 않습니다. 새 장비에서는 ROS의 시스템 Python을 보존하고 다음 격리 환경을 사용할 수 있습니다. 해당 아키텍처의 wheel이 없으면 설치를 중단하고 팀의 검증된 환경을 전달받으세요. 최신 버전으로 임의 대체한 환경은 현장 검증 조합과 다릅니다.

```bash
/usr/bin/python3 -m venv --system-site-packages ~/pinky_inference
~/pinky_inference/bin/python -m pip install \
  'numpy==1.26.4' 'opencv-python==4.10.0.84' \
  'torch==2.14.0' 'ultralytics==8.4.148'
```

ROS 실행 파일은 시스템 Python을 쓸 수 있으므로 venv 활성화만으로 충분하지 않습니다. 위 방식으로 설치했으면 **아래 실행 터미널에도** 다음을 설정합니다.

```bash
export PYTHONPATH="$HOME/pinky_inference/lib/python3.12/site-packages:${PYTHONPATH:-}"
python3 -c 'import rclpy, torch, cv2, numpy, ultralytics; print(torch.__version__, cv2.__version__, numpy.__version__, ultralytics.__version__)'
```

640 추론 입력 또는 FP16·INT8·NCNN·ONNX는 이번 공유 버전에 사용하지 않습니다. 640 입력은 기존 모델의 실제 차선 프레임에서 경로 검출에 실패했습니다.

### 로봇 패키지 빌드

```bash
source /opt/ros/jazzy/setup.bash
source ~/camera_ws/install/setup.bash
cd ~/pinky_robot_ws
# 전체 Gazebo/부가 패키지 대신 실제 실행에 필요한 경로만 의존성 설치
rosdep install --from-paths \
  src/pinky-lane-driving/pinky_bringup \
  src/pinky-lane-driving/pinky_description \
  src/pinky-lane-driving/pinky_fleet_safety \
  src/pinky-lane-driving/pinky_interfaces \
  src/pinky-lane-driving/pinky_lane_driving \
  src/pinky-lane-driving/pinky_led \
  src/pinky-lane-driving/pinky_navigation \
  src/pinky-lane-driving/pinky_robot_system \
  src/pinky-lane-driving/sllidar_ros2 \
  --ignore-src -r -y --rosdistro jazzy --skip-keys camera_ros
colcon build --symlink-install --packages-up-to pinky_robot_system
source ~/pinky_robot_ws/install/setup.bash
ros2 pkg prefix pinky_robot_system
ros2 pkg prefix camera_ros
```

`camera_ros`는 위에서 소스로 설치했기 때문에 rosdep에서만 제외합니다. 의존성 설치 실패를 무시하고 주행 단계로 넘어가지 마세요. 부가 기능용 `pinky_motion`도 코드에는 포함되지만 차선 단독 시험에는 필요하지 않습니다.

## 2. 주행 설정 준비

```bash
mkdir -p ~/pinky_calibration
# 대상 파일이 이미 있으면 덮어쓰기 전에 복사본을 보관하세요.
cp ~/pinky_robot_ws/src/pinky-lane-driving/pinky_lane_driving/config/lane_control_lane_only.json \
  ~/pinky_calibration/lane_control_lane_only.json
```

| 설정 | 현재 값 / 의미 |
|---|---|
| `control.min_lookahead` | 0.10m; 속도 항도 반영되므로 항상 고정 10cm 목표점이라는 뜻은 아님 |
| `control.steering_gain` | 1.2 |
| 최대 선속도 / 각속도 | 0.03m/s / 0.6rad/s |
| 영상 유효 시간 / 명령 timeout | 1.1s / 0.2s |
| 차선 일시 소실 | 마지막 측정 경로를 odom으로 변환, 최대 1.5s·0.06m·0.03m/s |
| 제어 정지 여유 / 복귀 최소 경로 길이 | 0.02m / 0.06m; 서로 다른 조건 |
| 차체 polygon | x: -0.08~0.06m, y: -0.06~0.06m; 추가 padding 0 |
| 라이다 자체 반사 필터 / 정지 여유 | 0.07m / 0.06m |
| 횡단보도 | `lane_control_lane_only.json`은 정지 조건 끔; `lane_control.json`은 일반 설정 |

**다른 로봇의 보정:** 위 JSON의 카메라 행렬·왜곡·호모그래피·ROI·mounting_id는 기존 robot1 기준입니다. 카메라 높이/각도/렌즈/차선 폭이 달라지면 복사만으로 적용 완료가 아닙니다. `pinky_bringup/config/pinky_camera.yaml`과 `pinky_lane_driving/config/ground_measurements.yaml`을 새 측정값으로 작성해 보정합니다.

```bash
ros2 run pinky_lane_driving lane_calibrate_ground \
  --camera "$HOME/pinky_calibration/new_camera.yaml" \
  --measurements "$HOME/pinky_calibration/new_ground_measurements.yaml" \
  --output "$HOME/pinky_calibration/new_calibrated.json"
```

위 명령은 설정 전체를 생성합니다. 생성된 제어값을 그대로 사용하지 말고, `new_calibrated.json`의 `calibration`·`mounting_id`와 실측 차선 폭을 시험용 JSON에 반영한 뒤 이 문서의 **0.03m/s 제한과 현재 튜닝값을 유지**합니다. 새 카메라 intrinsic 파일은 저장소의 `pinky_camera.yaml`에도 적용해 재빌드합니다. 차체 polygon/TF/라이다 높이도 실물과 맞춥니다. 기존 장비에서 벽 간격 2cm로 시험했다는 사실은 다른 기체의 안전 간격을 보증하지 않습니다.

## 3. 로봇 실행 — 아직 출발하지 않음

로봇 터미널 A에서 실행하고 켜둡니다. `lane_start_enabled:=false`를 유지합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/camera_ws/install/setup.bash
source ~/pinky_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=21
export LD_LIBRARY_PATH="/usr/local/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
# 앞서 venv로 추론 환경을 설치한 경우에만:
# export PYTHONPATH="$HOME/pinky_inference/lib/python3.12/site-packages:${PYTHONPATH:-}"
ros2 launch pinky_robot_system robot_system.launch.py \
  robot_id:=robot1 \
  start_motors:=true start_battery:=false start_lane_control:=true \
  lane_control_config:="$HOME/pinky_calibration/lane_control_lane_only.json" \
  lane_dry_run:=false hardware_watchdog_confirmed:=true \
  start_nav2:=false lane_start_enabled:=false \
  perception_imgsz:=448
```

`hardware_watchdog_confirmed:=true`는 모터 통신 상실 시 정지를 실제 확인한 장비에서만 설정합니다. 새 기체는 바퀴를 띄운 상태에서 모터 명령 timeout/비상정지를 먼저 확인하세요. 이 launch는 모터 드라이버를 시작하므로 현장 작업자가 준비된 상태에서 실행합니다. 이번 차선 단독 시험에서는 지도 인수가 필요 없습니다. Nav2를 켤 때는 해당 환경의 YAML+PGM 지도와 초기 위치 설정이 별도로 필요합니다.

### 로봇 터미널 B: 정지 상태 20초 점검

카메라 원본은 로봇 안에서만 구독하고 PC로 전송하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=21
python3 ~/pinky_robot_ws/src/pinky-lane-driving/tools/field_test/pinky_stationary_probe.py
```

출력에서 카메라 640×480, scan/odom/차선 진단 수신, `STOP`, cmd_vel 0, 로컬 estop=true, 유효 경로와 라이다 장애물 없음, watchdog 0.03m/s·0.2s·1.1s를 확인합니다. 이 probe는 관측 결과를 출력하는 도구이고 종료 코드 0이 모든 출발 조건 통과를 뜻하지 않습니다. 최종 시작 스크립트가 신선한 진단을 다시 검사합니다.

## 4. 관제 PC에서 시작·종료

1. [관제 가이드](https://github.com/INYUP-BAEK/pinky-fleet-control/blob/codex/lane-field-20261001/TEAM_LANE_GUIDE.md)에 따라 bridge와 UI를 각각 하나 실행합니다.
2. 감시기를 먼저 실행합니다. 로봇 배치·빈 코스·전원·현장 감시·즉시 비상정지 가능 여부를 확인합니다.
3. 관제 PC의 시작 스크립트로 로컬 주행 허가와 중앙 시험 요청을 켭니다.
4. 출구에서 UI의 **robot1 차선 종료 Bool**을 누릅니다. 감시기 요약의 중앙/로컬 권한 해제 성공, STOP·cmd_vel=0을 확인합니다.
5. 로봇 launch를 Ctrl+C로 종료합니다. 예전 systemd 방식이 실행 중인 경우 `systemctl --user stop pinky-lane-test.service`로 종료합니다. PC UI/bridge도 종료합니다.

재시험은 로봇 launch를 완전히 종료한 후 다시 시작합니다. 종료 Bool 상태가 남아 있는 세션에서 허가만 반복해서 켜지 마세요.

## 5. 현재 검증 범위와 점검할 현상

- 10cm 최소 추종 거리에서 사용자가 회전 안쪽 선 밟음 감소·중앙 주행 개선을 확인했습니다.
- 짧은 반대쪽 마스크로 경로가 소실되는 문제와 차체→경로 시작점의 인위적인 대각선 충돌 판정을 수정했습니다. 전방 정지 거리와 실제 명령 회전 궤적의 장애물 검사는 유지됩니다.
- 수정 단계별로 약 3.55m, 0.20m, 0.16m를 주행했습니다. 마지막 구간은 사용자의 종료 Bool로 끝났으며 **최종 코드로 전체 곡선 코스를 연속 완주한 검증은 아닙니다.** 사용자는 끝단을 직선으로 연장할 계획입니다.
- 다른 로봇·다른 카메라 보정·다른 벽 간격에서의 주행은 미검증입니다. `single_boundary`, `recent_path_no_boundaries`, `no_current_lane`, `lane_observation_stale`와 실제 움직임을 비교합니다.
- 카메라 영상/rosbag을 자동 저장하는 기능은 추가하지 않았습니다. 기존 감시기는 텍스트 상태·거리·종료 원인만 출력합니다.

하드웨어 없는 검사:

```bash
cd ~/pinky_robot_ws/src/pinky-lane-driving
./.agents/tools/harness.sh fast
# ROS 환경과 camera_ws를 source한 뒤, 별도 .agents/output에 빌드/검사
./.agents/tools/harness.sh ros pinky_robot_system pinky_lane_driving pinky_fleet_safety
```

단위 테스트·ROS 빌드·정지 점검·실주행은 서로 다른 검증입니다. 테스트 통과만으로 다른 기체의 주행 완료를 주장하지 않습니다.

### 공유 브랜치 포장 시 검증

현재 PC에서 로봇 의존 패키지 9개 빌드, 순수 알고리즘 테스트 128개, 격리 domain 177의 ROS 제어 테스트 12개를 통과했습니다. 패키지 검사 최종 집계는 오류/실패 0개이며 기존 skip 1개가 있습니다. XML 검사는 외부 ROS 스키마 서버 접근 오류가 있어 공식 `ros-infrastructure/rep`의 XSD를 로컬 catalog로 연결한 뒤 통과했습니다. 새 로봇에 대한 깨끗한 OS 설치와 실주행은 별도 확인 대상입니다. 모델·현장 영상·rosbag·자격증명은 포함하지 않습니다. tests의 작은 경로 좌표 fixture는 알고리즘 회귀 검증용이며 영상 데이터셋을 포함하지 않습니다.
