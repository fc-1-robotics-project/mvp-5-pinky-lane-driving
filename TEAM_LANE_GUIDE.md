# 새 로봇 설치·기존 로봇 업데이트

**로봇과 관제 PC 모두 `codex/two-robot-demo-20261005` 브랜치로 설치합니다.**

- 로봇: [jsh0116/pinky-lane-driving](https://github.com/fc-1-robotics-project/mvp-5-pinky-lane-driving/tree/codex/two-robot-demo-20261005)
- 관제: [INYUP-BAEK/pinky-fleet-control](https://github.com/fc-1-robotics-project/mvp-5-pinky-fleet-control/blob/codex/two-robot-demo-20261005/TEAM_LANE_GUIDE.md)
- 설치 후 실행·UI 사용·종료: [UI_INTEGRATION.md](UI_INTEGRATION.md)

## 1. 준비할 장비·파일

| 구분 | 준비 내용 |
|---|---|
| OS / ROS | Ubuntu 24.04 / ROS 2 Jazzy / 시스템 Python 3.12. 기준 로봇은 Ubuntu 24.04.4 aarch64 Raspberry Pi |
| 하드웨어 | Pinky 모터·펌웨어·UART·라이다·카메라가 동작하는 기본 이미지. 이 저장소는 OS/펌웨어 설치 이미지가 아님 |
| 지도 | 현장 Nav2 YAML **및 YAML의 `image:`가 가리키는 PGM/PNG** |
| 모델 | 팀에서 별도 전달받은 `best.pt` (아래 해시 확인) |
| 보정 | 새 기체의 카메라 intrinsic·지면 변환·장착 TF·차선 폭·차체 크기 |

ROS가 없으면 먼저 [ROS Jazzy 공식 Ubuntu 설치 안내](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)를 따릅니다. ROS 저장소가 등록된 뒤 로봇은 `ros-jazzy-ros-base`로 시작할 수 있고, 아래 rosdep으로 필요한 실행 패키지를 설치합니다.

현재 공유 프로필은 **기본/최대/한쪽 보완 0.09m/s, 차선 소실 유지/횡단보도 감속 0.06m/s, 횡단보도 정지 OFF, 라이다 물체 정지 ON**입니다. 전체 통합 시연 완주는 미확인이고 새 기체도 현장 점검이 필요합니다. 차선 외 Nav2 속도/장애물 설정은 `pinky_navigation/params/nav2_params.yaml`의 별도 설정입니다.

기준 하드웨어는 모터 `/dev/ttyAMA4`, C1 라이다 `/dev/ttyAMA0`, OV5647 카메라 640×480·orientation 180입니다. `ls -l /dev/ttyAMA{0,4}`와 `id`로 포트·접근 그룹을 확인합니다. 포트가 다르면 `pinky_bringup/config/pinky_params.yaml`과 `pinky_bringup/launch/bringup_robot.launch.xml`을 실제 장비에 맞춥니다. 기구 치수/TF는 `pinky_description`도 함께 확인합니다.

## 2. 소스 받기

같은 이름의 ROS 패키지가 이미 `src`에 있으면 중복 clone하지 말고 7절 업데이트 절차를 사용합니다. **로봇에는 로봇 저장소만**, 관제에는 관제 저장소를 설치합니다. 둘을 한 workspace에 clone하면 `pinky_interfaces`가 중복됩니다.

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-rosdep \
  python3-venv python3-pip python3-pytest libcamera-dev \
  ros-jazzy-camera-info-manager
source /opt/ros/jazzy/setup.bash
mkdir -p ~/pinky_robot_ws/src
git clone --branch codex/two-robot-demo-20261005 \
  https://github.com/fc-1-robotics-project/mvp-5-pinky-lane-driving.git \
  ~/pinky_robot_ws/src/pinky-lane-driving
# 처음 사용하는 장비에서만, rosdep이 초기화되지 않았다면 실행:
# sudo rosdep init
rosdep update
```

## 3. 카메라 및 추론 환경

### camera_ros

이미 `~/camera_ws`에서 카메라가 정상 동작하면 그 구성을 사용합니다. 새 설치의 기준 커밋은 현재 로봇에서 확인한 `8f792e27a6dbc81e4943a75765fc1b7b7d37b301`입니다.

```bash
mkdir -p ~/camera_ws/src
git clone https://github.com/christianrauch/camera_ros.git ~/camera_ws/src/camera_ros
git -C ~/camera_ws/src/camera_ros checkout 8f792e27a6dbc81e4943a75765fc1b7b7d37b301
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths ~/camera_ws/src --ignore-src -r -y --rosdistro jazzy
cd ~/camera_ws
colcon build --symlink-install --packages-select camera_ros
source ~/camera_ws/install/setup.bash
```

`/usr/local`의 libcamera 0.3.2를 쓰는 기체는 빌드 전에 `export PKG_CONFIG_PATH="/usr/local/lib/aarch64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH:-}"`를 설정합니다. 기준 커밋은 이 버전에서 `ControlTypePoint` switch 경고가 오류로 처리되어 실패하므로 `colcon build`에 `--cmake-args -DBUILD_TESTING=OFF -DCMAKE_CXX_FLAGS=-Wno-error=switch`를 추가합니다.

기준 기체는 `/usr/local/lib/aarch64-linux-gnu`의 libcamera 0.3.2 환경입니다. 새 이미지에서 시스템 libcamera만으로 OV5647이 동작한다는 보장은 없습니다. 이미 별도 libcamera를 설치했다면 rosdep의 해당 의존성을 `--skip-keys libcamera`로 제외하고 사용한 라이브러리와 빌드 경로를 맞춥니다. Raspberry Pi용 libcamera 지원/빌드는 [camera_ros 공식 안내](https://github.com/christianrauch/camera_ros#build-instructions)를 참고합니다. 라이브러리 버전을 바꿨으면 camera_ros도 다시 빌드합니다.

### 추론 환경

현재 로봇에서 확인한 버전(2026-10-06):

| 패키지 | 버전 |
|---|---|
| ultralytics | 8.4.148 |
| torch / torchvision | 2.14.0 / 0.29.0 |
| numpy | 1.26.4 |
| opencv-python | 4.10.0.84 |

이미 설치된 기준 로봇에는 재설치할 필요가 없습니다. 새 장비에서는 ROS 시스템 Python을 보존하도록 격리 환경을 사용합니다. 아래 명령은 해당 아키텍처용 wheel이 제공되는 환경이 전제입니다. 설치가 안 되면 임의 최신 버전으로 바꾸지 말고 팀의 검증된 wheel/이미지를 전달받습니다.

```bash
/usr/bin/python3 -m venv --system-site-packages ~/pinky_inference
~/pinky_inference/bin/python -m pip install \
  --index-url https://download.pytorch.org/whl/cpu \
  'torch==2.14.0' 'torchvision==0.29.0'
~/pinky_inference/bin/python -m pip install \
  'numpy==1.26.4' 'opencv-python==4.10.0.84' 'ultralytics==8.4.148'
```

torch는 반드시 CPU 저장소에서 먼저 설치합니다. 기본 PyPI의 aarch64 wheel은 Raspberry Pi에 필요 없는 NVIDIA CUDA 패키지 수 GB를 함께 받습니다.

모델은 Git에 포함하지 않습니다. 팀에서 받은 모델을 **로봇 패키지 빌드 전에** `~/pinky_robot_ws/src/pinky-lane-driving/pinky_lane_driving/models/best.pt`로 복사합니다.

```bash
mkdir -p ~/pinky_robot_ws/src/pinky-lane-driving/pinky_lane_driving/models
# 위 디렉터리에 전달받은 best.pt를 둔 뒤 확인
sha256sum ~/pinky_robot_ws/src/pinky-lane-driving/pinky_lane_driving/models/best.pt
```

기준 모델 크기: **6,541,917 bytes**, SHA256:

```text
5a01c38f6744f228b0d3e99c14989386185442de0efaf1235ad9a331950b7ede
```

YOLO 입력은 **448**입니다. 같은 모델의 640 입력은 실제 차선 프레임에서 경로 검출에 실패했습니다. FP16/INT8/NCNN/ONNX 비교는 이 브랜치 적용 범위에 포함하지 않습니다.

## 4. 의존성 설치·빌드

```bash
source /opt/ros/jazzy/setup.bash
source ~/camera_ws/install/setup.bash
cd ~/pinky_robot_ws
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

`camera_ros`는 별도 workspace에서 빌드했으므로 여기의 rosdep에서는 제외합니다. Gazebo/부가 모션 패키지는 통합 실물 실행의 빌드 대상이 아닙니다. 빌드 실패가 있으면 실행 단계로 넘어가지 않습니다.

시스템 Python으로 추론 패키지를 읽을 수 있는지 확인합니다. venv를 썼다면 **매 실행 터미널에도** 이 PYTHONPATH가 필요합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/camera_ws/install/setup.bash
source ~/pinky_robot_ws/install/setup.bash
export PYTHONPATH="$HOME/pinky_inference/lib/python3.12/site-packages:${PYTHONPATH:-}"
python3 -c 'import rclpy, torch, cv2, numpy, ultralytics; from pinky_interfaces.action import FollowLane; print(torch.__version__, cv2.__version__, numpy.__version__, ultralytics.__version__)'
ros2 pkg prefix --share pinky_lane_driving
```

`best.pt`를 빌드 후 복사했다면 `colcon build --symlink-install --packages-select pinky_lane_driving`을 다시 실행합니다. 설치 경로의 `share/pinky_lane_driving/models/best.pt`가 있어야 기본 launch에서 모델을 찾습니다.

## 5. 지도·기체 보정 준비

### 지도와 차선 프로필

```bash
mkdir -p ~/pinky_maps ~/pinky_calibration
# Nav2 YAML은 ~/pinky_maps/site.yaml로 준비합니다.
# YAML의 image: 항목이 가리키는 PGM/PNG도 해당 경로에 함께 준비합니다.
# 신규 설치 전용: 기존 운용 파일이 있다면 먼저 백업합니다.
cp -n ~/pinky_robot_ws/src/pinky-lane-driving/pinky_lane_driving/config/lane_control_lane_only.json \
  ~/pinky_calibration/lane_control_lane_only.json
```

저장소에는 `pinky_navigation/map/261003.yaml`과 대응 PGM이 포함됩니다. 새 현장 지도와 모델은 별도 준비하며, 포함된 지도를 그대로 쓸 때도 실제 현장과 일치하는지 확인합니다. 두 로봇을 같은 UI에서 운용할 때는 동일한 `map` 좌표계의 지도를 사용합니다.

### 새 기체 보정

1. 새 카메라의 intrinsic YAML과 실제 지면 대응점을 준비합니다. 형식 예시는 `pinky_bringup/config/pinky_camera.yaml`, `pinky_lane_driving/config/ground_measurements.yaml`입니다.
2. 아래 명령으로 새 보정 결과를 생성합니다.
3. 결과의 `calibration`, `mounting_id`, 실측 `path.width`를 운용 JSON에 반영합니다. 카메라 높이·각도·렌즈·ROI가 다르면 기존 robot1 보정을 그대로 사용하지 않습니다.
4. 새 intrinsic YAML을 `pinky_bringup/config/pinky_camera.yaml`에도 적용하고 재빌드합니다. 차체 polygon·라이다/카메라 TF·바퀴 치수도 실물과 맞춥니다.

```bash
ros2 run pinky_lane_driving lane_calibrate_ground \
  --camera "$HOME/pinky_calibration/new_camera.yaml" \
  --measurements "$HOME/pinky_calibration/new_ground_measurements.yaml" \
  --output "$HOME/pinky_calibration/new_calibrated.json"
```

생성기는 제어 설정까지 만듭니다. **측정 YAML의 초기 튜닝값은 현장 프로필과 다릅니다.** 생성 파일 전체로 운용 JSON을 덮어쓰지 말고 위 보정 항목만 반영합니다. 현장 프로필은 cruise/max/fallback 0.09m/s, blind/approach 0.06m/s, `min_lookahead=0.10`, `steering_gain=1.2`, 영상 신선도 1.1초, 명령 무응답 0.2초입니다. 기체별 미확인 항목이 있으면 먼저 정지·센서·비상정지 점검을 마칩니다.

## 6. 로봇 이름·네트워크·첫 실행

| 기체/장치 | robot_id | ROS_DOMAIN_ID |
|---|---|---:|
| 로봇 1 | robot1 | 21 |
| 로봇 2 | robot2 | 19 |
| 관제 PC | — | 22 |

다른 한 대로 robot1을 교체하면 이전 domain 21 기체를 종료합니다. 두 대를 동시에 사용하면 두 번째 로봇은 실행 명령의 `robot_id:=robot2`와 `ROS_DOMAIN_ID=19`를 함께 설정합니다. 이 두 ID/domain은 관제 코드에 이미 등록되어 있습니다. 다른 ID/domain은 관제의 `robot_config.py`, `domain_bridge.yaml`과 함께 수정·빌드합니다.

로봇과 PC는 DDS 통신 가능한 같은 네트워크, 시간 동기화가 필요합니다. `timedatectl status`로 시각을 확인합니다. `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`가 남아 있으면 다른 기체와 통신하지 못합니다. 표준 DDS에서는 [ROS 발견 범위 안내](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Improved-Dynamic-Discovery.html)의 SUBNET 설정을 사용합니다. IP는 SSH 접속용이며 코드에 고정된 로봇 IP는 없습니다.

**이제 [실행·UI 사용법의 1절](UI_INTEGRATION.md#1-로봇-ssh-터미널--한-번-실행)부터 진행합니다.** 로봇과 PC 각각 launch 한 개만 실행합니다. 현장 감시·즉시 정지가 가능한 조건에서 UI의 차선 단독 시험부터 확인합니다.

## 7. 기존 설치 업데이트

주행을 종료하고 로봇 launch를 닫은 상태에서 진행합니다. 아래 clone 경로가 아닌 평면 `src/pinky_lane_driving` 구성이라면 실제 Git 저장소 위치에서 갱신합니다. 기존 파일을 지우거나 중복 clone하지 않습니다.

```bash
cd ~/pinky_robot_ws/src/pinky-lane-driving
git status --short
# 변경이 있으면 보관·커밋한 뒤 진행합니다. 강제 reset은 하지 않습니다.
git fetch origin
git switch codex/two-robot-demo-20261005
git pull --ff-only origin codex/two-robot-demo-20261005
source /opt/ros/jazzy/setup.bash
source ~/camera_ws/install/setup.bash
cd ~/pinky_robot_ws
colcon build --symlink-install --packages-up-to pinky_robot_system
```

홈의 운용 JSON은 Git 업데이트로 바뀌지 않습니다. 실행 인자의 실제 JSON과 저장소 프로필을 비교합니다. 기체별 `calibration`, `mounting_id`, `path.width`, 차체 polygon/TF는 유지하고, 속도와 기능 설정은 아래 표를 확인해 필요한 항목만 반영합니다. 수정 전 운용 파일을 백업합니다.

| 항목 | 현재 저장소 값 |
|---|---|
| `behavior.cruise_speed`, `control.max_speed`, `path.fallback_speed` | 0.09m/s |
| `path.blind_speed`, `behavior.approach_speed` | 0.06m/s |
| `crosswalk_control_enabled` / `behavior.crosswalk_stop` | true / false (감속 통과) |
| `lidar_obstacle_stop_enabled` | true |
| `control.min_lookahead` / `control.steering_gain` | 0.10m / 1.2 |

`lane_control.json`과 `lane_control_lane_only.json`은 현재 같은 운용 값입니다. 파일명보다 내용을 기준으로 선택합니다. 차선 launch 워치독과 임무 서버의 상한은 0.09m/s이며 JSON만 증속하면 설정 불일치/속도 초과로 중단될 수 있습니다. 빌드 후 새 터미널에서 source하고 실행합니다. 세부 코드 설명은 [통합 시연용 상태 문서](doc/two_robot_demo_status.md)를 참고합니다.

## 8. 검사와 검증 범위

```bash
cd ~/pinky_robot_ws/src/pinky-lane-driving
./.agents/tools/harness.sh fast
source /opt/ros/jazzy/setup.bash
source ~/camera_ws/install/setup.bash
./.agents/tools/harness.sh ros pinky_robot_system pinky_lane_driving pinky_fleet_safety
```

2026-10-06 하드웨어 없는 알고리즘/감시 검사 **152개 통과**. 두 실제 로봇에 현재 소스 적용 및 실행용 14개 패키지 빌드를 완료했습니다. 이후 현장 통합 시연에서는 A 최종 Nav2 진행 중 관제 복구 대기가 두 로봇을 함께 멈추는 문제가 확인됐습니다. 전체 완주, 새 기체의 주행, 새 OS의 전체 설치는 검증 완료로 주장하지 않습니다. 격리 ROS 제어·임무 검사 25개도 통과했습니다. harness의 전체 결과 집계는 이전 Navigation lint/XML 실패 기록 때문에 실패했으며 전체 ROS 검사 통과와 구분합니다. 최신 패키지 검사 결과와 상태 계약은 `doc/two_robot_demo_status.md`를 확인합니다.

확인할 상태는 `single_boundary`, `recent_path_no_boundaries`, `no_current_lane`, `lane_observation_stale`, 실제 이동 거리·종료 이유입니다. 자동 영상/rosbag/CSV 수집 기능은 없습니다. `tools/field_test/pinky_stationary_probe.py`는 선택적 정지 진단용이고 정상 운용은 UI에서 합니다.
