# 통합 시연용 차선 종료 상태

기준일: **2026-10-06**. 브랜치 `codex/two-robot-demo-20261005`의 관제 통합 시연과 함께 사용한다. 팀 `develop`이 병합 대상이다.
전체 통합 시연의 현장 완주 여부는 별도 주행 검증 사항이다.

## 로봇 코드 흐름

| 코드 | 역할 |
|---|---|
| `pinky_robot_system/launch/robot_system.launch.py` | 센서·Nav2·차선·관제 안전 스택 통합 실행, 시작 허가는 OFF |
| `perception.py`, `ros_perception.py` | 로컬 카메라 640×480 → YOLO 448 → stamp·차선 polygon |
| `tracking.py`, `path.py`, `control.py`, `runtime.py`, `ros_control.py` | 기체 보정으로 지면 좌표 변환 → 양쪽/한쪽 경계 경로 → odom 보정 → 추종 명령·진단 |
| `obstacles.py`, `ros_watchdog.py` | scan/TF 기반 충돌 검사, 명령 신선도·속도 상한·로컬 허가 검증 |
| `lane_mission_server.py`, `mission_guard.py`, `exit_telemetry.py` | FollowLane 준비·허가 갱신·정체/고장 검사·상태 보고·종료 정리 |

표의 차선 파일은 `pinky_lane_driving/pinky_lane_driving/` 아래에 있다.
최종 차선 후보는 `drive_mode_mux`에서 Nav2/수동 후보와 모드로 선택한 뒤
`fleet_velocity_gate`의 관제 permit을 통과하여 모터 `/cmd_vel`로 전달한다.
카메라 영상은 PC로 전송하지 않으며 경로 없는 정상 프레임과 센서 고장을 구분한다.

한쪽 경계는 측정한 선과 차선 폭으로 보완한다. 양쪽 선 소실 시 새 경로를 만들지 않고
기존 측정 경로를 odom으로 변환해 최대 1.5초·6cm만 사용한다.
제어·영상 신선도 1.1초, 명령 무응답 0.2초 제한은 유지한다.

## 변경 범위

`pinky_lane_driving`만 수정한다. 기존 FollowLane 액션, 종료 Bool,
로컬 주행 허가 lease, 속도/조향/라이다 설정은 유지한다.

- `lane/diagnostics`: 현재 프레임의 검증된 `boundary_observation`
  (`left_visible`, `right_visible`)와 `observation_age_s` 추가.
  잘못된/중복/오래된 프레임은 양쪽 없음으로 처리하지 않는다.
- `lane/mission_status`: `status_time_s`, `exit_status.version=1`,
  가시성/영상 나이/센서·관제 허가 상태/`stationary_s` 추가.
  기존 약 0.6초 주기의 작은 JSON 상태 메시지에 포함한다. 영상·rosbag 수집은 추가하지 않는다.
- 실제 odom 속도, 위치 변화(3mm), 회전(0.02rad), 프레임·stamp 신선도로 정지를 측정한다.
  명령 속도 0만으로 실제 정지로 판단하지 않는다.
- 경로가 없는 정상 프레임과 카메라 고장을 구분한다.
  정상 프레임의 경로 소실은 기존 15초 차선 소실/정지 감시 대상으로 유지한다.
  카메라·TF·센서 오류, 로컬 허가 소실은 기존 고장 중단을 유지한다.

## 관제와 연결

관제가 기존 5Hz `fleet/pose`(AMCL 공분산 + 현재 localization TF)를 사용해
끝 반경 20cm 안에서 0.5초 도착이 유지되는지 확인한다. 차선 유무와 정지 대기는
필수 조건이 아니며, 최신 센서·허가 상태를 확인해 기존 `lane/finish` Bool을 발행한다. STOP·로컬 허가 OFF 정리 후 액션 성공을 반환한다.
새 관제는 이 정리 결과를 확인한 뒤 A는 Nav2로, B는 최종 STOP/HOLD로 전환한다.
AMCL 초기 위치를 임의 재설정하지 않는다. PC와 로봇의 시각 동기화가 필요하다.

배포 시 양쪽 로봇의 이 패키지를 빌드한 뒤 관제의 `vision_control`,
`multibot_control_ui`도 같은 브랜치 버전으로 빌드한다.
상세 UI 순서는 [관제 통합 시연 가이드](https://github.com/fc-1-robotics-project/mvp-5-pinky-fleet-control/blob/codex/two-robot-demo-20261005/multibot_control_ui/docs/TWO_ROBOT_DEMO.md) 참고.

## 하드웨어 없이 검증

```bash
./.agents/tools/harness.sh fast
./.agents/tools/harness.sh ros pinky_lane_driving
source /opt/ros/jazzy/setup.bash
source .agents/output/install/setup.bash
ROS_LOG_DIR=/tmp/pinky_demo_ros_logs ROS_DOMAIN_ID=177 \
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
PYTHONPATH="$PWD/pinky_lane_driving:$PWD/tests:${PYTHONPATH:-}" \
python3 -m unittest ros_control ros_mission -v
```

이동 후 정지할 때 `stationary_s`가 null로 남던 타이머 초기화 오류를 수정했다.
정지 상태 보고는 유지하며, 관제의 위치 기반 종료는 이 값에 의존하지 않는다.
실제 종료 위치와 Nav2 전환은 별도 현장 확인이 필요하다.

## 현재 프로필과 PR #4의 범위

팀 `develop`에 병합된 PR #3/#4를 포함한다. 이 PR은 그 조향/속도/장애물 판정 코드를 다시 변경하지 않는다.
`config/lane_control.json`과 `config/lane_control_lane_only.json`의 운용 값은 현재 같다.

| 설정 | 값 |
|---|---|
| `behavior.cruise_speed`, `control.max_speed`, `path.fallback_speed` | 0.09m/s |
| `path.blind_speed`, `behavior.approach_speed` | 0.06m/s |
| `crosswalk_control_enabled`, `behavior.crosswalk_stop` | true, false (감속 통과) |
| `lidar_obstacle_stop_enabled` | true |
| `control.min_lookahead`, `control.steering_gain`, `control.max_omega` | 0.10m, 1.2, 0.6rad/s |

PR #4는 회전 시 직진 반응/제동 구간의 라이다 점만 측정 차선 중심으로부터
`path.width/2 + sensors.straight_check_lane_margin_m`(여유 기본 0) 범위로 필터한다.
명령 조향 arc와 계획 경로의 충돌 검사는 필터하지 않는다.
차선 정보가 유효하지 않으면 보수적인 기존 검사를 사용한다. 전체 라이다 정지 제거가 아니다.

기체 홈의 `lane_control_config` JSON은 코드 업데이트로 덮어쓰지 않는다.
카메라 homography·차선 폭·차체 TF/polygon은 실측 기체별 값을 사용한다.
라이다/횡단보도/속도를 바꾸려면 파일명보다 실제 JSON과 launch 상한을 확인한다.

## 검증과 남은 문제

2026-10-06 하드웨어 없는 검사 152개, 격리 domain 177의 ROS 제어·임무 검사 25개 통과. `harness.sh ros pinky_lane_driving`의 대상 빌드와 core suite는 성공했으나 전체 결과 집계는 이전 `pinky_navigation` lint/XML 실패 기록 때문에 종료 코드 1이었다. 이를 전체 ROS 검사 통과로 표시하지 않는다. 두 기체의 현재 소스와 설치 모듈을 대조했고
실행용 14개 패키지 빌드를 완료했다. 빌드·모의 ROS 검사와 실제 주행은 별도 증거다.

실제 통합 시연에서 A 최종 Nav2가 목표 전에 멈췄다. 관제의 전체 복구 대기가 B 차선 실패를
A에도 STOP/HOLD로 적용하는 구조가 확인됐다. B 실패 직전 `sensor_failure`/`obstacle`이
기록됐으나 정확한 중단 원인은 확정되지 않았다. 로봇별 복구 분리와 의도적 HOLD의
Nav2 액션 정리는 관제 측 미해결 사항이다. 전체 완주나 새 기체 검증 완료를 주장하지 않는다.
