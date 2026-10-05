# 통합 시연용 차선 종료 상태

브랜치 `codex/two-robot-demo-20261005`의 관제 통합 시연과 함께 사용한다.
이번 작업에서는 실제 로봇에 적용하지 않았다.

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
끝 반경 7cm 안인지 확인한다. 그 안에서 양쪽 선이 모두 없거나 실제 3초 정지한 경우
기존 `lane/finish` Bool을 발행한다. STOP·로컬 허가 OFF 정리 후 액션 성공을 반환한다.
새 관제는 이 정리 결과를 확인한 뒤 A는 Nav2로, B는 최종 STOP/HOLD로 전환한다.
AMCL 초기 위치를 임의 재설정하지 않는다. PC와 로봇의 시각 동기화가 필요하다.

배포 시 양쪽 로봇의 이 패키지를 빌드한 뒤 관제의 `vision_control`,
`multibot_control_ui`도 같은 브랜치 버전으로 빌드한다.
상세 UI 순서는 관제 저장소 `multibot_control_ui/docs/TWO_ROBOT_DEMO.md` 참고.

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

실주행 및 7cm 도착 정밀도는 별도 현장 확인이 필요하다.
