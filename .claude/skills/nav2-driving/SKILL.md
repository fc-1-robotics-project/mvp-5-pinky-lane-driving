---
name: nav2-driving
description: "Pinky Nav2 지도 기반 주행(SLAM 지도 작성, AMCL 위치추정, Regulated Pure Pursuit 컨트롤러, 코스트맵·footprint, 런치 파일, Gazebo 시뮬레이션, 목표점·웨이포인트 전송)을 설정·수정·검증하는 방법. nav 주행, 네비게이션, 지도 만들기, 목표 지점 이동, Nav2 파라미터 튜닝 작업에 사용한다. 카메라 차선 추종은 lane-perception / drive-control 스킬을 쓴다."
---

# Nav2 주행

## 패키지 구성 (`pinky_navigation/`)
| 경로 | 용도 |
| --- | --- |
| `params/nav2_params.yaml` | AMCL, BT navigator, controller(`FollowPath` = RegulatedPurePursuit), planner(NavFn), smoother, costmap, behaviors |
| `params/mapper_params.yaml` | slam_toolbox |
| `launch/bringup_launch.xml` / `gz_bringup_launch.xml` | 실물 / 시뮬레이션 전체 (localization + navigation) |
| `launch/map_building.launch.xml` / `gz_map_building.launch.xml` | SLAM |
| `launch/*_view.launch.xml` | RViz |
| `map/*.yaml, *.pgm` | 지도 |
| `scripts/nav2_web_server.py` | 웹 목표 전송 |

Gazebo 로봇: `pinky_gz_sim/launch/launch_sim.launch.xml`. 실물 bringup: `pinky_bringup/launch/bringup_robot.launch.xml` (사용자만 실행).

## 기본 흐름 (시뮬레이션 먼저)
```bash
source /opt/ros/jazzy/setup.bash && source .agents/output/install/setup.bash
ros2 launch pinky_gz_sim launch_sim.launch.xml
ros2 launch pinky_navigation gz_map_building.launch.xml        # 지도 작성
ros2 run nav2_map_server map_saver_cli -f pinky_navigation/map/<name>
ros2 launch pinky_navigation gz_bringup_launch.xml map:=$PWD/pinky_navigation/map/<name>.yaml
ros2 launch pinky_navigation gz_nav2_view.launch.xml           # RViz에서 2D Pose Estimate → Goal
```
목표 전송을 코드로 할 때는 `nav2_simple_commander.robot_navigator.BasicNavigator`의 `setInitialPose`, `goToPose`/`followWaypoints`, `isTaskComplete`, `getResult`를 쓴다. 좌표는 `map` 프레임, 미터, yaw는 쿼터니언으로.

## 규칙과 이유
- `use_sim_time`: 시뮬레이션 true, 실물 false. 한 런치 안에서 섞으면 TF 외삽 오류로 AMCL이 튄다.
- `footprint`(현재 0.12 m 사각형)와 `footprint_padding`은 실제 로봇 치수와 맞춘다. 차선 주행 config의 `footprint_radius_m`(0.1)과 모순되지 않게 한다.
- 속도: Nav2 `desired_linear_vel: 0.2`, 차선 주행 `max_speed: 0.1`. 실물 첫 시험은 낮은 쪽에 맞춘다.
- 명령 경로: controller `cmd_vel` → remap `cmd_vel_nav` → velocity_smoother → `cmd_vel`. 차선 주행 watchdog과 동시에 `cmd_vel`을 쓰지 않는다. 둘을 함께 쓰려면 twist_mux 같은 명시적 선택기를 두고 검토 대상으로 표시한다.
- 파라미터 변경은 키/이전/이후/이유 표로 보고한다. 한 번에 한 묶음만 바꿔야 원인을 알 수 있다.

## 검증
```bash
./.agents/tools/harness.sh ros pinky_navigation     # 빌드 + 테스트 (항상)
python3 -c "import yaml,sys; yaml.safe_load(open('pinky_navigation/params/nav2_params.yaml'))"
ros2 launch pinky_navigation gz_bringup_launch.xml --show-args   # 런치 파싱
```
디스플레이/Gazebo가 없으면 여기까지만 하고 "시뮬레이션 미실행"이라고 적는다. 시뮬레이션을 돌렸다면 목표 도달 여부(`getResult`), 소요 시간, 복구 동작(spin/backup) 발생 횟수를 보고한다.

## 튜닝 가이드 (`controller_server.FollowPath`, costmap)
| 증상 | 먼저 볼 노브 |
| --- | --- |
| 목표 근처에서 빙빙 돔 | `xy_goal_tolerance`, `yaw_goal_tolerance`, `rotate_to_heading_min_angle` |
| 좁은 통로 통과 못 함 | `inflation_radius` ↓, `cost_scaling_factor` ↑ |
| 경로 흔들림 | `lookahead_dist` ↑ |
| 장애물 앞 급정지 | `use_collision_detection`, `max_allowed_time_to_collision_up_to_carrot` |
| 위치가 튐 | AMCL `alpha1..5`, `laser_max_range`, 초기 위치 |
