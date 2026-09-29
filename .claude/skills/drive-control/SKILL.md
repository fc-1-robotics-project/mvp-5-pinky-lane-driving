---
name: drive-control
description: "Pinky 차선 주행 알고리즘(Pure Pursuit 조향, 속도·가속·횡가속 제한, 가시 경로 제동, 횡단보도 정지/재출발 상태기계, 장애물 정지, 명령 중재, watchdog, ROS 제어 노드)을 구현·수정·튜닝하는 방법. 주행 알고리즘, 조향, lookahead, 속도 프로파일, 정지·재출발, watchdog, 제어 노드 토픽 작업에 사용한다. 경로 추출은 lane-perception, 지도 기반 목표 주행은 nav2-driving 스킬을 쓴다."
---

# 주행 알고리즘

## 구조
```
lane/observation ─► ros_control.ControlNode ─► runtime.DriveCore.tick
   odom, scan, TF ─┘        │                     ├─ tracking.LaneTracker  (경로)
   lane/estop ─────────────┘                      ├─ control.command       (Pure Pursuit → Proposal)
                                                  ├─ crosswalk / obstacles (이벤트)
                                                  └─ behavior.Behavior + arbitrate (최종 속도)
lane/command (String) ─► ros_watchdog ─► lane/dry_run_cmd_vel (기본) | cmd_vel (명시적 해제 시)
```
로봇 쪽 `fleet_velocity_gate`가 `/cmd_vel_candidate`를 RUN permit이 있을 때만 `/cmd_vel`로 넘긴다 (`doc/control-pc-pipeline.md`).

## 알고리즘 요점
- **Pure Pursuit** (`control.command`): 경로에서 로봇과 가장 가까운 점부터 호 길이 `L = clamp(v_prev·lookahead_time, min_lookahead, max_lookahead)`만큼 따라간 목표점 (x, y)를 잡고 곡률 `κ = 2y/(x²+y²)`, `ω = v·κ`.
- **속도** = 다음의 최솟값: 요청 속도, `max_speed`, 보이는 경로 끝(`stop_margin` 뺀 거리)에서 `braking_decel`·`latency`로 멈출 수 있는 속도, `v_prev + max_accel·dt`, `max_omega/|κ|`, `sqrt(max_lateral_accel/|κ|)`. 최솟값이라 가속만 램프를 타고 감속은 즉시 적용된다.
- **행동 상태기계** (`behavior.Behavior`): `FOLLOW → APPROACH → WAIT → PASS → FOLLOW`. 측정 속도가 `stopped_speed` 이하일 때만 WAIT, `wait_s` 후 재출발, 같은 이벤트 ID는 PASS가 끝날 때까지 다시 잡지 않는다.
- **중재** (`behavior.arbitrate`): 비상정지 > 센서 실패 > 장애물 > 횡단보도 > 차선. 최종 속도는 모든 제한의 최소값.
- **watchdog**: 수신 측 monotonic 시각, 순서 번호 역행·형식 오류·타임아웃이면 0.

## 규칙과 이유
- 무효·오래됨·측정 불가 → `Proposal()` (0). 새 분기를 만들면 그 분기의 0 출력 테스트도 만든다.
- 경로는 **현재** 로봇 프레임으로 변환해서 넣는다. 캡처 시각 프레임을 그대로 쓰면 지연 동안 움직인 만큼 조향이 틀린다.
- 한 시계 도메인에서 age를 계산한다. PC와 로봇 시각 차이는 1초 미만이어야 한다 (`tools/run_control_pc.sh`가 검사).
- 파라미터는 `config/*.json`에서만. 코드 상수로 튜닝하지 않는다.
- config의 `sensors.infinity_is_clear: true`는 "스캔 무반사 = 비어 있음" 가정이다. 이를 넓히는 변경은 검토자에게 명시한다.

## 변경 절차
1. `tests/test_control.py`, `test_behavior.py`, `test_runtime.py`, `test_watchdog.py`, `test_obstacles.py`, `test_crosswalk.py` 중 해당 테스트를 먼저 추가.
2. ```bash
   ./.agents/tools/harness.sh fast
   ./.agents/tools/harness.sh ros-smoke              # ros_*.py 변경 시 (ROS_DOMAIN_ID=177, localhost)
   ./.agents/tools/harness.sh ros pinky_lane_driving # 패키지/엔트리포인트 변경 시
   ```
3. 실물 연결(`PINKY_HARDWARE=1 tools/run_control_pc.sh`)은 사용자만 실행한다. 에이전트는 dry-run 출력 확인 명령만 제안한다: `ros2 topic echo /lane/dry_run_cmd_vel --once`.

## 튜닝 가이드 (config `control`, `behavior`)
| 증상 | 먼저 볼 노브 |
| --- | --- |
| 직선에서 좌우로 흔들림 | `min_lookahead` ↑, `lookahead_time` ↑ |
| 곡선 안쪽으로 파고듦 | `max_lookahead` ↓ |
| 곡선에서 너무 느림 | `max_lateral_accel` ↑ (한 번에 0.01씩) |
| 횡단보도 정지선 넘어감 | `stop_distance` ↑, `decel` ↓, `latency` 실측 반영 |
| 재출발이 늦음 | `wait_s`, `clear_s` |
실제 로봇 지연(`latency`)은 `doc/robot19-latency.md` 실측값을 쓴다. 바꾼 값과 이유를 보고서에 표로 남긴다.
