# Pinky 안전·정합성 체크리스트

`robot-safety-reviewer`가 항목별로 PASS / FAIL / UNVERIFIED를 판정한다. 변경과 무관한 항목은 `N/A`.
S로 시작하는 항목은 안전 항목이다. UNVERIFIED가 하나라도 있으면 전체 판정은 BLOCKED다.

## 안전 (S)
| ID | 확인 | 보는 법 |
| --- | --- | --- |
| S1 | 무효·오래된·측정 불가 입력에서 속도 출력이 0 | 해당 분기 테스트 존재 + 통과 |
| S2 | 우선순위 비상정지 > 센서 실패 > 장애물 > 횡단보도 > 차선 유지 | `behavior.arbitrate`, `runtime.tick` diff |
| S3 | 기본 출력이 `lane/dry_run_cmd_vel` (실제 `cmd_vel`은 명시적 파라미터로만) | `ros_watchdog.py` 기본값 |
| S4 | watchdog 타임아웃이 수신 측 monotonic 시각 기준, 발행자 침묵 시 정지 | `watchdog.py`, `ros-smoke` |
| S5 | 속도·각속도·가속·횡가속 한계가 config에서 오고 범위 검증됨 | `control.Limits`, config 로더 |
| S6 | Nav2와 차선 주행이 같은 `cmd_vel`을 동시에 발행하는 경로 없음 | 런치 remap, 토픽 이름 |
| S7 | 하드웨어 bringup·모터 발행·SSH 배포를 에이전트가 실행하지 않음 | 보고서의 실행 명령 목록 |
| S8 | 스캔 커버리지 미검증/no-hit을 "비어 있음"으로 취급하지 않음 | `obstacles.py`, `infinity_is_clear` 사용처 |

## 정합성 (C)
| ID | 확인 | 보는 법 |
| --- | --- | --- |
| C1 | 픽셀·미터 좌표 분리, 단위 m/rad/s, 프레임 x 전방·y 좌측 | diff의 변수명·변환 |
| C2 | 캡처 타임스탬프 보존, 신선도 판단에 사용 | observation → control 경로 |
| C3 | 캘리브레이션 없거나 불일치 시 `metric_valid=False` | `calibration.py`, 테스트 |
| C4 | 계약(`LanePath`, observation JSON, 토픽) 변경 시 소비자 측 테스트도 갱신 | 양쪽 테스트 diff |
| C5 | Nav2: `use_sim_time` 일관성, footprint가 실제 로봇과 맞음 | params/launch diff |
| C6 | QoS 의미 유지 (센서는 sensor QoS, 명령은 depth 1) | `create_subscription` diff |

## 증거 (E)
| ID | 확인 |
| --- | --- |
| E1 | 새 동작마다 실패하는 테스트가 먼저 있음 (RED → GREEN) |
| E2 | `./.agents/tools/harness.sh fast` 검토자가 직접 재실행해 통과 |
| E3 | 영향 패키지 `harness.sh ros PKG` 또는 `ros-smoke` 결과 (못 돌리면 UNVERIFIED) |
| E4 | 보고서가 증거 수준(단위/비전/ROS/시뮬/실물)을 구분하고 미검증을 밝힘 |
| E5 | 모델·영상·데이터셋·비밀정보·빌드 산출물이 diff에 없음 |
