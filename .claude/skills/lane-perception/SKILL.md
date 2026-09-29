---
name: lane-perception
description: "Pinky 차선 인식 파이프라인(YOLO 세그멘테이션 → 마스크 트레이스 → 왜곡 보정·호모그래피 지면 투영 → 좌/우 경계 선택 → 중심 경로 → 시간 추적)을 구현·수정·검증하는 방법. 차선 검출, 경로 추출, 캘리브레이션, 호모그래피, 한쪽 차선만 보일 때, replay 검증 작업에 사용한다. 속도·조향 제어 자체는 drive-control 스킬, 지도 기반 주행은 nav2-driving 스킬을 쓴다."
---

# 차선 인식

## 파이프라인과 파일
| 단계 | 파일 | 출력 |
| --- | --- | --- |
| JPEG 디코드, 모델 결과 → observation | `vision.py`, `ros_perception.py` | observation dict (`lane/observation`, String JSON) |
| 계약 검증 | `perception.observe` | 픽셀 폴리곤 (u 오른쪽, v 아래), `metric_valid=False` |
| 마스크 → 중심선 트레이스 | `tracing.py` | 차선별 픽셀 폴리라인 |
| 왜곡 보정 → 호모그래피 | `calibration.py`, `perception.project_ground` | `base_footprint` x/y [m] |
| 경계 짝짓기 → 중심 경로 | `path.select_path`, `paired_path` | `LanePath(points, valid, degraded, speed_limit, reason)` |
| 프레임 간 연속성 | `tracking.LaneTracker` | 오도메트리로 이전 경로 변환 후 이어붙임 |

클래스 ID: `0` 횡단보도, `1` 왼쪽 선, `2` 오른쪽 선. 우측 통행 2차선, 차선 폭은 config `path.width` (robot19: 0.15 m).

## 규칙과 이유
- **투영 전에 반드시 왜곡 보정.** 호모그래피는 보정된 픽셀 기준으로 측정됐다. 보정 없이 넣으면 가장자리에서 수 cm 오차가 곡률로 번진다.
- **캘리브레이션은 해상도·카메라 프레임·`mounting_id`가 일치할 때만 쓴다.** 카메라를 다시 달면 새 측정이 필요하다 (`doc/calibration.md`, `tools/fit_ground_homography.py`). 불일치면 `metric_valid=False`.
- **한쪽 경계만 보이면** 폭만큼 법선 오프셋한 경로를 `degraded=True`, `speed_limit=fallback_speed`로 내고, 마지막 양쪽 관측 후 `fallback_timeout`이 지나면 무효. 한쪽 관측으로 타이머를 갱신하지 않는다 — 갱신하면 잘못된 오프셋으로 무한히 달린다.
- **모호하면 버린다.** 폭 허용 범위(`width_tolerance`)를 벗어나거나 후보 차이가 `ambiguity_margin` 미만이면 invalid. 반대 차선 선을 잡는 것이 멈추는 것보다 나쁘다.
- **캡처 시각은 추론 완료 시각이 아니다.** observation의 `capture_time_s`를 바꾸지 않는다.

## 변경 절차
1. `tests/test_perception.py`, `test_path.py`, `test_tracking.py`, `test_calibration.py` 중 해당 파일에 실패하는 테스트를 먼저 추가한다. 합성 기하(직선, 120° 굽은 길, S자)를 코드로 만들어 쓴다.
2. 구현 후:
   ```bash
   ./.agents/tools/harness.sh fast          # 항상
   ./.agents/tools/harness.sh vision        # tracing/vision 변경 시 (OpenCV 필요)
   ./.agents/tools/harness.sh ros-smoke     # ros_perception/ros_camera 변경 시
   ```
3. 실제 영상 확인이 필요하면 `.agents/replay.local.json` 준비 후 `REPLAY_PYTHON=/home/seunghoon/dev_ws/ros/.venv_yolo/bin/python ./.agents/tools/harness.sh replay --config .agents/replay.local.json`. 출력은 `.agents/output/replay/run-*/` — 오버레이 몇 장을 직접 열어 확인하고 본 장수를 보고서에 적는다.

## 튜닝 노브 (config `path`)
`width`, `width_tolerance`, `max_near`, `sample_step`, `fallback_timeout`, `fallback_speed`, `ambiguity_margin`, `min_coverage`, `curve_extension`. 값을 바꾸면 이유와 이전 값을 보고서에 적는다. 조명·바닥이 바뀌면 다시 맞춰야 하는 값이다.
