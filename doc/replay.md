# 영상 회귀 검증

기존 fast/ros 하네스에 `replay`를 추가했다. 로컬 YOLO segmentation 모델을
주행 영상의 지정 구간에 적용하고 MP4와 프레임 로그를 저장한다. ROS 노드나
모터 명령을 실행하지 않는다. 신뢰하는 로컬 가중치 파일만 사용한다.

## 실행

`.agents/replay.example.json`을 참고해 Git에서 제외되는 `.agents/replay.local.json`에 모델과
영상 경로를 설정한다. 상대 경로는 설정 파일의 디렉터리 기준이다.
OpenCV, PyTorch, Ultralytics가 설치된 Python을 지정한다.

```bash
./.agents/tools/harness.sh fast
REPLAY_PYTHON=/home/seunghoon/dev_ws/ros/.venv_yolo/bin/python ./.agents/tools/harness.sh replay
# 특정 구간만 확인
REPLAY_PYTHON=/home/seunghoon/dev_ws/ros/.venv_yolo/bin/python ./.agents/tools/harness.sh replay --case first_crosswalk
```

현재 로컬 설정은 best (2).pt와 두 upright 주행 영상의 원본 경로를 참조한다.
기본 구간은 첫 영상 133–136초, 169–172초와 두 번째 영상 109–112초다.
기존 검토에서 선택한 급회전/횡단보도/여러 차선 구간이며 최종 대표성은
출력 영상을 보고 확인한다. 전체 구간의 모든 프레임 검증이 필요하면
sample_fps를 원본 FPS 이상으로 지정한다. 기본값 5fps는 빠른 육안 검수용이다.

## 결과

실행마다 `.agents/output/replay/run-*`에 새 결과를 생성하여 이전 실행을 보존한다.

- `*.mp4`: segmentation 및 원본 영상 시각 overlay. 일반 영상 플레이어로 재생/탐색.
- `*.jsonl`: 원본 프레임 번호, 시각, 클래스, confidence, 추론 소요 시간.
- `report.json`: 모델/영상 SHA-256, 라이브러리 버전, 실행 설정, 검출 수, 결과 상태.

`execution_passed`는 지정 프레임 추론과 결과 영상 디코딩의 성공이다.
정답 라벨 기반 정확도, 차로 선택, 경로 추종 또는 정지 동작의 성공은 아니다.
검출 0개도 유효한 관찰 결과로 남기며 `visual_review`는 pending이다.
모델 task와 클래스가 0=crosswalk, 1=left line, 2=right line인지 검사한다.

육안으로 잘못된 차선/마스크 누락/횡단보도 오류가 있는 원본 시각을 기록한다.
기존 학습 영상은 회귀 검증에 사용하고 일반화 성능은 새로운 주행에서 평가한다.
이 단계에는 경로 생성, 가상 속도 명령, LiDAR 및 폐루프 주행 검증이 없다.

## 설정 검증 기록

2026-09-16: fast 테스트 6개 통과, shell 구문 검사 통과.
OpenCV 5.0.0 / Ultralytics 8.4.152 / PyTorch 2.14.0+cu130 환경에서 CPU로
기본 세 구간을 실행했다. 각 15프레임, 총 45프레임의 추론과 출력 MP4의
전체 프레임 디코딩을 확인했다. 인식 품질의 육안 검수는 별도 진행한다.
