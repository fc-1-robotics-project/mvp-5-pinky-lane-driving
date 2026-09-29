---
name: pinky-robotics
description: "Pinky Pro(ROS 2 Jazzy) 로봇 개발 작업을 차선 인식·주행 알고리즘·Nav2 전문 에이전트에 나눠 맡기고 안전 검토자로 검증하는 오케스트레이터. 차선 인식, 차선 주행, Pure Pursuit, 횡단보도 정지, 장애물 정지, watchdog, nav 주행, Nav2, SLAM, 지도, 목표점 이동, 캘리브레이션 관련 구현·수정·버그 수정·튜닝 요청에 사용한다. 다시 실행, 검토 지적 반영, 이전 결과 개선, '주행 쪽만 다시', '검토만 다시' 같은 후속 요청에도 사용한다. 단순 개념 질문(예: Pure Pursuit가 뭐야)이나 로봇 실물 조작 요청에는 사용하지 않는다."
---

# Pinky 로보틱스 오케스트레이터

## 실행 모드: 서브에이전트 위임 + 생성·검증 루프

개발 에이전트가 만들고(Producer), `robot-safety-reviewer`가 검증한다(Reviewer). 에이전트끼리 대화할 필요는 없고 파일로 넘기면 충분해서 서브에이전트 위임을 쓴다.

| 에이전트 | `subagent_type` | 맡는 영역 |
| --- | --- | --- |
| 차선 인식 | `lane-perception-dev` | 세그멘테이션 → 트레이스 → 지면 투영 → 경로 |
| 주행 알고리즘 | `drive-control-dev` | Pure Pursuit, 행동 상태기계, 중재, watchdog, ROS 제어 노드 |
| Nav2 주행 | `nav2-dev` | SLAM, AMCL, Nav2 파라미터·런치, 목표 주행, Gazebo |
| 안전 검토 | `robot-safety-reviewer` | 체크리스트 판정, 검증 명령 재실행 (읽기 전용) |

데이터 흐름: 카메라 → `lane-perception-dev` 영역(`lane/observation`) → `drive-control-dev` 영역(`lane/command` → watchdog) → 로봇 게이트. Nav2는 별도 경로(`cmd_vel_nav` → smoother → `cmd_vel`)다.

## 작업 절차

### 0단계: 기존 작업 확인
작업 디렉터리는 `.agents/output/harness/` (이미 gitignore됨).
- 없으면 처음 실행. 만든다.
- 있고 "지적 반영/다시" 요청이면 최신 `*_review.md`를 읽고 FAIL 항목의 담당 에이전트만 다시 부른다.
- 있고 새 작업이면 `.agents/output/harness-{YYYYmmdd-HHMM}/`으로 옮기고 새로 시작한다.

### 1단계: 분해와 배정 (메인이 직접)
1. 요청을 위 세 영역으로 나눈다. 해당 없는 영역의 에이전트는 부르지 않는다.
2. `git rev-parse HEAD`를 기준 커밋으로 `00_plan.md`에 기록한다 (영역별 작업, 담당, 완료 조건).
3. 영역 간 계약(`LanePath`, observation JSON, 토픽 이름)이 바뀌면 인식 → 제어 순서로 **순차** 실행한다. 계약이 그대로면 **병렬**로 실행한다.

### 2단계: 개발 (서브에이전트 위임)
한 메시지에서 필요한 에이전트를 동시에 호출한다 (순차 조건이면 하나씩). 프롬프트에 넣을 것:
- 작업 내용, 완료 조건, 관련 이슈 번호
- `00_plan.md` 경로, 보고서 경로 `.agents/output/harness/01_{agent}_report.md`
- "RED 테스트 먼저, 하드웨어 명령 금지, 커밋하지 말 것"

같은 파일을 두 에이전트가 동시에 고치게 배정하지 않는다. 겹치면 순차로 바꾼다.

### 3단계: 안전 검토 (서브에이전트 위임)
`robot-safety-reviewer`를 부르며 기준 커밋과 모든 `01_*_report.md` 경로를 넘긴다. 반환 메시지를 `02_robot-safety-reviewer_review.md`로 저장한다.

### 4단계: 루프
- `VERDICT: FAIL` → FAIL 항목 담당 에이전트만 지적 파일 경로와 함께 다시 부른다 → 3단계. 최대 **3회**. 그래도 FAIL이면 멈추고 사용자에게 지적 목록을 보인다.
- `VERDICT: BLOCKED` → 루프를 돌지 않는다. 필요한 환경/실물 시험을 사용자에게 묻는다.
- `VERDICT: PASS` → 5단계.

### 5단계: 마무리 (메인이 직접)
1. `./.agents/tools/harness.sh fast`를 직접 한 번 더 실행한다.
2. 커밋은 저장소 규칙대로 RED(`test:`)와 GREEN(`feat:`/`fix:`)을 나눈다. 사용자가 커밋을 맡겼을 때만 한다.
3. 보고: 변경 요약, 실행한 명령과 결과, UNVERIFIED 항목, **실물 미검증** 여부.

## 증거 수준 (보고서에 반드시 구분)
1. 단위 테스트 (`fast`) 2. 비전 (`vision`, `replay`) 3. ROS 그래프 (`ros-smoke`, `ros PACKAGE`) 4. 시뮬레이션 (Gazebo) 5. 실물 저속 시험.
낮은 수준의 통과를 높은 수준의 통과로 적지 않는다. 5는 사용자만 실행한다.

## 오류 처리
- 에이전트 실패 → 한 번 재시도. 다시 실패하면 그 영역 없이 진행하고 보고에 누락을 적는다.
- ROS/모델/영상 없음처럼 재시도해도 같은 실패는 재시도하지 않는다. UNVERIFIED로 두고 필요한 것을 사용자에게 알린다.
- 에이전트가 남기지 않은 판단을 메인이 추측해 채우지 않는다.
- 사용자의 미커밋 변경이 있는 파일을 건드려야 하면 먼저 묻는다.

## 테스트 시나리오
### 정상 흐름
"곡선에서 lookahead를 속도에 비례하게 바꿔줘" → `drive-control-dev` 단독 → `01_drive-control-dev_report.md` → 검토 PASS → `fast` 통과 보고.
### 계약 변경 흐름
"observation에 차선 신뢰도 필드 추가하고 제어에서 낮으면 감속" → 인식 → 제어 순차 → 검토.
### 오류 흐름
검토자가 "stale observation에서 0이 아닌 속도" FAIL → `drive-control-dev` 재호출 (지적 파일 전달) → 3회 안에 PASS 못 하면 사용자에게 보고.
### 차단 흐름
Nav2 파라미터 변경, Gazebo 없음 → 검토자 BLOCKED → 시뮬레이션 실행 여부를 사용자에게 질문.
