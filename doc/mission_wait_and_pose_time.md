# 임무 대기와 위치 시각 계약

정상 장애물 정지, 장애물 해제 후 `clear_hold`, 횡단보도 `APPROACH`/`WAIT`의
의도적인 정지는 FollowLane의 15초 `persistent_stop`에 포함하지 않는다.
명령과 진단의 사유가 일치하고 경로·센서·로컬 허가·관제 permit이 유효해야 한다.
장애물 대기에는 현재 `scan_hit=true`, 해제 대기와 횡단보도에는 `scan_hit=false`가 필요하다.
`lane/diagnostics.behavior_state`로 횡단보도 상태를 확인한다.

장애물이 사라져도 기존 `clear_s` 동안 안정적으로 비어 있어야 같은 임무에서 재출발한다.
설명 없는 제로 출력의 15초 감시, 차선 소실 15초 감시, 센서 고장 2초 감시,
이동 명령에도 odom 진행이 없는 6초 감시, 임무의 `max_duration_sec`는 유지한다.
센서·TF 오류, E-stop, permit/로컬 허가 소실, 속도 제한, watchdog도 유지한다.
횡단보도 정지 설정 자체는 바꾸지 않았다.

`fleet/pose.header.stamp`는 발행 시각이 아니라 `map → base_footprint` 조회 결과의
원본 TF 시각이다. 0·음수·미래·잘못된 나노초·`transform_stale_sec` 초과 TF는 발행하지 않는다.
따라서 같은 TF를 주기적으로 다시 발행해도 관제의 source-age 검사가 원래 나이를 본다.
PC와 로봇은 같은 ROS 시계 기준을 사용하고 시각을 동기화해야 한다.

공분산은 최신으로 수신한 AMCL 필터 측정값이다. 원본 시각의 유효성과 역행,
유한값 및 대각 분산의 비음수 조건을 검사한다. 정상 정지 중에는 AMCL이
`update_min_d`/`update_min_a`를 넘지 않아 새 측정을 발행하지 않을 수 있으므로,
TF 발행 주기마다 새 공분산을 요구하는 고정 수신 lease는 추가하지 않았다.
현재 메시지에는 별도 공분산 시각 필드가 없으므로 TF 시각이 공분산 재측정 시각을
보장하지 않는다. 두 원본 시각을 독립적으로 소비하려면 별도 메시지 계약이 필요하다.
이는 [Nav2 Jazzy AMCL 발행 조건](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/)을 따른다.

단위 테스트·격리 ROS 검사는 실제 정지 거리, 지연, 재출발 및 localization 정확도의
현장 증거를 대신하지 않는다. 실제 로봇과 시뮬레이션은 별도 검증이 필요하다.
