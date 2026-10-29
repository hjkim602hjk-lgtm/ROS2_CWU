# 실물 시험 도구 (PC에서 실행, Pi는 ssh `rpi`)

결과·녹화는 `$FIELD` (기본 `~/ros_CWU/log/field`)에 쌓입니다. 근거와 결과표는
`docs/2026-09-29-실물-주행-시험-기록.md`. 2026-10-29 펌웨어 전환으로 Nav2·`go_home` 전제 도구
(`restart_stack.sh`, `trial.sh`, `loop_trial.sh`)는 지웠습니다(커밋 84e5663까지에 있음).

| 파일 | 하는 일 |
|---|---|
| `start_mission1.sh` / `stop_mission1.sh` | 미션 1 시작(경기 명령) / 시험용 정지 |
| `find_obstacle.py [방향°]` | (Pi) 로봇 기준 해당 방향 ±45°, 2 m 안 물체 묶음의 위치·크기 |
| `save_scan.py OUT` | (Pi) 스캔 5장 중앙값 + SLAM 자세 저장 |
| `home_err.py START END` | 두 스캔 정합 → 실제 위치 차 (SLAM 자세를 초기값으로) |
| `timeline.py BAG PAIRS` | map→odom 튐, 구간별 SLAM/odom 오차 |
| `final_pose.py BAG x y deg` | 녹화(또는 재생 결과)의 최종 SLAM 자세 vs 실제 |
| `rviz.sh` | (PC, 시험용) RViz(지도·웹캠) + 웹캠 디코더. RViz를 닫으면 디코더도 종료 |
