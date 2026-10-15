# 실물 복귀 시험 도구 (PC에서 실행, Pi는 ssh `rpi`)

결과·녹화는 `$FIELD` (기본 `~/ros_CWU/log/field`)에 쌓입니다. 근거와 결과표는
`docs/2026-09-29-실물-주행-시험-기록.md` "Nav2 출발점 복귀 시험" 절.

| 파일 | 하는 일 |
|---|---|
| `restart_stack.sh` | 도구를 Pi로 복사 → SLAM(drive, cmd_vel_safe) + Nav2 재시작. **로봇 현재 자세가 출발점** |
| `trial.sh N [초]` | 출발 스캔 → 녹화 → 자동주행 → `go_home` → 도착 스캔 → 실제 오차·odom 보정·SLAM 튐 분석 |
| `loop_trial.sh N CX CY R ccw\|cw` | 위와 같되 장애물(중심 CX,CY, map 좌표) 반경 R로 한 바퀴 돌고 복귀 |
| `find_obstacle.py [방향°]` | (Pi) 로봇 기준 해당 방향 ±45°, 2 m 안 물체 묶음의 위치·크기 |
| `save_scan.py OUT` | (Pi) 스캔 5장 중앙값 + SLAM 자세 저장 |
| `home_err.py START END` | 두 스캔 정합 → 실제 위치 차 (SLAM 자세를 초기값으로) |
| `timeline.py BAG PAIRS` | map→odom 튐, 구간별 SLAM/odom 오차 |
| `final_pose.py BAG x y deg` | 녹화(또는 재생 결과)의 최종 SLAM 자세 vs 실제 |
| `rviz.sh` | (PC, 시험용) RViz(지도·비용지도·경로·웹캠) + 웹캠 디코더. RViz를 닫으면 디코더도 종료 |
