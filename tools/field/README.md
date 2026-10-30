# 실물 시험 도구 (PC에서 실행, Pi는 ssh `rpi`)

근거와 결과표는 `docs/2026-09-29-실물-주행-시험-기록.md`. 미션 로그는 Pi `~/mission1.log`,
보드 텔레메트리는 Pi `~/telemetry_날짜_시각.csv`(`start_mission1.sh`가 `tools/telemetry_csv.py`를 같이 띄움). 2026-10-29 펌웨어 전환으로 Nav2·`go_home` 전제 도구
(`restart_stack.sh`, `trial.sh`, `loop_trial.sh`)는 지웠습니다(커밋 84e5663까지에 있음).

| 파일 | 하는 일 |
|---|---|
| `deploy.sh` | (PC, 시험 전) 지금 코드(커밋 안 한 것 포함)를 Pi `~/ros_CWU`로 보내고 Pi에서 빌드. 덮어쓰기 전 Pi `~/ros_CWU_backup_날짜_시각.tgz` |
| `start_mission1.sh` / `stop_mission1.sh` | 미션 1 시작(경기 명령) / 시험용 정지. 시작 때 Pi 코드가 PC와 다르면 경고 |
| `logs.sh [글자]` | (PC, 시험용) Pi의 `~/mission1.log` 실시간 로그. 글자를 주면 그 줄만 (`logs.sh 격자` = 위치 보정, `logs.sh →` = 상태 전이). Ctrl+C로 보기만 종료 |
| `plot_record.py 기록.npz` | (PC, 시험 뒤) 미션이 끝나거나 멈출 때 Pi `~/mission1_record_날짜_시각.npz`에 저장된 지도·경로를 그림으로. `scp rpi:~/mission1_record_*.npz .` 로 가져와 실행 |
| `find_obstacle.py [방향°]` | (Pi) 로봇 기준 해당 방향 ±45°, 2 m 안 물체 묶음의 위치·크기 |
| `save_scan.py OUT` | (Pi) 스캔 5장 중앙값 + SLAM 자세 저장 |
| `home_err.py START END` | 두 스캔 정합 → 실제 위치 차 (SLAM 자세를 초기값으로) |
| `timeline.py BAG PAIRS` | map→odom 튐, 구간별 SLAM/odom 오차 |
| `final_pose.py BAG x y deg` | 녹화(또는 재생 결과)의 최종 SLAM 자세 vs 실제 |
| `rviz.sh` | (PC, 시험용) RViz(지도·웹캠) + 웹캠 디코더(`tools/webcam_view.py`). RViz를 닫으면 디코더도 종료 |
| `webcam_fx.py` | (Pi) 웹캠으로 빨간 큐브 검출 크기·위치 확인 (`cwu_perception.detect`) — 초점거리 fx 맞출 때 |
| `50-webcam-no-autosuspend.rules` | (Pi, sudo로 `/etc/udev/rules.d/`에 설치) 웹캠 USB 절전 끄기 — 깨어나며 -110으로 멈추던 문제 |

## 실물 시험 순서 (2026-10-31, 미션이 SLAM 대신 odom + 격자 보정 + 자체 지도를 쓴 뒤)

1. `tools/field/deploy.sh` → "배포 완료" 확인
2. 로봇을 출발 칸 **한가운데**, 칸 축에 맞춰 경기장이 왼쪽에 오게 놓음 (격자 보정은 출발 자세를 칸 중심·축으로 삼고, 반 칸 20 cm·4° 안의 어긋남만 바로잡음)
3. `tools/field/start_mission1.sh` → `tools/field/logs.sh 격자`로 위치 보정 값 확인: `격자 보정: x, y cm, 각도°`가 ±15 cm·±4° 안에 머물러야 함. 계속 커지면 바퀴 odom 보정값(`encoder.yaml`) 문제
4. `tools/field/rviz.sh`: "Map"은 SLAM 지도(보기용), "Mission map"은 미션이 실제로 판단에 쓰는 지도(`/mission_map`, 3 Hz). 둘이 어긋나면 SLAM 쪽이 틀어진 것
