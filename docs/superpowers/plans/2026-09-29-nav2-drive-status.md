# Nav2 실물 구동 연결 — Codex 구현 초안

Claude 검토 전 초안. 소프트웨어 연결과 실제 주행 합격은 별개다.

## 설계 의도

Nav2 → Collision Monitor → `/cmd_vel_safe` → 단일 시리얼 모터·엔코더 노드 → Nucleo PI 제어로 연결한다.
센서/명령/TF가 끊기면 정지하며, 실측값을 채우기 전에는 구동 실행을 거부한다.
기존 작업 중인 엔코더 보정값·파서 검사·웹 UI 변경은 보존했다.

## 구현

- `src/cwu_base/cwu_base/motor_bridge.py`: 포트 독점, 설정 확인·세션, 20Hz 명령, `/odom`·TF, 입력 만료, 재연결 시 이전 명령 폐기.
- `src/cwu_base/cwu_base/motor_safety.py`: 엄격한 프레임 파싱·차동구동 변환·설정 및 입력 유효성 검사.
- `src/cwu_base/config/motor.yaml`: 보정 확인 기본 false, 미측정 값 -1. 기존 반지름·트레드·평균 틱은 임시 실측값으로 보존.
- `src/cwu_base/cwu_base/encoder_odom.py`: 기존 읽기 전용 노드에도 포트 독점 적용.
- `src/cwu_base/{setup.py,package.xml}`, `test/test_motor_safety.py`, `test/run_motor_check.py`, `test/run_drive_safety_check.py`: 설치·의존성·검사.
- `firmware/nucleo/src/changwon_robot_stm.ino`, `src/wheel_control.h`, `test/control_check.cpp`, `platformio.ini`, `README.md`: 20ms PI, 출력/적분/가속 제한, 최대 300ms 만료, 비차단 통신. 기존 핀·모터 반전 유지. 기존 원본은 `legacy/`에 보존.
- `src/cwu_nav/cwu_nav/drive_config.py`, `config/nav2.yaml`, `config/navigate*_safe.xml`: 실측 속도/반경 반영, 전방·후방·회전 외곽을 포함하는 정지 영역, 후진·회전 복구 제거. 정지·대기·재계획 사용.
- `src/cwu_nav/launch/{autonomy,nav2}.launch.py`: `drive:=false` 기본값, 켜면 엔코더 전용 노드 제외. 첫 실물 시험은 SLAM Toolbox·카메라/서보 비활성 고정.
- `src/cwu_nav/{setup.py,package.xml}`, `test/{test_drive_config,run_nav2_check}.py`: 설치·검사.
- `src/cwu_slam/cwu_slam/bringup.py`, `config/demo.yaml`: Nav2 데모도 `/cmd_vel_safe` 소비.
- README, 기존 후속 계획 및 이 문서: 실제 구현·검증 범위 기록.

## 통신 (115200 baud, 8N1, 줄바꿈 ASCII)

```text
CFG,session,max_ticks,kpL,kiL,kpR,kiR,pwm_limit,accel_ticks,timeout_ms,left_sign,right_sign
ACK,session
VEL,session,left_ticks_per_second,right_ticks_per_second
STOP
ENC,millis,raw_left_count,raw_right_count,session
READY,1
```

설정은 정지 상태에서만 받으며 정상 ACK 전에는 구동하지 않는다. 새 연결은 새 세션이다.
잘못된 MCU 명령·만료는 출력을 즉시 0으로 만들고 설정을 해제한다. STOP은 설정을 유지한다.
VEL의 두 속도가 0이면 감속 램프를 거치지 않고 정지한다. 단일 문자 f/b/l/r/1/2는 구동 명령이 아니다.
PWM 0은 실제 바퀴가 멈췄다는 뜻이 아니다. 제동거리와 정지 시간을 별도로 측정해야 한다.

Humble Collision Monitor는 누락/오래된 스캔을 건너뛰므로 모터 노드에 별도 입력 검사를 둔다.
[Humble scan.cpp](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_collision_monitor/src/scan.cpp).
정지 영역 반폭은 `robot_radius + braking_distance + safety_margin + safety_latency * max_wheel_speed`다.
반경은 그리퍼 최대 확장을 포함해 측정한다. 정지 영역이 통로를 막으면 값을 임의로 줄이지 말고
안전 속도를 낮춰 다시 제동거리를 측정한다.

## 개발 시험 실행

아래 명령은 온보드에서 실행한다. 경기 중 외부 목표 지정·로그 수집에 의존하지 않는다
(규정 2.1–2.3). 모든 보정은 경기장 공개 전에 확정한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# 먼저 motor.yaml과 mount.yaml을 실측으로 완성. 출고 기본값은 실행을 거부한다.
ros2 launch cwu_nav autonomy.launch.py drive:=true camera:=false follow:=false \
  encoder_port:=/dev/ttyACM0 port:=/dev/ttyUSB0
```

펌웨어 자동 업로드·실물 구동은 이번 작업에서 수행하지 않았다.
읽기 전용 encoder.launch.py는 기존 `ENC,left,right` 펌웨어용이다.
새 펌웨어에는 motor_bridge를 사용한다. 두 노드를 동시에 실행하지 않는다.

## 반드시 남은 실물 측정

| 항목 | 상태 / 합격 기준 |
|---|---|
| 보드에 업로드된 펌웨어 버전·해시 대조 | 미검증. 저장소 원본/새 빌드와 실제 보드는 구분 |
| 좌/우 각각 10회전 틱·부호 | 미검증. 현재 평균 3009.5는 재검증 필요 |
| LiDAR x/y/z·roll/pitch/yaw, 확장 외곽 | 미측정 |
| PI 이득·PWM·속도/가속 한계 | 미측정 |
| 명령 중단 → PWM 0 → 실제 정지 시간/거리 | 미측정. 전진·회전·케이블 분리·Pi 종료 포함 |
| 부하 회전 시 ST-Link 시리얼 멈춤 | 기존 주석만 존재, 재현 여부 미검증 |
| 바퀴 띄운 양방향 속도 추종 | 미검증 |
| 바닥 1m 직진·360도 회전 각 3회 | 미검증. 거리 ≤5%, 방향 ≤5도 |
| Nav2 목표·시작점 왕복 3회 | 미검증. 매회 무충돌·위치 오차 ≤0.15m |
| 고정 장애물·막힌 경로 | 실물 미검증. 접촉 없이 우회 또는 정지 |
| Pi 4 10분 주행·CPU/메모리/온도 | 미검증 |

각 실물 시험은 날짜·펌웨어 해시·YAML 사본·원시 시리얼·ROS 로그·측정 도구와 결과를 보관한다.
사람이 옆에서 전원을 차단할 수 있는 개발 환경에서만 시작한다.

## 열린 질문

실측 이득과 제동 한계, LiDAR 장착값, 최대 확장 외곽, 보드에 실제 올라간 버전,
부하 시 통신 중단 원인은 아직 확정되지 않았다. 가상 검증으로 이를 대신하지 않는다.

## 소프트웨어 검증 기록 (2026-09-29~30)

- `colcon build --symlink-install --executor sequential`: 6 패키지 성공.
- `colcon test --packages-select cwu_base cwu_nav cwu_slam`: 이번 실행 45개 pytest 통과.
  `colcon test-result --verbose` 집계 56개, 오류·실패 0 (다른 기존 결과 포함).
- `g++ -std=c++11 -Wall -Wextra -Werror -pedantic firmware/nucleo/test/control_check.cpp -o /tmp/cwu-control-check && /tmp/cwu-control-check`: 통과.
  파싱·PI/PWM/적분 제한·부호/반전·잘린 명령·300ms 만료·롤오버 확인.
- `pio run -d firmware/nucleo`: 성공. RAM 2,152 bytes, flash 42,024 bytes. 업로드하지 않음.
- `run_motor_check.py`: 가상 시리얼 CFG/ACK·왕복 통신·재부팅·기존 명령 폐기·정지 확인.
- `run_drive_safety_check.py`: 실제 DDS + Collision Monitor + PTY 포트의 통합 검사 통과.
  `/odom` 발행자 1개와 TF, scan/encoder/TF/명령 단절, 무효 scan, 장애물, 모니터 종료 시 STOP 확인.
  로그: `/tmp/cwu-drive-safety-821wytss`.
- `run_encoder_check.py`: 기존 읽기 전용 경로·카운터 롤오버 통과, 1m 입력에 odom 0.990m.
- `run_nav2_check.py`: Collision Monitor 경유 목표·복귀 통과(각 오차 약 0.10m),
  장애물 내부 목표 거부 status=6. 로그: `/tmp/cwu-nav2-check-nj_8aq7n`.
  종료 시 기존 Nav2 컴포지션 프로세스가 SIGKILL까지 필요했음. 주행 중 크래시는 없음.
- 기본 YAML로 `drive:=true camera:=false follow:=false` 실행 시 보정 오류로 사전 거부 확인.
- `run_demo_check.py --backend slam_toolbox`와 `--backend cartographer`: 스캔·움직이는 odom·TF·지도 생성 및 YAML/PGM 저장 회귀 검사 통과.
- `git diff --check`: 통과.

검사 과정에서 MCU가 만료 시각에 새 명령을 받으면 watchdog 호출 전에 타이머를 갱신할 수
있던 경계를 수정했다. 이제 매 수신 바이트에서 먼저 만료를 검사한다.
ROS 검사 스크립트의 부모·자식 DDS 설정 불일치도 수정했다.
ROS 로컬 UDP는 샌드박스 밖에서만 가능했고, 시스템 Python(`/usr/bin/python3`)으로 검증했다.

SSH 보드레이트 확인: Pi의 소스/설치본 `encoder.yaml`과 `/dev/ttyACM0` 포트 상태가
115200 baud, 8N1이다. 저장소 Nucleo의 `Serial.begin(115200)`과 일치한다.
실제 업로드된 바이너리 버전까지 확인한 결과는 아니다.
