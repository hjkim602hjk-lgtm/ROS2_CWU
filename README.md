---
title: ROS2 창원대학교 대회 로봇
aliases:
  - 실행 가이드
tags:
  - cwu/guide
---

# ROS2 창원대학교 대회 로봇

**GitHub 저장소**: [hjkim602hjk-lgtm/ROS2_CWU](https://github.com/hjkim602hjk-lgtm/ROS2_CWU)

## 프로젝트 개요

노션 링크: https://app.notion.com/invite/972d858f72a0d7a3db161013a118c949d5c30e4d

**하드웨어**: Nucleo F446RE / 라즈베리파이 4 / 모터 드라이버 Cytron
**센서**: YDLIDAR G4, Intel RealSense D415, PSD 거리 센서 4개(정면 그리퍼 1개 + 좌·우·뒤 각 1개)
**라즈베리파이**: Ubuntu Server 22.04, `ssh ubuntu@raspberrypi.local` (비밀번호는 저장소에 두지 않습니다)
**개발 환경**: VSCode, 아두이노 측은 PlatformIO 확장 사용 예정

### 라즈베리파이 접속 환경 — 작업 전 반드시 확인

- **파이는 Ubuntu Server 22.04를 설치한 헤드리스 환경이며, 연결할 모니터가 없습니다. 관리·실행은 개발 PC에서 SSH로만 합니다.**
- 모니터·키보드를 연결하거나 파이의 로컬 터미널에서 명령을 실행하라고 안내하지 않습니다. 이 환경은 사용자에게 매번 다시 묻지 않고 작업 전제로 유지합니다.
- 개발 PC의 기존 SSH 별칭은 `rpi`이며, PC 터미널에서 `ssh rpi`로 접속합니다. 마지막 접속 대상은 `ubuntu@192.168.0.75`였으나 IP는 바뀔 수 있으므로 고정 주소로 단정하지 않습니다. `raspberrypi.local`도 이름 해석이 되는 환경에서만 사용할 수 있습니다.
- USB-C는 현재 전원 연결에 사용합니다. SSH 접근에는 별도의 Wi-Fi 또는 유선랜 연결이 필요합니다.
- 접속 장애가 나면 먼저 PC에서 기존 SSH 설정·네트워크·대상 주소·22번 포트 응답을 확인합니다. `Connection refused`만으로 그 주소가 파이인지, SSH 서비스가 왜 거부되는지 확정하지 않습니다.
- **SSH가 끊긴 상태에서 `sudo systemctl start ssh`나 `hostname -I`를 파이에서 실행하라고 요청하지 않습니다.** 원격에서 확인 가능한 항목을 먼저 점검하고, 더 진행할 수 없을 때만 필요한 물리 조치나 정보를 구체적으로 안내합니다.
- 이후 실행 안내는 **PC에서 실행할 명령**과 **SSH 접속 후 파이에서 실행할 명령**을 구분합니다. 비밀번호는 문서나 저장소에 기록하지 않습니다.
- SSH는 개발·점검과 허용된 경기 시작 명령에만 사용하며, 미션 시작 후에는 외부 통신에 의존하지 않습니다.

**목표 1**: 미로 속 목표물을 인식·접근하고, 그리퍼의 PSD가 설정 거리 이내 물체를 감지하면 자동으로 집은 후 시작점으로 복귀
**목표 2**: 경기장 중앙의 목표물에 접근해 같은 PSD 기반 자동 파지로 집고 뺏기지 않게 유지

### PSD 기반 자동 파지 (2026-09-16 반영)

확정한 동작은 **그리퍼에 PSD 설치 → 설정 거리 이내 물체 감지 → 자동 파지**입니다.
통합 설계 초안은 D415 컬러 영상으로 빨간 목표물을 확인·정렬한 뒤 파지를 활성화하고,
PSD의 유효한 근접 감지가 지속되면 주행을 정지한 다음 그리퍼를 닫는 흐름입니다.
PSD 거리 감지만으로 목표물 식별이나 파지 성공을 판정하지 않습니다.

PSD 모델·연결 방식·감지 거리·그리퍼 구동기 사양은 아직 미확정입니다.
감지 거리와 보정값은 YAML에 두고 경기장 공개 전에 확정합니다.
**PSD 입력 및 그리퍼 제어 코드는 아직 미구현**이며, 기존 SLAM launch로 그리퍼가 움직이지 않습니다.
상태 전이·입출력·실물 확인 항목은 [[docs/superpowers/specs/2026-09-16-psd-gripper-design|PSD 자동 파지 설계 초안]]을 따릅니다.

### PSD 배치 (2026-09-30 확정)

| 위치 | 개수 | 역할 |
|---|---|---|
| 정면 그리퍼 | 1 | 자동 파지 시작 조건. **파지를 시작할 수 있는 PSD는 이것 하나뿐** |
| 좌·우·뒤 | 각 1 (총 3) | 주변 근접 감지. 구체적인 용도·임계값·ROS 인터페이스는 미정 |

PSD 모델과 출력 방식은 아직 미확정입니다. 아날로그 출력이면 Nucleo ADC(A2~A5가 비어 있음)에 연결하는 안을 검토하며,
출력 전압이 Nucleo 입력 허용 범위(3.3 V) 안인지 먼저 확인합니다.

### Nucleo ↔ Pi 통신: USB → UART 전환 (2026-09-30)

ST-Link USB 시리얼이 모터 잡음으로 반복해서 멈춰([[docs/2026-09-29-실물-주행-시험-기록|실물 주행 시험 기록]]),
Nucleo 하드웨어 UART를 Pi GPIO UART에 직결합니다. 펌웨어(`platformio.ini`의 `SERIAL_UART_INSTANCE=4`)와 ROS 기본 포트(`/dev/ttyAMA0`)는 바꿨습니다.
**새 펌웨어 업로드·Pi UART 설정·실물 통신 확인은 아직 안 했습니다.** 새 펌웨어를 올리면 ST-Link USB(`/dev/ttyACM0`)로는 더 이상 데이터가 나오지 않습니다.

| Nucleo-F446RE | | Raspberry Pi 4 |
|---|---|---|
| PC10 (CN7 1번, UART4 TX) | → | GPIO15 / RXD (10번 핀) |
| PC11 (CN7 2번, UART4 RX) | ← | GPIO14 / TXD (8번 핀) |
| GND | — | GND (6번 핀) |

- 둘 다 3.3 V 로직이라 레벨 변환기가 필요 없습니다. 전원선(3.3 V/5 V)은 연결하지 않습니다.
- D3~D10은 모터·엔코더가 쓰므로 USART1(D8/D2)·USART6(PC7=D9)은 쓸 수 없습니다.
- 펌웨어: 빌드 플래그로 `Serial`을 UART4(RX=PC11, TX=PC10)에 연결하므로 코드는 그대로입니다. 업로드는 계속 USB로 합니다.
- Pi: `/boot/firmware/config.txt`에 `enable_uart=1`, `dtoverlay=disable-bt` 추가, `cmdline.txt`에서 `console=serial0,115200` 삭제,
  `sudo systemctl disable hciuart`(있을 때) 후 재부팅 → `/dev/ttyAMA0`. 사용자가 `dialout` 그룹이어야 합니다.

## 미션 1·2 작동 방식

**정리일: 2026-09-16 / 상태: codex 설계 초안, Claude 검토·실물 검증 전.**
아래는 구현할 목표 동작입니다. 현재 SLAM·엔코더 처리, Nav2 주행 설정, D415 목표물 검출까지
연결돼 있습니다. 모터 연결·안전 정지·MCU PI 제어는 구현 초안이며 실물 검증 전입니다.
기본 구동은 꺼져 있고 실측 보정 전에는 실행을 거부합니다. PSD·그리퍼·미션 상태머신은 미구현입니다.

### 공통 센서 역할과 파지 순서

센서를 하나씩 켰다 끄는 순서가 아니라, **센서 데이터를 계속 받아 현재 단계에 필요한 판단에 사용**합니다.
G4와 엔코더는 탐색·접근·운반 중에도 계속 위치 추정과 주변 감시에 사용합니다.
PSD 자동 파지는 목표물을 확인하고 정렬한 접근 단계에서만 활성화합니다.

| 구성 | 역할 |
|---|---|
| G4 LiDAR | 벽·장애물·주변 물체까지의 거리를 측정하여 지도 작성과 충돌 회피에 사용 |
| 좌우 바퀴 엔코더 | 바퀴 회전량으로 이동 거리·회전을 추정하여 `/odom`과 TF 제공. 바퀴 미끄러짐은 이것만으로 정확히 알 수 없음 |
| SLAM — 센서 데이터를 처리하는 소프트웨어 | G4와 엔코더 기반 TF를 이용해 지도와 로봇 위치 추정 |
| D415 카메라 | **컬러 영상만** 사용. 빨간 목표물이 어느 쪽에 있는지(방향)를 판단. 거리는 측정하지 않음 |
| 그리퍼 PSD | 집는 위치로 들어온 물체의 근접 거리 감지. 목표물 식별이나 확보 성공 판정과는 별개 |
| 그리퍼 구동기·피드백 | 실제로 열고 닫아 목표물을 유지. 완료·유지·놓침 판정에 쓸 피드백은 하드웨어 확인 후 결정 |
| 라즈베리파이·Nucleo | Pi가 인식·경로·미션 상태를 판단하고 Nucleo가 구동을 담당하는 통합 초안. 실제 통신 형식은 확정 필요 |

**양 미션이 공유하는 파지 흐름:**

```text
D415로 목표물 확인·정렬 → 그리퍼 개방 → 자동 파지 활성화·저속 접근
→ PSD의 유효한 근접 감지가 설정 시간 동안 지속
→ 주행 정지 요청 → 실제 정지와 목표물·거리 조건 재확인
→ 그리퍼 닫기 → 동작 완료·실제 유지 확인
```

PSD가 벽이나 상대 로봇을 감지했다는 이유만으로 탐색 중 그리퍼를 닫지 않습니다.
감지 거리·확인 시간·접근 속도 등은 실측 후 YAML에 저장하며, 경기장 공개 전에 보정을 끝냅니다.

### 미션 1 — 목표물을 찾아 시작점으로 운반 (10분)

**목표:** 장애물이 있는 3.6 m × 3.6 m 경기장에서 빨간 50 mm 큐브를 확보해 자신의 시작점으로 운반합니다.
목표물 초기 위치는 `(5,5)`이지만 이동될 수 있으므로 카메라로 실제 위치를 확인합니다.

| 순서 | 로봇 동작 | 사용하는 센서·판단 |
|---|---|---|
| 1. 초기화·시작점 기록 | 센서·위치 추정이 준비되면 출발 전 자신의 시작 위치를 복귀 기준으로 기록 | G4·엔코더·SLAM. 부팅 위치가 곧 경기 시작점이라고 가정하지 않음 |
| 2. 탐색·중앙 접근 | 새 지도를 만들면서 장애물을 피해 탐색. 경기장 좌표계를 추정한 뒤 중앙을 우선 탐색 | G4로 벽·장애물 관측, 엔코더·SLAM으로 위치 추정. 사전 장애물 지도 사용 안 함 |
| 3. 목표물 발견·정렬 | 빨간 큐브가 화면 세로 중심선에 오도록 돌고, 정렬된 뒤 저속 전진 | D415 컬러 영상의 좌우 오차로 회전·전진, G4로 주변 충돌 감시 |
| 4. 근접 감지·파지 | 공통 파지 흐름에 따라 PSD 조건 확인 → 정지 확인 → 닫기 → 유지 확인 | D415·PSD·주행 상태·그리퍼 피드백. 유지 확인 후에만 복귀 시작 |
| 5. 시작점 복귀 | 목표물을 잡은 채 온라인 지도에서 경로를 구해 장애물과 상대 로봇을 피해 이동 | G4·엔코더·SLAM으로 주행, 그리퍼 피드백으로 유지 확인 |
| 6. 도착·종료 | 목표물을 시작점까지 운반했는지 확인하고 정지. 기존 팀 설계의 놓기 단계에서는 파지를 비활성화한 뒤 개방 | 위치 추정·유지 상태 확인. 개방은 팀 설계이며 규정상 목표는 시작점까지 운반 |

**예외 처리:** 목표물이 없거나 접근 중 놓치면 정지 후 재탐색합니다. 파지 실패는 제한된 재시도 또는
재탐색으로 처리하고, 운반 중 놓침이 확인되면 정지 후 재확보합니다.
시작점 기록과 복귀 정확도는 SLAM 지도 보정·바퀴 미끄러짐을 포함한 실물 왕복 시험으로 확인해야 합니다.

### 미션 2 — 중앙 목표물을 확보하고 종료까지 유지 (5분)

상세 센서 역할·상태 전이·안전 조건은 [[docs/superpowers/specs/2026-09-16-mission2-control-design|미션2 제어 설계 초안]]에 정리했습니다.

**목표:** 내부 장애물이 없는 경기장에서 네 로봇이 경쟁하며, 경기 종료 시 목표물을 확보하고 있어야 합니다.
미션 1과 같은 센서·그리퍼를 사용하며, 확보 후에는 유지와 모서리 탈락 회피를 우선합니다.

| 순서 | 로봇 동작 | 사용하는 센서·판단 |
|---|---|---|
| 1. 초기화·경기장 파악 | 센서 상태를 확인하고 벽 관측으로 중앙·모서리 위치를 추정 | G4·엔코더·위치 추정. SLAM 원점만으로 중앙·코너를 알 수는 없음 |
| 2. 중앙 접근·목표물 탐색 | 주변 로봇을 피하면서 중앙 쪽을 탐색하고 빨간 목표물을 확인 | G4로 주변 물체 거리, D415 컬러로 목표물 색상·방향 확인 |
| 3. 정렬·근접 감지·파지 | 미션 1과 같은 공통 파지 흐름 실행 | D415 정렬 → PSD 근접 감지 → 실제 정지 확인 → 그리퍼 닫기·유지 확인 |
| 4. 확보 유지·탈락 회피 | 목표물을 유지하면서 상대 접근에 대응하고 모서리 탈락 영역으로 들어가지 않도록 이동 | G4·위치 추정으로 주변·코너 감시, 그리퍼 피드백으로 유지 확인 |
| 5. 놓침·재확보 | 놓침이 확인되면 정지 후 목표물을 다시 찾아 접근·파지 | D415로 재탐색하고 PSD 파지 조건을 다시 활성화. 목표물이 항상 중앙에 있다고 가정하지 않음 |
| 6. 경기 종료 | 종료 시점까지 확보 상태 유지 | 실물 확보 기준은 로봇을 수직으로 들었을 때 목표물도 함께 들리는 것(규정 19항) |

G4의 거리값만으로 상대 로봇의 정체를 확정하지 않으며, 벽·지도와 비교한 주변 물체 추적이 필요합니다.
밀리거나 미끄러지는 상황은 엔코더만으로 판단하지 않고 LiDAR 기반 위치 변화와 함께 확인해야 합니다.
이 대응 로직과 코너 회피는 아직 구현·검증 전입니다.

### 공통 예외 처리와 경기 실행 조건

- PSD 입력이 무효·오래됨·끊김 상태이거나 정지 확인에 실패하면 접근을 정지하고 새 파지를 금지합니다.
- 이미 확보한 상태에서는 PSD 오류만으로 그리퍼를 열지 않습니다. 실제 놓침 확인과 센서 오류를 구분합니다.
- 놓기 전에는 자동 파지를 비활성화하여 센서 앞에 남아 있는 목표물을 즉시 다시 집지 않도록 합니다.
- 모든 인식·주행·파지 판단은 온보드에서 수행합니다. 허용된 실행 명령 이후 외부 통신을 차단하며, 미션 중 SSH·웹 UI·외부 시각 동기화에 의존하지 않습니다.
- 두 미션의 하드웨어는 동일하게 유지하고, 경기장 공개 후 코드·설정·지도·보정값을 참가자가 수정하지 않습니다.

상세 구현 순서는 [[docs/superpowers/plans/2026-09-10-competition-plan|후속 개발 계획]],
경기 조건은 [[docs/competition_rules|대회 규정 정리]]를 참고합니다.
**열린 항목:** PSD 모델·배선·감지 거리, 그리퍼 구동기·유지 피드백, Nucleo 통신 형식 및 실물 주행 보정.

---


ROS 2 Humble / Ubuntu 22.04 / YDLIDAR G4 / 엔코더 기반 이동 로봇용 구성입니다.
SLAM Toolbox 또는 Cartographer가 지도와 위치를 추정합니다. 라즈베리파이 4에서는 센서 드라이버,
엔코더 오도메트리와 SLAM을 모두 온보드로 실행합니다.

**진행 위치 (2026-09-29): Nav2 → Collision Monitor → 모터·엔코더 연결 구현 초안.**
실물 주행·정지거리·PI 보정은 미검증입니다. PSD·그리퍼·미션 상태머신은 아직 미구현입니다.
Nav2 주행은 가상 입력으로 목표 이동·복귀·장애물 회피까지 확인했습니다.
D415 인식과 정렬 서보는 합성 입력만 확인했고, 실물 센서와 Pi 4 성능은 둘 다 미검증입니다.
웹 UI는 코드가 있으며 브라우저·파이 동작 재검증이 필요합니다.
전체 순서와 실물 측정 기록표는 [[docs/superpowers/plans/2026-09-10-competition-plan|후속 개발 계획]]을 따릅니다.
해당 계획은 Claude 검토 전 초안입니다.

현재 포함된 기능:

- G4 드라이버 실행, 센서 장착 TF, SLAM Toolbox·Cartographer 전용 런치와 설정
- **Nav2 주행 설정과 런치 (`cwu_nav`)** — 사전 지도 없이 온라인 SLAM 위에서 동작
- **D415 드라이버 설정과 빨간 목표물 검출 (`cwu_perception`)** — `/target/bearing` 발행 (방향만, 깊이 없음)
- **목표물 정렬 시각 서보 (`cwu_nav/target_follower`)** — 방향 오차를 `/cmd_vel_servo`로
- 엔코더 오도메트리(`cwu_base`) — 시리얼 틱을 `/odom`과 `odom → base_link` TF로 변환
- 장비 없이 실행하는 **별도 가상 센서 데모**와 지도 생성·저장 통합 검증
- 브라우저로 지도·위치·토픽 주기를 보는 **웹 UI** (RViz 대체)
- 선택적인 개발용 RViz 화면

**LiDAR 경로는 2026-09-10에 실물 검증했습니다.** 라즈베리파이에서 G4 드라이버가
`frame_id=laser_frame`, 930점, -180°~180°, 약 9.7 Hz로 `/scan`을 발행하고
`base_link → laser_frame` TF가 연결되는 것까지 확인했습니다. 파이 온보드 전체
빌드는 약 1분 20초입니다(SDK 26s → 드라이버 45s → cwu_slam 6s).

**엔코더 보정값은 2026-09-29 팀 측정으로 입력됐습니다.** 반지름 0.04265m,
트레드 0.20686m, 회전당 평균 틱 3009.5이며 좌우 10회전 재측정이 필요합니다.
통합 구동은 `motor.yaml`을 사용하고, 엔코더 전용 시험은 `encoder.yaml`을 사용합니다.
실물 전개 절차·변경 파일·열린 질문은
[구동 구현 기록](docs/superpowers/plans/2026-09-29-nav2-drive-status.md)에 있습니다.

## 0. 라즈베리파이 시계 (최초 1회)

파이 4에는 RTC가 없어 전원을 내리면 시계가 과거로 되돌아갑니다. 게다가 이 현장
네트워크는 UDP 123이 공유기·외부 모두 차단돼 있어 기본 `systemd-timesyncd`가
영구히 실패합니다(`System clock synchronized: no`). 시계가 틀리면 `/scan`
타임스탬프가 어긋나 TF 조회가 전부 실패하고, HTTPS 인증서도 "아직 유효하지 않음"으로
거부돼 `git`·`apt`가 막힙니다.

```bash
ssh -t rpi 'sudo bash ~/ros_CWU/tools/setup_pi_clock.sh'
```

HTTP 응답의 `Date` 헤더에서 시각을 받는 `htpdate`를 설치·설정합니다(TCP 80/443은
열려 있음). 1회 실행하면 이후 부팅마다 자동으로 보정됩니다. 확인:

```bash
ssh rpi 'systemctl is-active htpdate; date -u'   # PC의 date -u와 비교
```

## 1. 빌드

현재 PC에는 SLAM/지도 저장 의존성을 설치했습니다. 새 PC 또는 라즈베리파이의
Ubuntu 22.04 64-bit + ROS 2 Humble 환경에서는 다음을 실행합니다.

```bash
sudo apt update
sudo apt install ros-humble-nav2-map-server ros-humble-rviz2 \
  python3-colcon-common-extensions python3-rosdep \
  python3-pytest python3-yaml build-essential cmake
cd ~/ros_CWU
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
MAKEFLAGS=-j2 colcon build --symlink-install --executor sequential
source install/setup.bash
```

SLAM 백엔드는 3.3장에서 사용할 것을 별도 설치합니다. 둘 다 비교하려면 두 설치 명령을 실행합니다.
`rosdep`을 처음 쓰는 시스템은 `sudo rosdep init`과 `rosdep update`를 먼저 합니다.
ARM 라즈베리파이로 옮길 때 `build/`, `install/`, `log/`는 복사하지 않고 다시 빌드합니다.
`colcon.meta`가 SDK → G4 드라이버 순서를 지정하며 SDK도 작업공간 내부에 설치합니다.
SDK의 빈 라이브러리 경로 내보내기 문제를 해결하는 작은 드라이버 CMake 패치를
적용했습니다(`patches/ydlidar-workspace-library.patch`). 제조사 자체 테스트는 빌드에서 제외했습니다.
자체 SLAM 패키지 테스트는 아래에서 별도로 실행합니다.

하드웨어 드라이버는 선택 의존성이므로 `--packages-up-to cwu_slam`만으로는 빌드되지
않습니다. 위 전체 빌드 명령을 사용하세요. 데모만 필요하면 다음으로 충분합니다.

```bash
colcon build --symlink-install --packages-select cwu_slam cwu_base cwu_nav cwu_perception
```

`cwu_base`는 `pyserial`이 필요합니다. 위 `rosdep install`이 `python3-serial`로
설치하며, 수동으로는 `sudo apt install python3-serial`입니다.

제조사 소스가 없는 새 체크아웃에서는 작업공간 루트에서 `dependencies.repos`의
커밋을 사용하거나 다음처럼 가져옵니다.

```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git src/ydlidar_sdk
git -C src/ydlidar_sdk checkout 01cdda4f2b36dff2a706d0535c64228d863c7411
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git src/ydlidar_ros2_driver
git -C src/ydlidar_ros2_driver checkout 4ef70d3f32a85704ade0be54b214f3763b1ab3e8
git -C src/ydlidar_ros2_driver apply ../../patches/ydlidar-workspace-library.patch
```

## 2. 장비 없는 데모

```bash
cd ~/ros_CWU
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch cwu_slam demo.launch.py rviz:=true
```

GUI가 없으면 `rviz:=true`를 생략합니다. 가상 로봇이 원을 돌며 3.6 m 방과 내부
상자에 대한 `/scan`, `/odom`, TF를 발행합니다. SLAM이 **스캔으로부터** 지도를 만듭니다.
데모 방은 테스트용이며 실물 실행에 로드되지 않습니다. 가상 입력은 이상적인
오도메트리와 순간 스캔으로, 슬립·노이즈·스캔 중 움직임을 재현하지 않습니다.
따라서 데모 통과는 실물 정확도나 라즈베리파이 성능을 보증하지 않습니다.

데모는 기본적으로 실제 시계를 사용합니다. 외부 `/clock` 발행기 없이
`use_sim_time:=true`를 설정하면 타이머가 동작하지 않습니다. 종료는 Ctrl+C입니다.

## 3. 실제 G4 + 엔코더 연결

```text
G4 driver (cwu_slam) ── /scan ──────────────────┐
cwu_base encoder_odom ── odom → base_link TF ───┼─ SLAM Toolbox ── /map
Mount TF (cwu_slam) ── base_link → laser_frame ─┘                 map → odom TF
```

TF별 발행자는 반드시 하나여야 합니다.

| 인터페이스 | 제공자 | 조건 |
|---|---|---|
| `/scan` (`sensor_msgs/LaserScan`) | G4 드라이버 | `frame_id=laser_frame`, 실제 측정 타임스탬프 |
| `/odom` (`nav_msgs/Odometry`) | `cwu_base/encoder_odom` | `header.frame_id=odom`, `child_frame_id=base_link` |
| `odom → base_link` | `cwu_base/encoder_odom` 또는 robot_localization | 스캔 시각에 조회 가능한 동적 TF |
| `base_link → laser_frame` | 이 패키지 또는 robot_state_publisher | 측정한 센서 장착 위치·방향 |
| `map → odom` | SLAM Toolbox | 다른 노드에서 중복 발행하지 않음 |

**SLAM Toolbox는 `/odom` 토픽 자체가 아니라 TF를 사용합니다.** `/odom`만 발행해도
SLAM 입력 조건이 충족되는 것은 아닙니다. 이동 로봇에 고정 `odom → base_link` TF를
넣어 문제를 숨기면 안 됩니다. 보통 엔코더 오도메트리를 20–50 Hz 정도로 발행하고,
LiDAR와 동일한 ROS 시계의 측정 시각을 사용합니다.

실물 투입 전에 다음 하드웨어 값을 확정합니다. 경기장 공개 뒤 조정하는 값이 아닙니다.

- `src/cwu_slam/config/mount.yaml`: 현재 원점/무회전인 **미보정 기본값**입니다.
  실제 장착 x/y/z(미터), roll/pitch/yaw(라디안)를 측정합니다.
  REP-103 기준 +x 전방, +y 좌측, +z 위쪽입니다.
- `ydlidar_g4.yaml`: 제조사 G4 설정을 바탕으로 230400 baud, 9 kHz, 10 Hz를 사용합니다.
  실물에서 포인트 방향과 반전 설정을 확인합니다. 직렬 장치는 가능하면
  `/dev/serial/by-id/...` 고정 경로를 사용합니다.

실물 G4에서 확인된 동작 두 가지입니다. 둘 다 정상이므로 "고치려" 하지 마십시오.

- **무효 측정은 `inf`가 아니라 `0.0`으로 옵니다.** 드라이버가 `ranges`를 0으로
  초기화한 뒤 `range >= range_min`인 점만 덮어쓰기 때문입니다. 제조사 예제의
  `invalid_range_is_inf`는 읽히기만 하고 사용되지 않는 죽은 파라미터라 설정에서
  뺐습니다. `/scan` 소비자는 `range_min`~`range_max` 밖의 값을 버려야 합니다.
  실측 기준 930점 중 절반가량이 `0.0`입니다.
- **`Real points 931 > fixed points 930` 경고가 매 스캔 나옵니다.**
  `fixed_resolution: true`가 스캔 크기를 930점으로 고정하면서 초과분을 잘라내기
  때문입니다. 이 값을 `false`로 바꾸면 `angle_increment`가 스캔마다 달라져
  LaserScan 배열 크기가 변하고, slam_toolbox가 첫 스캔과 점 수가 다른 스캔을
  거부해 SLAM이 조용히 멈춥니다. 경고를 감수하고 `true`로 두는 것이 맞습니다.
- 엔코더: **차동구동(좌우 2륜)으로 확정**되었습니다. `cwu_base/encoder_odom`이 틱을
  적분해 `/odom`과 TF를 냅니다. 되감김 보정·공분산·통신 끊김 감지는 구현돼 있고,
  남은 것은 `src/cwu_base/config/encoder.yaml`에 채울 **실측값**뿐입니다.

### 3.1 엔코더 설정값 채우기 (1회)

`encoder.yaml`의 `-1` 값이 남아 있으면 노드가 시작을 거부합니다. 추측값으로 돌리면
SLAM은 정상처럼 보이면서 어긋난 지도를 만들기 때문에, 실패를 앞당기는 쪽을 택했습니다.

**통신 형식** — Nucleo F446RE 펌웨어는 존재하지만 시리얼 프로토콜이 문서화돼 있지
않습니다. 보드를 파이 USB에 연결한 뒤 형식을 역추적합니다.

```bash
python3 tools/sniff_encoder_serial.py          # 포트·보드레이트 자동 탐색
```

데이터 송신은 하지 않지만 포트를 열 때 보드에 따라 리셋될 수 있으므로 모터 전원을 분리하고 확인합니다. 보드레이트 후보를 훑어 인쇄 가능
바이트 비율로 최적값을 고르고, 줄 단위 텍스트면 필드 개수와 각 필드의 증감 추세까지
출력합니다. 단조 증가/감소하는 필드가 누적 틱입니다. 여기서 `port`, `baud`,
`left_field`, `right_field`를 얻습니다. 왼쪽 바퀴만 앞으로 굴려 어느 필드가 움직이는지
보면 좌우를 가르고, 앞으로 굴릴 때 값이 줄면 해당 `*_sign`을 `-1.0`으로 둡니다.

노드는 줄에서 정규식으로 뽑은 정수 필드를 번호로 참조하므로 구분자(`,` 공백 `:`)는
상관없습니다. 다만 **줄 단위 텍스트 프로토콜**을 전제합니다. 스니퍼가 바이너리
프레이밍이라고 하면 파서를 따로 붙여야 합니다.

**바퀴 제원** — `wheel_radius`(m), `wheel_separation`(좌우 접지점 간격, m),
`ticks_per_rev`(감속비 반영 후 바퀴 1회전당 틱). 틱 수가 확실하지 않으면 바퀴를 손으로
정확히 10바퀴 굴리며 스니퍼의 틱 변화를 읽어 10으로 나눕니다.

**검증** — 안전하게 수동 이동하며 1 m 직진과 360° 회전을 각각 3회 측정합니다.
초기 개발 합격 기준은 각 회차 직진 거리 오차 5% 이내, 회전 후 방향 오차 5° 이내입니다.
회전은 시작·끝 yaw뿐 아니라 연속 yaw를 펼친 누적 회전각도 확인합니다.
정지 중에도 MCU가 같은 틱 값을 주기적으로 보내는지 `/odom` 주기로 확인합니다.
현재 노드는 수신 시에만 TF를 갱신하며, 통신 단절 시 오류 로그만 남기고 모터를 정지시키지 않습니다.

1 m 직진 후 `/odom`의 x가 1.0에 가까운지, 제자리 360° 회전 후 yaw가
제자리로 오는지 봅니다. 직진 오차는 `wheel_radius`, 회전 오차는 `wheel_separation`으로
보정합니다. 이 두 값은 계산값이 아니라 **보정 손잡이**입니다.

### 3.2 실행

```bash
source /opt/ros/humble/setup.bash
source ~/ros_CWU/install/setup.bash
ros2 launch cwu_slam slam.launch.py port:=/dev/ttyUSB0 encoder_port:=/dev/ttyAMA0
```

`slam.launch.py`가 G4 드라이버, 장착 TF, 엔코더 오도메트리, SLAM Toolbox를 함께 띄웁니다.
`port`는 LiDAR, `encoder_port`는 Nucleo입니다. 두 장치 모두 `/dev/serial/by-id/...`
고정 경로를 쓰면 부팅 순서에 따라 뒤바뀌는 일을 막을 수 있습니다.

URDF가 센서 TF를 이미 발행하면 `publish_mount_tf:=false`를 추가합니다.
G4 드라이버를 별도로 실행 중이면 `start_lidar:=false`, 엔코더 오도메트리를 다른
노드(robot_localization 등)가 맡으면 `start_encoder:=false`를 추가합니다.
이 실물 launch에는 가상 데이터나 모터 명령이 없습니다. 로봇이 실제로 이동해야
새 영역을 지도에 넣을 수 있습니다.

### 3.3 SLAM Toolbox / Cartographer 선택 실행

두 전용 런치는 공통 G4·엔코더·장착 TF를 사용하며 백엔드를 이름대로 고정합니다.
비교할 때는 먼저 실행한 런치를 Ctrl+C로 종료한 뒤 다른 것을 실행합니다.
같은 ROS 도메인에서 동시에 실행하면 `/map`과 `map → odom` TF가 중복됩니다.

| 구분 | SLAM Toolbox | Cartographer |
|---|---|---|
| 전용 런치 | `slam_toolbox.launch.py` | `cartographer.launch.py` |
| 설정 | `src/cwu_slam/config/slam.yaml` | `src/cwu_slam/config/cartographer_2d.lua` |
| 입력 | `/scan`, `odom → base_link` TF | `/scan`, `/odom`, `odom → base_link` TF |
| 지도 생성 | 비동기 온라인 SLAM 노드 | Cartographer + 점유 격자 노드 |
| 출력 | `/map`, `map → odom` TF | `/map`, `map → odom` TF |

#### A. SLAM Toolbox

```bash
sudo apt install ros-humble-slam-toolbox
# 실물: encoder.yaml 및 mount.yaml 실측 설정을 먼저 완료합니다.
ros2 launch cwu_slam slam_toolbox.launch.py port:=/dev/ttyUSB0 encoder_port:=/dev/ttyAMA0
# 장비 없는 데모: 위 실물 런치를 종료한 뒤 실행합니다.
ros2 launch cwu_slam slam_toolbox.launch.py demo:=true rviz:=true
# 자동 검증: 데모를 따로 켜지 않고 실행합니다.
python3 src/cwu_slam/test/run_demo_check.py --backend slam_toolbox
```

#### B. Cartographer

```bash
sudo apt install ros-humble-cartographer-ros
# 실물: SLAM Toolbox와 같은 센서·엔코더 설정을 사용합니다.
ros2 launch cwu_slam cartographer.launch.py port:=/dev/ttyUSB0 encoder_port:=/dev/ttyAMA0
# 장비 없는 데모: 위 실물 런치를 종료한 뒤 실행합니다.
ros2 launch cwu_slam cartographer.launch.py demo:=true rviz:=true
# 자동 검증: 데모를 따로 켜지 않고 실행합니다.
python3 src/cwu_slam/test/run_demo_check.py --backend cartographer
```

GUI가 없으면 `rviz:=true`를 생략합니다. 각 자동 검사는 선택한 백엔드 전용 런치로
같은 가상 입력을 제공하고 지도·TF·지도 저장을 확인한 뒤 종료합니다.
Cartographer 설정은 라이브러리의 네이티브 형식인 Lua에 유지합니다.
설정 변경은 경기장 공개 전에 완료하고, 대회에서는 선택한 백엔드 하나를 온보드로 실행합니다.

기존 `slam.launch.py`, `demo.launch.py`와 `backend:=...` 인자도 호환 유지합니다.
기존 런치의 기본값은 SLAM Toolbox입니다. bag 재생 시에는 선택한 전용 런치에
`start_lidar:=false start_encoder:=false publish_mount_tf:=false use_sim_time:=true`를
추가합니다. 기록된 `map → odom` TF를 제외하는 조건은 5장을 따릅니다.

직렬 접근이 거부되면 현재 사용자의 `dialout` 그룹 권한을 확인합니다. 필요한 경우
`sudo usermod -aG dialout "$USER"` 실행 후 로그아웃/로그인합니다.
이 작업에서는 사용자 그룹이나 USB 규칙을 변경하지 않았습니다.

### 3.4 웹 UI (RViz 대신 브라우저로 보기)

파이는 headless라 RViz를 띄우려면 X 포워딩이 필요하고 느립니다. 대회 중에는 외부 PC
자체가 금지입니다. 같은 WiFi의 폰이나 노트북 브라우저로 보는 쪽이 실용적입니다.

웹 UI를 사용할 때만 `sudo apt install ros-humble-rosbridge-suite`로 별도 설치합니다.
SLAM Toolbox·Cartographer·rosbridge는 코어 패키지의 필수 의존성에서 제외했으므로 `rosdep install`로 설치되지 않습니다.
`slam.launch.py`가 떠 있는 상태에서 다른 터미널에 띄웁니다.

```bash
ros2 launch cwu_slam web.launch.py
```

브라우저에서 `http://raspberrypi.local:8080` (또는 파이의 IP)으로 접속합니다.
지도, map 프레임 기준 로봇 위치·방향, `/scan` 점, 토픽별 수신 주기, 그리고 지도 저장
버튼이 나옵니다. `http_port`(기본 8080)와 `ws_port`(기본 9090)로 포트를 바꿉니다.
`port`라는 이름은 rosbridge의 XML launch가 이미 쓰고 있어 피했습니다.

페이지는 `src/cwu_slam/web/index.html` 한 장이고 rosbridge의 JSON 프로토콜에
WebSocket으로 직접 붙습니다. roslib.js 같은 외부 라이브러리를 CDN에서 받지 않으므로
인터넷이 없는 경기장에서도 그대로 뜹니다.

`/scan` 점은 `base_link` 위치에 그리므로 LiDAR 장착 오프셋만큼 어긋납니다. 상태 확인용
표시이며 측정 도구가 아닙니다. 정확한 겹침이 필요하면 RViz를 쓰십시오.

**대회 중에는 이 UI를 켜지 마십시오.** 규정상 미션 시작 후 외부 통신이 금지됩니다.
개발·리허설·시작 직전 점검용입니다.

## 4. Nav2 자율주행과 D415 카메라

SLAM 위에 Nav2 주행과 D415 목표물 인식·정렬을 올린 구성입니다. 미션 상태머신은 아직 없습니다.
설계와 인터페이스는 [[docs/superpowers/specs/2026-09-16-mission1-control-design|미션1 제어 설계]]를 따릅니다.

> [!warning] 실물 구동 기본 비활성 · Claude 검토 전 초안
> `drive:=true`일 때만 `/cmd_vel_safe`를 받는 모터 연결을 실행합니다.
> `motor.yaml`의 미측정 값과 보정 확인을 먼저 완성해야 합니다. 새 펌웨어는 자동 업로드하지 않습니다.
> 가상 검사 통과는 실물 정지·주행 합격을 뜻하지 않습니다.

### 4.1 설치

Nav2와 RealSense는 용량이 커서 코어 의존성에서 제외했습니다. 쓸 때만 설치합니다.

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-regulated-pure-pursuit-controller ros-humble-nav2-collision-monitor
sudo apt install ros-humble-realsense2-camera   # D415를 쓸 때만
```

### 4.2 장비 없는 주행 데모

```bash
cd ~/ros_CWU
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch cwu_nav autonomy.launch.py demo:=true camera:=false follow:=false rviz:=true
```

가상 센서가 `drive:=cmd_vel` 모드로 돌아 Nav2가 보낸 속도만큼 움직입니다.
RViz에서 목표를 찍거나 다른 터미널에서 액션을 보냅니다.

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: -0.8}, orientation: {w: 1.0}}}}"
```

자동 검증은 같은 데모로 목표 이동, 시작점 복귀, 장애물 거부를 확인합니다.

```bash
python3 src/cwu_nav/test/run_nav2_check.py
```

```text
PASS: (0.93, -0.73) 도달, 오차 0.10 m
PASS: (0.07, -0.06) 도달, 오차 0.09 m
PASS: 상자 안 (1.10, 1.10) 주행 거부됨 status=6
PASS: SLAM → Nav2 → Collision Monitor → /cmd_vel_safe → 이동 경로가 연결됩니다
```

마지막 두 줄이 중요합니다. 상자 안 목표가 **성공하면** 코스트맵에 장애물이 안 찍힌 것이고,
그 상태로 실물에 올리면 로봇이 벽으로 들어갑니다.

종료할 때 lifecycle 노드 상태 전환 오류와 컨테이너 SIGKILL 로그가 나옵니다.
`Goal succeeded` 이후 Ctrl+C 시점에만 나오는 Nav2 컴포지션의 알려진 종료 경합이며
주행 결과와는 무관합니다.

기존 `demo.launch.py`의 고정 원운동은 `/cmd_vel`을 무시하므로 이 검증을 대신하지 못합니다.
`demo_drive:=circle`을 주면 원운동으로 되돌아갑니다.

### 4.3 실물 실행

```bash
ros2 launch cwu_nav autonomy.launch.py \
  port:=/dev/ttyUSB0 encoder_port:=/dev/ttyAMA0
```

G4 드라이버, 장착 TF, 엔코더 오도메트리, SLAM Toolbox, Nav2, D415 드라이버,
목표물 검출기, 시각 서보를 함께 띄웁니다. 카메라를 빼려면 `camera:=false`,
서보만 빼려면 `follow:=false`입니다.
Nav2만 따로 띄우려면 `ros2 launch cwu_nav nav2.launch.py`,
카메라만 따로면 `ros2 launch cwu_perception camera.launch.py`입니다.

실측과 바퀴를 띄운 검증이 끝난 뒤 저속 실물 구동 시험:

```bash
ros2 launch cwu_nav autonomy.launch.py drive:=true camera:=false follow:=false \
  encoder_port:=/dev/ttyAMA0 port:=/dev/ttyUSB0
```

`drive:=true`에서는 엔코더 전용 노드를 제외하고 통합 모터 노드 하나만 포트를 소유합니다.
카메라·서보·가상 시간·데모와 실물 구동을 함께 켜면 거부합니다.
읽기 전용 엔코더 노드는 기존 `ENC,left,right` 펌웨어용입니다. 새 펌웨어는 프레임이 다릅니다.
구동 속도·확장 반경·제동거리·정지 영역은 `motor.yaml`의 실측값에서 함께 적용합니다.
측정값은 경기장 공개 전에 확정합니다. 위 목표 지정 절차는 개발 시험용이며 경기 중 외부 통신에 의존하지 않습니다.

### 4.4 D415 카메라 — 방향만 씁니다

**깊이는 쓰지 않습니다.** 카메라가 답하는 것은 "목표물이 어느 쪽인가" 하나이고,
"얼마나 남았는가"는 그리퍼 PSD가 답합니다. 그래서 `realsense.yaml`은 깊이 스트림과
정렬을 끄고 컬러 하나만 켭니다. 되돌리려면 `enable_depth`와 `align_depth.enable`
두 플래그이며, 규정상 경기장 공개 전에만 가능합니다.

`src/cwu_perception/config/realsense.yaml`이 드라이버 설정을,
`target.yaml`이 빨간 목표물 검출 임계값을 담습니다. 검출기는 컬러 영상만 받아
`/target/bearing`(`geometry_msgs/PointStamped`)을 **카메라 컬러 광학 프레임의
단위 방향 벡터로** 발행합니다. 길이가 1이라는 것 외에 어떤 거리도 주장하지 않습니다.
목표물이 안 보이면 발행하지 않으므로, 소비자는 `header.stamp`로 상실을 판정합니다.

해상도는 424×240 @ 15 fps입니다. Pi 4 4GB에서 SLAM·Nav2와 동시에 도는 것이
목적입니다. 검출 유효 거리가 모자라면 해상도를 올리고 그때마다 동시 실행 CPU를
다시 측정합니다.

카메라 없이 검출기의 ROS 연결만 확인합니다.

```bash
python3 src/cwu_perception/test/run_detector_check.py
```

### 4.5 목표물 정렬 시각 서보

`src/cwu_nav/config/servo.yaml`. `target_follower`가 `/target/bearing`을 받아
목표물이 화면 세로 중심선에 오도록 돌고, 정렬된 뒤에만 천천히 전진합니다.
출력은 `/cmd_vel_servo`이며 **모터로 바로 가지 않습니다** — Nav2의 `/cmd_vel`과
이 토픽 중 무엇을 내보낼지는 안전 감시자(`cwu_safety`, 미구현)가 미션 상태를 보고
고릅니다. 전진을 멈추는 신호는 PSD뿐이므로 `approach_speed`는 0.08 m/s로 낮췄습니다.

목표물 표본이 `sample_timeout_s`보다 오래되면 0을 발행합니다. 검출 콜백이 아니라
고정 주기로 내보내는 이유가 이것입니다. 목표물을 놓쳤을 때 마지막 명령이 남아
계속 도는 것을 막으려면, 표본이 끊긴 것 자체가 정지 명령이 되어야 합니다.

로봇 없이 서보의 ROS 연결과 정지 경로를 확인합니다.

```bash
python3 src/cwu_nav/test/run_follower_check.py
```

**경기장 공개 전에 반드시 끝낼 것:** `realsense.yaml`의 자동 노출·자동 화이트밸런스가
지금 켜져 있습니다. 규정상 공개 후에는 파라미터를 못 고치므로, 경기장과 같은 조명에서
수렴값을 읽어 고정값으로 바꿔야 합니다. 절차는 해당 YAML 주석에 있습니다.

### 4.6 장착값 보정 (LiDAR와 카메라)

`src/cwu_slam/config/mount.yaml` 한 파일이 `laser_mount`와 `camera_mount`를 모두 담습니다.
둘 다 **현재 전부 0인 미보정 기본값**이며, 실행 시 경고가 나옵니다.
LiDAR는 3장, 카메라는 로봇 앞 0.5 m에 빨간 큐브를 두고 `/target/bearing`을 보는 방법을
YAML 주석에 적어 두었습니다. 카메라는 pitch가 특히 중요합니다. 깊이가 없어 접근을
영상으로만 하므로, 먼 목표물과 접근 마지막 구간의 목표물이 둘 다 화면에 남아야 합니다. URDF가 이 TF를 발행하면 `publish_mount_tf:=false`입니다.

### 4.7 Nav2 설정에서 손대는 값

`src/cwu_nav/config/nav2.yaml`. 플래너·컨트롤러는 자작하지 않고 기성품만 씁니다.

| 값 | 현재 | 언제 바꾸나 |
|---|---|---|
| `robot_radius` | 0.22 m | 그리퍼·배선 포함 실제 최대 확장 반경을 재고 나서 |
| `desired_linear_vel` | 0.26 m/s | 실제 제동거리를 재고 나서 |
| `inflation_radius` | 0.35 m | 키우면 800 mm 통로까지 막히므로 경로가 안 나올 때만 |
| 컨트롤러 | Regulated Pure Pursuit | 미션2에서 상대 로봇 회피가 부족하면 DWB로 바꾸고 CPU 재측정 |

전역 코스트맵은 사전 지도를 쓰지 않고 SLAM의 `/map`을 그대로 받습니다(규정 2.3).
`map_server`와 AMCL은 띄우지 않습니다.

## 5. 지도 저장 및 입력 확인

SLAM이 실행 중인 동안 같은 ROS 환경을 source한 다른 터미널에서:

```bash
mkdir -p ~/ros_CWU/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros_CWU/maps/current \
  --ros-args -p save_map_timeout:=10.0
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map laser_frame
```

`current.pgm`과 `current.yaml`은 점유 지도의 결과물이며 SLAM 포즈 그래프는 아닙니다.
실제 SLAM은 매 실행마다 새 지도를 만들며 이 파일을 자동 로드하지 않습니다.
스캔이 보여도 지도가 없으면 TF의 존재·시각·프레임 이름부터 확인합니다.

실제 기록 데이터가 있다면 가상 센서 없이 재생할 수 있습니다.

```bash
ros2 launch cwu_slam slam.launch.py start_lidar:=false start_encoder:=false \
  publish_mount_tf:=false use_sim_time:=true
# 다른 터미널: /scan, /tf, /tf_static이 포함된 기록이어야 합니다.
ros2 bag play <bag_directory> --clock
```

기록은 `/scan`, `/odom`, `odom → base_link` TF와 센서 장착 TF를 포함해야 합니다.
재생 시 `map → odom`은 새 SLAM만 발행해야 하므로, 기록된 `/tf`에 이 변환이
들어 있으면 그대로 재생하지 말고 제외한 입력 bag을 준비합니다.

bag에 센서 장착 TF가 없으면 올바른 `mount_file`로 `publish_mount_tf:=true`를 사용합니다.

## 6. 테스트

2026-09-13 개발 PC에서 전체 4개 패키지 빌드, 단위 테스트 29개와 아래 두 가상 통합 검사를 통과했습니다.
두 전용 런치에서 SLAM Toolbox(72×72), Cartographer(84×84) 지도 생성·TF·PGM/YAML 저장을 확인했습니다.
Cartographer는 시작 시 TF 과거 시각 조회 경고가 발생했으며 이후 검사는 통과했습니다.
실물 주행·웹 UI·Pi 성능·장시간 루프 폐쇄 품질 검증은 포함하지 않습니다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select cwu_slam cwu_base --event-handlers console_direct+
colcon test-result --verbose
# 별도 데모 실행 없이 시작/검증/저장/종료까지 자동 수행
python3 src/cwu_slam/test/run_demo_check.py
# 가상 시리얼(pty)로 엔코더 노드의 파싱·되감김·적분·발행을 검증. 실물 불필요
python3 src/cwu_base/test/run_encoder_check.py
# 합성 컬러 프레임으로 목표물 검출기의 토픽·방향 부호를 검증. 카메라 불필요
python3 src/cwu_perception/test/run_detector_check.py
# 합성 방향 표본으로 시각 서보의 회전 부호·정렬 전진·표본 만료 정지를 검증. 로봇 불필요
python3 src/cwu_nav/test/run_follower_check.py
# SLAM → Nav2 → /cmd_vel → 이동 → 복귀를 가상 센서로 검증. Nav2 설치 필요
python3 src/cwu_nav/test/run_nav2_check.py
```

실물 G4가 연결된 파이에서는 하드웨어 점검 두 가지를 추가로 실행합니다.

```bash
# ROS 없이 직렬 프로토콜만으로 센서 자체를 확인(health, 모델/펌웨어, 실측 거리, 방향별 분포)
python3 src/cwu_slam/test/g4_serial_check.py /dev/serial/by-id/usb-Silicon_Labs_CP2102_*-if00-port0

# slam.launch.py가 떠 있는 상태에서 /scan과 장착 TF를 확인
python3 src/cwu_slam/test/g4_scan_check.py
```

`g4_serial_check.py`는 드라이버가 주파수를 설정하기 전의 stock 상태를 읽으므로
스캔 주파수가 6~7 Hz로 나옵니다. ROS 드라이버로 띄우면 `frequency: 10.0`이 적용돼
`g4_scan_check.py`에서 약 9.7 Hz가 나오는 것이 정상입니다.

통합 검증은 기본 ROS 도메인 67과 localhost 통신을 사용하며 `/tmp/cwu-<backend>-check-*`에
지도와 로그를 남깁니다. 같은 도메인에 다른 로봇/데모를 실행하지 마세요.
`ROS_DOMAIN_ID=68 python3 ...`처럼 빈 도메인을 지정할 수 있습니다.
소켓 통신이 차단된 샌드박스에서는 ROS 통합 테스트에 별도 실행 권한이 필요합니다.

`run_encoder_check.py`는 pty를 열어 되감김 구간을 지나는 1 m 직진 틱을 흘려 넣고
`/odom`의 x와 `odom → base_link` TF가 일치하는지 확인합니다. 실물 엔코더·바퀴 제원은
검증하지 않습니다 — 그것은 위 3.1의 1 m/360° 실측이 담당합니다.

기본 테스트는 레이 교차, 가까운 장애물 차폐, 평행/후방 선분, 범위 제한을 검사합니다.
통합 검증은 움직이는 odom, 유효한 스캔, 전체 TF 연결, 지도 셀과 저장 파일을 확인합니다.
엔코더 기구학 테스트는 카운터 되감김, 틱 환산, 직진·제자리회전·호 적분, 좌우 부호를
검사합니다. 실물 G4/엔코더, 실제 루프 폐쇄 품질, Pi 메모리·CPU 부하는 별도 실험이 필요합니다.

경기에서는 RViz/웹 UI/외부 PC에 의존하지 않고 모든 계산을 온보드로 실행해야 합니다.
규칙은 [[docs/competition_rules|대회 규정 정리]]와 [[Agent|개발 원칙]]을 참고합니다.
규정 정리는 규정(안)을 바탕으로 하므로 최종 판단은 주최 측 원본 규정을 따릅니다.

## 참고

주석을 직접 넣기 어려운 보조 파일의 역할:

- `colcon.meta`: JSON 형식의 빌드 설정입니다. SDK를 G4 드라이버보다 먼저
  빌드하도록 지정하고 제조사 테스트·예제 빌드를 끕니다. JSON은 주석을 지원하지 않습니다.
- `src/cwu_slam/resource/cwu_slam`: ament가 ROS 패키지를 찾을 때 사용하는
  빈 등록 파일입니다. 실행 코드나 설정값을 담는 파일이 아닙니다.
- `patches/ydlidar-workspace-library.patch`: 제조사 드라이버가 작업공간에 설치된
  SDK 라이브러리를 찾도록 CMake를 수정하는 재현용 패치입니다.

- [SLAM Toolbox Humble](https://docs.ros.org/en/humble/p/slam_toolbox/)
- [G4 ROS 2 드라이버](https://github.com/YDLIDAR/ydlidar_ros2_driver/tree/humble)
- [YDLidar SDK](https://github.com/YDLIDAR/YDLidar-SDK)

## 관련 문서

- [[docs/프로젝트 목차|프로젝트 목차]] — 전체 문서와 파일 탐색
- [[docs/competition_rules|대회 규정과 제약]]
- [[docs/superpowers/specs/2026-09-08-slam-design|SLAM 인터페이스 설계]]
- [[docs/superpowers/specs/2026-09-16-mission1-control-design|미션1 제어 설계 — SLAM·Nav2·D415]]
- [[docs/superpowers/specs/2026-09-16-mission2-control-design|미션2 제어 설계 — 확보·유지·코너 회피]]
- [[docs/superpowers/specs/2026-09-16-psd-gripper-design|PSD 자동 파지 설계 초안]]
- [[docs/superpowers/plans/2026-09-10-competition-plan|후속 개발 계획]]
