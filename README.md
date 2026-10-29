# STM32 + Raspberry Pi + MDD10A Encoder PID Test

NUCLEO-F446RE, Raspberry Pi, Cytron MDD10A, 2채널 DC 모터 + 6선 엔코더를 이용한 UART / 엔코더 / 직진 PID 테스트 구성입니다.

## 사용 하드웨어

- STM32 NUCLEO-F446RE
- Raspberry Pi
- Cytron MDD10A Dual 10A Motor Driver
- WGM40 계열 6선 엔코더 DC 모터 x2
- 3S 11.1V 배터리

---

## 1. STM32 ↔ Raspberry Pi UART

ST-Link USB 시리얼이 모터 잡음으로 반복해서 멈춰([[docs/2026-09-29-실물-주행-시험-기록|실물 주행 시험 기록]]),
Nucleo 하드웨어 UART4를 Pi GPIO UART에 직결해 씁니다. USB는 업로드(`st-flash`)에만 씁니다.

| Nucleo-F446RE | | Raspberry Pi 4 |
|---|---|---|
| PC10 (UART4 TX, CN7-1) | → | GPIO15 / RXD (10번 핀) |
| PC11 (UART4 RX, CN7-2) | ← | GPIO14 / TXD (8번 핀) |
| GND | — | GND (6번 핀) |

- 115200 baud, 둘 다 3.3 V 로직이라 레벨 변환기가 필요 없습니다. Pi 쪽 포트는 `/dev/ttyAMA0`입니다.
- 펌웨어는 스케치 안에서 `Uart PiSerial(PC_11, PC_10)`로 직접 엽니다(기본 PlatformIO ini로 빌드).
- 보드 명령·응답 규약은 `README_STM_PI_INTERFACE.md` 0절입니다.

## 미션 1 작동 방식 (2026-10-29 펌웨어)

보드 펌웨어는 팀원의 `RobotIntegrated.ino` + `MotorControl.h`에서 미션 1용으로 하트비트와 정면 PSD 정지를 빼고
(정지 판단은 Pi), 텔레메트리 20 Hz, 잡기 99/78로 바꾼 판입니다.
이 펌웨어는 연속 속도 명령이 없고 **"`[F100~1000]` mm 직진"과 "`[TL/TR10~180]`° 제자리 회전"만** 받으므로,
Nav2 없이 400 mm 격자 칸 단위로 "돌고 → 한 칸 직진"을 하나씩 보냅니다. 하드웨어 없는 검사는 통과했고 실물 미션은 아직입니다.

### 노드와 역할

| 구성 | 역할 |
|---|---|
| G4 LiDAR | 벽·장애물 거리. SLAM 지도와, 직진 전 "가는 길이 비었나" 검사 |
| `encoder_odom` (cwu_base) | 보드 CSV(20 Hz) → `/odom`·TF, `/base/telemetry`(PSD·서보), `/base/event`(`# DONE`·`# STOP` …). `/base/command` 문자열을 허용 목록만 보드로 전달. 종료·통신 끊김(0.5 s)이면 `q` |
| slam_toolbox (cwu_slam) | `/map`과 map → odom. 출발 자세가 map 원점 = 출발 칸 중심 |
| 카메라 (cwu_perception) | **컬러만**. 빨간 목표물이 화면 세로 중앙선에서 얼마나 벗어났는지(방향)만 `/target/bearing`. 거리는 안 잼 |
| `mission1` (cwu_nav) | 상태머신. `grid.py`로 칸 경로를 짜고 다음 명령 하나를 `/base/command`로 보냄. 직진 중 정면 PSD가 550 이상 두 표본 연속이면 `q`로 세움 |
| 보드(Nucleo) | 직진·회전·집게 실행만. PSD로 멈추지 않고 하트비트도 없음 (거리·각도 끝, 15 s 제한, 엔코더 이상이면 멈춤) |

### 진행 순서

```text
RELEASE  7 (집게 열기, 다 열려야 다음으로)
LOOK     제자리 TL/TR 45°씩. 라이다로 트인 방향만 카메라가 지나가게 (첫 번째는 반시계 90°만)
GOTO     중앙 힌트 칸 (4,4)과 그 옆 칸 → 이후 아직 못 본 칸 경계. 칸 BFS → 회전 또는 F400 하나씩
APPROACH 화면 오차 ≥ 5°면 그만큼 회전(최소 10°), 아니면 F100. 화면에서 빠지면 F100 최대 3번 더
GRASP    직진 중 정면 PSD 550 이상(두 표본) → mission이 q로 세움. 접근 중이고 직전에 중앙에 보였으면 파지: 8 (잡기 99/78)
RETURN   출발 칸 (0,0)으로 같은 칸 이동. 경로가 없으면 막힘 표시를 비우고 옆 칸으로 옮겨 재시도(최대 3번)
DONE     집게를 닫은 채 정지
```

- 명령 하나를 보내면 `# DONE …`을 기다리고, 0.5 s 뒤 SLAM 자세로 다음 명령을 다시 계산합니다(옆으로 밀린 만큼 바로잡음).
- 직진 전에 라이다로 몸 폭(±0.165 m) 직선이 비었는지 봅니다. 막혔거나 직진 중 정면 PSD로 세우면 그 칸을 막힘으로 두고 돌아갑니다.
- 60 s 안에 보드 정지가 8번 넘으면 미션을 멈춥니다. 명령 하나가 20 s 안에 안 끝나면 `q`.
- 운반 중에는 쥔 큐브가 정면 PSD 앞에 있어 PSD 정지를 안 보므로 충돌 방지는 라이다 검사만 합니다.
- 보드에는 하트비트가 없어서 Pi가 죽으면 진행 중인 명령 하나(최대 F1000, 15 s)는 끝까지 갑니다.
- 50 mm 큐브는 라이다(약 170 mm 높이)에 안 보입니다. 큐브 정지는 정면 PSD 몫입니다.

### 실행

```bash
# 로봇을 출발 칸에 "경기장이 로봇 왼쪽"이 되게 놓고 (PC에서, 경기 시작 명령 한 번)
tools/field/start_mission1.sh      # Pi에서 SLAM + 카메라 + mission1, 로그 Pi ~/mission1.log, 텔레메트리 ~/telemetry_*.csv
tools/field/stop_mission1.sh       # 시험용 정지 (경기에서는 쓰지 않음)

# Pi에서 보드에 명령 하나 직접 보내기 (encoder_odom이 떠 있을 때)
ros2 topic pub --once /base/command std_msgs/String "{data: TL90}"
ros2 topic echo /base/event
```

설정은 `src/cwu_nav/config/mission1.yaml`(격자·접근·파지), `src/cwu_base/config/encoder.yaml`(보정값 = 펌웨어 `# CAL`),
`src/cwu_slam/config/slam.yaml`입니다. 경기장 공개 후에는 고치지 않습니다.

### 하드웨어 없이 검사

```bash
colcon build --packages-select cwu_base cwu_nav cwu_slam && source install/setup.bash
python3 -m pytest src/cwu_nav/test src/cwu_base/test      # 격자 플래너·판단·시리얼 규약
python3 src/cwu_nav/test/run_mission1_check.py            # 가짜 보드로 미션 1 전체 흐름
python3 src/cwu_base/test/run_encoder_check.py            # 가짜 시리얼로 /odom·TF
```

### 남은 일

1. 400 mm 칸에서 TL90 ×4 → SLAM 방향 오차 (회전 약 0.7 rad/s, odom 5 Hz)
2. 개활지 접근·파지·복귀 3회 → 모의 경기장 3회
3. 미션 2는 이 펌웨어 기준으로 아직 설계 안 함

---

## 2. 모터 엔코더 배선

6선 모터의 색상 기능은 다음과 같이 사용합니다.

| 색상 | 기능 |
|---|---|
| 빨강 | 모터 출력선 1 |
| 검정 | 모터 출력선 2 |
| 초록 | 엔코더 GND |
| 파랑 | 엔코더 VCC |
| 노랑 | 엔코더 A상 |
| 흰색 | 엔코더 B상 |

### 왼쪽 모터

| 모터 선 | 연결 |
|---|---|
| 빨강 | MDD10A M1A |
| 검정 | MDD10A M1B |
| 초록 | STM32 GND |
| 파랑 | 엔코더 정격 VCC |
| 노랑 | STM32 D4 |
| 흰색 | STM32 D5 |

### 오른쪽 모터

| 모터 선 | 연결 |
|---|---|
| 빨강 | MDD10A M2A |
| 검정 | MDD10A M2B |
| 초록 | STM32 GND |
| 파랑 | 엔코더 정격 VCC |
| 노랑 | STM32 D9 |
| 흰색 | STM32 D10 |

> 주의: 모터의 검정선은 GND선이 아닙니다. 빨강/검정 둘 다 MDD10A 모터 출력단에 연결합니다.

---

## 3. STM32 ↔ MDD10A 제어 배선

| STM32 | MDD10A | 역할 |
|---|---|---|
| D6 | PWM1 | 왼쪽 모터 PWM |
| D7 | DIR1 | 왼쪽 모터 방향 |
| D11 | PWM2 | 오른쪽 모터 PWM |
| D8 | DIR2 | 오른쪽 모터 방향 |
| GND | GND | 제어 기준 GND |

코드 기준 핀 정의:

```cpp
constexpr uint32_t LEFT_PWM  = 6;        // D6
constexpr uint32_t LEFT_DIR  = 7;        // D7
constexpr uint32_t RIGHT_PWM = PA7_ALT3; // D11 / PA7
constexpr uint32_t RIGHT_DIR = 8;        // D8
```

---

## 4. 엔코더 입력 핀

| 용도 | STM32 핀 |
|---|---|
| Left Encoder A | D4 |
| Left Encoder B | D5 |
| Right Encoder A | D9 |
| Right Encoder B | D10 |

---

## 5. 전원 배선

테스트 단계에서는 전원을 분리해서 사용합니다.

```text
노트북 USB  → STM32 NUCLEO-F446RE
USB-C 전원 → Raspberry Pi
3S 11.1V   → MDD10A → Left / Right Motor
```

MDD10A 전원:

```text
배터리 (+) → MDD10A VIN+
배터리 (-) → MDD10A GND / VIN-
```

공통 GND:

```text
MDD10A GND
STM32 GND
Raspberry Pi GND
Left Encoder GND
Right Encoder GND
```

모두 기준 전압을 공유해야 합니다.

> STM32는 테스트 중 노트북 USB로 전원을 공급하므로 E5V에 별도 5V를 넣지 않습니다.

---

## 6. 전체 STM32 핀 정리

| STM32 핀 | 기능 |
|---|---|
| D4 | Left Encoder A |
| D5 | Left Encoder B |
| D6 | Left Motor PWM |
| D7 | Left Motor DIR |
| D8 | Right Motor DIR |
| D9 | Right Encoder A |
| D10 | Right Encoder B |
| D11 | Right Motor PWM |
| PC10 | Raspberry Pi UART TX |
| PC11 | Raspberry Pi UART RX |
| GND | Common Ground |

---

## 7. Raspberry Pi UART 테스트

> 7~10절은 옛 시험 펌웨어 `STM32_Encoder_PID_Test.ino` 기준입니다. 지금 보드 펌웨어 명령은 `README_STM_PI_INTERFACE.md` 0절을 보세요.

Pi에서:

```bash
minicom -D /dev/serial0 -b 115200
```

Minicom 설정에서 Hardware Flow Control / Software Flow Control은 모두 `No`로 설정합니다.

STM32 코드가 실행되면 Pi에서 명령을 한 글자씩 전송할 수 있습니다.

| 명령 | 동작 |
|---|---|
| `h` | 도움말 출력 |
| `l` | 왼쪽 모터 약 1초 테스트 |
| `r` | 오른쪽 모터 약 1초 테스트 |
| `b` | 양쪽 모터 동일 PWM 테스트 |
| `p` | 엔코더 기반 P/PID 보정 테스트 |
| `s` 또는 Space | 즉시 정지 |
| `z` | 정지 + 엔코더 카운트 초기화 |

---

## 8. PID 기본 설정

현재 테스트 코드는 초기 상태에서 사실상 P 제어만 사용합니다.

```cpp
constexpr float KP = 0.50f;
constexpr float KI = 0.00f;
constexpr float KD = 0.00f;
```

기본 PWM:

```cpp
constexpr int BASE_PWM = 80;
constexpr int MAX_PWM  = 130;
```

오차는 좌우 엔코더 펄스 증가량 차이로 계산합니다.

```text
error = Left Encoder Delta - Right Encoder Delta
```

양수 오차라면 왼쪽 바퀴가 상대적으로 빠른 것으로 보고 왼쪽 PWM을 줄이고 오른쪽 PWM을 높입니다.

---

## 9. 첫 테스트 순서

1. MDD10A 배터리를 빼고 STM32 코드 업로드
2. STM32 / Pi UART 연결 확인
3. 바퀴를 바닥에서 띄움
4. MDD10A 배터리 연결
5. `h` 명령으로 양방향 UART 확인
6. `l`로 왼쪽 모터 방향 + 왼쪽 엔코더 확인
7. `r`로 오른쪽 모터 방향 + 오른쪽 엔코더 확인
8. `b`로 양쪽 모터 동일 PWM 확인
9. `p`로 좌우 엔코더 기반 보정 확인

---

## 10. 모터 / 엔코더 방향 반전

모터가 로봇의 전진 방향과 반대로 돌면:

```cpp
constexpr bool INVERT_LEFT_MOTOR  = true;
constexpr bool INVERT_RIGHT_MOTOR = true;
```

필요한 쪽만 `true`로 변경합니다.

모터는 전진하는데 엔코더 값이 음수로 증가하면:

```cpp
constexpr int LEFT_ENCODER_SIGN  = -1;
constexpr int RIGHT_ENCODER_SIGN = -1;
```

필요한 쪽만 `-1`로 변경합니다.

---

## 파일

- `STM32_Encoder_PID_Test.ino` : STM32 UART + Motor + Encoder + PID 테스트 코드
- `README.md` : 배선 및 테스트 절차
