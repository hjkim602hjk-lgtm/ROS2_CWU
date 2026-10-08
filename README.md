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

UART는 PC10 / PC11을 사용합니다.

| STM32 | Raspberry Pi | 역할 |
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
| A0 (PA0, UART4 TX) | → | GPIO15 / RXD (10번 핀) |
| A1 (PA1, UART4 RX) | ← | GPIO14 / TXD (8번 핀) |
| GND | — | GND (6번 핀) |

- 둘 다 3.3 V 로직이라 레벨 변환기가 필요 없습니다. 전원선(3.3 V/5 V)은 연결하지 않습니다.
- D3~D10은 모터·엔코더가 쓰므로 USART1(D8/D2)·USART6(PC7=D9)은 쓸 수 없습니다.
- 펌웨어: 빌드 플래그로 `Serial`을 UART4(RX=PA1, TX=PA0)에 연결하므로 코드는 그대로입니다. 업로드는 계속 USB로 합니다.
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
115200 baud
```

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
