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
| PC10 | GPIO15 / Physical Pin 10 | STM32 TX → Pi RX |
| PC11 | GPIO14 / Physical Pin 8 | STM32 RX ← Pi TX |
| GND | Physical Pin 6 / GND | 공통 GND |

STM32 코드에서는 다음 UART 객체를 사용합니다.

```cpp
Uart PiSerial(PC_11, PC_10);  // RX, TX
```

통신 속도:

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
