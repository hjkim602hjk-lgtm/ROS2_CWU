# STM32 ↔ Raspberry Pi UART / Encoder PID Firmware

## 1. 프로젝트 개요

이 문서는 **NUCLEO-F446RE + Raspberry Pi + Cytron MDD10A + 엔코더 DC 모터** 구성에서 사용한 UART 통신 및 엔코더/PID 테스트 펌웨어 설정을 정리한 문서입니다.

현재 확인된 상태:

- STM32 ↔ Raspberry Pi UART 통신 성공
- STM32 → Pi 문자열 송신 성공
- Pi → STM32 명령 송신 성공
- 좌/우 엔코더 카운트 수신 성공
- 왼쪽 엔코더 방향은 소프트웨어에서 부호 반전 필요
- Python(`pyserial`)을 이용한 UART 테스트 및 CSV 로깅 가능

---

## 2. 사용 하드웨어

- MCU: **STM32 NUCLEO-F446RE**
- SBC: **Raspberry Pi**
- Motor Driver: **Cytron MDD10A**
- Motor: 6선 엔코더 DC 기어드모터
- Battery: **3S 11.1V**
- STM32 개발환경: **Arduino IDE 2.x / STM32duino**
- Raspberry Pi OS: Ubuntu 22.04 계열
- UART Baudrate: **115200**

---

# 3. UART 배선

STM32의 기존 Arduino D핀들과 충돌하지 않도록 **PC10 / PC11**을 UART로 사용합니다.

| STM32 | Raspberry Pi | 기능 |
|---|---|---|
| **PC10 (TX)** | Physical Pin 10 / GPIO15 (RX) | STM32 → Pi |
| **PC11 (RX)** | Physical Pin 8 / GPIO14 (TX) | Pi → STM32 |
| **GND** | Physical Pin 6 / GND | 공통 GND |

UART는 반드시 **TX ↔ RX 교차 연결**합니다.

```text
STM32 PC10 (TX) ─────────→ Pi GPIO15 (RX), physical pin 10
STM32 PC11 (RX) ←───────── Pi GPIO14 (TX), physical pin 8
STM32 GND       ────────── Pi GND, physical pin 6
```

테스트 시 전원 구성:

```text
STM32        : 노트북 USB
Raspberry Pi : USB-C 전원
MDD10A       : 3S 11.1V 배터리
```

UART 테스트 단계에서는 STM32의 E5V에 외부 5V를 별도로 넣지 않았습니다.

---

# 4. STM32 UART 객체

사용 중인 STM32duino Core에서는 다음 방식으로 UART 객체를 생성합니다.

```cpp
#include <Arduino.h>

Uart PiSerial(PC_11, PC_10);  // RX, TX
```

주의:

```cpp
HardwareSerial PiSerial(PC_11, PC_10);
```

및

```cpp
Serial3.begin(115200);
```

방식은 현재 사용 환경에서 컴파일/링크 문제가 발생했으므로 사용하지 않았습니다.

UART 초기화:

```cpp
void setup() {
    PiSerial.begin(115200);
}
```

---

# 5. UART 단독 송신 테스트 펌웨어

STM32 → Raspberry Pi 통신 확인에 사용한 최소 테스트 코드입니다.

```cpp
#include <Arduino.h>

Uart PiSerial(PC_11, PC_10);  // RX, TX

void setup() {
    PiSerial.begin(115200);
}

void loop() {
    PiSerial.println("HELLO FROM STM32");
    delay(1000);
}
```

Raspberry Pi에서:

```bash
minicom -D /dev/serial0 -b 115200
```

정상일 경우:

```text
HELLO FROM STM32
HELLO FROM STM32
HELLO FROM STM32
```

가 1초마다 출력됩니다.

---

# 6. 전체 STM32 핀 배치

## Motor Driver

| 기능 | STM32 |
|---|---|
| Left Motor PWM | D6 |
| Left Motor DIR | D7 |
| Right Motor DIR | D8 |
| Right Motor PWM | D11 |

MDD10A 연결:

```text
STM32 D6  → MDD10A PWM1
STM32 D7  → MDD10A DIR1

STM32 D11 → MDD10A PWM2
STM32 D8  → MDD10A DIR2

STM32 GND → MDD10A GND
```

---

# 7. 엔코더 배선

모터는 6선이며 색 순서는 다음과 같습니다.

```text
빨강 / 검정 / 초록 / 파랑 / 노랑 / 흰색
```

현재 사용한 기능 구분:

| 색상 | 기능 |
|---|---|
| 빨강 | 모터 출력선 |
| 검정 | 모터 출력선 |
| 초록 | Encoder GND |
| 파랑 | Encoder VCC |
| 노랑 | Encoder A |
| 흰색 | Encoder B |

## 왼쪽 모터

```text
빨강 → MDD10A M1 출력
검정 → MDD10A M1 출력

초록 → GND
파랑 → Encoder VCC
노랑 → STM32 D4
흰색 → STM32 D5
```

## 오른쪽 모터

```text
빨강 → MDD10A M2 출력
검정 → MDD10A M2 출력

초록 → GND
파랑 → Encoder VCC
노랑 → STM32 D9
흰색 → STM32 D10
```

엔코더 STM32 핀:

```text
D4  = Left Encoder A
D5  = Left Encoder B

D9  = Right Encoder A
D10 = Right Encoder B
```

---

# 8. 엔코더 방향 설정

실제 테스트 결과:

- 오른쪽 엔코더: 전진 방향에서 **양수 증가**
- 왼쪽 엔코더: 전진 방향에서 **음수 증가**

따라서 펌웨어에서는 다음과 같이 설정합니다.

```cpp
constexpr int LEFT_ENCODER_SIGN  = -1;
constexpr int RIGHT_ENCODER_SIGN = +1;
```

이 설정 후 전진 시 좌/우 엔코더 값이 모두 양수 방향으로 증가해야 합니다.

---

# 9. STM32 테스트 명령

Raspberry Pi에서 STM32로 한 글자 명령을 보냅니다.

| 명령 | 기능 |
|---|---|
| `h` | Help |
| `l` | 왼쪽 모터 단독 테스트 |
| `r` | 오른쪽 모터 단독 테스트 |
| `b` | 양쪽 모터 동일 PWM |
| `p` | PID 보정 테스트 |
| `s` | 즉시 정지 |
| `z` | 정지 + 엔코더 카운트 초기화 |

테스트 순서 권장:

```text
h
l
r
b
p
```

`p`는 좌/우 엔코더 방향과 모터 방향 확인 후 실행합니다.

---

# 10. STM32 출력 데이터 형식

펌웨어에서 다음 CSV 형태로 데이터를 전송합니다.

```text
ms,mode,L,R,dL,dR,PWM_L,PWM_R,error
```

예:

```text
338400,r,1,202,0,49,0,80,-49.00
338600,r,3,416,0,51,0,80,-51.00
338800,r,7,641,0,57,0,80,-57.00
```

각 항목:

| 항목 | 의미 |
|---|---|
| `ms` | STM32 실행 시간 [ms] |
| `mode` | 현재 동작 모드 |
| `L` | 왼쪽 누적 엔코더 카운트 |
| `R` | 오른쪽 누적 엔코더 카운트 |
| `dL` | 제어 주기 동안 왼쪽 증가량 |
| `dR` | 제어 주기 동안 오른쪽 증가량 |
| `PWM_L` | 왼쪽 PWM |
| `PWM_R` | 오른쪽 PWM |
| `error` | 좌우 엔코더 차이 |

---

# 11. PID 기본 설정

초기 테스트는 P 제어만 사용합니다.

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

오차:

```text
error = Left Encoder Increment - Right Encoder Increment
```

양의 오차:

```text
왼쪽이 더 빠름
→ 왼쪽 PWM 감소
→ 오른쪽 PWM 증가
```

현재 단계에서는 **좌/우 바퀴 속도 동기화용 PID**입니다.

---

# 12. Raspberry Pi UART 확인

UART 장치 확인:

```bash
ls -l /dev/serial*
```

일반적으로:

```text
/dev/serial0
```

를 사용합니다.

Minicom 실행:

```bash
minicom -D /dev/serial0 -b 115200
```

종료:

```text
Ctrl + A
X
Enter
```

---

# 13. Python / PySerial 설치

Ubuntu에서:

```bash
sudo apt update
sudo apt install python3-pip -y
pip3 install pyserial
```

설치 확인:

```bash
python3 -c "import serial; print(serial.__version__)"
```

환경에 따라 pip 대신 다음 패키지를 사용할 수도 있습니다.

```bash
sudo apt install python3-serial
```

---

# 14. Python UART 실행

Python 프로그램 예시 파일명:

```text
uart_pid_test.py
```

실행:

```bash
python3 uart_pid_test.py
```

Python 프로그램은 다음 기능을 수행합니다.

- `/dev/serial0` 연결
- STM32 명령 송신
- 엔코더/PWM/error 데이터 수신
- 실시간 터미널 출력
- CSV 자동 저장

생성 CSV 예:

```text
pid_log_20261002_110500.csv
```

---

# 15. 현재 테스트 결과

## UART

STM32 → Pi:

```text
HELLO FROM STM32
```

연속 수신 성공.

Pi → STM32:

```text
l
r
b
```

명령 수신 및 동작 성공.

따라서 **UART 양방향 통신 정상**.

## Encoder

왼쪽 테스트 예:

```text
4200,l,8,0,...
4400,l,-80,0,...
4600,l,-165,0,...
4800,l,-262,0,...
```

왼쪽 엔코더 방향이 음수로 측정됨.

따라서:

```cpp
LEFT_ENCODER_SIGN = -1;
```

적용.

오른쪽 테스트:

```text
338400,r,1,202,...
338600,r,3,416,...
338800,r,7,641,...
339000,r,11,863,...
```

오른쪽 엔코더는 정상적으로 양수 증가.

---

# 16. 안전 사항

- 펌웨어 업로드 시 가능하면 **모터 배터리를 분리**
- 첫 모터 테스트는 **바퀴를 바닥에서 띄운 상태**에서 진행
- UART에는 5V를 연결하지 않음
- Pi와 STM32는 **GND 공통**
- 모터의 빨강/검정 선은 STM32 GND가 아니라 **MDD10A 모터 출력 단자**로 연결
- 배터리 극성 확인 후 MDD10A에 연결
- 엔코더 방향이 반대라고 모터 배선을 바로 바꾸지 말고 `ENCODER_SIGN`으로 먼저 보정 가능

---

# 17. 최종 핀 요약

```text
NUCLEO-F446RE

D4   → Left Encoder A
D5   → Left Encoder B

D6   → MDD10A PWM1
D7   → MDD10A DIR1

D8   → MDD10A DIR2

D9   → Right Encoder A
D10  → Right Encoder B

D11  → MDD10A PWM2

PC10 → Raspberry Pi RX
PC11 ← Raspberry Pi TX

GND  ↔ Raspberry Pi GND
GND  ↔ MDD10A GND
GND  ↔ Encoder GND
```

---

## 다음 작업

1. `LEFT_ENCODER_SIGN = -1` 적용
2. `l`, `r` 재검증
3. `b`로 동일 PWM에서 좌우 속도 차이 측정
4. `p`로 P 제어 테스트
5. CSV 로그 저장
6. 주행 거리 및 편차 분석
7. Kp 재튜닝
8. 필요 시 Ki / Kd 추가
