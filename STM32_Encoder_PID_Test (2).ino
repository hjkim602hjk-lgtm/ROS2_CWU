#include <Arduino.h>  // STM32duino GPIO, PWM, interrupts, UART
#include <math.h>     // lroundf(): float -> nearest integer

// NUCLEO-F446RE / STM32 core 3.0.0
// Upload with the motor battery disconnected. First tests: wheels off ground.
// Commands arrive from Raspberry Pi UART, NOT the laptop Serial Monitor.
Uart PiSerial(PC_11, PC_10);  // RX, TX; keep the working UART wiring

// Motor driver: MDD10A, PWM + DIR
constexpr uint32_t LEFT_PWM  = 6;   // D6  / PB10 -> PWM1
constexpr uint32_t LEFT_DIR  = 7;   // D7  / PA8  -> DIR1
// PA7_ALT3 selects non-complementary TIM14_CH1 on the SAME physical D11 pin.
constexpr uint32_t RIGHT_PWM = PA7_ALT3; // D11 / PA7 -> PWM2
constexpr uint32_t RIGHT_DIR = 8;   // D8  / PA9  -> DIR2

// Encoder: yellow=A, white=B (confirmed motor wiring)
constexpr uint32_t LEFT_A  = 4;   // D4
constexpr uint32_t LEFT_B  = 5;   // D5
constexpr uint32_t RIGHT_A = 9;   // D9
constexpr uint32_t RIGHT_B = 10;  // D10

// Calibrate these independently. Forward means ROBOT forward, not CW/CCW.
constexpr bool INVERT_LEFT_MOTOR  = false;
constexpr bool INVERT_RIGHT_MOTOR = false;
constexpr int LEFT_ENCODER_SIGN   = +1;
constexpr int RIGHT_ENCODER_SIGN  = +1;

// Initial BENCH settings, not the previous tuned gains.
constexpr int BASE_PWM = 80;       // 8-bit PWM: 0..255
constexpr int MAX_PWM = 130;       // software output cap for this test
constexpr float MAX_CORRECTION = 30.0f;
constexpr float KP = 0.50f;
constexpr float KI = 0.00f;         // initially P-only
constexpr float KD = 0.00f;
constexpr uint32_t RUN_MS = 1000;   // stop deadline, checked every loop
constexpr uint32_t CONTROL_MS = 50;
constexpr uint32_t PRINT_MS = 200;
constexpr uint32_t NO_PULSE_MS = 350;

static_assert(BASE_PWM > MAX_CORRECTION &&
              BASE_PWM + MAX_CORRECTION <= MAX_PWM && MAX_PWM <= 255,
              "Invalid PWM limits");
static_assert((LEFT_ENCODER_SIGN == 1 || LEFT_ENCODER_SIGN == -1) &&
              (RIGHT_ENCODER_SIGN == 1 || RIGHT_ENCODER_SIGN == -1),
              "Encoder signs must be +1 or -1");

volatile int32_t rawLeft = 0, rawRight = 0;
int32_t countLeft = 0, countRight = 0, deltaLeft = 0, deltaRight = 0;
int pwmLeft = 0, pwmRight = 0;
char mode = 's';  // s=stop, l=left, r=right, b=both/open-loop, p=PID
uint32_t startedAt = 0, sampledAt = 0, printedAt = 0;
uint32_t lastLeftPulse = 0, lastRightPulse = 0;
float error = 0.0f, integral = 0.0f, previousError = 0.0f;
bool havePIDSample = false;

// x1 decoding: count A rising edges; B determines the direction.
void leftISR() {
  rawLeft += (digitalRead(LEFT_B) == HIGH) ? 1 : -1;
}
void rightISR() {
  rawRight += (digitalRead(RIGHT_B) == HIGH) ? 1 : -1;
}

float limitFloat(float value, float lo, float hi) {
  return value < lo ? lo : (value > hi ? hi : value);
}

// Test commands only request forward duty; invert flags select its polarity.
void setMotors(int left, int right) {
  pwmLeft = constrain(left, 0, MAX_PWM);
  pwmRight = constrain(right, 0, MAX_PWM);
  digitalWrite(LEFT_DIR, INVERT_LEFT_MOTOR ? LOW : HIGH);
  digitalWrite(RIGHT_DIR, INVERT_RIGHT_MOTOR ? LOW : HIGH);
  analogWrite(LEFT_PWM, pwmLeft);
  analogWrite(RIGHT_PWM, pwmRight);
}

void stopMotors(const char *reason) {
  setMotors(0, 0);  // remove PWM before printing anything
  mode = 's';
  integral = 0.0f;
  havePIDSample = false;
  PiSerial.print("# STOP: ");
  PiSerial.println(reason);
}

void resetMeasurements() {
  noInterrupts();
  rawLeft = 0;
  rawRight = 0;
  interrupts();
  countLeft = countRight = deltaLeft = deltaRight = 0;
  error = integral = previousError = 0.0f;
  havePIDSample = false;
  sampledAt = millis();
  lastLeftPulse = lastRightPulse = sampledAt;
}

void printHelp() {
  PiSerial.println("# READY: l=left r=right b=both p=PID s=stop z=zero h=help");
  PiSerial.println("# Each run ~1000ms. Check motor directions and encoder signs first.");
  PiSerial.println("ms,mode,L,R,dL,dR,PWM_L,PWM_R,error");
}

void startTest(char command) {
  // Do not extend an active test or change its mode mid-run.
  if (mode != 's') return;
  resetMeasurements();
  mode = command;
  startedAt = millis();
  setMotors(command == 'r' ? 0 : BASE_PWM,
            command == 'l' ? 0 : BASE_PWM);
  PiSerial.print("# START: ");
  PiSerial.println(command);
}

void readCommands() {
  // Bound work per loop so an input flood cannot starve the stop timer.
  for (int n = 0; n < 16 && PiSerial.available(); ++n) {
    char c = static_cast<char>(PiSerial.read());
    if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
    if (c == 's' || c == ' ') {
      stopMotors("COMMAND");
      // Do not execute queued motion commands after an emergency stop.
      while (PiSerial.available()) PiSerial.read();
      return;
    }
    if (c == 'z') {
      stopMotors("ZERO");
      resetMeasurements();
      while (PiSerial.available()) PiSerial.read();
      return;
    }
    if (mode != 's') continue;
    if (c == 'h') printHelp();
    else if (c == 'l' || c == 'r' || c == 'b' || c == 'p') startTest(c);
    // Ignore Enter/newline and any unknown character.
  }
}

void sampleAndControl(uint32_t now) {
  const uint32_t elapsed = now - sampledAt;
  if (elapsed < CONTROL_MS) return;
  sampledAt = now;

  // Snapshot both counters together; never print from an ISR.
  int32_t left, right;
  noInterrupts();
  left = rawLeft;
  right = rawRight;
  interrupts();
  left *= LEFT_ENCODER_SIGN;
  right *= RIGHT_ENCODER_SIGN;
  deltaLeft = left - countLeft;
  deltaRight = right - countRight;
  countLeft = left;
  countRight = right;
  if (deltaLeft != 0) lastLeftPulse = now;
  if (deltaRight != 0) lastRightPulse = now;

  // Error is the signed pulse-increment difference, normalized to 50ms.
  error = (static_cast<float>(deltaLeft) - deltaRight) * CONTROL_MS / elapsed;
  if (mode == 's') return;
  if (elapsed > 250) {
    stopMotors("CONTROL_DELAY");
    return;
  }
  if ((pwmLeft > 0 && now - lastLeftPulse >= NO_PULSE_MS) ||
      (pwmRight > 0 && now - lastRightPulse >= NO_PULSE_MS)) {
    stopMotors("NO_ENCODER_PULSE");
    return;
  }
  if (mode != 'p') return;
  if (countLeft < -2 || countRight < -2) {
    stopMotors("CHECK_ENCODER_SIGN");
    return;
  }
  // Initially open-loop; wait for positive counts on BOTH encoders.
  if (countLeft < 3 || countRight < 3) return;

  const float dt = elapsed * 0.001f;
  const float derivative = havePIDSample ? (error - previousError) / dt : 0.0f;
  const float candidateI = KI == 0.0f ? 0.0f :
      limitFloat(integral + error * dt, -2000.0f, 2000.0f);
  const float requested = KP * error + KI * candidateI + KD * derivative;
  // Conditional integration: do not wind up farther into output saturation.
  if ((requested >= -MAX_CORRECTION && requested <= MAX_CORRECTION) ||
      (requested > MAX_CORRECTION && error < 0) ||
      (requested < -MAX_CORRECTION && error > 0)) integral = candidateI;
  const int correction = static_cast<int>(lroundf(
      limitFloat(requested, -MAX_CORRECTION, MAX_CORRECTION)));
  previousError = error;
  havePIDSample = true;
  // Positive error: left is faster -> slow left, speed up right.
  setMotors(BASE_PWM - correction, BASE_PWM + correction);
}

void printTelemetry(uint32_t now) {
  if (now - printedAt < PRINT_MS) return;
  printedAt = now;
  PiSerial.print(now);        PiSerial.print(',');
  PiSerial.print(mode);       PiSerial.print(',');
  PiSerial.print(countLeft);  PiSerial.print(',');
  PiSerial.print(countRight); PiSerial.print(',');
  PiSerial.print(deltaLeft);  PiSerial.print(',');
  PiSerial.print(deltaRight); PiSerial.print(',');
  PiSerial.print(pwmLeft);    PiSerial.print(',');
  PiSerial.print(pwmRight);   PiSerial.print(',');
  PiSerial.println(error, 2);
}

void setup() {
  // No motor commands at startup. Hardware power must still be isolated
  // during upload/reset/wiring; software cannot guarantee reset-time levels.
  pinMode(LEFT_PWM, OUTPUT);  digitalWrite(LEFT_PWM, LOW);
  pinMode(RIGHT_PWM, OUTPUT); digitalWrite(RIGHT_PWM, LOW);
  pinMode(LEFT_DIR, OUTPUT);  digitalWrite(LEFT_DIR, LOW);
  pinMode(RIGHT_DIR, OUTPUT); digitalWrite(RIGHT_DIR, LOW);
  analogWriteResolution(8);
  analogWriteFrequency(10000);  // 10kHz, below MDD10A's 20kHz limit
  setMotors(0, 0);

  pinMode(LEFT_A, INPUT_PULLUP);
  pinMode(LEFT_B, INPUT_PULLUP);
  pinMode(RIGHT_A, INPUT_PULLUP);
  pinMode(RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), rightISR, RISING);
  resetMeasurements();
  PiSerial.begin(115200);
  printHelp();
}

void loop() {
  uint32_t now = millis();
  if (mode != 's' && now - startedAt >= RUN_MS) stopMotors("TIME_LIMIT");
  readCommands();
  now = millis();
  if (mode != 's' && now - startedAt >= RUN_MS) stopMotors("TIME_LIMIT");
  sampleAndControl(now);
  printTelemetry(now);
}
