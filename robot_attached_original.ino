#include <Arduino.h>
#include <Servo.h>

// ============================================================
// Raspberry Pi UART
// STM32 PC11 = RX
// STM32 PC10 = TX
// ============================================================

Uart PiSerial(PC_11, PC_10);

// ============================================================
// PIN MAP
// ============================================================

// ---------- Servo ----------
constexpr uint32_t SERVO_LEFT_PIN  = D2;
constexpr uint32_t SERVO_RIGHT_PIN = D3;

// ---------- Encoder ----------
constexpr uint32_t LEFT_ENC_A  = D4;
constexpr uint32_t LEFT_ENC_B  = D5;

constexpr uint32_t RIGHT_ENC_A = D9;
constexpr uint32_t RIGHT_ENC_B = D10;

// ---------- Motor Driver : Cytron MDD10A ----------
constexpr uint32_t LEFT_PWM  = D6;
constexpr uint32_t LEFT_DIR  = D7;

constexpr uint32_t RIGHT_DIR = D8;

// Physical pin = D11 / PA7
// Previous STM32duino setup used PA7_ALT3 successfully.
constexpr uint32_t RIGHT_PWM = PA7_ALT3;

// ---------- PSD ----------
constexpr uint32_t PSD_FL = A0;   // Front Left
constexpr uint32_t PSD_FR = A1;   // Front Right
constexpr uint32_t PSD_R  = A2;   // Right
constexpr uint32_t PSD_L  = A3;   // Left

// ============================================================
// SETTINGS
// ============================================================

// Motor direction
// If a wheel rotates backward when FORWARD is commanded,
// change only that wheel to true.
constexpr bool INVERT_LEFT_MOTOR  = false;
constexpr bool INVERT_RIGHT_MOTOR = false;

// Encoder direction correction
// From previous real test:
// left encoder was negative during forward motion.
constexpr int LEFT_ENCODER_SIGN  = -1;
constexpr int RIGHT_ENCODER_SIGN = +1;

// ---------- Motor PWM ----------
constexpr int BASE_PWM = 80;
constexpr int MAX_PWM  = 130;

constexpr int TURN_PWM = 90;

// ---------- PID ----------
float Kp = 0.50f;
float Ki = 0.00f;
float Kd = 0.00f;

constexpr float MAX_CORRECTION = 30.0f;

constexpr uint32_t CONTROL_MS = 50;

// ---------- PSD ----------
// Existing test threshold.
// Must be calibrated again on actual robot.
int PSD_THRESHOLD = 550;

// ---------- Test duration ----------
constexpr uint32_t MOTOR_TEST_MS = 1000;

// ---------- Telemetry ----------
constexpr uint32_t TELEMETRY_MS = 200;

// ============================================================
// Servo
// ============================================================

Servo servoLeft;
Servo servoRight;

int servoLeftAngle  = 90;
int servoRightAngle = 90;

// ============================================================
// Encoder
// ============================================================

volatile int32_t leftEncoderCount  = 0;
volatile int32_t rightEncoderCount = 0;

// ============================================================
// Runtime state
// ============================================================

enum RunMode
{
  MODE_STOP,
  MODE_LEFT_TEST,
  MODE_RIGHT_TEST,
  MODE_BOTH_TEST,
  MODE_PID_TEST,
  MODE_PID_CONTINUOUS,
  MODE_AUTO_PSD
};

RunMode currentMode = MODE_STOP;

uint32_t modeStartTime = 0;
uint32_t lastControlTime = 0;
uint32_t lastTelemetryTime = 0;

int currentLeftPWM  = 0;
int currentRightPWM = 0;

bool telemetryStream = false;

// PID state
float pidIntegral = 0.0f;
float previousError = 0.0f;

int32_t previousLeftCount  = 0;
int32_t previousRightCount = 0;

int zeroPulseLeftCycles  = 0;
int zeroPulseRightCycles = 0;

// ============================================================
// ENCODER ISR
// ============================================================

void leftEncoderISR()
{
  if (digitalRead(LEFT_ENC_B))
  {
    leftEncoderCount += LEFT_ENCODER_SIGN;
  }
  else
  {
    leftEncoderCount -= LEFT_ENCODER_SIGN;
  }
}

void rightEncoderISR()
{
  if (digitalRead(RIGHT_ENC_B))
  {
    rightEncoderCount += RIGHT_ENCODER_SIGN;
  }
  else
  {
    rightEncoderCount -= RIGHT_ENCODER_SIGN;
  }
}

// ============================================================
// MOTOR FUNCTIONS
// ============================================================

void driveOneMotor(
    uint32_t pwmPin,
    uint32_t dirPin,
    int command,
    bool invertMotor)
{
  command = constrain(command, -255, 255);

  bool forward = (command >= 0);

  if (invertMotor)
  {
    forward = !forward;
  }

  digitalWrite(dirPin, forward ? HIGH : LOW);

  analogWrite(pwmPin, abs(command));
}

void setMotors(int leftPWM, int rightPWM)
{
  leftPWM  = constrain(leftPWM, -MAX_PWM, MAX_PWM);
  rightPWM = constrain(rightPWM, -MAX_PWM, MAX_PWM);

  currentLeftPWM  = leftPWM;
  currentRightPWM = rightPWM;

  driveOneMotor(
      LEFT_PWM,
      LEFT_DIR,
      leftPWM,
      INVERT_LEFT_MOTOR);

  driveOneMotor(
      RIGHT_PWM,
      RIGHT_DIR,
      rightPWM,
      INVERT_RIGHT_MOTOR);
}

void stopMotors()
{
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  currentLeftPWM = 0;
  currentRightPWM = 0;
}

// ============================================================
// PSD
// ============================================================

int readPSD_FL()
{
  return analogRead(PSD_FL);
}

int readPSD_FR()
{
  return analogRead(PSD_FR);
}

int readPSD_R()
{
  return analogRead(PSD_R);
}

int readPSD_L()
{
  return analogRead(PSD_L);
}

bool frontObstacle()
{
  return (readPSD_FL() >= PSD_THRESHOLD ||
          readPSD_FR() >= PSD_THRESHOLD);
}

bool anyObstacle()
{
  return (
      readPSD_FL() >= PSD_THRESHOLD ||
      readPSD_FR() >= PSD_THRESHOLD ||
      readPSD_R() >= PSD_THRESHOLD ||
      readPSD_L() >= PSD_THRESHOLD);
}

// ============================================================
// PID
// ============================================================

void resetPID()
{
  pidIntegral = 0.0f;
  previousError = 0.0f;

  noInterrupts();

  previousLeftCount  = leftEncoderCount;
  previousRightCount = rightEncoderCount;

  interrupts();

  zeroPulseLeftCycles  = 0;
  zeroPulseRightCycles = 0;

  lastControlTime = millis();
}

void runPID()
{
  uint32_t now = millis();

  uint32_t dt_ms = now - lastControlTime;

  if (dt_ms < CONTROL_MS)
  {
    return;
  }

  int32_t leftNow;
  int32_t rightNow;

  noInterrupts();

  leftNow  = leftEncoderCount;
  rightNow = rightEncoderCount;

  interrupts();

  int32_t dL = leftNow - previousLeftCount;
  int32_t dR = rightNow - previousRightCount;

  previousLeftCount  = leftNow;
  previousRightCount = rightNow;

  lastControlTime = now;

  // Normalize encoder difference to CONTROL_MS.
  float error =
      ((float)(dL - dR) * CONTROL_MS) /
      (float)dt_ms;

  float dt = dt_ms / 1000.0f;

  pidIntegral += error * dt;

  // Simple anti-windup
  pidIntegral = constrain(pidIntegral, -100.0f, 100.0f);

  float derivative =
      (error - previousError) / dt;

  previousError = error;

  float correction =
      Kp * error +
      Ki * pidIntegral +
      Kd * derivative;

  correction =
      constrain(
          correction,
          -MAX_CORRECTION,
          MAX_CORRECTION);

  // error > 0
  // = left wheel faster
  // -> reduce left / increase right
  int pwmLeft =
      BASE_PWM - (int)correction;

  int pwmRight =
      BASE_PWM + (int)correction;

  pwmLeft =
      constrain(pwmLeft, 0, MAX_PWM);

  pwmRight =
      constrain(pwmRight, 0, MAX_PWM);

  setMotors(pwmLeft, pwmRight);

  // Encoder fail safety
  if (abs(dL) == 0)
    zeroPulseLeftCycles++;
  else
    zeroPulseLeftCycles = 0;

  if (abs(dR) == 0)
    zeroPulseRightCycles++;
  else
    zeroPulseRightCycles = 0;

  // About 350 ms with CONTROL_MS = 50 ms
  if (zeroPulseLeftCycles >= 7 ||
      zeroPulseRightCycles >= 7)
  {
    stopMotors();

    currentMode = MODE_STOP;

    PiSerial.println("# STOP: ENCODER_NO_PULSE");
    Serial.println("# STOP: ENCODER_NO_PULSE");
  }
}

// ============================================================
// SIMPLE PSD AUTO TEST
// ============================================================

void runPSDAuto()
{
  int fl = readPSD_FL();
  int fr = readPSD_FR();
  int left = readPSD_L();
  int right = readPSD_R();

  bool frontBlocked =
      fl >= PSD_THRESHOLD ||
      fr >= PSD_THRESHOLD;

  bool leftBlocked =
      left >= PSD_THRESHOLD;

  bool rightBlocked =
      right >= PSD_THRESHOLD;

  // Clear -> forward
  if (!frontBlocked &&
      !leftBlocked &&
      !rightBlocked)
  {
    setMotors(BASE_PWM, BASE_PWM);
    return;
  }

  // Front obstacle:
  // choose direction with more free space.
  // Smaller PSD reading = generally farther away
  // in the usable GP2Y0A41 range.
  if (frontBlocked)
  {
    if (left < right)
    {
      // More space on left -> turn left
      setMotors(-TURN_PWM, TURN_PWM);
    }
    else
    {
      // More space on right -> turn right
      setMotors(TURN_PWM, -TURN_PWM);
    }

    return;
  }

  // Obstacle on left -> turn right
  if (leftBlocked)
  {
    setMotors(TURN_PWM, -TURN_PWM);
    return;
  }

  // Obstacle on right -> turn left
  if (rightBlocked)
  {
    setMotors(-TURN_PWM, TURN_PWM);
    return;
  }

  stopMotors();
}

// ============================================================
// TELEMETRY
// ============================================================

void printTelemetry(Stream &out)
{
  int32_t l;
  int32_t r;

  noInterrupts();

  l = leftEncoderCount;
  r = rightEncoderCount;

  interrupts();

  int fl = readPSD_FL();
  int fr = readPSD_FR();
  int sideR = readPSD_R();
  int sideL = readPSD_L();

  out.print(millis());
  out.print(",");

  out.print((int)currentMode);
  out.print(",");

  out.print(l);
  out.print(",");

  out.print(r);
  out.print(",");

  out.print(currentLeftPWM);
  out.print(",");

  out.print(currentRightPWM);
  out.print(",");

  out.print(fl);
  out.print(",");

  out.print(fr);
  out.print(",");

  out.print(sideL);
  out.print(",");

  out.print(sideR);
  out.print(",");

  out.print(servoLeftAngle);
  out.print(",");

  out.println(servoRightAngle);
}

void printHeader(Stream &out)
{
  out.println(
      "ms,mode,L,R,PWM_L,PWM_R,"
      "PSD_FL,PSD_FR,PSD_L,PSD_R,"
      "SERVO_L,SERVO_R");
}

// ============================================================
// STATUS
// ============================================================

void printStatus(Stream &out)
{
  int32_t l;
  int32_t r;

  noInterrupts();

  l = leftEncoderCount;
  r = rightEncoderCount;

  interrupts();

  out.println();
  out.println("========== ROBOT STATUS ==========");

  out.print("Encoder L : ");
  out.println(l);

  out.print("Encoder R : ");
  out.println(r);

  out.print("PSD FL : ");
  out.println(readPSD_FL());

  out.print("PSD FR : ");
  out.println(readPSD_FR());

  out.print("PSD L  : ");
  out.println(readPSD_L());

  out.print("PSD R  : ");
  out.println(readPSD_R());

  out.print("Servo L: ");
  out.println(servoLeftAngle);

  out.print("Servo R: ");
  out.println(servoRightAngle);

  out.print("PWM L  : ");
  out.println(currentLeftPWM);

  out.print("PWM R  : ");
  out.println(currentRightPWM);

  out.print("Kp : ");
  out.println(Kp);

  out.print("Ki : ");
  out.println(Ki);

  out.print("Kd : ");
  out.println(Kd);

  out.print("PSD Threshold : ");
  out.println(PSD_THRESHOLD);

  out.println("==================================");
}

// ============================================================
// HELP
// ============================================================

void printHelp(Stream &out)
{
  out.println();
  out.println("===== STM32 ROBOT TEST =====");

  out.println("h : help");
  out.println("s : sensor/status snapshot");
  out.println("t : telemetry stream ON/OFF");

  out.println();
  out.println("--- MOTOR ---");

  out.println("l : left motor 1 sec");
  out.println("r : right motor 1 sec");
  out.println("b : both motors 1 sec open loop");

  out.println("p : PID straight 1 sec");
  out.println("P : PID straight continuous");

  out.println("a : PSD auto avoidance");
  out.println("q : STOP");

  out.println();
  out.println("--- ENCODER ---");

  out.println("z : reset encoder counts");

  out.println();
  out.println("--- SERVO LEFT D2 ---");

  out.println("1 : Left servo 30 deg");
  out.println("2 : Left servo 90 deg");
  out.println("3 : Left servo 150 deg");

  out.println();
  out.println("--- SERVO RIGHT D3 ---");

  out.println("4 : Right servo 30 deg");
  out.println("5 : Right servo 90 deg");
  out.println("6 : Right servo 150 deg");

  out.println();
  out.println("7 : Both servos center");

  out.println("============================");
}

// ============================================================
// COMMAND
// ============================================================

void startMode(RunMode mode)
{
  stopMotors();

  currentMode = mode;

  modeStartTime = millis();

  resetPID();
}

void processCommand(char c)
{
  // Ignore line endings
  if (c == '\n' || c == '\r')
    return;

  switch (c)
  {
    // ---------------- HELP ----------------
    case 'h':
      printHelp(PiSerial);
      printHelp(Serial);
      break;

    // ---------------- STATUS ----------------
    case 's':
      printStatus(PiSerial);
      printStatus(Serial);
      break;

    // ---------------- STREAM ----------------
    case 't':
      telemetryStream = !telemetryStream;

      PiSerial.print("Telemetry = ");
      PiSerial.println(
          telemetryStream ? "ON" : "OFF");

      Serial.print("Telemetry = ");
      Serial.println(
          telemetryStream ? "ON" : "OFF");

      break;

    // ---------------- MOTOR ----------------
    case 'l':
      startMode(MODE_LEFT_TEST);
      break;

    case 'r':
      startMode(MODE_RIGHT_TEST);
      break;

    case 'b':
      startMode(MODE_BOTH_TEST);
      break;

    // ---------------- PID ----------------
    case 'p':
      startMode(MODE_PID_TEST);
      break;

    case 'P':
      startMode(MODE_PID_CONTINUOUS);
      break;

    // ---------------- PSD AUTO ----------------
    case 'a':
      startMode(MODE_AUTO_PSD);
      break;

    // ---------------- STOP ----------------
    case 'q':
    case ' ':
      stopMotors();
      currentMode = MODE_STOP;

      PiSerial.println("# STOP");
      Serial.println("# STOP");

      break;

    // ---------------- RESET ENCODER ----------------
    case 'z':

      noInterrupts();

      leftEncoderCount = 0;
      rightEncoderCount = 0;

      interrupts();

      PiSerial.println("# ENCODERS RESET");
      Serial.println("# ENCODERS RESET");

      break;

    // ==================================================
    // SERVO LEFT
    // ==================================================

    case '1':
      servoLeftAngle = 30;
      servoLeft.write(servoLeftAngle);
      break;

    case '2':
      servoLeftAngle = 90;
      servoLeft.write(servoLeftAngle);
      break;

    case '3':
      servoLeftAngle = 150;
      servoLeft.write(servoLeftAngle);
      break;

    // ==================================================
    // SERVO RIGHT
    // ==================================================

    case '4':
      servoRightAngle = 30;
      servoRight.write(servoRightAngle);
      break;

    case '5':
      servoRightAngle = 90;
      servoRight.write(servoRightAngle);
      break;

    case '6':
      servoRightAngle = 150;
      servoRight.write(servoRightAngle);
      break;

    // ==================================================
    // BOTH CENTER
    // ==================================================

    case '7':

      servoLeftAngle = 90;
      servoRightAngle = 90;

      servoLeft.write(servoLeftAngle);
      servoRight.write(servoRightAngle);

      break;
  }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  // ---------- Communication ----------
  Serial.begin(115200);
  PiSerial.begin(115200);

  // ---------- ADC ----------
  // Keep values in same 0~1023 scale
  // as previous PSD threshold tests.
  analogReadResolution(10);

  // ---------- PWM ----------
  analogWriteResolution(8);

  // ---------- Motor ----------
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);

  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  stopMotors();

  // ---------- Encoder ----------
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);

  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(
      digitalPinToInterrupt(LEFT_ENC_A),
      leftEncoderISR,
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(RIGHT_ENC_A),
      rightEncoderISR,
      RISING);

  // ---------- Servo ----------
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);

  servoLeft.write(90);
  servoRight.write(90);

  delay(500);

  // ---------- Startup ----------
  printHeader(PiSerial);

  PiSerial.println("# ROBOT READY");
  PiSerial.println("# Send h for help");

  Serial.println("# ROBOT READY");
  Serial.println("# Send h for help");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
  // ========================================================
  // COMMAND INPUT
  // ========================================================

  while (PiSerial.available())
  {
    processCommand(PiSerial.read());
  }

  while (Serial.available())
  {
    processCommand(Serial.read());
  }

  // ========================================================
  // MODE CONTROL
  // ========================================================

  switch (currentMode)
  {
    // ------------------------------------------------------
    case MODE_STOP:
      break;

    // ------------------------------------------------------
    case MODE_LEFT_TEST:

      setMotors(BASE_PWM, 0);

      if (millis() - modeStartTime >= MOTOR_TEST_MS)
      {
        stopMotors();
        currentMode = MODE_STOP;

        PiSerial.println("# LEFT MOTOR TEST DONE");
        Serial.println("# LEFT MOTOR TEST DONE");
      }

      break;

    // ------------------------------------------------------
    case MODE_RIGHT_TEST:

      setMotors(0, BASE_PWM);

      if (millis() - modeStartTime >= MOTOR_TEST_MS)
      {
        stopMotors();
        currentMode = MODE_STOP;

        PiSerial.println("# RIGHT MOTOR TEST DONE");
        Serial.println("# RIGHT MOTOR TEST DONE");
      }

      break;

    // ------------------------------------------------------
    case MODE_BOTH_TEST:

      setMotors(BASE_PWM, BASE_PWM);

      if (millis() - modeStartTime >= MOTOR_TEST_MS)
      {
        stopMotors();
        currentMode = MODE_STOP;

        PiSerial.println("# BOTH MOTOR TEST DONE");
        Serial.println("# BOTH MOTOR TEST DONE");
      }

      break;

    // ------------------------------------------------------
    case MODE_PID_TEST:

      // Emergency obstacle stop
      if (frontObstacle())
      {
        stopMotors();
        currentMode = MODE_STOP;

        PiSerial.println("# STOP: PSD FRONT OBSTACLE");
        Serial.println("# STOP: PSD FRONT OBSTACLE");

        break;
      }

      runPID();

      if (millis() - modeStartTime >= MOTOR_TEST_MS)
      {
        stopMotors();
        currentMode = MODE_STOP;

        PiSerial.println("# PID TEST DONE");
        Serial.println("# PID TEST DONE");
      }

      break;

    // ------------------------------------------------------
    case MODE_PID_CONTINUOUS:

      if (frontObstacle())
      {
        stopMotors();
        currentMode = MODE_STOP;

        PiSerial.println("# STOP: PSD FRONT OBSTACLE");
        Serial.println("# STOP: PSD FRONT OBSTACLE");

        break;
      }

      runPID();

      break;

    // ------------------------------------------------------
    case MODE_AUTO_PSD:

      runPSDAuto();

      break;
  }

  // ========================================================
  // TELEMETRY
  // ========================================================

  if (
      telemetryStream ||
      currentMode != MODE_STOP)
  {
    if (
        millis() - lastTelemetryTime >=
        TELEMETRY_MS)
    {
      lastTelemetryTime = millis();

      printTelemetry(PiSerial);
      printTelemetry(Serial);
    }
  }
}
