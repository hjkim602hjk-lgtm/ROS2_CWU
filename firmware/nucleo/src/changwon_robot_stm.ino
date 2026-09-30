// CWU autonomous wheel controller, protocol v1. Draft; bench calibration required.
// Pin assignment and motor polarity preserved from the 2026-09-29 bench firmware.
#include <Arduino.h>
#include "wheel_control.h"

const int L_DIR = D8, L_PWM = D9, R_DIR = D7, R_PWM = D6;
const int ENC_L_A = D5, ENC_L_B = D4, ENC_R_A = D10, ENC_R_B = D3;
volatile uint32_t encL = 0, encR = 0;
cwu::Controller control;
char tx[96] = "READY,1\n";
size_t tx_size = 8, tx_sent = 0;
uint32_t pending_ack = 0, last_report = 0;

void leftEncoderISR() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) ++encL;
  else --encL;
}
void rightEncoderISR() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) ++encR;
  else --encR;
}
void motor(int dir, int pin, int pwm) {
  // Remove PWM before changing polarity; left mounting is inverted.
  analogWrite(pin, 0);
  if (pwm) { digitalWrite(dir, pwm > 0 ? HIGH : LOW); analogWrite(pin, abs(pwm)); }
}
void applyOutputs() {
  static int previous_left = 999, previous_right = 999;
  if (previous_left != control.pwm[0]) {
    motor(L_DIR, L_PWM, -control.pwm[0]); previous_left = control.pwm[0];
  }
  if (previous_right != control.pwm[1]) {
    motor(R_DIR, R_PWM, control.pwm[1]); previous_right = control.pwm[1];
  }
}
void setup() {
  pinMode(L_DIR, OUTPUT); pinMode(L_PWM, OUTPUT);
  pinMode(R_DIR, OUTPUT); pinMode(R_PWM, OUTPUT);
  applyOutputs();
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, CHANGE);
  Serial.begin(115200);
}
void loop() {
  uint32_t now = millis();
  control.watchdog(now);
  applyOutputs();
  // Bounded receive work keeps floods from starving watchdog and PI updates.
  for (unsigned i = 0; i < 32 && Serial.available(); ++i) {
    int result = control.receive(char(Serial.read()), millis());
    if (result == 1) pending_ack = control.session;
    if (result == -1) pending_ack = 0;
    applyOutputs();
  }
  uint32_t left, right;
  noInterrupts(); left = encL; right = encR; interrupts();
  now = millis();
  control.update(now, left, right);
  applyOutputs();
  // Never block on serial output. ACK waits for the current complete frame;
  // telemetry can be dropped when the host stops reading.
  if (tx_sent == tx_size) {
    tx_sent = tx_size = 0;
    if (pending_ack && pending_ack == control.session) {
      tx_size = snprintf(tx, sizeof(tx), "ACK,%lu\n", (unsigned long)pending_ack);
      pending_ack = 0;
    } else if (uint32_t(now - last_report) >= 100) {
      last_report = now;
      const int64_t signed_left = left <= INT32_MAX ? int64_t(left) : int64_t(left) - 4294967296LL;
      const int64_t signed_right = right <= INT32_MAX ? int64_t(right) : int64_t(right) - 4294967296LL;
      tx_size = snprintf(tx, sizeof(tx), "ENC,%lu,%ld,%ld,%lu\n", (unsigned long)now,
                         (long)signed_left, (long)signed_right, (unsigned long)control.session);
    }
  }
  const int available = Serial.availableForWrite();
  if (available > 0 && tx_sent < tx_size) {
    size_t count = tx_size - tx_sent;
    if (count > size_t(available)) count = size_t(available);
    tx_sent += Serial.write((const uint8_t*)tx + tx_sent, count);
  }
}
