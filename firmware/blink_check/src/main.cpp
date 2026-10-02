// Link check: LED blinks 1 Hz; "BLINK,n" goes out on UART4 (A0/A1 -> ttyAMA0)
// and ST-Link VCP (USART2 -> ttyACM0). Bytes received on either port are echoed back.
#include <Arduino.h>
Uart vcp(PA_3, PA_2);
uint32_t n = 0, last = 0;
void echo(Stream& s, const char* tag) {
  while (s.available()) { s.print(tag); s.println(char(s.read())); }
}
void setup() {
  // Motor PWM/DIR low so the drivers stay off.
  for (int p : {D6, D7, D8, D9}) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200); vcp.begin(115200);
}
void loop() {
  if (millis() - last >= 500) {
    last = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    ++n; Serial.print("BLINK,"); Serial.println(n); vcp.print("BLINK,"); vcp.println(n);
  }
  echo(Serial, "ECHO_UART4,"); echo(vcp, "ECHO_VCP,");
}
