#include "../src/wheel_control.h"
#include <cassert>
#include <cmath>
#include <cstdio>
using cwu::Controller;
static int send(Controller& c, const char* s, uint32_t now) {
  int result = 0;
  for (; *s; ++s) { int r = c.receive(*s, now); if (r) result = r; }
  return result;
}
static void configure(Controller& c, uint32_t now = 0) {
  assert(send(c, "CFG,42,1000,1,2,1,2,80,1000,300,1,-1\n", now) == 1);
  assert(c.session == 42 && c.pwm[0] == 0);
}
int main() {
  Controller c;
  assert(send(c, "VEL,42,100,100\n", 0) == -1 && c.session == 0);
  configure(c);
  send(c, "VEL,42,100,100\n", 0);
  c.update(20, 0, 0);
  assert(c.pwm[0] > 0 && c.pwm[0] <= 21); // acceleration bound
  c.update(40, 1, -1);
  assert(c.pwm[0] == c.pwm[1]); // encoder signs
  for (uint32_t t = 60; t <= 280; t += 20) {
    send(c, "VEL,42,1000,1000\n", t);
    c.update(t, 1, -1);
    assert(std::abs(c.pwm[0]) <= 80 && std::abs(c.integral[0]) <= 80);
  }
  send(c, "VEL,42,-100,-100\n", 281);
  assert(c.integral[0] == 0 && c.pwm[0] == 0);
  c.update(300, 1, -1);
  assert(c.pwm[0] < 0);
  send(c, "VEL,42,0,0\n", 300);
  assert(c.pwm[0] == 0 && c.pwm[1] == 0 && c.integral[0] == 0);
  send(c, "STOP\n", 301);
  assert(c.pwm[0] == 0 && c.integral[0] == 0);
  send(c, "VEL,42,100,100\n", 302);
  c.update(602, 1, -1);
  assert(c.session == 0 && c.pwm[0] == 0); // watchdog disarms
  const char* bad[] = {"VEL,42,nan,0\n", "VEL,42,inf,0\n", "VEL,42,1001,0\n",
    "VEL,42,1x,0\n", "VEL,43,1,0\n", "VEL,42,,0\n", "VEL,42,1,0,\n", "f\n",
    "CFG,42,1000,1,2,1,2,80,1000,301,1,-1\n", "CFG,42,1000,1,2,1,2,256,1000,300,1,-1\n"};
  for (const auto* s : bad) {
    configure(c, 1000);
    assert(send(c, s, 1001) == -1 && c.session == 0 && c.pwm[0] == 0);
  }
  configure(c, 2000);
  send(c, "VEL,42,100,100\n", 2001);
  assert(send(c, "CFG,43,1000,1,2,1,2,80,1000,300,1,-1\n", 2002) == -1);
  configure(c, 3000);
  send(c, "VEL,42,100,100\n", 3001);
  send(c, "VEL,42,1", 3100); // truncated packet cannot refresh watchdog
  c.update(3301, 0, 0);
  assert(c.session == 0 && c.pwm[0] == 0);
  Controller late;
  configure(late);
  send(late, "VEL,42,100,100\n", 1);
  assert(send(late, "VEL,42,100,100\n", 301) == -1 && late.session == 0);
  Controller overflow;
  configure(overflow);
  for (int i = 0; i < 300; ++i) overflow.receive('x', 10);
  assert(overflow.session == 0);
  send(overflow, "VEL,42,1,1\n", 11);
  assert(overflow.session == 0);
  Controller rollover;
  configure(rollover, UINT32_MAX - 100);
  send(rollover, "VEL,42,100,100\n", UINT32_MAX - 100);
  rollover.update(200, 0, 0);
  assert(rollover.session == 0);
  puts("firmware parser / PI / watchdog checks passed");
}
