#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

namespace cwu {
inline float clamp(float x, float bound) { return fmaxf(-bound, fminf(bound, x)); }
// Shared with the native check: no Arduino calls or heap allocation.
struct Controller {
  uint32_t session = 0;
  int pwm[2] = {0, 0};
  float integral[2] = {0, 0};
  float target[2] = {0, 0}, ramp[2] = {0, 0};
  float kp[2] = {}, ki[2] = {}, max_ticks = 0, accel = 0, limit = 0;
  int sign[2] = {1, 1};
  uint32_t timeout = 300, last_command = 0, last_step = 0;
  uint32_t previous[2] = {};
  bool active = false;
  char line[192] = {};
  unsigned used = 0;
  bool discard = false;

  void stop(bool disarm = false) {
    for (unsigned i = 0; i < 2; ++i) pwm[i] = integral[i] = target[i] = ramp[i] = 0;
    active = false;
    if (disarm) session = 0;
  }
  void watchdog(uint32_t now) {
    if (active && uint32_t(now - last_command) >= timeout) stop(true);
  }
  // Return 1 = configuration accepted; -1 = fault/disarmed; 0 = no ACK.
  int receive(char ch, uint32_t now) {
    watchdog(now); // A late packet must not renew an already expired command.
    if (ch == '\n') {
      if (discard) { discard = false; used = 0; return -1; }
      line[used] = 0;
      used = 0;
      return command(now);
    }
    if (discard) return 0;
    if (ch < 32 || ch > 126 || used >= sizeof(line) - 1) {
      discard = true; used = 0; stop(true); return -1;
    }
    line[used++] = ch;
    return 0;
  }
  static bool number(const char* s, double& v) {
    if (!*s) return false;
    // Decimal ASCII only: reject whitespace, hex floats and nonfinite values.
    for (const char* p = s; *p; ++p)
      if (!strchr("0123456789+-.eE", *p)) return false;
    char* end;
    errno = 0;
    v = strtod(s, &end);
    return !errno && *end == 0 && isfinite(v);
  }
  int command(uint32_t now) {
    if (strcmp(line, "STOP") == 0) { stop(); return 0; }
    char* field[12];
    unsigned n = 1;
    field[0] = line;
    for (char* p = line; *p; ++p) {
      if (*p == ',') {
        if (n == 12) { stop(true); return -1; }
        *p = 0; field[n++] = p + 1;
      }
    }
    double v[11] = {};
    for (unsigned i = 1; i < n; ++i)
      if (!number(field[i], v[i-1])) { stop(true); return -1; }
    const bool valid_session = v[0] >= 1 && v[0] <= UINT32_MAX && floor(v[0]) == v[0];
    if (strcmp(field[0], "CFG") == 0 && n == 12 && valid_session && !active &&
        v[1] > 0 && v[1] <= 1000000 && v[2] > 0 && v[2] <= 1000000 &&
        v[3] >= 0 && v[3] <= 1000000 && v[4] > 0 && v[4] <= 1000000 &&
        v[5] >= 0 && v[5] <= 1000000 && v[6] >= 1 && v[6] <= 255 &&
        floor(v[6]) == v[6] && v[7] > 0 && v[7] <= 10000000 &&
        v[8] >= 20 && v[8] <= 300 && floor(v[8]) == v[8] &&
        fabs(v[9]) == 1 && fabs(v[10]) == 1) {
      stop(); session = uint32_t(v[0]); max_ticks = v[1];
      kp[0] = v[2]; ki[0] = v[3]; kp[1] = v[4]; ki[1] = v[5];
      limit = v[6]; accel = v[7]; timeout = uint32_t(v[8]);
      sign[0] = int(v[9]); sign[1] = int(v[10]);
      return 1;
    }
    if (strcmp(field[0], "VEL") == 0 && n == 4 && valid_session &&
        session != 0 && uint32_t(v[0]) == session &&
        fabs(v[1]) <= max_ticks && fabs(v[2]) <= max_ticks) {
      for (unsigned i = 0; i < 2; ++i) {
        if (v[i+1] == 0 || target[i] * v[i+1] < 0) {
          integral[i] = ramp[i] = 0; pwm[i] = 0;
        }
        target[i] = v[i+1];
      }
      active = true; last_command = now;
      return 0;
    }
    stop(true); return -1;
  }
  void update(uint32_t now, uint32_t left, uint32_t right) {
    watchdog(now);
    const uint32_t elapsed = now - last_step;
    if (elapsed < 20) return;
    const uint32_t counts[2] = {left, right};
    const float dt = elapsed * 0.001f;
    last_step = now;
    for (unsigned i = 0; i < 2; ++i) {
      const uint32_t raw_delta = counts[i] - previous[i];
      // Explicit modular difference avoids signed overflow at encoder rollover.
      const int64_t delta = raw_delta <= INT32_MAX ? int64_t(raw_delta) : int64_t(raw_delta) - 4294967296LL;
      previous[i] = counts[i];
      if (!active || target[i] == 0) { pwm[i] = 0; continue; }
      ramp[i] += clamp(target[i] - ramp[i], accel * dt);
      const float error = ramp[i] - float(delta) * sign[i] / dt;
      const float candidate = clamp(integral[i] + ki[i] * error * dt, limit);
      const float output = kp[i] * error + candidate;
      // Conditional integration: permit unwinding, forbid saturation windup.
      if (fabsf(output) <= limit || output * error < 0) integral[i] = candidate;
      pwm[i] = int(clamp(kp[i] * error + integral[i], limit));
    }
  }
};
}  // namespace cwu
