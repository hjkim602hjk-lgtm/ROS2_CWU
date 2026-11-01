#pragma once

// Hardware-independent arithmetic, shared by the sketch and host tests.
#include <stdint.h>
#include <math.h>

namespace motor {
constexpr float COUNTS_PER_REV = 6400.0f; // 16 cycles * AB X4 * 100: verify one output revolution.
constexpr float LEGACY_TO_X4 = 4.0f;
constexpr uint32_t SAMPLE_US = 10000;
constexpr uint32_t MAX_CONTROL_GAP_US = 50000;
constexpr float MAX_PLAUSIBLE_RPM = 120.0f; // Diagnostic margin, not a motor rating.

inline float clamp(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// Unsigned ISR counters wrap by definition. Valid if travel between reads is < 2^31.
inline int32_t delta(uint32_t now, uint32_t before) {
  const uint32_t d = now - before;
  return d <= INT32_MAX ? static_cast<int32_t>(d)
                       : -1 - static_cast<int32_t>(UINT32_MAX - d);
}

// Extend a hardware timer without an overflow ISR. Read at least every half-wrap.
struct Counter16 {
  uint16_t previous = 0;
  uint32_t total = 0;
  void reset(uint16_t raw) { previous = raw; total = 0; }
  uint32_t read(uint16_t raw, int sign) {
    const uint16_t d = static_cast<uint16_t>(raw - previous);
    const int32_t step = d <= INT16_MAX ? d : static_cast<int32_t>(d) - 65536;
    total += static_cast<uint32_t>(step * sign);
    previous = raw;
    return total;
  }
};

struct SpeedEstimator {
  uint32_t counts[3] = {};
  uint32_t times[3] = {};
  uint8_t head = 0, samples = 0;
  float rpm = 0;

  void reset(uint32_t count, uint32_t now) {
    for (uint8_t i = 0; i < 3; ++i) { counts[i] = count; times[i] = now; }
    head = samples = 0;
    rpm = 0;
  }

  bool update(uint32_t count, uint32_t now) {
    const uint8_t previous = head;
    const uint32_t dt = now - times[previous];
    if (!dt || dt > MAX_CONTROL_GAP_US) return false;
    const float instant = delta(count, counts[previous]) * 60000000.0f /
                          (COUNTS_PER_REV * dt);
    if (!isfinite(instant) || fabsf(instant) > MAX_PLAUSIBLE_RPM) return false;
    head = (head + 1) % 3;
    counts[head] = count;
    times[head] = now;
    if (samples < 2) ++samples;
    const uint8_t oldest = (head + 3 - samples) % 3;
    const uint32_t window = now - times[oldest];
    rpm = delta(count, counts[oldest]) * 60000000.0f / (COUNTS_PER_REV * window);
    return isfinite(rpm);
  }
};

struct Gains { float kp, ki, feedForward; };

struct SpeedPI {
  float integral = 0;
  int direction = 0;
  void reset() { integral = 0; direction = 0; }

  float update(float target, float measured, float dt, float applied,
               const Gains &g, float limit) {
    if (!isfinite(target) || !isfinite(measured) || !isfinite(dt) ||
        dt <= 0 || dt > MAX_CONTROL_GAP_US * 1.0e-6f || target == 0) {
      reset();
      return 0;
    }
    const int sign = target > 0 ? 1 : -1;
    if (sign != direction) { integral = 0; direction = sign; }
    const float error = fabsf(target) - measured * sign;
    const float proportional = g.feedForward * fabsf(target) + g.kp * error;
    const float candidate = clamp(integral + g.ki * error * dt, -limit, limit);
    const float demand = proportional + candidate;
    // Conditional integration includes both PWM saturation and the output ramp.
    // Integral can always unwind toward the valid range.
    const bool high = demand > limit || demand > applied * sign + 8.0f;
    const bool low = demand < 0;
    if ((!high || error <= 0) && (!low || error >= 0)) integral = candidate;
    return sign * clamp(proportional + integral, 0, limit);
  }
};

inline float approach(float value, float target, float step) {
  return value + clamp(target - value, -step, step);
}

inline float distanceSpeed(float remaining, float cruise, float acceleration,
                           float creep) {
  if (remaining <= 0) return 0;
  return fminf(cruise, fmaxf(creep, sqrtf(2.0f * acceleration * remaining)));
}
} // namespace motor
