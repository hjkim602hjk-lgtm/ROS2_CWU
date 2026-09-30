# Nucleo autonomous wheel control — draft

The original fixed-PWM test firmware is preserved in
`legacy/changwon_robot_stm_20260929.ino.txt`. The autonomous firmware retains its
pins, encoder interrupts and left motor polarity inversion. It does not accept
single-character drive commands. Nothing has been uploaded or tested on hardware.

## Build and hardware-free check

```sh
pio run -d firmware/nucleo
g++ -std=c++11 -Wall -Wextra -Werror firmware/nucleo/test/control_check.cpp -o /tmp/cwu-control-check
/tmp/cwu-control-check
```

## Protocol version 1

USB serial: 115200 baud, ASCII, LF terminated, at most 191 bytes before LF.
Numbers must be finite decimal values. Extra fields, CR, whitespace, invalid
characters, overflow and out-of-range values stop and disarm the controller.

| Frame | Meaning |
| --- | --- |
| `READY,1` | Boot report; Pi must discard previous commands and configure again. |
| `STOP` | Immediate zero PWM and cleared PI/ramp/target; preserves configuration. |
| `CFG,session,max_ticks,kpL,kiL,kpR,kiR,pwm_limit,accel_ticks,timeout_ms,left_sign,right_sign` | Accepted only with stopped controller; clears control state. |
| `ACK,session` | Configuration accepted. Host must receive matching ACK before sending VEL. |
| `VEL,session,left_ticks_s,right_ticks_s` | Signed forward-positive wheel targets. Only valid matching-session VEL refreshes watchdog. |
| `ENC,millis,left_raw,right_raw,session` | 10 Hz raw signed cumulative encoder counts, unsigned 32-bit MCU time; session 0 means disarmed. |

The Pi generates a fresh nonzero uint32 session for each connection/configuration.
It sends STOP, then CFG, waits for matching ACK, and requires a fresh safe velocity
command. A watchdog or malformed frame disarms to session 0 and needs another CFG.
An incomplete line never refreshes the watchdog. Disconnects do not restore targets.

Configuration ranges are parser limits, **not calibrated safe settings**:

- `max_ticks`: (0, 1,000,000] ticks/s; `accel_ticks`: (0, 10,000,000] ticks/s².
- `kpL/kpR`: (0, 1,000,000]; `kiL/kiR`: [0, 1,000,000]. Gains produce 8-bit PWM units.
- `pwm_limit`: integer 1–255; `timeout_ms`: integer 20–300.
- `left_sign/right_sign`: ±1; multiply raw encoder deltas to yield forward-positive feedback. These do not change motor wiring polarity.

PI runs every 20 ms using measured elapsed time, bounded integral and conditional
anti-windup. Targets ramp at the configured acceleration limit. Zero targets,
STOP, faults, timeout and direction reversal clear integral and output immediately.
STOP bypasses deceleration. Zero PWM does not establish mechanical stopping or
successful braking. Config is allowed only after software stop; the host/operator
must separately establish that wheels have physically stopped before calibration.
Encoder counts and time wrap modulo 2³².

Serial receive work is bounded each loop. Serial output writes only available TX
buffer capacity and can drop telemetry while the host stalls. The watchdog remains
independent of TX progress; confirmation and telemetry share complete-line framing.

## Required bench evidence before floor driving

Confirm actual flashed version, encoder signs and ten-revolution counts per wheel;
measure both-direction speed tracking, PI gains, PWM/velocity/acceleration limits,
command-loss stopping time/distance, unplug/process-kill behavior, and the previously
reported loaded-turn serial stall. Gains and physical limits belong in the Pi YAML;
no measured values are claimed by the example values in the native test.
