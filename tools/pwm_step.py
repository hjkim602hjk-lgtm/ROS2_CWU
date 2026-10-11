#!/usr/bin/env python3
"""Floor PWM -> wheel speed table for ROBOT40_V3 (feeds the [G] feed-forward gains).

Usage on the Pi (stop the SLAM launch first; it owns the port):
    python3 pwm_step.py                     # PWM table ([D])
    python3 pwm_step.py v ff static kp ki   # closed-loop check ([V] 300/577/800 ticks/s)
Drives [D p,p] forward through PWM steps, the same steps backward (returns near
the start), then [D p,-p] turns. Each step 0.8 s; speed = steady last 0.4 s.
Needs ~0.6 m clear ahead. Ticks are reported "forward = +" (V3 raw ticks fall
when driving forward).
"""
import sys
import time

import serial

STEPS = [40, 50, 60, 70, 80, 100]
FWD = -1  # V3 telemetry ticks decrease when the robot drives forward


def run(ser, left, right, seconds=0.8, kind=b'D', trace=False):
    """Drive one step; return (left, right) forward-positive ticks/s over the last 0.4 s."""
    samples, buf, end = [], b'', time.monotonic() + seconds
    while time.monotonic() < end:
        ser.write(b'![%s%d,%d]' % (kind, left, right))
        time.sleep(0.05)
        buf += ser.read(4096)
        *lines, buf = buf.split(b'\n')
        for line in lines:
            parts = line.decode(errors='replace').strip().split(',')
            if line.startswith(b'# ABORT') or line.startswith(b'# REJECT'):
                print('  보드:', line.decode(errors='replace').strip())
            elif len(parts) >= 12 and parts[2].lstrip('-').isdigit():
                samples.append((int(parts[0]), int(parts[2]), int(parts[3])))
    if trace:  # per-sample wheel speeds: shows rise time / overshoot of the PI loop
        for (t0, l0, r0), (t1, l1, r1) in zip(samples, samples[1:]):
            if t1 > t0:
                print(f'    {t1 - samples[0][0]:5d}ms  L {FWD * (l1 - l0) * 1000 / (t1 - t0):6.0f}'
                      f'  R {FWD * (r1 - r0) * 1000 / (t1 - t0):6.0f}')
    tail = [s for s in samples if s[0] >= samples[-1][0] - 400] if samples else []
    if len(tail) < 2:
        return float('nan'), float('nan')
    dt = (tail[-1][0] - tail[0][0]) / 1000
    return (FWD * (tail[-1][1] - tail[0][1]) / dt, FWD * (tail[-1][2] - tail[0][2]) / dt)


def main():
    ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0)
    ser.write(b'q')
    time.sleep(0.3)
    ser.reset_input_buffer()
    if len(sys.argv) > 1 and sys.argv[1] == 'v':
        ser.write(('![G%s,%s,%s,%s]' % tuple(sys.argv[2:6])).encode())
        time.sleep(0.2)
        print(ser.read(4096).decode(errors='replace').strip())
        try:
            for sign in (1, -1):
                for target in (300, 577, 800):
                    print(f'목표 {sign * target} 틱/s')
                    left, right = run(ser, sign * target, sign * target, 1.0, b'V', True)
                    print(f'  정상상태 L {left:.0f}  R {right:.0f}', flush=True)
                run(ser, 0, 0, 0.6, b'V')
        finally:
            ser.write(b'q')
            ser.close()
        return
    print(' PWM_L PWM_R   L틱/s   R틱/s')
    try:
        for sign in (1, -1):
            for p in STEPS:
                left, right = run(ser, sign * p, sign * p)
                print(f'{sign * p:6d}{sign * p:6d}{left:8.0f}{right:8.0f}', flush=True)
            run(ser, 0, 0, 0.6)
        for p in (60, 80):
            for sign in (1, -1):
                left, right = run(ser, -sign * p, sign * p)
                print(f'{-sign * p:6d}{sign * p:6d}{left:8.0f}{right:8.0f}', flush=True)
                run(ser, 0, 0, 0.6)
    finally:
        ser.write(b'q')
        ser.close()


if __name__ == '__main__':
    main()
