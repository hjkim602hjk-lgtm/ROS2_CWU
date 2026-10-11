#!/usr/bin/env python3
"""Ticks-per-revolution measurement with ROBOT40_V3 firmware (wheels lifted!).

Usage on the Pi:  python3 tick_measure.py l|r [port]
Turns one wheel steadily (firmware w/e, heartbeat every 0.1 s, 10 s cap)
until Enter, then stops and prints the tick count at Enter and at rest.
"""
import select
import sys
import time

import serial

side = sys.argv[1] if len(sys.argv) > 1 else ''
if side not in ('l', 'r'):
    sys.exit('usage: tick_measure.py l|r [port]')
port = sys.argv[2] if len(sys.argv) > 2 else '/dev/ttyAMA0'
field = 2 if side == 'l' else 3  # telemetry CSV: ms,mode,L,R,...


def counts(data):
    """Encoder values of the chosen wheel in every telemetry line of data."""
    out = []
    for line in data.decode(errors='replace').splitlines():
        parts = line.strip().split(',')
        if len(parts) >= 12 and parts[field].lstrip('-').isdigit():
            out.append(int(parts[field]))
    return out


ser = serial.Serial(port, 115200, timeout=0.02)
ser.write(b'qz')
time.sleep(0.5)
ser.reset_input_buffer()
input(f'{"왼쪽" if side == "l" else "오른쪽"} 바퀴. Enter를 누르면 돌기 시작, 다시 Enter를 누르면 멈춥니다. ')
ser.write(b'!' + (b'w' if side == 'l' else b'e'))
count, beat, buf, msg = 0, time.monotonic(), b'', ''
try:
    while True:
        if time.monotonic() - beat >= 0.1:
            ser.write(b'!')
            beat = time.monotonic()
        buf += ser.read(256)
        if b'\n' in buf:
            *lines, buf = buf.split(b'\n')
            data = b'\n'.join(lines)
            for line in data.decode(errors='replace').splitlines():
                if line.startswith('# ABORT') or line.startswith('# REJECT'):
                    msg = line.strip()
            found = counts(data)
            if found:
                count = found[-1]
                print(f'\r틱 {count:7d}   ', end='', flush=True)
            if msg:
                print('\n보드:', msg, '- 멈췄습니다.')
                break
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            break
except KeyboardInterrupt:
    pass
finally:
    ser.write(b'q')
at_enter = count
time.sleep(1.5)  # let the wheel coast to rest; the ISR keeps counting
ser.reset_input_buffer()
ser.write(b's')
time.sleep(0.5)
rest = (counts(ser.read(2000)) or [at_enter])[-1]
ser.close()
print(f'\nEnter 순간 틱 = {at_enter}  (최대 0.2초 전 값)')
print(f'멈춘 뒤 틱    = {rest}')
