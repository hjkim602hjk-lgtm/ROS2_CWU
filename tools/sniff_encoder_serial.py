#!/usr/bin/env python3
"""Nucleo F446RE가 보내는 엔코더 데이터의 형식을 역추적합니다.

펌웨어는 있지만 프로토콜(보드레이트·프레이밍·필드 의미)이 문서화돼 있지 않을 때
씁니다. 장치로 아무것도 송신하지 않고 수신만 하므로 펌웨어 동작에 영향이 없습니다.

사용법:
    python3 tools/sniff_encoder_serial.py                 # 포트 자동 탐색
    python3 tools/sniff_encoder_serial.py /dev/ttyACM0    # 포트 지정
    python3 tools/sniff_encoder_serial.py /dev/ttyACM0 115200   # 보드레이트도 지정

보드레이트를 주지 않으면 후보를 하나씩 시도해 "인쇄 가능한 ASCII 비율"로 점수를
매겨 가장 그럴듯한 값을 고릅니다. 보드레이트가 틀리면 바이트가 깨져 그 비율이
급격히 떨어지므로 이 방법이 잘 듣습니다.
"""
import glob
import os
import re
import sys
import time

import serial

# Nucleo ST-Link 가상 COM은 보통 ttyACM, 외부 USB-시리얼은 ttyUSB로 잡힙니다.
CANDIDATE_BAUDS = [115200, 9600, 57600, 38400, 230400, 19200, 460800]
SAMPLE_SECS = 1.5


def read_sample(port, baud, secs=SAMPLE_SECS):
    """해당 보드레이트로 잠깐 열어 원시 바이트를 모읍니다."""
    try:
        with serial.Serial(port, baud, timeout=0.2) as s:
            s.reset_input_buffer()
            buf = bytearray()
            end = time.time() + secs
            while time.time() < end:
                buf += s.read(1024)
            return bytes(buf)
    except (serial.SerialException, OSError) as e:
        print(f'  {baud:>7} baud: 열기 실패 ({e})')
        return b''


def score(buf):
    """텍스트 프로토콜다움을 0~1로 점수화합니다. 보드레이트가 맞을수록 높습니다."""
    if not buf:
        return 0.0
    printable = sum(1 for b in buf if 32 <= b < 127 or b in (10, 13, 9))
    return printable / len(buf)


def show(buf, limit=256):
    """앞부분을 hex와 ASCII로 나란히 보여줍니다."""
    chunk = buf[:limit]
    for i in range(0, len(chunk), 16):
        row = chunk[i:i + 16]
        hexs = ' '.join(f'{b:02x}' for b in row)
        text = ''.join(chr(b) if 32 <= b < 127 else '.' for b in row)
        print(f'    {i:04x}  {hexs:<47}  |{text}|')


def analyse(buf):
    """줄 단위 텍스트로 보이면 숫자 필드를 뽑아 구조를 추정합니다."""
    lines = [l for l in buf.split(b'\n') if l.strip()]
    if len(lines) < 3:
        print('  줄바꿈으로 나뉘지 않습니다 — 바이너리 프레이밍일 가능성이 큽니다.')
        print('  반복되는 선두 바이트(싱크 워드)를 위 hex 덤프에서 찾아보십시오.')
        return
    # 처음/마지막 줄은 잘렸을 수 있으므로 가운데만 봅니다.
    body = lines[1:-1] or lines
    print(f'  줄 {len(lines)}개 수신, 평균 길이 {sum(len(l) for l in body)/len(body):.1f} B')
    print('  예시:')
    for l in body[:5]:
        print(f'    {l.decode("ascii", "replace").strip()!r}')

    nums = [re.findall(rb'-?\d+', l) for l in body]
    counts = {len(n) for n in nums}
    if counts == {0}:
        print('  숫자 필드를 찾지 못했습니다.')
        return
    print(f'  줄당 숫자 개수: {sorted(counts)}')
    if len(counts) == 1 and 0 not in counts:
        k = counts.pop()
        cols = list(zip(*[[int(x) for x in n] for n in nums]))
        print(f'  고정 {k}개 필드로 보입니다. 필드별 변화:')
        for i, col in enumerate(cols):
            delta = [b - a for a, b in zip(col, col[1:])]
            trend = ('단조 증가' if all(d >= 0 for d in delta)
                     else '단조 감소' if all(d <= 0 for d in delta)
                     else '증감 혼재')
            print(f'    필드{i}: {col[0]} → {col[-1]}  ({trend}, '
                  f'평균 증분 {sum(delta)/len(delta) if delta else 0:+.1f})')
        print('  → 단조 증가/감소하는 필드가 누적 엔코더 틱일 가능성이 높습니다.')
        print('    바퀴를 손으로 앞뒤로 굴리며 다시 실행해 좌우 부호를 확인하십시오.')
    else:
        print('  줄마다 필드 수가 달라 고정 포맷이 아닙니다(또는 수신이 깨졌습니다).')


def main():
    args = sys.argv[1:]
    port = args[0] if args else None
    baud = int(args[1]) if len(args) > 1 else None

    if port is None:
        found = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
        # CP2102는 YDLIDAR입니다. 엔코더 포트를 찾는 중이므로 제외합니다.
        lidar = {os.path.realpath(p) for p in glob.glob('/dev/serial/by-id/*CP2102*')}
        found = [p for p in found if p not in lidar]
        if not found:
            print('엔코더로 보이는 시리얼 장치가 없습니다.')
            print('Nucleo를 파이 USB에 연결한 뒤 다시 실행하십시오 '
                  '(보통 /dev/ttyACM0으로 잡힙니다).')
            return 1
        port = found[0]
        print(f'포트 자동 선택: {port}  (후보: {found})')

    bauds = [baud] if baud else CANDIDATE_BAUDS
    best = (0.0, None, b'')
    for b in bauds:
        buf = read_sample(port, b)
        sc = score(buf)
        print(f'  {b:>7} baud: {len(buf):5d} B 수신, 인쇄가능 비율 {sc:.0%}')
        if sc > best[0]:
            best = (sc, b, buf)

    sc, b, buf = best
    if not buf:
        print('\n어떤 보드레이트에서도 데이터가 오지 않습니다. '
              '펌웨어가 주기적으로 송신하는지, 케이블이 데이터용인지 확인하십시오.')
        return 1

    print(f'\n=== 최적 추정: {b} baud (인쇄가능 {sc:.0%}) ===')
    show(buf)
    print()
    if sc > 0.9:
        analyse(buf)
    else:
        print('  인쇄 가능 비율이 낮아 바이너리 프로토콜로 보입니다.')
        print('  위 hex 덤프에서 일정 간격으로 반복되는 바이트를 찾으면 그것이')
        print('  프레임 싱크 워드이고, 그 간격이 프레임 길이입니다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
