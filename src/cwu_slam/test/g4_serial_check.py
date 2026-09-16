#!/usr/bin/env python3
"""YDLIDAR G4 하드웨어 확인. ROS 빌드 없이 시리얼 프로토콜만 사용합니다.
health/device info를 읽고 3초간 스캔해 실제 거리값이 나오는지 검사합니다."""
import sys, time, statistics, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
BAUD = 230400
SECS = 3.0


def cmd(s, c):
    s.write(bytes([0xA5, c]))
    s.flush()


def header(s, timeout=2.0):
    """A5 5A 동기 후 (length, mode, type) 반환."""
    end, prev = time.time() + timeout, b''
    while time.time() < end:
        b = s.read(1)
        if not b:
            continue
        if prev == b'\xa5' and b == b'\x5a':
            rest = s.read(5)
            assert len(rest) == 5, '헤더가 잘렸습니다'
            v = int.from_bytes(rest[:4], 'little')
            return v & 0x3FFFFFFF, v >> 30, rest[4]
        prev = b
    raise SystemExit(f'{PORT}: 응답 없음 — 전원/배선/baud({BAUD}) 확인')


s = serial.Serial(PORT, BAUD, timeout=1)
s.dtr = False               # support_motor_dtr=false: DTR로 모터 제어하지 않음
time.sleep(0.1)
cmd(s, 0x65)                # 이전 스캔이 돌고 있으면 정지
time.sleep(0.2)
s.reset_input_buffer()

cmd(s, 0x91)
n, _, t = header(s)
p = s.read(n)
status, err = p[0], int.from_bytes(p[1:3], 'little')
print(f'[health] status={status} errcode=0x{err:04x}  (0 = 정상)')

cmd(s, 0x90)
n, _, t = header(s)
p = s.read(n)
print(f'[device] model=0x{p[0]:02x} firmware={p[2]}.{p[1]} hardware={p[3]} '
      f'sn={"".join(str(b) for b in p[4:20])}')

cmd(s, 0x60)
n, mode, t = header(s)
assert t == 0x81, f'스캔 응답 타입이 0x{t:02x} 입니다 (0x81 기대)'

buf, t0 = bytearray(), time.time()
while time.time() - t0 < SECS:
    buf += s.read(4096)
elapsed = time.time() - t0
cmd(s, 0x65)                # 모터 정지
s.close()

i = pkts = starts = zeros = 0
pts = []                    # (angle_deg, dist_m)
while i + 10 <= len(buf):
    if buf[i] != 0xAA or buf[i + 1] != 0x55:
        i += 1
        continue
    ct, cnt = buf[i + 2], buf[i + 3]
    fsa = int.from_bytes(buf[i + 4:i + 6], 'little')
    lsa = int.from_bytes(buf[i + 6:i + 8], 'little')
    end = i + 10 + 2 * cnt
    if end > len(buf):
        break
    pkts += 1
    starts += ct & 0x01
    a0, a1 = (fsa >> 1) / 64.0, (lsa >> 1) / 64.0
    span = (a1 - a0) % 360.0
    for k in range(cnt):
        d = int.from_bytes(buf[i + 10 + 2 * k:i + 12 + 2 * k], 'little') / 4000.0
        if d <= 0:
            zeros += 1
            continue
        # ponytail: 각도 보정항(거리 의존 atan) 생략한 선형 보간. 방향 감각용으로 충분,
        # 정밀 각도가 필요하면 SDK 드라이버 값을 쓸 것.
        pts.append((( a0 + span * k / max(cnt - 1, 1)) % 360.0, d))
    i = end

freq = (starts - 1) / elapsed if starts > 1 else 0.0
d = [x[1] for x in pts]
print(f'[scan] {elapsed:.1f}s / {len(buf)}B / 패킷 {pkts} / 회전 {starts} '
      f'→ {freq:.1f} Hz')
print(f'[point] 유효 {len(pts)} / 무효(0) {zeros} '
      f'→ 회전당 약 {len(pts)/max(starts-1,1):.0f} 점')
assert status == 0, f'라이다 health 이상: status={status} err=0x{err:04x}'
assert pkts > 0, '스캔 패킷이 하나도 안 옵니다 (모터가 도는지 확인)'
assert d, '유효 거리값이 0개입니다'
print(f'[range] min={min(d):.3f}m median={statistics.median(d):.3f}m max={max(d):.3f}m')
# 이 스크립트는 주파수 설정 명령을 보내지 않으므로 장치의 stock 값(G4 기본 7 Hz대)이
# 그대로 나옵니다. ydlidar_g4.yaml의 frequency: 10.0은 ROS 드라이버가 기동할 때
# 적용되며 g4_scan_check.py에서 약 9.7 Hz로 확인됩니다. 여기서는 모터가 정상 범위로
# 돌고 있는지만 봅니다.
assert 5.0 < freq < 15.0, f'스캔 주파수 {freq:.1f}Hz — 모터 회전이 비정상입니다'
# G4 사양은 0.12~12 m이지만 근거리에서 0.119 m처럼 사양보다 근소하게 낮은 값이
# 정상적으로 나옵니다(거리 분해능이 0.25 mm 단위라 경계에서 걸칩니다). 사양을
# 그대로 하한으로 쓰면 실행마다 통과·실패가 갈립니다. 진짜 고장은 0.01 m나 50 m
# 같은 값으로 드러나므로 물리적으로 말이 되는 범위인지만 봅니다.
assert 0.10 <= min(d) and max(d) <= 12.5, \
    f'거리값 {min(d):.3f}~{max(d):.3f}m가 G4 사양(0.12~12m)에서 크게 벗어납니다'

print('\n[방향 확인] 30도 구간별 중앙 거리 (0도 = 센서 정면 기준):')
for lo in range(0, 360, 30):
    seg = sorted(x[1] for x in pts if lo <= x[0] < lo + 30)
    bar = '#' * min(int(statistics.median(seg) * 8), 40) if seg else ''
    print(f'  {lo:3d}-{lo+30:3d}° n={len(seg):4d} '
          + (f'{statistics.median(seg):6.3f}m {bar}' if seg else '   (측정 없음)'))
print('\nOK: G4 하드웨어 정상 — 실제 거리 데이터 수신 확인')
