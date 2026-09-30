# 모터 명령 중 원시 ENC 줄을 시간과 함께 기록하는 일회성 진단 스크립트입니다.
# encoder_odom을 끈 상태에서 실행합니다. 어떤 경우든 끝나면 s를 보냅니다.
# 사용법: python3 raw_enc_log.py <명령 f|b|l|r> <최대 초> <라벨> [목표 틱]
# 목표 틱: 좌우 |누적 틱| 평균이 이 값에 닿으면 정지. 시간 대비 불가능하게 튄 줄은 버리고
# 마지막 정상 줄을 기준으로 삼아, 끊긴 동안 간 거리도 다음 정상 줄에서 모두 셉니다.
import re
import signal
import sys
import time

import serial

cmd, secs, label = sys.argv[1].encode(), float(sys.argv[2]), sys.argv[3]
target = float(sys.argv[4]) if len(sys.argv) > 4 else float('inf')
travel, stopped_at, ref = 0.0, None, None
# ENC가 이 시간(초) 동안 안 오면 눈을 가린 채 달리는 셈이라 명령을 끊습니다.
# ST-Link는 송신만 멈추고 수신은 살아 있을 수 있어 펌웨어 명령 만료로는 못 막습니다.
# 주행 중 0.2~0.4초 공백은 흔해서(엔코더가 누적값이라 거리는 복구됨) 완전 멈춤만 잡습니다.
BLIND = 1.0
MAX_TICKS_PER_SEC = 3000  # 약 0.27 m/s, PWM 100 바닥 속도(약 1400틱/초)의 2배
ENC = re.compile(r'ENC,(-?\d+),(-?\d+)')


def bail(*_):
    raise SystemExit(1)


signal.signal(signal.SIGHUP, bail)
signal.signal(signal.SIGTERM, bail)

rows, other = [], []
# 쓰기 제한 시간: ST-Link가 양방향으로 멈추면 write가 막혀 루프 전체가 섭니다(2026-09-30 5초 멈춤).
# 그때는 명령도 안 가므로 펌웨어 명령 만료가 모터를 세웁니다. 여기선 루프만 살려 둡니다.
port = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.05, write_timeout=0.05)


def send(b):
    try:
        port.write(b)
    except serial.SerialTimeoutException:
        pass

try:
    port.reset_input_buffer()
    t0 = time.time()
    last_enc = t0
    sent = t0 - 1
    while time.time() - t0 < secs + 1.0:  # 정지 후 1초까지 기록
        blind = cmd and time.time() - last_enc > BLIND
        if (time.time() - t0 >= secs or travel >= target or blind) and cmd:
            print(f'ENC {BLIND}초 끊김으로 정지' if blind else '', end='')
            send(b's')
            cmd = b''
            stopped_at = (round(time.time() - t0, 2), round(travel))
            secs = time.time() - t0  # 정지 후 1초까지만 더 기록
        elif cmd and time.time() - sent >= 0.1:  # 펌웨어 명령 만료보다 빠르게 재전송
            send(cmd)
            sent = time.time()
        line = port.readline().decode(errors='replace').strip()
        if not line:
            continue
        m = ENC.fullmatch(line)
        if m:
            now, l, r = time.time(), int(m[1]), int(m[2])
            if ref is None:
                ref = (now, l, r)
            elif max(abs(l - ref[1]), abs(r - ref[2])) <= MAX_TICKS_PER_SEC * (now - ref[0]) + 100:
                travel += (abs(l - ref[1]) + abs(r - ref[2])) / 2
                ref = (now, l, r)
            rows.append((now - t0, l, r))
            last_enc = time.time()
        else:
            other.append((round(time.time() - t0, 2), line))
finally:
    send(b's')
    time.sleep(0.05)
    send(b's')
    port.close()

with open(f'/tmp/raw_enc_{label}.csv', 'w') as f:
    f.write('t_sec,left,right\n')
    f.writelines(f'{t:.3f},{l},{r}\n' for t, l, r in rows)

gaps = [(round(a[0], 2), round(b[0] - a[0], 2)) for a, b in zip(rows, rows[1:])
        if b[0] - a[0] > 0.15]
dl = [b[1] - a[1] for a, b in zip(rows, rows[1:])]
dr = [b[2] - a[2] for a, b in zip(rows, rows[1:])]
print(f'줄 {len(rows)}개 (정상이면 약 {int((secs + 1) * 10)}개), 기타 줄: {other}')
print(f'정지 시점(초, 누적 틱): {stopped_at}')
print(f'150 ms 넘는 공백: {gaps or "없음"}')
if len(rows) < 2:
    raise SystemExit("ENC 줄이 2개 미만이라 분석할 수 없습니다")
print(f'왼쪽 총 {rows[-1][1] - rows[0][1]:+d}틱, 증분 +{sum(d > 0 for d in dl)}/-{sum(d < 0 for d in dl)}회, '
      f'최대 |증분| {max(map(abs, dl), default=0)}')
print(f'오른쪽 총 {rows[-1][2] - rows[0][2]:+d}틱, 증분 +{sum(d > 0 for d in dr)}/-{sum(d < 0 for d in dr)}회, '
      f'최대 |증분| {max(map(abs, dr), default=0)}')
print('증분 샘플 (L,R):', list(zip(dl, dr))[:40])
