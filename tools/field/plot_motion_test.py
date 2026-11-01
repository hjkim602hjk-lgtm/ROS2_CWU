"""motion_test.py 결과 그림·요약 (PC): 목표 속도 대 바퀴별 실제 속도·PWM, 회전 명령 대 실제 각도.

  python3 tools/field/plot_motion_test.py motion_speed_*.csv [motion_turn_*.csv]
바퀴 속도 = 엔코더 틱 차이 / counts_per_mm(encoder.yaml) / 보드 시간(stm_ms) 차이. 계단마다 마지막 1 s 평균을 정상 상태로,
목표의 90%에 처음 닿은 시각을 올라가는 시간으로 봅니다.
"""
import csv
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

for _f in font_manager.findSystemFonts():  # 한글
    if 'NanumGothic' in _f or 'NotoSansCJK' in _f or 'Noto Sans CJK' in _f:
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=_f).get_name()
        break
plt.rcParams['axes.unicode_minus'] = False

YAML = Path(__file__).resolve().parents[2] / 'src/cwu_base/config/encoder.yaml'


def calibration():
    text = YAML.read_text()
    get = lambda key: float(re.search(rf'{key}:\s*([0-9.]+)', text).group(1))  # noqa: E731
    return get('counts_per_mm_left'), get('counts_per_mm_right'), get('wheel_separation')


def speeds(path):
    cl, cr, sep = calibration()
    rows = list(csv.DictReader(open(path)))
    t0, out = float(rows[0]['pi_time']), []
    for a, b in zip(rows, rows[1:]):
        dt = (int(b['stm_ms']) - int(a['stm_ms'])) / 1000
        if dt <= 0:
            continue
        v, w = float(b['cmd_v']), float(b['cmd_w'])
        out.append({'t': float(b['pi_time']) - t0, 'phase': b['phase'],
                    'tl': (v - w * sep / 2) * 1000, 'tr': (v + w * sep / 2) * 1000,
                    'l': (int(b['enc_l']) - int(a['enc_l'])) / cl / dt,
                    'r': (int(b['enc_r']) - int(a['enc_r'])) / cr / dt,
                    'pl': int(b['pwm_l']), 'pr': int(b['pwm_r'])})
    for k in ('l', 'r'):  # 3점 이동 평균 (20 Hz 틱 양자화)
        vals = [s[k] for s in out]
        for i, s in enumerate(out):
            s[k] = sum(vals[max(0, i - 1):i + 2]) / len(vals[max(0, i - 1):i + 2])
    return out


def summary(samples):
    print('단계        목표 mm/s   왼쪽 정상(오차)     오른쪽 정상(오차)    90% 도달 L/R s   PWM 정상 L/R   L-R 차')
    phases = [p for i, p in enumerate(s['phase'] for s in samples) if i == 0 or p != samples[i - 1]['phase']]
    for ph in phases:
        if not re.fullmatch(r'[+-]\d\.\d\d', ph):
            continue
        seg = [s for s in samples if s['phase'] == ph]
        target = seg[0]['tl']
        end = seg[-1]['t']
        steady = [s for s in seg if s['t'] >= end - 1.0]
        mean = lambda k: sum(s[k] for s in steady) / len(steady)  # noqa: E731
        rise = []
        for k in ('l', 'r'):
            hit = next((s['t'] - seg[0]['t'] for s in seg if abs(s[k]) >= 0.9 * abs(target)), None)
            rise.append('-' if hit is None else f'{hit:.2f}')
        ml, mr = mean('l'), mean('r')
        print(f'{ph:>6}      {target:7.0f}    {ml:7.0f} ({(ml - target) / target * 100:+5.1f}%)   '
              f'{mr:7.0f} ({(mr - target) / target * 100:+5.1f}%)    {rise[0]:>5}/{rise[1]:<5}      '
              f'{mean("pl"):4.0f}/{mean("pr"):<4.0f}    {mean("pl") - mean("pr"):+4.0f}')


def cruise_summary(samples):
    """1 m 직진처럼 계단이 아닌 단계: 최고 속도, 최고의 90% 넘는 구간의 좌우 PWM·속도 평균."""
    names = [p for i, p in enumerate(s['phase'] for s in samples) if i == 0 or p != samples[i - 1]['phase']]
    for ph in names:
        if re.fullmatch(r'[+-]\d\.\d\d', ph) or ph.endswith('정지') or ph == 'idle' or ph.startswith('회전'):
            continue
        seg = [s for s in samples if s['phase'] == ph]
        peak = max(abs((s['l'] + s['r']) / 2) for s in seg)
        top = [s for s in seg if abs((s['l'] + s['r']) / 2) >= 0.9 * peak]
        mean = lambda k: sum(s[k] for s in top) / len(top)  # noqa: E731
        print(f'[{ph}] 최고 {peak:.0f} mm/s, 등속 구간 {len(top) / 20:.1f} s: 속도 L/R {mean("l"):.0f}/{mean("r"):.0f} mm/s, '
              f'PWM L/R {mean("pl"):.0f}/{mean("pr"):.0f} (L-R {mean("pl") - mean("pr"):+.0f}, '
              f'{(mean("pl") / mean("pr") - 1) * 100:+.0f}%)')


def plot_speed(path):
    samples = speeds(path)
    summary(samples)
    cruise_summary(samples)
    t = [s['t'] for s in samples]
    fig, (a, b) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    a.plot(t, [s['tl'] for s in samples], 'k--', lw=1, label='목표')
    a.plot(t, [s['l'] for s in samples], color='#1f77b4', lw=1.2, label='왼쪽 실제')
    a.plot(t, [s['r'] for s in samples], color='#d62728', lw=1.2, label='오른쪽 실제')
    a.set_ylabel('바퀴 속도 (mm/s)')
    a.legend(loc='upper right')
    a.grid(alpha=0.3)
    b.plot(t, [s['pl'] for s in samples], color='#1f77b4', lw=1.2, label='왼쪽 PWM')
    b.plot(t, [s['pr'] for s in samples], color='#d62728', lw=1.2, label='오른쪽 PWM')
    b.axhline(255, color='gray', ls=':', lw=1)
    b.set_ylabel('PWM')
    b.set_xlabel('시간 (s)')
    b.legend(loc='upper right')
    b.grid(alpha=0.3)
    fig.suptitle(f'속도 추종 시험 {Path(path).stem}')
    out = Path(path).with_suffix('.png')
    fig.savefig(out, dpi=100, bbox_inches='tight')
    print(f'그림: {out}')


def plot_turn(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        return
    print('\n명령°   odom°    실제°    오차°')
    for r in rows:
        odom, actual, err = (float(r[k]) for k in ('odom_deg', 'actual_deg', 'error_deg'))
        print(f"{r['cmd_deg']:>5}  {odom:7.1f}  {actual:7.1f}  {err:+6.1f}")
    if not all(re.fullmatch(r'-?\d+', r['cmd_deg']) for r in rows):
        return  # 1 m 왕복 결과는 표만
    fig, ax = plt.subplots(figsize=(8, 4))
    labels = [f"{int(r['cmd_deg']):+d}" for r in rows]
    err = [float(r['error_deg']) for r in rows]
    ax.bar(range(len(rows)), err, color=['#1f77b4' if int(r['cmd_deg']) > 0 else '#d62728' for r in rows])
    ax.set_xticks(range(len(rows)))
    ax.set_xticklabels(labels)
    ax.axhline(0, color='k', lw=0.8)
    ax.set_ylabel('실제 - 명령 (°)')
    ax.set_title('회전 각도 오차 (파랑 = 반시계 TL, 빨강 = 시계 TR)')
    ax.grid(alpha=0.3, axis='y')
    out = Path(path).with_suffix('.png')
    fig.savefig(out, dpi=100, bbox_inches='tight')
    print(f'그림: {out}')


if __name__ == '__main__':
    plot_speed(sys.argv[1])
    if len(sys.argv) > 2:
        plot_turn(sys.argv[2])
