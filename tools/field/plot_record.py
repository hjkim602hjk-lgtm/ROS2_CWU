"""미션 1 기록(~/mission1_record_*.npz, 미션 끝에 저장)을 지도 위 경로 그림으로 (PC에서).

  python3 tools/field/plot_record.py 기록.npz [그림.png]
좌표는 사용자 좌표 (출발 = 9행 1열, 처음 가던 쪽 = 열 증가, 왼쪽 = 행 감소). 점선 사각형 = 카메라로 본 칸.
"""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

for _f in font_manager.findSystemFonts():  # 한글 제목
    if 'NanumGothic' in _f or 'NotoSansCJK' in _f or 'Noto Sans CJK' in _f:
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=_f).get_name()
        break
plt.rcParams['axes.unicode_minus'] = False

COLORS = {'CRUISE': '#1f77b4', 'GOTO': '#ff7f0e', 'LOOK': '#9467bd', 'APPROACH': '#d62728', 'GRASP': '#d62728',
          'RETURN': '#2ca02c', 'DONE': '#2ca02c'}


def user(x, y, size=0.4):
    return 1 + x / size, 9 - y / size


def main():
    z = np.load(sys.argv[1])
    out = sys.argv[2] if len(sys.argv) > 2 else sys.argv[1].replace('.npz', '.png')
    hits, misses = z['hits'].astype(int), z['misses'].astype(int)
    res, (x0, y0) = float(z['resolution']), z['origin']
    seen = hits + misses
    img = np.where(seen == 0, 0.75, np.where(hits * 2 >= seen, 0.0, 1.0))
    rows, cols = np.nonzero(seen)
    r0, r1, c0, c1 = rows.min(), rows.max() + 1, cols.min(), cols.max() + 1
    left, bottom = user(x0 + c0 * res, y0 + r0 * res)
    right, top = user(x0 + c1 * res, y0 + r1 * res)
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(img[r0:r1, c0:c1], cmap='gray', origin='lower', extent=[left, right, bottom, top], vmin=0, vmax=1,
              interpolation='nearest')
    for k in range(-5, 16):
        ax.axhline(k + 0.5, color='c', lw=0.3)
        ax.axvline(k + 0.5, color='c', lw=0.3)
    for i, j in z['camera_seen']:
        u, v = user(i * 0.4, j * 0.4)
        ax.add_patch(plt.Rectangle((u - 0.45, v - 0.45), 0.9, 0.9, fill=False, ec='#9467bd', lw=0.6, ls=':'))
    track, states = z['track'], z['track_state']
    for k in range(1, len(track)):
        (u0, v0), (u1, v1) = user(*track[k - 1][1:3]), user(*track[k][1:3])
        ax.plot([u0, u1], [v0, v1], color=COLORS.get(states[k - 1], '#999'), lw=2)
    t = track[:, 0] - track[0, 0]
    for k in range(0, len(track), 20):
        u, v = user(*track[k][1:3])
        ax.text(u, v, f'{t[k]:.0f}', fontsize=6)
    ax.plot(1, 9, 's', color='g', ms=10)
    ax.set_xlim(min(left, right), max(left, right))
    ax.set_ylim(max(bottom, top), min(bottom, top))
    ax.set_aspect('equal')
    ax.set_title(f'{sys.argv[1].split("/")[-1]}  (파랑 직진, 주황 GOTO, 보라 둘러보기, 빨강 접근, 초록 복귀, 숫자 = 초)')
    plt.savefig(out, dpi=110, bbox_inches='tight')
    print(out)


if __name__ == '__main__':
    main()
