"""시뮬레이션 한 판의 실제 경로를 경기장 위에 그림 (run_maze.sh가 판마다 부름).

  /usr/bin/python3 tools/sim/plot_track.py SEED TRACK RESULT_LINE OUT.png
  TRACK = cwu_sim track_recorder 출력 ("t x y yaw" 출발 기준 m, "E t 상태 전이"). SEED = 숫자 | example | custom(SIM_CELLS)
  색 = 시뮬레이션 경과 s, 큐브를 잡은 곳·복귀 시작을 표시.
"""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
from matplotlib import font_manager  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

from cwu_sim.maze import CELL, TARGET, layout, parse_cells, xy  # noqa: E402

for path in ('/usr/share/fonts/truetype/nanum/NanumSquareB.ttf', '/usr/share/fonts/truetype/nanum/NanumGothic.ttf'):
    if os.path.exists(path):
        font_manager.fontManager.addfont(path)
        matplotlib.rcParams['font.family'] = font_manager.FontProperties(fname=path).get_name()
        break

seed, track, result, out = sys.argv[1:5]
if seed == 'example':
    start, yaw, obstacles = layout(0, True)
elif seed == 'custom':
    start, yaw, obstacles = layout(0, cells=parse_cells(os.environ['SIM_CELLS']))
else:
    start, yaw, obstacles = layout(int(seed))
sx, sy = xy(start)
c, s = math.cos(yaw), math.sin(yaw)
pts, events = [], []
for line in open(track, encoding='utf-8'):
    f = line.split()
    if f and f[0] == 'E':
        events.append((float(f[1]), ' '.join(f[2:])))
    elif len(f) == 4:
        t, px, py = float(f[0]), float(f[1]), float(f[2])
        pts.append((t, sx + px * c - py * s, sy + px * s + py * c))  # 출발 기준 → 경기장 좌표
t0 = pts[0][0] if pts else 0.0

fig, ax = plt.subplots(figsize=(7, 7.6))
for i in range(1, 10):
    for j in range(1, 10):
        x, y = xy((i, j))
        ax.add_patch(Rectangle((x - CELL / 2, y - CELL / 2), CELL, CELL, fill=(i, j) in obstacles,
                               fc='#45455a', ec='#cccccc', lw=0.6))
ax.add_patch(Rectangle((-1.8, -1.8), 3.6, 3.6, fill=False, ec='#222', lw=3))
ax.plot(*xy(TARGET), 's', color='red', ms=9, label='큐브 (5,5)')
ax.plot(sx, sy, 'o', color='#1f6fff', ms=16, alpha=0.5, label=f'출발 {start}')
if pts:
    ts, xs, ys = zip(*pts)
    ax.plot(xs, ys, color='#555', lw=0.8, zorder=2)
    sc = ax.scatter(xs, ys, c=[t - t0 for t in ts], cmap='viridis', s=10, zorder=3)
    fig.colorbar(sc, ax=ax, fraction=0.04, pad=0.02, label='경과 시간 (s)')
    grasp = [t for t, e in events if '→ GRASP' in e]
    if grasp:
        g = min(pts, key=lambda p: abs(p[0] - grasp[0]))
        ax.plot(g[1], g[2], 'D', color='orange', ms=11, mec='k', zorder=4, label=f'파지 ({grasp[0] - t0:.0f} s)')
    ax.plot(xs[-1], ys[-1], '*', color='gold', ms=18, mec='k', zorder=5, label='끝 위치')
ax.set_xlim(-1.9, 1.9)
ax.set_ylim(-1.9, 1.9)
ax.set_aspect('equal')
ax.set_xticks([xy((i, 1))[0] for i in range(1, 10)], [str(i) for i in range(1, 10)])
ax.set_yticks([xy((1, j))[1] for j in range(1, 10)], [str(j) for j in range(1, 10)])
ax.set_title(os.environ.get('SIM_TITLE', f'seed {seed}') + ' — 실제 경로\n' + result.split('| 막힘')[0], fontsize=10)
ax.legend(loc='upper right', fontsize=8)
fig.tight_layout()
fig.savefig(out, dpi=120)
