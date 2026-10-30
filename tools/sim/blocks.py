"""mission1 시뮬레이션 로그 요약: 라이다 '막힘' 판정 중 실제로는 빈 칸(가짜 막힘) 수, 회전·직진 명령 수.

  python3 tools/sim/blocks.py SEED LOG ['x y ...']   (run_maze.sh가 부름. 정답 칸은 cwu_sim.maze.layout(SEED))
"""
import math
import os
import re
import sys

from cwu_sim.maze import N, layout, parse_cells

seed, log = sys.argv[1], open(sys.argv[2], encoding='utf-8', errors='replace').read()
start, yaw, obstacles = (layout(0, True) if seed == 'example' else
                         layout(0, cells=parse_cells(os.environ['SIM_CELLS'])) if seed == 'custom' else layout(int(seed)))
c, s = round(math.cos(yaw)), round(math.sin(yaw))


def world(a, b):
    """mission map cell (출발 칸 기준, x = 출발 정면) → arena cell."""
    return start[0] + a * c - b * s, start[1] + a * s + b * c


claims = [world(int(a), int(b)) for a, b in re.findall(r'라이다: \((-?\d+), (-?\d+)\) 칸으로 가는 직선이 막힘', log)]
false = [w for w in claims if 1 <= w[0] <= N and 1 <= w[1] <= N and w not in obstacles]
turns = len(re.findall(r'\]: \w+: T[LR]\d+', log))
runs = len(re.findall(r'\]: \w+: F\d+', log))
home = '?'
if len(sys.argv) > 3 and len(sys.argv[3].split()) >= 2:  # Gazebo 실제 위치 'x y ...' (world 원점 = 출발 자세)
    home = f'{math.hypot(*map(float, sys.argv[3].split()[:2])):.2f}'
print(f'집까지 {home} m | 막힘 {len(claims)}(가짜 {len(false)}) 회전 {turns} 직진 {runs}')
