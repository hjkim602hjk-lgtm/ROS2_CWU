# 미션 1 경기장(400 mm 격자)을 칸 단위로 보고 다음 이동 명령을 정하는 순수 함수 모음입니다.
# 2026-10-29 펌웨어는 연속 속도 명령이 없고 "[F100~1000] 전진", "[TL/TR 10~180] 제자리 회전"만 받으므로
# Nav2 대신 칸 BFS로 경로를 짜고 "돌고 → 한 칸 직진"을 한 번에 하나씩 보냅니다.
# map 원점 = 출발 칸 중심(SLAM 시작 자세), 칸 축 = 출발 자세 축. 로봇 없이 pytest로 검증합니다.

"""Grid-cell planning for discrete moves (turn in place, then drive straight). No ROS."""
from collections import deque
import math

FREE, UNKNOWN, BLOCKED = 'free', 'unknown', 'blocked'
NEIGHBOURS = ((1, 0), (0, 1), (-1, 0), (0, -1))


def cell_of(x, y, size):
    """Map metres -> cell (i, j). Cell (0, 0) is the start cell."""
    return round(x / size), round(y / size)


def center(cell, size):
    return cell[0] * size, cell[1] * size


def cell_state(grid, resolution, origin, cell, size, core=0.24, min_occupied=3, max_unknown=0.5):
    """FREE / UNKNOWN / BLOCKED from the SLAM OccupancyGrid inside the cell's central square.

    grid: (height, width) array, -1 unknown, 0..100 occupied %. 경기장 장애물은 칸을 통째로 차지하고
    벽은 칸 경계에 있으므로, 경계를 뺀 가운데 core 정사각형만 봅니다 (SLAM 벽 두께 ±5 cm가 옆 칸으로 번져도
    가운데까지는 안 옴). 점유 칸이 min_occupied 이상이면 막힘, 미탐색 비율이 max_unknown 넘으면 미탐색.
    지도 밖은 미탐색입니다.
    """
    cx, cy = center(cell, size)
    half = core / 2
    c0 = int(math.floor((cx - half - origin[0]) / resolution))
    c1 = int(math.ceil((cx + half - origin[0]) / resolution))
    r0 = int(math.floor((cy - half - origin[1]) / resolution))
    r1 = int(math.ceil((cy + half - origin[1]) / resolution))
    height, width = grid.shape
    patch = grid[max(r0, 0):max(min(r1, height), 0), max(c0, 0):max(min(c1, width), 0)]
    total = max((r1 - r0) * (c1 - c0), 1)
    if (patch >= 50).sum() >= min_occupied:
        return BLOCKED
    seen = patch.size - (patch < 0).sum()
    return UNKNOWN if 1 - seen / total > max_unknown else FREE


def bfs(start, passable, is_goal):
    """Shortest 4-connected cell path start..goal (inclusive) to the nearest is_goal cell, else None.

    출발 칸은 막힘 판정이어도 지나갈 수 있다고 봅니다(로봇이 이미 그 안에 있음).
    """
    previous = {start: None}
    queue = deque([start])
    while queue:
        cell = queue.popleft()
        if is_goal(cell):
            path = []
            while cell is not None:
                path.append(cell)
                cell = previous[cell]
            return path[::-1]
        for di, dj in NEIGHBOURS:
            nxt = (cell[0] + di, cell[1] + dj)
            if nxt not in previous and passable(nxt):
                previous[nxt] = cell
                queue.append(nxt)
    return None


def frontier(states):
    """Cells worth looking from: not blocked and next to an unknown cell (or unknown themselves)."""
    out = set()
    for cell, state in states.items():
        if state == BLOCKED:
            continue
        if state == UNKNOWN or any(states.get((cell[0] + di, cell[1] + dj)) == UNKNOWN
                                   for di, dj in NEIGHBOURS):
            out.add(cell)
    return out


def next_move(pose, path, size, turn_min_deg=5.0, arrive=0.1, max_cells=1):
    """Next board command toward `path` from the SLAM pose, as ('T', deg) / ('F', mm), or None when arrived.

    목표 = 경로에서 지금 칸 다음부터 같은 직선으로 최대 max_cells칸 앞의 칸 중심. 그 중심을 향한 방향 오차가
    turn_min_deg 이상이면 회전(+ 반시계 = TL), 아니면 중심까지 직진. 매 이동 뒤 SLAM 자세로 다시 부르므로
    옆으로 밀린 만큼 다음 조준에서 바로잡힙니다. 펌웨어 최소 회전이 10°라 5~10° 오차도 10°로 돌려
    남는 오차를 5° 안에 둡니다(400 mm 직진에 옆 35 mm — 400 mm 통로·지름 330 mm 몸체 여유와 같음).
    펌웨어 최소 직진 100 mm보다 가까우면 도착으로 봅니다.
    pose: (x, y, yaw) map. path: [지금 칸, ..., 목표 칸].
    """
    x, y, _ = pose
    if len(path) < 2:
        cx, cy = center(path[-1], size)
        return None if math.hypot(cx - x, cy - y) < arrive else _aim(pose, (cx, cy), turn_min_deg)
    step = (path[1][0] - path[0][0], path[1][1] - path[0][1])
    k = 1
    while (k < max_cells and k + 1 < len(path)
           and (path[k + 1][0] - path[k][0], path[k + 1][1] - path[k][1]) == step):
        k += 1
    return _aim(pose, center(path[k], size), turn_min_deg)


def _aim(pose, target, turn_min_deg):
    x, y, yaw = pose
    heading = math.atan2(target[1] - y, target[0] - x)
    err = math.degrees(math.atan2(math.sin(heading - yaw), math.cos(heading - yaw)))
    if abs(err) >= turn_min_deg:
        return 'T', math.copysign(min(max(round(abs(err)), 10), 180), err)
    return 'F', max(100, min(1000, round(math.hypot(target[0] - x, target[1] - y) * 1000)))


def command_text(move):
    """('T', -90) -> 'TR90', ('T', 45) -> 'TL45', ('F', 400) -> 'F400' (encoder_odom /base/command)."""
    kind, value = move
    if kind == 'T':
        return f"T{'L' if value > 0 else 'R'}{abs(int(value))}"
    return f'F{int(value)}'


def scan_points(ranges, angle_min, angle_inc, yaw_offset):
    """LaserScan -> [(x, y)] in base_link. 무효 측정(0.0·inf·nan)은 버립니다."""
    points = []
    for i, r in enumerate(ranges):
        if math.isfinite(r) and r > 0.0:
            a = angle_min + i * angle_inc + yaw_offset
            points.append((r * math.cos(a), r * math.sin(a)))
    return points


def corridor_clear(points, distance, half_width, front, min_points=3):
    """True when no LiDAR points sit in the straight strip the body sweeps driving `distance` metres.

    몸체 앞끝(front)에서 distance까지, 폭 ±half_width. 잡음 한두 점은 무시. 라이다(높이 약 170 mm)보다
    낮은 50 mm 큐브는 안 보이므로 큐브 정지는 펌웨어 정면 PSD 몫입니다.
    """
    hits = sum(1 for x, y in points if 0.0 < x <= front + distance and abs(y) < half_width)
    return hits < min_points
