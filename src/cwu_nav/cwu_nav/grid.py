# 미션 1 경기장(400 mm 격자)을 칸 단위로 보고 다음 이동 명령을 정하는 순수 함수 모음입니다.
# 2026-10-29 펌웨어는 연속 속도 명령이 없고 "[F100~1000] 전진", "[TL/TR 10~180] 제자리 회전"만 받으므로
# Nav2 대신 칸 BFS로 경로를 짜고 "돌고 → 한 칸 직진"을 한 번에 하나씩 보냅니다.
# map 원점 = 출발 칸 중심(SLAM 시작 자세), 칸 축 = 출발 자세 축. 로봇 없이 pytest로 검증합니다.

"""Grid-cell planning for discrete moves (turn in place, then drive straight). No ROS."""
from collections import deque
import heapq
import math

import numpy as np

FREE, UNKNOWN, BLOCKED = 'free', 'unknown', 'blocked'
NEIGHBOURS = ((1, 0), (0, 1), (-1, 0), (0, -1))


def cell_of(x, y, size):
    """Map metres -> cell (i, j). Cell (0, 0) is the start cell."""
    return round(x / size), round(y / size)


def center(cell, size):
    return cell[0] * size, cell[1] * size


def cell_state(grid, resolution, origin, cell, size, core=0.24, block_fraction=0.3, max_unknown=0.5):
    """FREE / UNKNOWN / BLOCKED from the SLAM OccupancyGrid inside the cell's central square.

    grid: (height, width) array, -1 unknown, 0..100 occupied %. 경기장 장애물은 칸을 통째로 차지하고
    벽은 칸 경계에 있으므로, 경계를 뺀 가운데 core 정사각형만 봅니다. 그 넓이의 block_fraction 이상이 점유일 때만
    막힘: 출발 자세가 몇 도만 틀어져도 먼 벽 한 줄이 칸 가운데를 지나가는데, 점 3개 기준으로는 그 줄 하나가 칸을
    막아 출발 구역이 갇혔음 (2026-10-29 실물). 실제 막힘은 직진 전 라이다 검사·정면 PSD가 그때그때 판단합니다.
    미탐색 비율이 max_unknown 넘으면 미탐색. 지도 밖은 미탐색입니다. 겉면만 보인 장애물 칸은 seen_faces가 막힘으로.
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
    if (patch >= 50).sum() >= block_fraction * total:
        return BLOCKED
    seen = patch.size - (patch < 0).sum()
    return UNKNOWN if 1 - seen / total > max_unknown else FREE


def seen_faces(grid, resolution, origin, states, size, face_points=3, span=0.24):
    """UNKNOWN cells with an occupied face on the side of a FREE neighbour → BLOCKED (returns a new dict).

    라이다는 400 mm 상자의 겉면만 맞혀 속이 끝까지 미탐색이라, 장애물 칸이 탐색 목표·경로에 들어가 "막힘 → 갈 곳 없음 →
    비우기"를 반복했음 (2026-10-30 Gazebo 예시 미로). 빈 칸에서 쏴서 맞힌 면(그 빈 칸과 맞닿은 경계 ±1 화소)만 봄:
    테두리 어디든 점이 있으면 막힘으로 보면, 양옆이 장애물인 빈 통로 칸(속이 아직 미탐색)까지 막혀 (5,5) 쪽 길이
    끊겼음 (2026-10-30 Gazebo 사용자 미로 (6,3)). 경기장 밖 칸은 외벽 면이 안쪽 빈 칸과 맞닿아 막힘이 됨.
    """
    out = dict(states)
    for cell, state in states.items():
        if state != UNKNOWN:
            continue
        for d in NEIGHBOURS:
            if states.get((cell[0] + d[0], cell[1] + d[1])) != FREE:
                continue
            if (_face(grid, resolution, origin, cell, d, size, span) >= 50).sum() >= face_points:
                out[cell] = BLOCKED
                break
    return out


def _face(grid, resolution, origin, cell, d, size, span, across=1):
    """Map pixels on cell's boundary toward neighbour d: ±across pixels across, the middle span along (axis 1 = along).

    양 끝까지 보면 양옆 장애물 모서리 점만으로 차서, 위아래가 막힌 빈 통로 입구 칸이 막힘이 됐음 → 큐브 줄 (4,5)~(7,5)가
    통째로 닫혀 420 s 동안 못 들어감 (2026-10-31 Gazebo 시드 1). 실제 장애물 면·바깥벽은 경계 전체에 걸쳐 있음.
    """
    height, width = grid.shape
    cx, cy = center(cell, size)
    bx, by = cx + d[0] * size / 2, cy + d[1] * size / 2
    hx, hy = (across * resolution, span / 2) if d[0] else (span / 2, across * resolution)
    c0 = max(int(math.floor((bx - hx - origin[0]) / resolution)), 0)
    c1 = max(min(int(math.ceil((bx + hx - origin[0]) / resolution)), width), 0)
    r0 = max(int(math.floor((by - hy - origin[1]) / resolution)), 0)
    r1 = max(min(int(math.ceil((by + hy - origin[1]) / resolution)), height), 0)
    band = grid[r0:r1, c0:c1]
    return band.T if d[0] else band


def wall_faces(grid, resolution, origin, states, size, span=0.24, parts=3, across=2):
    """{frozenset((a, b))} for neighbouring non-BLOCKED cells a, b with a wall along their whole shared boundary.

    장애물은 칸을 통째로 차지하므로 빈 칸 둘 사이 경계에 이어진 벽은 바깥벽. 라이다가 벽 위·틈으로 본 바깥이나 방향 오차로
    번진 스캔이 경기장 밖 칸을 빈 칸으로 만들어, 그 칸을 보러 두 자리 사이를 오가다 아래쪽 미탐색 구역으로 못 돌아왔음
    (2026-10-31 실물 8차, 경기장 밖 (9,2)). 경기장 크기는 안 씀 (연습장이 축소판). 벽 = 경계 가운데 span을 parts 토막으로
    나눠 토막마다 점유 화소가 있음 (장애물 모서리 점은 한쪽 끝에만 있음, 20 cm 넘는 틈은 길). 실물 지도의 벽은 경계선에서
    5 cm쯤 비켜 찍혀(위치 오차) ±across 화소를 봄
    """
    out = set()
    for cell, state in states.items():
        if state == BLOCKED:
            continue
        for d in ((1, 0), (0, 1)):
            nxt = (cell[0] + d[0], cell[1] + d[1])
            if states.get(nxt, BLOCKED) == BLOCKED:
                continue
            hit = (_face(grid, resolution, origin, cell, d, size, span, across) >= 50).any(axis=0)
            if hit.size >= parts and all(part.any() for part in np.array_split(hit, parts)):
                out.add(frozenset((cell, nxt)))
    return out


def walled_off(states, sources, walls):
    """States with every cell not reachable from sources without crossing a wall face (wall_faces) made BLOCKED.

    연습장처럼 벽에 틈이 있으면 바깥이 이어져 효과가 없음 → 카메라 시선도 벽 면을 못 지나감 (_sight)
    """
    reached = {c for c in sources if c in states}
    queue = deque(reached)
    while queue:
        cell = queue.popleft()
        for d in NEIGHBOURS:
            nxt = (cell[0] + d[0], cell[1] + d[1])
            if nxt not in reached and states.get(nxt, BLOCKED) != BLOCKED and frozenset((cell, nxt)) not in walls:
                reached.add(nxt)
                queue.append(nxt)
    return {c: s if c in reached else BLOCKED for c, s in states.items()}


def bfs(start, passable, is_goal, heading=None):
    """Shortest 4-connected cell path start..goal (inclusive) to the nearest is_goal cell, else None.

    출발 칸은 막힘 판정이어도 지나갈 수 있다고 봅니다(로봇이 이미 그 안에 있음).
    같은 길이면 직진이 많은 길: 각 칸에서 들어온 방향(출발 칸은 heading, 예 (1, 0))을 먼저 넓힙니다.
    회전마다 멈추고 방향 오차가 생기므로 꺾임이 적을수록 좋습니다 (2026-10-29 사용자 요청).
    """
    previous, came = {start: None}, {start: heading}
    queue = deque([start])
    while queue:
        cell = queue.popleft()
        if is_goal(cell):
            path = []
            while cell is not None:
                path.append(cell)
                cell = previous[cell]
            return path[::-1]
        first = came[cell]
        for di, dj in ((first,) if first else ()) + tuple(n for n in NEIGHBOURS if n != first):
            nxt = (cell[0] + di, cell[1] + dj)
            if nxt not in previous and passable(nxt):
                previous[nxt], came[nxt] = cell, (di, dj)
                queue.append(nxt)
    return None


def axis(yaw):
    """Grid step (di, dj) closest to the heading yaw (rad): 0 → (1, 0), π/2 → (0, 1)."""
    return NEIGHBOURS[round(yaw / (math.pi / 2)) % 4]


def bfs_avoiding(start, passable, is_goal, avoid, heading=None):
    """bfs that stays off the `avoid` cells (already driven through) when it can, else plain bfs.

    탐색 중 지나간 칸을 다시 지나지 않게 하되, 그 칸을 거쳐야만 닿는 곳(외길·막다른 통로)이면 지나갑니다.
    완전히 막으면 외길에서 갇힙니다 (2026-10-29 사용자 요청).
    """
    return (bfs(start, lambda c: c not in avoid and passable(c), is_goal, heading)
            or bfs(start, passable, is_goal, heading))


def return_route(start, states, passable, is_goal, walls, unknown_cost=4):
    """Cheapest 4-connected path start..goal on the own map: no blocked cell, no wall face (wall_faces) crossed,
    a FREE cell costs 1 and an UNKNOWN one unknown_cost (모르는 칸은 지나갈 수는 있지만 확인된 빈 칸 길이 먼저).

    복귀는 탐색하며 쌓은 장애물·벽 지도로 새로 짠 최단 경로 (2026-10-31 실물 13차 사용자 요청). 출발 칸은 막힘이어도 됨.
    """
    cost, previous = {start: 0}, {start: None}
    heap = [(0, 0, start)]
    order = 0
    while heap:
        c, _, cell = heapq.heappop(heap)
        if c > cost[cell]:
            continue
        if is_goal(cell):
            path = []
            while cell is not None:
                path.append(cell)
                cell = previous[cell]
            return path[::-1]
        for di, dj in NEIGHBOURS:
            nxt = (cell[0] + di, cell[1] + dj)
            if not passable(nxt) or frozenset((cell, nxt)) in walls:
                continue
            nc = c + (unknown_cost if states.get(nxt) == UNKNOWN else 1)
            if nc < cost.get(nxt, math.inf):
                cost[nxt], previous[nxt] = nc, cell
                order += 1
                heapq.heappush(heap, (nc, order, nxt))
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
    target = center(aim_cell(path, max_cells), size)
    if len(path) < 2 and math.hypot(target[0] - x, target[1] - y) < arrive:
        return None
    return _aim(pose, target, turn_min_deg)


def aim_cell(path, max_cells=1):
    """The cell next_move aims at: up to max_cells cells along the first straight run of path."""
    if len(path) < 2:
        return path[-1]
    step = (path[1][0] - path[0][0], path[1][1] - path[0][1])
    k = 1
    while (k < max_cells and k + 1 < len(path)
           and (path[k + 1][0] - path[k][0], path[k + 1][1] - path[k][1]) == step):
        k += 1
    return path[k]


def _aim(pose, target, turn_min_deg):
    x, y, yaw = pose
    heading = math.atan2(target[1] - y, target[0] - x)
    err = math.degrees(math.atan2(math.sin(heading - yaw), math.cos(heading - yaw)))
    if abs(err) >= turn_min_deg:
        return 'T', math.copysign(min(max(round(abs(err)), 10), 180), err)
    return 'F', max(100, min(4000, round(math.hypot(target[0] - x, target[1] - y) * 1000)))


def command_text(move):
    """('T', -90) -> 'TR90', ('T', 45) -> 'TL45', ('F', 400) -> 'F400' (encoder_odom /base/command)."""
    kind, value = move
    if kind == 'T':
        return f"T{'L' if value > 0 else 'R'}{abs(int(value))}"
    return f'F{int(value)}'


def compose(a, b):
    """2D pose a ∘ b: b (x, y, yaw) given in frame a → a's parent frame."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]


def invert(a):
    """Inverse 2D pose: compose(a, invert(a)) == (0, 0, 0)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return -c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]


def mark_scan(hits, misses, origin, resolution, pose, points, max_range=4.0, wall_hits=1):
    """Add one LiDAR scan (points in base_link) to the hit/miss counters of an occupancy grid (in place).

    SLAM 지도 대신 격자 보정한 자세로 직접 쌓는 지도. SLAM은 긴 통로에서 1~2.6 m 튀어 지도 자체가 망가졌고, 지금 보정을
    옛 지도에 거꾸로 적용하면 일찍 그린 출발 칸 벽이 칸 안으로 들어와 집 칸이 막힘이 됐음 (2026-10-31 Gazebo 사용자 맵
    2·3·4). 빔 끝 화소 = 맞음, 로봇에서 끝 바로 앞까지 = 빔이 지나감 (한 스캔에 화소마다 한 번만 셈).
    이미 벽(맞음 wall_hits번 이상·지나감보다 많음)인 화소를 지나는 빔은 거기까지만 지나감으로 세고 끝은 버림: 라이다가 낮은
    벽 위·틈으로 경기장 밖을 봐 밖이 빈 칸이 되고, 그걸 보러 오가다 미탐색 구역으로 못 돌아왔음 (2026-10-31 실물 8차).
    """
    if not points:
        return
    height, width = hits.shape
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    xy = np.asarray(points, float)
    ex, ey = xy[:, 0] * c - xy[:, 1] * s + x, xy[:, 0] * s + xy[:, 1] * c + y
    dist = np.hypot(ex - x, ey - y)
    keep = dist <= max_range
    ex, ey, dist = ex[keep], ey[keep], dist[keep]

    def flat(px, py):
        col = np.floor((px - origin[0]) / resolution).astype(int)
        row = np.floor((py - origin[1]) / resolution).astype(int)
        ok = (col >= 0) & (col < width) & (row >= 0) & (row < height)
        return np.unique(row[ok] * width + col[ok])

    step = resolution / 2
    t = np.arange(step, max_range, step)                    # 로봇에서 거리
    frac = t[None, :] / np.maximum(dist[:, None], 1e-6)
    inside = t[None, :] < dist[:, None] - resolution          # 끝 화소 바로 앞까지만 빔이 지나간 곳
    px, py = x + (ex - x)[:, None] * frac, y + (ey - y)[:, None] * frac
    col = np.floor((px - origin[0]) / resolution).astype(int)
    row = np.floor((py - origin[1]) / resolution).astype(int)
    ok = (col >= 0) & (col < width) & (row >= 0) & (row < height)
    wall = (hits >= wall_hits) & (hits > misses)
    crossed = np.zeros(inside.shape, bool)
    crossed[ok] = wall[row[ok], col[ok]]
    crossed &= inside
    through = crossed.any(axis=1)                              # 끝보다 앞에서 벽을 지난 빔
    first = np.where(through, crossed.argmax(axis=1), inside.shape[1])
    inside &= np.arange(inside.shape[1])[None, :] <= first[:, None]
    hit = flat(ex[~through], ey[~through])
    miss = flat(px[inside], py[inside])
    miss = np.setdiff1d(miss, hit, assume_unique=True)
    hits.flat[hit] += 1
    misses.flat[miss] += 1


def occupancy(hits, misses, min_misses=1):
    """Counters → OccupancyGrid-style values: -1 never seen, else 100·hits/(hits+misses).

    min_misses: 맞은 적 없이 빔이 이보다 적게 지나간 화소는 미탐색. 벽이 굳기 전 첫 스캔 한두 장이 벽 너머를 칠한 것을
    빈 칸으로 안 침 (mark_scan, 2026-10-31 실물 8차).
    """
    seen = hits.astype(np.int32) + misses
    known = (hits > 0) | (misses >= min_misses)
    return np.where(known, (100 * hits.astype(np.int32)) // np.maximum(seen, 1), -1).astype(np.int8)


def wall_heading(points, step=6, min_segments=20, max_range=2.0):
    """Dominant wall direction modulo 90° in the points' own frame (rad, -π/4..π/4), or None if too few wall pieces.

    규정상 벽·장애물 면은 모두 서로 평행·직각이라, 라이다로 본 벽 방향만으로 로봇 방향을 90° 단위 안에서 바로 앎
    (2026-10-31 사용자 제안). 스캔 순서로 step칸 떨어진 두 점을 이은 조각 중 가운데 점이 그 선에서 1.5 cm 안인 것(같은
    평면)만 씀(바로 이웃한 점끼리는 점 잡음 1 cm에 방향이 무작위). 방향은 4배 각(90° 주기) 히스토그램의 봉우리 → 그 둘레
    ±10°만 길이 가중 평균. 전체 평균에서 시작하면 경기장 밖(최대 11 m) 물건이나 모서리 조각이 평균을 끌어 +35°로
    튀었음 (2026-10-31 실물 3차) → 봉우리로 고르고 max_range 안 점만 씀.
    """
    xy = np.asarray([q for q in points if math.hypot(*q) <= max_range], float)
    if len(xy) < step + min_segments:
        return None
    quads, weights = [], []
    for k in (step, 2 * step):                                   # 점이 촘촘한 가까운 벽은 긴 조각이 덜 흔들림
        a, b, mid = xy[:-k], xy[k:], xy[k // 2:len(xy) - k + k // 2]
        d = b - a
        length = np.hypot(d[:, 0], d[:, 1])
        off = np.abs(d[:, 0] * (mid[:, 1] - a[:, 1]) - d[:, 1] * (mid[:, 0] - a[:, 0])) / np.maximum(length, 1e-9)
        piece = (length > 0.05) & (length < 0.4) & (off < 0.015)
        quads.append(4 * np.arctan2(d[piece, 1], d[piece, 0]))
        weights.append(length[piece] ** 2)                      # 긴 조각일수록 방향 오차가 작음
    quad, weight = np.concatenate(quads) % (2 * math.pi), np.concatenate(weights)
    if len(quad) < min_segments:
        return None
    bins = 72                                                    # 4배 각 5° = 실제 1.25°
    hist = np.bincount((quad / (2 * math.pi) * bins).astype(int) % bins, weights=weight, minlength=bins)
    smooth = sum(np.roll(hist, k) for k in range(-2, 3))
    peak = (np.argmax(smooth) + 0.5) * 2 * math.pi / bins
    near = np.abs(np.angle(np.exp(1j * (quad - peak)))) < math.radians(40)   # 4배 각 ±40° = 실제 ±10°
    if weight[near].sum() < 0.3 * weight.sum() or near.sum() < min_segments:
        return None                                              # 뚜렷한 벽 방향이 없음
    return math.atan2((weight[near] * np.sin(quad[near])).sum(), (weight[near] * np.cos(quad[near])).sum()) / 4


def lattice_fix(points, pose, size, max_yaw=math.radians(4.0), max_shift=0.12, tol=0.02, min_points=15):
    """(dx, dy, dyaw) that moves LiDAR wall points (base_link) onto the cell boundary lines x, y = size/2 + k·size.

    규정상 벽·장애물 면은 모두 400 mm 칸 경계선 위에 있음. SLAM 위치가 실제보다 10~20 cm·4° 어긋나 벽이 칸 경로 띠에
    들어와 빈 칸을 막힘으로 보고 먼 길로 돌다가 SLAM이 더 크게 틀어졌음 (2026-10-31 Gazebo 사용자 맵 4: 최대 1.38 m).
    yaw는 ±max_yaw를 0.25° 간격으로 훑고, 각 축은 경계선까지 남는 거리 히스토그램의 봉우리(±max_shift 안)로 정함.
    한 축의 벽 점이 min_points보다 적으면 그 축은 0 (통로 방향은 알 수 없음). 둘 다 적으면 None.
    """
    if len(points) < min_points:
        return None
    x, y, yaw = pose
    xy = np.asarray(points, float)
    bins = int(round(size / 0.005))
    width = max(1, int(round(tol / 0.005)))
    shifts = -(((np.arange(bins) + 0.5) * 0.005 + size / 2) % size - size / 2)  # 히스토그램 칸 i의 잔차를 0으로 옮기는 이동
    allowed = np.abs(shifts) <= max_shift
    best = None
    for dyaw in np.arange(-max_yaw, max_yaw + 1e-9, math.radians(0.25)):
        c, s = math.cos(yaw + dyaw), math.sin(yaw + dyaw)
        wx = xy[:, 0] * c - xy[:, 1] * s + x
        wy = xy[:, 0] * s + xy[:, 1] * c + y
        axes, score = [], 0.0
        for w in (wx, wy):
            r = (w - size / 2) % size
            hist = np.bincount((r / 0.005).astype(int) % bins, minlength=bins)
            near = sum(np.roll(hist, k) for k in range(-width + 1, width))  # 잔차 ±tol 안 점 수
            d = float(shifts[int(np.argmax(np.where(allowed, near, -1)))])
            for _ in range(2):  # 봉우리 안 점들의 평균 잔차로 다듬음 (히스토그램 칸 폭보다 정확히)
                res = (r + d + size / 2) % size - size / 2
                inliers = np.abs(res) < tol
                if inliers.any():
                    d -= float(res[inliers].mean())
            res = (r + d + size / 2) % size - size / 2
            inliers = np.abs(res) < tol
            # 점 수만 세면 짧은 벽에서는 몇 도 돌려도 같은 점이 띠 안에 남아 yaw를 못 가림 → 경계선에 가까울수록 큰 점수
            score += float(np.sum(1 - (res[inliers] / tol) ** 2))
            axes.append((int(inliers.sum()), d))
        if best is None or score > best[0]:
            best = (score, dyaw, axes)
    score, dyaw, ((nx, dx), (ny, dy)) = best
    if nx < min_points and ny < min_points:
        return None
    return (dx if nx >= min_points else 0.0, dy if ny >= min_points else 0.0, float(dyaw))


def scan_points(ranges, angle_min, angle_inc, yaw_offset):
    """LaserScan -> [(x, y)] in base_link. 무효 측정(0.0·inf·nan)은 버립니다."""
    points = []
    for i, r in enumerate(ranges):
        if math.isfinite(r) and r > 0.0:
            a = angle_min + i * angle_inc + yaw_offset
            points.append((r * math.cos(a), r * math.sin(a)))
    return points


def camera_view(pose, points, states, size, half_fov, max_range, min_range=0.3, margin=0.15, side=0.1):
    """Cells (not BLOCKED) whose centre is in the camera's view, with the line of sight checked on the live LiDAR scan.

    탐색 완료를 라이다 지도가 아니라 "카메라로 본 칸"으로 판단하려고 씁니다 (2026-10-30 사용자 우선순위 1).
    시선을 지도로 보면(빈 칸만 봄으로 침) 돌아서 처음 보는 쪽은 지도가 아직 미탐색이라(돌 때·돈 직후는 지도를 안 쌓음)
    봤어도 기록이 안 됐고, 나중에 빈 칸이 되면 "못 본 칸"이 되어 본 쪽을 다시 돌아보러 왔음 (2026-10-31 실물: 사용자
    "꾸준히 있는 문제"). 지금 스캔에서 그 칸 중심 방향(옆 ±side)에 칸 중심보다 margin 넘게 앞서 맞은 점이 없으면 보임.
    pose: 스캔을 찍은 때의 로봇 자세(칸 좌표). points: 그 스캔(base_link). min_range 안쪽 바닥은 카메라 화면 아래로 빠짐.
    """
    x, y, yaw = pose
    pts = np.asarray(points, float).reshape(-1, 2)
    ang, rng = np.arctan2(pts[:, 1], pts[:, 0]), np.hypot(pts[:, 0], pts[:, 1])
    out = set()
    for cell, state in states.items():
        if state == BLOCKED:
            continue
        cx, cy = center(cell, size)
        d = math.hypot(cx - x, cy - y)
        bearing = math.remainder(math.atan2(cy - y, cx - x) - yaw, 2 * math.pi)
        if not min_range <= d <= max_range or abs(bearing) > half_fov:
            continue
        near = np.abs(np.remainder(ang - bearing + math.pi, 2 * math.pi) - math.pi) <= math.atan2(side, d)
        if not np.any(rng[near] < d - margin):
            out.add(cell)
    return out


def _sight(x, y, cell, states, size, max_range, min_range, walls=frozenset()):
    """Camera at (x, y) can see cell's centre with any heading: in range, line of sight only over FREE cells and
    across no wall face (wall_faces: 벽 너머 경기장 밖 칸은 어디서도 안 보임, 2026-10-31 실물 8차)."""
    cx, cy = center(cell, size)
    d = math.hypot(cx - x, cy - y)
    if not min_range <= d <= max_range:
        return False
    here, steps = cell_of(x, y, size), max(1, int(d / (size / 4)))
    last = here
    for k in range(1, steps + 1):
        c = cell_of(x + (cx - x) * k / steps, y + (cy - y) * k / steps, size)
        if c != last and frozenset((last, c)) in walls or k < steps and c != here and states.get(c) != FREE:
            return False
        last = c
    return True


def viewpoints(states, targets, size, max_range, min_range=0.3, walls=frozenset()):
    """{FREE cell: targets its centre sees (turning as needed)} for every FREE cell that sees at least one target.

    아직 못 본 칸 자체로 가지 않고 그 칸이 화각에 들어오는 자리로 감 (2026-10-30 사용자 요청: "굳이 다 보이는 부분을
    찾아가는 이유를 모르겠다", 화각 70°·1.3 m). 못 본 칸을 목적지로 삼으면 출구가 하나인 구역을 끝까지 들어갔음.
    """
    out, reach = {}, int(max_range / size) + 1
    for t in targets:
        for di in range(-reach, reach + 1):
            for dj in range(-reach, reach + 1):
                v = (t[0] + di, t[1] + dj)
                if states.get(v) == FREE and _sight(*center(v, size), t, states, size, max_range, min_range, walls):
                    out.setdefault(v, set()).add(t)
    return out


def best_heading(pose, targets, size, half_fov):
    """(heading rad, targets within ±half_fov of it) that covers the most targets; ties go to the smallest turn."""
    x, y, yaw = pose
    bearings = {t: math.atan2(center(t, size)[1] - y, center(t, size)[0] - x) for t in targets}

    def covered(h):
        return {t for t, b in bearings.items() if abs(math.remainder(b - h, 2 * math.pi)) <= half_fov}
    h = max(bearings.values(), key=lambda b: (len(covered(b)), -abs(math.remainder(b - yaw, 2 * math.pi))))
    return h, covered(h)


def ray_cells(pose, heading, states, size, reach=8, tol=0.12):
    """Unblocked cells near the ray (centre within tol m), nearest first, up to the first blocked cell on it.

    카메라는 방향만 줌(깊이 없음). 큐브는 칸 중심에 놓이므로(규정: (5,5) 칸 중심) 방향선이 중심 가까이 지나는 빈 칸이 후보.
    방향선이 처음 지나는 막힌 칸 너머는 카메라에 안 보이므로 뺌. 미탐색 칸도 후보: 빈 칸만 보면 아직 라이다가 못 본
    큐브 칸을 건너뛰고 막힌 칸 너머 먼 빈 칸을 큐브 칸으로 잡아 65 s를 제자리에서 잃었음 (2026-10-31 Gazebo 시드 6).
    """
    x, y, _ = pose
    c, s = math.cos(heading), math.sin(heading)
    out, wall = [], reach * size
    for cell, state in states.items():
        cx, cy = center(cell, size)
        along, side = (cx - x) * c + (cy - y) * s, -(cx - x) * s + (cy - y) * c
        if state == BLOCKED and along > 0 and abs(side) < size / 2:
            wall = min(wall, along)
        elif state != BLOCKED and size / 2 < along <= reach * size and abs(side) < tol:
            out.append((along, cell))
    return [cell for along, cell in sorted(out) if along < wall]


def pocket_entries(cell, states):
    """FREE 4-neighbours to stand on and drive straight into `cell` when a side of it is blocked; [] when open all round.

    좌우가 막힌 400 mm 주머니 속 큐브에 비스듬히 들어가면 몸이 입구 모서리에 걸림 (2026-10-30 Gazebo 시드 6: 두 번 걸리고
    414 s에야 파지). 입구 칸 한가운데에 서면 큐브가 칸 축 위 정면에 있어 곧게 들어감.
    """
    around = [(cell[0] + di, cell[1] + dj) for di, dj in NEIGHBOURS]
    if not any(states.get(n) == BLOCKED for n in around):
        return []
    return [n for n in around if states.get(n) == FREE]


def wall_push(points, keep, gain, reach=0.25):
    """Extra steering (+ = left) that pushes the body off side walls nearer than `keep` (m); both sides near = centre.

    points: 진행 방향이 +x인 좌표(to_line). 400 mm 통로·지름 330 mm 몸체라 여유가 35 mm뿐이어서 SLAM 위치가 몇 cm만
    틀려도 벽에 붙어 달렸음 (2026-10-30 Gazebo: 옆 PSD 900, 몸 옆 16 cm 점으로 주행 중 정지가 판마다 수십 번).
    """
    left = min((y for x, y in points if -0.1 < x < reach and 0 < y < keep), default=keep)
    right = min((-y for x, y in points if -0.1 < x < reach and -keep < y < 0), default=keep)
    return gain * (left - right)


def to_line(points, pose, origin, target):
    """base_link LiDAR points -> a frame at `origin` (map) whose +x points at `target` (map).

    통로 검사를 "이동이 실제로 따라갈 직선"에서 합니다. 로봇이 향한 방향으로 검사하면 최대 turn_min_deg 틀어진
    띠를 보고, 로봇 위치에서 검사하면 400 mm 통로에서 옆으로 3~4 cm만 밀려도 벽이 몸 폭 띠에 걸려 빈 칸을
    막힘으로 봤음 (2026-10-30 Gazebo 랜덤 경기장: 시드 8에서 막힘 판정 105번 중 74번이 빈 칸).
    origin = 지금 칸 중심이면 칸 경로가 열렸는지, origin = 로봇 위치면 지금 갈 길이 열렸는지.
    """
    x, y, yaw = pose
    h = math.atan2(target[1] - origin[1], target[0] - origin[0])
    c, s = math.cos(yaw - h), math.sin(yaw - h)
    dx, dy = x - origin[0], y - origin[1]
    ox, oy = dx * math.cos(h) + dy * math.sin(h), -dx * math.sin(h) + dy * math.cos(h)
    return [(px * c - py * s + ox, px * s + py * c + oy) for px, py in points]


def _gaps(points, half_width, front):
    """For each LiDAR point in the body's straight path, how far the round body can drive before touching it.

    몸체를 반경 front의 원으로 보고, 옆으로 y만큼 비낀 점까지는 x - √(front² − y²)만큼 남습니다. 사각형으로 보면
    몸 옆 모서리의 벽 점이 "몸 앞끝보다 가깝다"로 잡혀 벽 옆을 지나갈 때마다 섰음 (2026-10-30 실물: ABORT LIDAR 때 옆 PSD 410·433).
    """
    for x, y in points:
        if x > 0.0 and abs(y) < half_width:
            yield x - math.sqrt(max(0.0, front * front - y * y))


def corridor_clear(points, distance, half_width, front, min_points=3):
    """True when no LiDAR points sit in the straight strip the round body sweeps driving `distance` metres.

    폭 ±half_width, 몸체 원(반경 front) 앞 가장자리부터 distance까지. 잡음 한두 점은 무시. 라이다(높이 약 170 mm)보다
    낮은 50 mm 큐브는 안 보이므로 큐브 정지는 정면 PSD 몫입니다.
    """
    return sum(1 for gap in _gaps(points, half_width, front) if gap <= distance) < min_points


def blocking_points(points, half_width, front, within, inside=None):
    """The points free_distance would stop for: in the strip, closer than `within` (m), not deeper than `inside`."""
    out = []
    for x, y in points:
        if x > 0.0 and abs(y) < half_width:
            gap = x - math.sqrt(max(0.0, front * front - y * y))
            if gap < within and (inside is None or gap > -inside):
                out.append((x, y))
    return out


def free_distance(points, half_width, front, limit, min_points=3, inside=None):
    """How far (m) the round body can drive straight before LiDAR points enter its path, capped at limit.

    잡음 두 점까지는 무시하고 세 번째로 가까운 점까지. inside: 몸 원 안쪽으로 이만큼 넘게 들어온 점은 버림 — 이미 몸 옆에
    붙은 벽 모서리·잡음이라 앞으로 가도 더 부딪히지 않음. 400 mm 출구를 지날 때마다 주행 중 정지만 반복했음
    (2026-10-30 Gazebo 시드 1: 큐브를 쥐고 주머니를 못 빠져나와 복귀 포기, 옆 PSD 900).
    """
    gaps = sorted(g for g in _gaps(points, half_width, front) if inside is None or g > -inside)
    if len(gaps) < min_points:
        return limit
    return max(0.0, min(limit, gaps[min_points - 1]))


def sweep_hits(points, turn, radius, half=math.radians(35)):
    """Points an in-place turn of `turn` rad (CCW +) would sweep the gripper into (±half around the front, out to radius).

    집게(쥐면 앞 22 cm)는 400 mm 통로 옆벽(20 cm)보다 길어, 제자리 회전 때 집게가 지나가는 쪽에 벽이 있으면 닿음
    (2026-10-31 실물 13차: 복귀 중 TL77에서 왼쪽 벽 충돌). 처음 집게 자리(앞 ±half)는 빼고 지나갈 부분만 봄.
    """
    out = []
    for x, y in points:
        r = math.hypot(x, y)
        if not 0.06 < r < radius:
            continue
        a = math.atan2(y, x) % (2 * math.pi) if turn > 0 else (-math.atan2(y, x)) % (2 * math.pi)
        if half < a < abs(turn) + half:
            out.append((x, y))
    return out


def open_frontier(grid, resolution, origin, states, size, span=0.24):
    """Frontier cells whose shared boundary with the unknown side was seen as open floor (free pixels, no wall).

    새 길 입구를 볼 칸과 같은 순위로 가까운 순에 넣었더니 바깥벽 너머 모르는 칸 (10,5)를 목표로 11 s를 씀 (2026-10-31
    실물 15차): 경계에 벽 점이 아직 덜 찍혔어도 빔이 그 경계를 지나간 적이 있어야(빈 화소) 입구로 침.
    """
    out = set()
    for cell, state in states.items():
        if state != FREE:
            continue
        for d in NEIGHBOURS:
            nxt = (cell[0] + d[0], cell[1] + d[1])
            if states.get(nxt) != UNKNOWN:
                continue
            # 경계 너머 모르는 칸 쪽 10 cm 띠 (빈 칸 쪽 화소는 늘 비어 있어 같이 보면 모든 경계가 입구가 됨)
            cx, cy = center(nxt, size)
            bx, by = cx - d[0] * size / 2, cy - d[1] * size / 2
            xs = sorted((bx, bx + d[0] * 0.1)) if d[0] else (cx - span / 2, cx + span / 2)
            ys = sorted((by, by + d[1] * 0.1)) if d[1] else (cy - span / 2, cy + span / 2)
            c0, c1 = (max(int(round((v - origin[0]) / resolution)), 0) for v in xs)
            r0, r1 = (max(int(round((v - origin[1]) / resolution)), 0) for v in ys)
            band = grid[r0:r1, c0:c1]
            if ((band >= 0) & (band < 50)).sum() >= 2 and not (band >= 50).any():
                out |= {cell, nxt}
    return out
