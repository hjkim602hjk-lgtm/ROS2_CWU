# 미션 2(배틀럼블) 상태머신(mission2.py)이 쓰는 판단을 ROS 없이 계산하는 순수 함수 모음입니다.
# 경기장 좌표: 원점 = 경기장 중앙(3600 mm 정사각형), 축 = 벽 방향. 칸 (i,j) 중심 = ((i-5)·0.4, (j-5)·0.4) m.
# 출발 칸 네 곳과 모서리 탈락 칸이 중앙 기준 90° 대칭이라 어느 출발 칸이든 (2,2)로 봅니다 (설계 4.1).
# 설계: docs/superpowers/specs/2026-10-30-mission2-design.md. 로봇 없이 pytest로 검증합니다.

"""Pure mission-2 decisions: arena frame, corner guard, ring, push, flee. No ROS."""
import math

import numpy as np

HALF = 1.8            # 벽 x, y = ±1.8 m
CORNER = 1.4          # 모서리 칸 = |x| > 1.4 그리고 |y| > 1.4
START = (-1.2, -1.2, 0.0)   # (2,2) 칸 중심, +x 방향 (경기장 중앙이 로봇 왼쪽)
W_MAX = 1.2           # 펌웨어 회전 한계 rad/s


def in_corner(x, y, margin=0.0):
    """True when (x, y) is inside a corner cell grown by margin (로봇 반지름·여유)."""
    return abs(x) > CORNER - margin and abs(y) > CORNER - margin


def _corner_depth(x, y, margin):
    return min(abs(x) - (CORNER - margin), abs(y) - (CORNER - margin))


def corner_guard(pose, v, w, margin, lookahead):
    """(v, w) with v zeroed when driving lookahead metres on would go deeper into a corner zone.

    모든 상태에서 마지막에 거칩니다. 나가는 쪽(깊이가 줄어드는 쪽)은 막지 않아 밀려 들어갔을 때 빠져나올 수 있습니다.
    회전은 막지 않습니다.
    """
    if v == 0.0:
        return v, w
    x, y, yaw = pose
    d = math.copysign(lookahead, v)
    nx, ny = x + d * math.cos(yaw), y + d * math.sin(yaw)
    ahead = _corner_depth(nx, ny, margin)
    if ahead > 0.0 and ahead >= _corner_depth(x, y, margin) - 1e-9:
        return 0.0, w
    return v, w


def _square_s(x, y, h):
    """Arc position (0..8h) of the nearest point on the square |x|,|y| = h, counter-clockwise from (h, -h)."""
    cx, cy = max(-h, min(h, x)), max(-h, min(h, y))
    if abs(cx) < h and abs(cy) < h:   # 안쪽: 가장 가까운 변으로
        gaps = (h - cx, h - cy, cx + h, cy + h)
        k = gaps.index(min(gaps))
        cx, cy = ((h, cy), (cx, h), (-h, cy), (cx, -h))[k]
    if cx == h and cy < h:
        return cy + h
    if cy == h and cx > -h:
        return 2 * h + (h - cx)
    if cx == -h and cy > -h:
        return 4 * h + (h - cy)
    return 6 * h + (cx + h)


def _square_xy(s, h):
    s %= 8 * h
    side, t = int(s // (2 * h)), s % (2 * h)
    return ((h, t - h), (h - t, h), (-h, h - t), (t - h, -h))[side]


def ring_target(pose, half=0.8, lead=0.3):
    """Point lead metres counter-clockwise along the (3,3)~(7,7) ring square from the pose's nearest ring point."""
    return _square_xy(_square_s(pose[0], pose[1], half) + lead, half)


def pursue(pose, target, v_max, k, turn_only_rad):
    """(v, w) toward target: turn in place while the heading error exceeds turn_only_rad."""
    err = math.remainder(math.atan2(target[1] - pose[1], target[0] - pose[0]) - pose[2], 2 * math.pi)
    w = max(-W_MAX, min(W_MAX, k * err))
    return (v_max * math.cos(err) if abs(err) < turn_only_rad else 0.0), w


def wall_fix(points, tol, min_points):
    """(dx, dy) to add to the pose so the LiDAR points (arena frame) sit on the walls x, y = ±1.8, or None.

    벽 근처(±tol) 점의 잔차 중앙값: 벽 앞에 붙은 상대 로봇 점이 섞여도 벽 점이 훨씬 많아 끌려가지 않습니다.
    한 축의 점이 min_points보다 적으면 그 축은 0. 둘 다 적으면 None.
    """
    if not points:
        return None
    xy = np.asarray(points, float)
    out, found = [], False
    for k in (0, 1):
        c = xy[:, k]
        near = np.abs(np.abs(c) - HALF) < tol
        if near.sum() >= min_points:
            out.append(float(np.median(np.sign(c[near]) * HALF - c[near])))
            found = True
        else:
            out.append(0.0)
    return tuple(out) if found else None


def robots(points, wall_margin, gap, min_points, max_extent):
    """Centres of point clusters inside the walls (other robots), points in scan order, arena frame.

    ponytail: 보이는 호의 평균이라 실제 중심보다 로봇 반지름 절반쯤 관측자 쪽. 밀기 정렬이 어긋나면 바깥쪽으로 보정.
    """
    lim = HALF - wall_margin
    inside = [p for p in points if abs(p[0]) < lim and abs(p[1]) < lim]
    groups, cur = [], []
    for p in inside:
        if cur and math.dist(cur[-1], p) > gap:
            groups.append(cur)
            cur = []
        cur.append(p)
    if cur:
        if groups and math.dist(groups[0][0], cur[-1]) <= gap:   # 스캔 시작·끝에 걸친 묶음
            groups[0] = cur + groups[0]
        else:
            groups.append(cur)
    return [(sum(x for x, _ in g) / len(g), sum(y for _, y in g) / len(g)) for g in groups
            if len(g) >= min_points and math.dist(g[0], g[-1]) <= max_extent]


def robot_on_bearing(own_xy, bearing, robot_xys, tol):
    """The robot whose direction from own_xy is closest to bearing (within tol + its angular radius), else None."""
    best = None
    for r in robot_xys:
        d = math.dist(own_xy, r)
        diff = abs(math.remainder(math.atan2(r[1] - own_xy[1], r[0] - own_xy[0]) - bearing, 2 * math.pi))
        if diff < tol + math.atan2(0.17, max(d, 0.17)) and (best is None or diff < best[0]):
            best = (diff, r)
    return None if best is None else best[1]


def held_by(own_xy, bearing, robot_xys, tol, center_min):
    """The opponent that seems to hold the red target seen at bearing, else None (target on the floor).

    목표물은 LiDAR보다 낮아 안 보이므로 "빨간 방향선 위 로봇"을 보유자로 봅니다. 단 그 로봇이 중앙에서 center_min 안이거나
    중앙보다 멀리 있으면(바닥 큐브가 그 앞 중앙에 있을 수 있음) 보유로 보지 않습니다.
    """
    r = robot_on_bearing(own_xy, bearing, robot_xys, tol)
    if r is None or math.hypot(*r) < center_min or math.dist(own_xy, r) > math.hypot(*own_xy) + 0.1:
        return None
    return r


def push_plan(own_pose, opp, behind, lateral_tol, pushing=False):
    """(target, pushing): get behind opp (away from its nearest corner), then drive through it toward the corner.

    밀기 중(pushing=True)에는 옆 허용을 두 배로: 상대가 옆으로 조금 미끄러질 때마다 뒤 자리로 다시 돌아가 밀기가 끊겼음
    (2026-10-30 Gazebo 배틀 2). 조준점은 상대 너머 0.5 m — 모서리를 직접 겨누면 접촉이 빠짐.
    ponytail: 뒤 자리로 갈 때 상대를 돌아가는 경로는 안 짬. 상대에 걸려 정렬이 안 되면 우회 지점 추가.
    """
    corner = (math.copysign(1.6, opp[0]), math.copysign(1.6, opp[1]))
    dx, dy = corner[0] - opp[0], corner[1] - opp[1]
    n = math.hypot(dx, dy) or 1.0
    dx, dy = dx / n, dy / n
    rx, ry = own_pose[0] - opp[0], own_pose[1] - opp[1]
    along, lateral = rx * dx + ry * dy, abs(rx * dy - ry * dx)
    if along < (0.0 if pushing else -0.15) and lateral < lateral_tol * (2 if pushing else 1):
        return (opp[0] + 0.5 * dx, opp[1] + 0.5 * dy), True
    return (opp[0] - behind * dx, opp[1] - behind * dy), False


def flee_target(own_pose, opp, step, margin):
    """A point step metres away from opp (turning up to ±90° if needed) that is inside the walls and off the corners."""
    ax, ay = own_pose[0] - opp[0], own_pose[1] - opp[1]
    base = math.atan2(ay, ax)
    lim = HALF - 0.25
    for turn in (0, 45, -45, 90, -90):
        a = base + math.radians(turn)
        tx = max(-lim, min(lim, own_pose[0] + step * math.cos(a)))
        ty = max(-lim, min(lim, own_pose[1] + step * math.sin(a)))
        if not in_corner(tx, ty, margin):
            return tx, ty
    return 0.0, 0.0


def robot_ahead(points, half_width, reach, min_points=3):
    """True when at least min_points LiDAR points (base_link) are in the band ahead: 0 < x < reach, |y| < half_width.

    목표물(50 mm)은 LiDAR(170 mm)보다 낮아 안 보이므로, 집게 앞 띠에 점이 있으면 상대 로봇입니다 (동시 확보 금지).
    """
    return sum(1 for x, y in points if 0.0 < x < reach and abs(y) < half_width) >= min_points
