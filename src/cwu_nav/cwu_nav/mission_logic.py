# 미션 1 상태머신(mission1.py)이 쓰는 판단을 ROS 없이 계산하는 순수 함수 모음입니다.
# 다음에 어디를 볼지(중앙 힌트·미탐색 경계), 보드가 멈춘 이유가 파지인지 장애물인지,
# 막혔을 때 어느 쪽으로 빠질지를 정합니다. 로봇 없이 pytest로 검증합니다.

"""Pure mission-1 decisions: where to look next, what a board stop means. No ROS."""
import math

import cv2
import numpy as np

# /base/telemetry 필드 번호 (README_STM_PI_INTERFACE.md 6절, PSD는 raw ADC)
FRONT, REAR, LEFT, RIGHT, SERVO_L, SERVO_R = 6, 7, 8, 9, 10, 11


def hint_goal(hint, standoff):
    """Point `standoff` short of the hint on the line from the start (map origin).

    목표물이 힌트 자리에 그대로 있으면 그 위로 올라타지 않고, 카메라가 볼 거리에서 멈춥니다.
    """
    distance = math.hypot(*hint)
    scale = max(0.0, distance - standoff) / distance if distance > 0 else 0.0
    return hint[0] * scale, hint[1] * scale


def frontier_goal(grid, resolution, origin, robot, clearance, min_cells, min_dist,
                  blacklist=(), blacklist_radius=0.4, passage=0.075):
    """Nearest frontier cell centre in map metres, else None.

    grid: OccupancyGrid data as a (height, width) array, -1 unknown, 0..100 occupied %.
    A frontier cell is free and touches unknown; clusters under min_cells are noise.
    Cells closer than `clearance` to an occupied cell cannot hold the robot.
    Only frontiers joined to the robot through seen floor at least `passage` from walls count:
    라이다가 벽 틈으로 넘겨 본 바닥은 직선으로 가까워도 못 갑니다 (2026-10-16 11회차,
    0.64 m 앞 벽 너머 경계로 Nav2가 23 s 재시도).
    """
    free = (grid >= 0) & (grid < 50)
    occupied = grid >= 50
    near_unknown = cv2.dilate((grid < 0).astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    frontier = (free & near_unknown).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(frontier, connectivity=8)
    big = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= min_cells]
    room = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, 3) * resolution
    _, area = cv2.connectedComponents((free & (room >= passage)).astype(np.uint8), connectivity=8)
    # 로봇이 벽에 바짝 붙어 제 칸이 통로 기준에 못 미쳐도 반경 안 통로 칸에서 출발
    r = int((robot[1] - origin[1]) / resolution)
    c = int((robot[0] - origin[0]) / resolution)
    k = int(math.ceil(passage / resolution))
    around = area[max(r - k, 0):max(r + k + 1, 0), max(c - k, 0):max(c + k + 1, 0)]
    reachable = np.isin(area, around[around > 0])
    rows, cols = np.nonzero(np.isin(labels, big) & (room >= clearance) & reachable)
    if not len(rows):
        return None
    xs = origin[0] + (cols + 0.5) * resolution
    ys = origin[1] + (rows + 0.5) * resolution
    dist = np.hypot(xs - robot[0], ys - robot[1])
    ok = dist >= min_dist
    for bx, by in blacklist:
        ok &= np.hypot(xs - bx, ys - by) > blacklist_radius
    if not ok.any():
        return None
    i = int(np.argmin(np.where(ok, dist, np.inf)))
    return float(xs[i]), float(ys[i])


def is_grasp(event, telemetry, front_min, approaching, target_recent):
    """True when the board stopped on the front PSD while closing in on a seen target.

    PSD 근접은 정체를 증명하지 않으므로, 접근 중이고 카메라가 방금 목표물을 중앙에 봤을 때만
    파지로 봅니다. 그 외의 PSD 정지는 벽·장애물·상대 로봇입니다.
    front_min은 보드 임계(550)가 아니라 "정면에 무언가 가깝다"는 기준입니다. 보드는 표본 하나가
    550을 넘으면 멈추지만 20 Hz 텔레메트리에는 그 표본이 안 잡힐 수 있습니다(실물 3회차: 큐브 앞
    정지 때 텔레메트리 최대 461~520). 집게가 열리고 앞이 비면 정면은 185 이하입니다.
    """
    return (approaching and target_recent and 'PSD_BLOCKED' in event
            and telemetry[FRONT] >= front_min)


def backoff_direction(telemetry, threshold):
    """Straight-line escape after a PSD stop: -1 reverse, +1 forward, 0 none/boxed in.

    펌웨어는 직진에 정면 또는 후면만, 회전에는 네 방향을 모두 검사합니다. 그래서 측면이
    막혔을 때 빠지는 길은 직진뿐입니다.
    """
    front = telemetry[FRONT] >= threshold
    rear = telemetry[REAR] >= threshold
    side = max(telemetry[LEFT], telemetry[RIGHT]) >= threshold
    if (front or side) and not rear:
        return -1
    if rear and not front:
        return 1
    return 0


def side_block(ranges, angle_min, angle_inc, yaw_offset, reach, half_width, min_points=3):
    """+1 (left) / -1 (right) when LiDAR points sit beside the front half of the body, else 0.

    직진하면 몸통 옆이 그 점을 긁거나 밉니다 (2026-10-16 11회차: 큐브 접근 중 옆면에 걸린 장애물을
    그대로 밀고 감 — collision monitor 원은 몸체 바깥선과 같아 여유가 없고, 측면 PSD 정지는 꺼져 있음).
    50 mm 큐브는 라이다(약 170 mm) 아래라 여기 안 잡힙니다. 정면(x > reach)은 collision monitor·정면 PSD 몫.
    무효 측정(0.0·inf)은 버리고, 잡음 한두 점은 무시합니다.
    """
    side = 0
    count = 0
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0:
            continue
        a = angle_min + i * angle_inc + yaw_offset
        x, y = r * math.cos(a), r * math.sin(a)
        if 0.0 <= x <= reach and abs(y) <= half_width:
            count += 1
            side += 1 if y > 0 else -1
    if count < min_points:
        return 0
    return 1 if side >= 0 else -1


def spin_plan(ranges, angle_min, angle_inc, yaw_offset, open_dist, half_fov_deg, min_gap_deg):
    """Rotation (ccw, cw) in degrees that sweeps the camera over every open direction.

    벽 높이 200 mm는 50 mm 큐브를 가리므로 벽을 향해 도는 것은 시간 낭비입니다. 라이다 한 번의 스캔에서
    open_dist보다 멀리 트인 방향(무효 0.0·inf는 먼 곳으로 봄)만 남기고, min_gap_deg보다 좁은 틈은 버립니다.
    카메라가 정면 ±half_fov_deg를 보므로 반시계로 θ 돌면 [-half, θ+half]가 화면에 들어옵니다.
    yaw_offset: base_link 기준 라이다 각도 (rad). 0도 = 로봇 정면. 트인 방향이 없으면 (0, 0).
    """
    bins = np.zeros(360, bool)
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0 or r > open_dist:
            bins[int(round(math.degrees(angle_min + i * angle_inc + yaw_offset))) % 360] = True
    if bins.all():
        open_deg = np.arange(360)
    else:
        start = int(np.argmin(bins))  # 닫힌 칸에서 시작하면 원형 구간이 끊기지 않음
        rolled, keep, run = np.roll(bins, -start), np.zeros(360, bool), []
        for k in range(361):
            if k < 360 and rolled[k]:
                run.append(k)
            else:
                if len(run) >= min_gap_deg:
                    keep[run] = True
                run = []
        open_deg = (np.nonzero(keep)[0] + start) % 360
    if not len(open_deg):
        return 0.0, 0.0
    half = half_fov_deg
    ccw = max(0.0, float(max((d + half) % 360 for d in open_deg)) - 2 * half)
    cw = max(0.0, float(max((-d + half) % 360 for d in open_deg)) - 2 * half)
    return ccw, cw


def wrap(angle):
    """Angle folded into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
