# 미션 1 판단 함수의 하드웨어 없는 검사입니다.
# 프론티어가 벽에 붙은 칸·잡음·실패한 곳을 고르지 않는지, PSD 정지를 파지와 장애물로
# 바르게 가르는지, 막힌 쪽 반대로 빠지는지를 확인합니다.

"""Checks for mission_logic.py; no ROS, no robot."""
import math

import numpy as np

from cwu_nav.mission_logic import (backoff_direction, clear_turn, dead_end, frontier_goal, hint_goal, is_grasp,
                                   side_block, spin_plan)

RES = 0.05


def telemetry(front=0, rear=0, left=0, right=0):
    return [0, 0, 0, 0, 0, 0, front, rear, left, right, 40, 140]


def corridor_map():
    """2 m x 1 m seen floor with walls on top/bottom, unknown beyond x = 2 m."""
    grid = np.full((20, 60), -1, np.int8)   # rows = y (1 m), cols = x (3 m)
    grid[:, :40] = 0                        # seen free floor x < 2 m
    grid[0, :40] = grid[-1, :40] = 100      # walls along y = 0 and y = 1 m
    return grid


def test_hint_goal_stops_short_of_the_centre():
    x, y = hint_goal((1.6, 1.6), 0.6)
    assert math.isclose(math.hypot(x, y), math.hypot(1.6, 1.6) - 0.6)
    assert math.isclose(x, y)               # same direction as the hint
    assert hint_goal((0.3, 0.0), 0.6) == (0.0, 0.0)


def test_frontier_is_the_open_edge_away_from_walls():
    goal = frontier_goal(corridor_map(), RES, (0.0, 0.0), (0.5, 0.5),
                         clearance=0.2, min_cells=3, min_dist=0.5)
    assert goal is not None
    x, y = goal
    assert math.isclose(x, 1.975)           # last seen column, next to unknown
    assert 0.2 <= y <= 0.8                  # not hugging either wall


def test_blacklisted_and_too_close_frontiers_are_skipped():
    args = dict(clearance=0.2, min_cells=3, min_dist=0.5)
    first = frontier_goal(corridor_map(), RES, (0.0, 0.0), (0.5, 0.5), **args)
    assert frontier_goal(corridor_map(), RES, (0.0, 0.0), (0.5, 0.5),
                         blacklist=[first], blacklist_radius=0.4, **args) is None
    assert frontier_goal(corridor_map(), RES, (0.0, 0.0), (1.9, 0.5),
                         clearance=0.2, min_cells=3, min_dist=1.0) is None


def test_floor_seen_over_a_wall_is_not_a_goal_but_any_gap_is_a_path():
    """Robot room x < 1 m; wall at x = 1 m; floor seen beyond it up to the unknown at x = 2 m."""
    def room_with_gap(gap_rows):
        grid = corridor_map()
        grid[:, 0] = 100                     # closed on the left too
        grid[:, 20] = 100                    # wall at x = 1 m
        grid[gap_rows, 20] = 0
        return grid
    args = dict(clearance=0.15, min_cells=3, min_dist=0.5, passage=0.075)
    # closed wall (LiDAR saw over it): straight-line close, but no way through → no goal
    assert frontier_goal(room_with_gap(slice(0, 0)), RES, (0.0, 0.0), (0.5, 0.5), **args) is None
    # a 400 mm opening drawn as 0.25 m (walls one cell thick into it) is still a path
    goal = frontier_goal(room_with_gap(slice(8, 13)), RES, (0.0, 0.0), (0.5, 0.5), **args)
    assert goal is not None and math.isclose(goal[0], 1.975)


def scan_of(points):
    """360 one-degree rays from (x, y) points in the robot frame (laser yaw 0); others invalid 0.0."""
    ranges = [0.0] * 360
    for x, y in points:
        ranges[int(round(math.degrees(math.atan2(y, x)))) % 360] = math.hypot(x, y)
    return ranges, 0.0, math.radians(1.0)


def test_side_block_sees_an_obstacle_beside_the_front_half_only():
    def block(points):
        ranges, a0, inc = scan_of(points)
        return side_block(ranges, a0, inc, 0.0, reach=0.165, half_width=0.165)
    beside_left = [(x, 0.155) for x in (0.02, 0.05, 0.08, 0.11)]  # inside the body width: straight ahead hits it
    assert block(beside_left) == 1
    assert block([(x, -y) for x, y in beside_left]) == -1
    # wall running alongside, 0.17~0.21 m (5 mm~4.5 cm off the body, 14회차): passes by, never pushed
    assert block([(x / 100, 0.17 + x / 400) for x in range(0, 17)]) == 0
    # 400 mm corridor, robot centred: walls 0.20 m to each side → clear
    assert block([(x, s * 0.20) for x in (0.0, 0.05, 0.1, 0.15) for s in (1, -1)]) == 0
    # wall straight ahead (front PSD / collision monitor job) and points behind → clear
    assert block([(0.30, y) for y in (-0.05, 0.0, 0.05)] + [(-0.1, 0.17), (-0.05, 0.17), (-0.12, 0.16)]) == 0
    # one or two noise points do not stop the approach
    assert block([(0.05, 0.155), (0.08, 0.155)]) == 0


def test_clear_turn_steers_away_from_a_converging_wall():
    guard = dict(reach=0.165, half_width=0.165, max_deg=25)
    # 15회차: right wall closing in ahead (0.20 m beside the body, 0.15 m at x = 0.15)
    wall = [(x / 100, -(0.20 - x / 300)) for x in range(-10, 31)]
    ranges, a0, inc = scan_of(wall)
    assert side_block(ranges, a0, inc, 0.0, 0.165, 0.165) == -1
    turn = clear_turn(ranges, a0, inc, 0.0, side=-1, **guard)
    assert turn is not None and 0 < turn <= math.radians(25)          # turn left, away from it
    assert not side_block(ranges, a0, inc, -turn, 0.165, 0.165)
    # mirrored: wall on the left → turn right
    ranges, a0, inc = scan_of([(x, -y) for x, y in wall])
    assert clear_turn(ranges, a0, inc, 0.0, side=1, **guard) < 0
    # boxed in on both sides inside the body width → no way past, back off instead
    ranges, a0, inc = scan_of([(x / 100, s * 0.15) for x in range(-5, 20) for s in (1, -1)])
    assert clear_turn(ranges, a0, inc, 0.0, side=1, **guard) is None


def box_scan(front, left, right, back=math.inf):
    """360 one-degree rays (laser yaw 0) hitting flat walls at these distances; inf = open."""
    ranges = []
    for d in range(360):
        a = math.radians(d)
        c, s = math.cos(a), math.sin(a)
        hits = [w / v for w, v in ((front, c), (back, -c), (left, s), (right, -s)) if v > 1e-9]
        ranges.append(min(hits))
    return ranges, 0.0, math.radians(1.0)


def test_dead_end_needs_walls_ahead_left_and_right_and_the_way_back_open():
    limits = dict(front_max=0.35, side_max=0.35, back_min=0.5)

    def de(front, left, right, back=math.inf):
        ranges, a0, inc = box_scan(front, left, right, back)
        return dead_end(ranges, a0, inc, 0.0, **limits)
    assert de(0.2, 0.2, 0.2)                 # last cell of a 400 mm dead-end corridor
    assert de(0.2, 0.2, 0.2, back=0.6)       # one-cell pocket off a corridor
    assert not de(0.6, 0.2, 0.2)             # corridor whose end is still a cell away (may turn there)
    assert not de(0.2, 0.2, math.inf)        # L-turn: open to the right
    assert not de(0.2, math.inf, math.inf)   # T-junction
    assert not de(math.inf, 0.2, 0.2)        # straight corridor
    assert not de(0.2, 0.2, 0.2, back=0.2)   # closed box: nowhere to turn back to
    # turned around in the dead end, facing the exit: the end wall is now behind → not again
    assert not de(math.inf, 0.2, 0.2, back=0.2)
    # laser mounted backwards (yaw pi): same dead end seen through the offset
    ranges, a0, inc = box_scan(0.2, 0.2, 0.2)
    flipped = ranges[180:] + ranges[:180]
    assert dead_end(flipped, a0, inc, math.pi, **limits)


def test_single_cell_noise_is_not_a_frontier():
    grid = np.zeros((20, 20), np.int8)
    grid[10, 10] = -1                        # one unknown speck inside seen floor
    assert frontier_goal(grid, RES, (0.0, 0.0), (0.0, 0.0),
                         clearance=0.1, min_cells=12, min_dist=0.1) is None


def test_grasp_only_on_front_psd_while_approaching_a_seen_target():
    near = telemetry(front=600)
    assert is_grasp('# ABORT PSD_BLOCKED', near, 550, True, True)
    assert not is_grasp('# ABORT PSD_BLOCKED', near, 550, False, True)   # exploring: wall
    assert not is_grasp('# ABORT PSD_BLOCKED', near, 550, True, False)   # target not seen
    assert not is_grasp('# ABORT PSD_BLOCKED', telemetry(left=600), 550, True, True)
    assert is_grasp('# ABORT PSD_BLOCKED', telemetry(front=461), 300, True, True)  # 보드만 550 표본을 봄
    assert not is_grasp('# ABORT PSD_BLOCKED', telemetry(front=185, left=600), 300, True, True)
    assert not is_grasp('# ABORT ENCODER_NO_PULSE', near, 550, True, True)


def test_backoff_moves_straight_away_from_the_blocked_side():
    assert backoff_direction(telemetry(front=600), 550) == -1
    assert backoff_direction(telemetry(right=600), 550) == -1   # side: rotation blocked
    assert backoff_direction(telemetry(rear=600), 550) == 1
    assert backoff_direction(telemetry(rear=600, left=600), 550) == 1
    assert backoff_direction(telemetry(front=600, rear=600), 550) == 0
    assert backoff_direction(telemetry(), 550) == 0


def scan(open_from=None, open_to=None, near=0.3, far=2.0):
    """360 rays at 1 deg, base frame (0 = ahead, +CCW); open (far) between open_from..open_to deg."""
    rays = []
    for d in range(360):
        a = (d + 180) % 360 - 180
        inside = open_from is not None and open_from <= a <= open_to
        rays.append(far if inside else near)
    return rays, math.radians(0), math.radians(1)


def test_spin_skips_walls():
    rays, a0, inc = scan()
    assert spin_plan(rays, a0, inc, 0.0, 0.6, 30, 5) == (0.0, 0.0)          # 사방이 벽: 돌지 않음
    rays, a0, inc = scan(-20, 20)
    assert spin_plan(rays, a0, inc, 0.0, 0.6, 30, 5) == (0.0, 0.0)          # 정면만 트임: 이미 보임
    rays, a0, inc = scan(80, 100)
    ccw, cw = spin_plan(rays, a0, inc, 0.0, 0.6, 30, 5)
    assert ccw == 70.0 and cw > ccw                                         # 왼쪽만 트임: 반시계 70도
    rays, a0, inc = scan(-180, 180)
    assert spin_plan(rays, a0, inc, 0.0, 0.6, 30, 5) == (299.0, 299.0)      # 다 트임: 한 바퀴 - 화각


def test_spin_ignores_narrow_slits_and_uses_lidar_mount():
    rays, a0, inc = scan(88, 90)                                            # 3도 틈
    assert spin_plan(rays, a0, inc, 0.0, 0.6, 30, 5) == (0.0, 0.0)
    rays, a0, inc = scan(80, 100)
    ccw, _ = spin_plan(rays, a0, inc, math.pi, 0.6, 30, 5)                  # 라이다가 거꾸로 달림
    assert ccw == 250.0                                                     # 트인 곳은 로봇 기준 오른쪽(−90도)
