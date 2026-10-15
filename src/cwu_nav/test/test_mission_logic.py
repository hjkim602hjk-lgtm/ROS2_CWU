# 미션 1 판단 함수의 하드웨어 없는 검사입니다.
# 프론티어가 벽에 붙은 칸·잡음·실패한 곳을 고르지 않는지, PSD 정지를 파지와 장애물로
# 바르게 가르는지, 막힌 쪽 반대로 빠지는지를 확인합니다.

"""Checks for mission_logic.py; no ROS, no robot."""
import math

import numpy as np

from cwu_nav.mission_logic import backoff_direction, frontier_goal, hint_goal, is_grasp, spin_plan

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
