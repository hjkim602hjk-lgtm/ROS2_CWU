# 미션 2 판단 함수(mission2_logic.py) 검사. 로봇·ROS 없이 돕니다.
import math

from cwu_nav.mission2_logic import (corner_guard, flee_target, held_by, in_corner, push_plan, pursue, ring_target,
                                    robot_ahead, robot_on_bearing, robots, wall_fix)


def wall_points(dx=0.0, dy=0.0, step=0.02):
    """Points on the four walls (±1.8) seen from a pose that is off by (dx, dy): arena points shifted by -d."""
    s = [i * step - 1.8 for i in range(int(3.6 / step) + 1)]
    pts = [(x, 1.8) for x in s] + [(x, -1.8) for x in s] + [(1.8, y) for y in s] + [(-1.8, y) for y in s]
    return [(x - dx, y - dy) for x, y in pts]


def test_in_corner():
    assert in_corner(-1.6, -1.6)
    assert not in_corner(-1.6, -1.2)   # (1,2) 칸: 모서리 아님
    assert in_corner(-1.3, -1.3, margin=0.15)


def test_corner_guard_stops_driving_into_corner():
    v, w = corner_guard((-1.2, -1.2, math.radians(-135)), 0.15, 0.3, margin=0.2, lookahead=0.5)
    assert v == 0.0 and w == 0.3


def test_corner_guard_lets_robot_leave_corner():
    v, _ = corner_guard((-1.3, -1.3, math.radians(45)), 0.15, 0.0, margin=0.2, lookahead=0.5)
    assert v == 0.15


def test_corner_guard_free_elsewhere():
    assert corner_guard((0.0, 0.0, 0.0), 0.15, 0.0, margin=0.2, lookahead=0.5) == (0.15, 0.0)


def test_ring_target_counter_clockwise():
    x, y = ring_target((0.8, 0.0, 0.0), half=0.8, lead=0.3)   # 오른쪽 변 가운데 → 반시계 = +y
    assert math.isclose(x, 0.8) and math.isclose(y, 0.3)
    x, y = ring_target((0.0, 0.0, 0.0), half=0.8, lead=0.3)   # 안쪽에서도 고리 위 점
    assert math.isclose(max(abs(x), abs(y)), 0.8)


def test_ring_target_wraps_corner():
    x, y = ring_target((0.8, 0.7, 0.0), half=0.8, lead=0.3)
    assert math.isclose(y, 0.8) and math.isclose(x, 0.6)


def test_pursue_turns_first_when_behind():
    v, w = pursue((0, 0, 0), (-1, 0), 0.15, 1.5, 0.6)
    assert v == 0.0 and w != 0.0
    v, w = pursue((0, 0, 0), (1, 0.01), 0.15, 1.5, 0.6)
    assert v > 0.1


def test_wall_fix_recovers_shift():
    dx, dy = wall_fix(wall_points(0.15, -0.1), tol=0.3, min_points=15)
    assert abs(dx - 0.15) < 0.01 and abs(dy + 0.1) < 0.01


def test_wall_fix_ignores_robot_near_wall():
    pts = wall_points() + [(1.6 + 0.01 * k, 0.3 + 0.01 * k) for k in range(15)]   # 벽 앞 로봇
    dx, dy = wall_fix(pts, tol=0.3, min_points=15)
    assert abs(dx) < 0.01 and abs(dy) < 0.01


def test_wall_fix_none_without_walls():
    assert wall_fix([(0.1, 0.1)] * 50, tol=0.3, min_points=15) is None


def test_robots_cluster_and_drop_walls():
    blob = [(0.5 + 0.02 * k, 0.4) for k in range(8)]
    found = robots(wall_points() + blob, wall_margin=0.15, gap=0.1, min_points=3, max_extent=0.5)
    assert len(found) == 1 and math.dist(found[0], (0.57, 0.4)) < 0.1


def test_robot_on_bearing():
    opp = [(1.0, 0.0), (0.0, 1.0)]
    assert robot_on_bearing((0, 0), 0.05, opp, tol=0.15) == (1.0, 0.0)
    assert robot_on_bearing((0, 0), math.pi, opp, tol=0.15) is None


def test_push_plan_goes_behind_then_pushes():
    opp = (0.8, 0.8)            # 가장 가까운 모서리 = (+,+)
    target, pushing = push_plan((0.0, 0.8, 0.0), opp, behind=0.4, lateral_tol=0.12)
    assert not pushing and target[0] < 0.8 and target[1] < 0.8      # 상대 뒤(중앙 쪽) 자리로
    target, pushing = push_plan((0.5, 0.5, math.pi / 4), opp, behind=0.4, lateral_tol=0.12)
    assert pushing and target[0] > 0.8 and target[1] > 0.8


def test_push_plan_keeps_pushing_when_opponent_slides():
    opp = (0.8, 0.8)
    own = (0.5 + 0.12, 0.5 - 0.12, 0.0)          # 옆으로 0.17 m 어긋남
    assert not push_plan(own, opp, behind=0.4, lateral_tol=0.12)[1]
    assert push_plan(own, opp, behind=0.4, lateral_tol=0.12, pushing=True)[1]


def test_flee_target_moves_away_but_not_into_corner():
    tx, ty = flee_target((0.0, 0.0, 0.0), (0.3, 0.0), step=0.4, margin=0.2)
    assert tx < 0.0
    tx, ty = flee_target((-1.0, -1.0, 0.0), (-0.7, -0.7), step=0.4, margin=0.2)
    assert not in_corner(tx, ty, 0.2)


def test_robot_ahead():
    assert robot_ahead([(0.25, 0.02)] * 4, half_width=0.12, reach=0.35)
    assert not robot_ahead([(0.25, 0.3)] * 4, half_width=0.12, reach=0.35)
    assert not robot_ahead([(0.25, 0.0)] * 2, half_width=0.12, reach=0.35)   # 잡음 두 점


def test_held_by():
    own = (-1.2, -1.2)
    holder = (-0.2, -1.0)                  # 중앙에서 떨어져 있고 중앙보다 가까움
    b = math.atan2(holder[1] - own[1], holder[0] - own[0])
    assert held_by(own, b, [holder], 0.1, 0.35) == holder
    beyond = (0.5, 0.5)                    # 중앙 너머: 바닥 큐브가 그 앞 중앙에 있을 수 있음
    assert held_by(own, math.radians(45), [beyond], 0.1, 0.35) is None
    near_centre = (0.1, 0.1)
    assert held_by(own, math.radians(45), [near_centre], 0.1, 0.35) is None


def test_wall_fix_near_corner_ignores_other_wall():
    # (2,2) 근처에서 본 벽: 아래 벽·왼쪽 벽이 모서리에서 만남. 모서리 근처의 아래 벽 점이 "x 벽 근처"로 섞여 잔차를 끌면
    # 보정할수록 더 섞여 위치가 1 m 넘게 틀어졌음 (2026-10-30 Gazebo 배틀 1)
    s = [i * 0.01 for i in range(130)]
    pts = [(-1.8 + d, -1.8) for d in s] + [(-1.8, -1.8 + d) for d in s]
    dx, dy = wall_fix([(x - 0.1, y - 0.05) for x, y in pts], tol=0.3, min_points=15)
    assert abs(dx - 0.1) < 0.01 and abs(dy - 0.05) < 0.01
