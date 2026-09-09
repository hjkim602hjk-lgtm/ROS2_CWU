# 가상 LiDAR 거리 계산을 하드웨어 없이 검사하는 pytest 테스트 파일입니다.
# 벽 교점, 센서 위치·방향, 가까운 장애물의 차폐, 평행·후방 선분과 측정 범위를 검증합니다.

"""Catch incorrect intersection math, missing nearest-hit selection and range limits."""
import math

import pytest

from cwu_slam.geometry import raycast


WALLS = [(2, -2, 2, 2), (2, 2, -2, 2), (-2, 2, -2, -2), (-2, -2, 2, -2)]


@pytest.mark.parametrize('angle, expected', [
    (0, 2), (math.pi / 2, 2), (math.pi, 2), (-math.pi / 2, 2),
    (math.pi / 4, math.sqrt(8)),
])
def test_room_intersections(angle, expected):
    assert raycast(0, 0, angle, WALLS, 0.1, 10) == pytest.approx(expected)


def test_translated_origin_and_rotated_beam():
    assert raycast(1, -1, math.pi / 2, WALLS, 0.1, 10) == pytest.approx(3)


def test_occluder_wins_regardless_of_segment_order():
    assert raycast(0, 0, 0, WALLS + [(1, -1, 1, 1)], 0.1, 10) == 1


def test_parallel_and_behind_segments_do_not_hit():
    assert math.isinf(raycast(0, 0, 0, [(0, 1, 2, 1), (-1, -1, -1, 1)], .1, 10))


def test_outside_segment_is_not_an_infinite_line():
    assert math.isinf(raycast(0, 0, 0, [(1, 1, 1, 2)], .1, 10))


@pytest.mark.parametrize('minimum, maximum', [(0.1, 1), (3, 10)])
def test_out_of_range_is_infinity(minimum, maximum):
    assert math.isinf(raycast(0, 0, 0, WALLS, minimum, maximum))


def test_near_blind_zone_does_not_see_through_obstacle():
    assert math.isinf(raycast(0, 0, 0, WALLS + [(.05, -1, .05, 1)], .1, 10))
