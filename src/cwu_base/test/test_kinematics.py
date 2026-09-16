# 엔코더 오도메트리 수식을 하드웨어 없이 검사하는 pytest 테스트 파일입니다.
# 카운터 되감김, 틱 환산, 직진·제자리회전·호 적분과 좌우 부호를 검증합니다.

"""Catch wrap-around errors, straight/turn confusion and bad arc integration."""
import math

import pytest

from cwu_base.kinematics import (differential_step, integrate, normalise,
                                 ticks_to_metres, wrap_delta)

SEP = 0.2


def test_plain_delta_without_wrapping():
    assert wrap_delta(100, 150, 32) == 50
    assert wrap_delta(150, 100, 32) == -50


@pytest.mark.parametrize('bits, top', [(16, 1 << 16), (32, 1 << 32)])
def test_counter_wrap_is_a_small_step_not_a_jump(bits, top):
    assert wrap_delta(top - 3, 2, bits) == 5       # 앞으로 넘어감
    assert wrap_delta(2, top - 3, bits) == -5      # 뒤로 넘어감


def test_zero_bits_disables_wrap_correction():
    assert wrap_delta(0, 1 << 40, 0) == 1 << 40


def test_one_revolution_is_the_wheel_circumference():
    assert ticks_to_metres(1000, 1000, 0.0325) == pytest.approx(2 * math.pi * 0.0325)


def test_straight_line_has_no_rotation():
    assert differential_step(0.5, 0.5, SEP) == pytest.approx((0.5, 0.0, 0.0))


def test_spin_in_place_does_not_translate():
    dx, dy, dtheta = differential_step(-0.1, 0.1, SEP)
    assert (dx, dy) == pytest.approx((0.0, 0.0))
    assert dtheta == pytest.approx(1.0)


def test_quarter_circle_arc_matches_geometry():
    # 반지름 1 m 원호를 90도. 중심이 왼쪽이므로 (1, 1)이 아니라 (1, 1)이 아닌
    # body frame 기준 (sin90, 1-cos90) x R = (1, 1)입니다.
    turn = math.pi / 2
    left, right = (1 - SEP / 2) * turn, (1 + SEP / 2) * turn
    dx, dy, dtheta = differential_step(left, right, SEP)
    assert (dx, dy, dtheta) == pytest.approx((1.0, 1.0, turn))


def test_arc_is_not_the_straight_line_approximation():
    left, right = 0.4, 0.6
    dx, dy, _ = differential_step(left, right, SEP)
    assert dy > 0.0                      # 직선 근사라면 0이 나옵니다.
    assert dx < (left + right) / 2       # 호의 현은 호보다 짧습니다.


def test_integration_applies_displacement_in_the_current_heading():
    pose = integrate((0.0, 0.0, math.pi / 2), 1.0, 1.0, SEP)
    assert pose == pytest.approx((0.0, 1.0, math.pi / 2), abs=1e-9)


def test_full_circle_returns_to_the_start():
    turn, steps = 2 * math.pi, 720
    left, right = (1 - SEP / 2) * turn / steps, (1 + SEP / 2) * turn / steps
    pose = (0.0, 0.0, 0.0)
    for _ in range(steps):
        pose = integrate(pose, left, right, SEP)
    assert pose[:2] == pytest.approx((0.0, 0.0), abs=1e-3)


def test_swapped_wheels_turn_the_other_way():
    assert differential_step(0.1, -0.1, SEP)[2] == -differential_step(-0.1, 0.1, SEP)[2]


def test_heading_stays_folded():
    assert normalise(3 * math.pi) == pytest.approx(math.pi)
    assert integrate((0.0, 0.0, math.pi - 0.01), -0.1, 0.1, SEP)[2] < 0
