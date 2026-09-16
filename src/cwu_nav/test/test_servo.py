# 시각 서보 계산의 하드웨어 없는 검사입니다.
# 부호(목표물이 오른쪽이면 오른쪽으로 돈다)와 정렬 전 전진 금지가 핵심입니다.
# 부호가 뒤집히면 로봇은 목표물 반대편으로 돌아 영영 못 찾습니다.

"""Checks for servo.py; no ROS, no robot."""
import math

import pytest

from cwu_nav.servo import follow, yaw_error

# k_yaw, max_yaw, min_yaw, align_tol, approach_speed
GAINS = (1.2, 0.8, 0.0, 0.09, 0.08)


def test_target_on_the_centre_line_has_no_error():
    assert yaw_error(0.0, 1.0) == 0.0


def test_target_to_the_right_is_a_positive_error():
    # 광학 프레임 +x는 오른쪽입니다.
    assert yaw_error(0.5, 1.0) > 0


def test_turning_is_opposite_to_the_error_sign():
    # 목표물이 오른쪽(+오차)이면 시계방향, 즉 음의 각속도로 돌아야 합니다.
    _, right = follow(0.3, *GAINS)
    _, left = follow(-0.3, *GAINS)
    assert right < 0 < left


def test_no_forward_motion_until_aligned():
    linear, angular = follow(0.5, *GAINS)
    assert linear == 0.0
    assert angular != 0.0


def test_forward_motion_once_aligned():
    linear, _ = follow(0.01, *GAINS)
    assert linear == pytest.approx(0.08)


def test_angular_speed_is_clamped():
    # 시야를 넘겨 버리면 목표물이 화면 밖으로 나가 제어가 끊깁니다.
    assert follow(math.pi, *GAINS)[1] == pytest.approx(-0.8)
    assert follow(-math.pi, *GAINS)[1] == pytest.approx(0.8)


def test_zero_error_commands_no_rotation():
    assert follow(0.0, *GAINS)[1] == 0.0


def test_min_yaw_lifts_small_commands_but_not_zero():
    # 정지 마찰 보정: 작은 오차에서도 실제로 돌게 하되, 오차 0에서 떨지 않아야 합니다.
    gains = (1.2, 0.8, 0.15, 0.09, 0.08)
    assert follow(0.01, *gains)[1] == pytest.approx(-0.15)
    assert follow(0.0, *gains)[1] == 0.0
