# 시각 서보 계산의 하드웨어 없는 검사입니다.
# 부호(목표물이 오른쪽이면 양수 오차 → mission1이 시계 방향 TR로 돎)가 핵심입니다.
# 부호가 뒤집히면 로봇은 목표물 반대편으로 돌아 영영 못 찾습니다.

"""Checks for servo.py; no ROS, no robot."""
import math

from cwu_nav.servo import yaw_error


def test_target_on_the_centre_line_has_no_error():
    assert yaw_error(0.0, 1.0) == 0.0


def test_target_to_the_right_is_a_positive_error():
    # 광학 프레임 +x는 오른쪽입니다.
    assert yaw_error(0.5, 1.0) > 0
    assert math.isclose(math.degrees(yaw_error(math.tan(math.radians(17)), 1.0)), 17.0)
