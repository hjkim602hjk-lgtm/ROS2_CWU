"""Mission.corridor_mid: middle of the one-cell corridor ahead from LiDAR points (no ROS needed)."""
import math
from types import SimpleNamespace

import pytest

mission1 = pytest.importorskip('cwu_nav.mission1')


def wall_points(offset):
    """Corridor 0.4 m wide whose middle is `offset` m left of the path line, walls from 0.2 to 0.6 m ahead."""
    pts = []
    for k in range(9):
        x = 0.2 + 0.05 * k
        pts += [(x, offset + 0.2), (x, offset - 0.2)]
    return pts


def mid(points, pose=(0.0, 0.0, 0.0)):
    stub = SimpleNamespace(points=lambda: points)
    return mission1.Mission.corridor_mid(stub, pose, (0.0, 0.0), (1.0, 0.0))


def test_corridor_offset_measured():
    assert mid(wall_points(0.08)) == pytest.approx(0.08, abs=1e-6)
    assert mid(wall_points(-0.05)) == pytest.approx(-0.05, abs=1e-6)


def test_robot_turned_sideways_still_measures_in_path_frame():
    # 로봇이 경로 왼쪽(+90°)을 보고 있어도 경로 기준으로 같은 값
    turned = [(y, -x) for x, y in wall_points(0.06)]
    assert mid(turned, (0.0, 0.0, math.pi / 2)) == pytest.approx(0.06, abs=1e-6)


def test_one_wall_or_nothing_is_not_a_corridor():
    assert mid([(x, y) for x, y in wall_points(0.0) if y > 0]) is None
    assert mid([]) is None
