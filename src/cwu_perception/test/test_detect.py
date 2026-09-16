# 목표물 검출 계산의 하드웨어 없는 검사입니다.
# 회색 배경 위 빨간 사각형 합성 이미지로 색 마스크·덩어리 선택·방향 계산을 확인합니다.

"""Synthetic-image checks for detect.py; no camera, no ROS, no depth."""
import numpy as np
import pytest

from cwu_perception.detect import bearing, largest_blob, red_mask

RED_1 = ([0, 120, 60], [10, 255, 255])
RED_2 = ([170, 120, 60], [179, 255, 255])
ARENA_GRAY = 127


def scene(box=(40, 30, 20, 20), color=(0, 0, 255)):
    """Gray arena floor with one coloured box at (x, y, w, h), in BGR."""
    image = np.full((120, 160, 3), ARENA_GRAY, np.uint8)
    x, y, w, h = box
    image[y:y + h, x:x + w] = color
    return image


def find(image, min_area_px=60, max_aspect=2.0, min_fill=0.5):
    mask = red_mask(image, RED_1[0], RED_1[1], RED_2[0], RED_2[1])
    return largest_blob(mask, min_area_px, max_aspect, min_fill)


def test_red_box_found_at_its_centre():
    blob = find(scene(box=(40, 30, 20, 20)))
    assert blob is not None
    u, v = blob
    assert u == pytest.approx(49.5, abs=1.0)
    assert v == pytest.approx(39.5, abs=1.0)


def test_gray_arena_alone_detects_nothing():
    # 회색 RGB(127,127,127)은 채도가 0이라 어떤 색상 구간에도 걸리면 안 됩니다.
    assert find(scene(box=(0, 0, 0, 0))) is None


def test_blue_object_is_not_a_target():
    assert find(scene(color=(255, 0, 0))) is None


def test_small_red_speck_is_rejected():
    assert find(scene(box=(40, 30, 5, 5))) is None


def test_long_thin_red_streak_is_rejected():
    # 반사광이나 배선처럼 길쭉한 빨강은 큐브가 아닙니다.
    assert find(scene(box=(20, 30, 60, 6))) is None


def test_largest_of_two_red_regions_wins():
    image = scene(box=(10, 10, 12, 12))
    image[60:90, 100:130] = (0, 0, 255)
    blob = find(image)
    assert blob is not None
    assert blob[0] == pytest.approx(114.5, abs=1.5)


def test_bearing_of_centre_pixel_points_straight_ahead():
    assert bearing(80.0, 60.0, 300.0, 300.0, 80.0, 60.0) == (0.0, 0.0, 1.0)


def test_bearing_right_and_below_centre_is_positive_x_and_y():
    # REP-145 광학 프레임: +x 오른쪽, +y 아래, +z 전방.
    x, y, z = bearing(110.0, 90.0, 300.0, 300.0, 80.0, 60.0)
    assert x > 0 and y > 0 and z > 0
    assert x == pytest.approx(y)


def test_bearing_is_a_unit_vector():
    # 길이가 1이라는 것 외에 어떤 거리도 주장하지 않아야 합니다.
    x, y, z = bearing(10.0, 110.0, 300.0, 300.0, 80.0, 60.0)
    assert (x * x + y * y + z * z) ** 0.5 == pytest.approx(1.0)


def test_bearing_grows_with_distance_from_the_centre_line():
    near = bearing(90.0, 60.0, 300.0, 300.0, 80.0, 60.0)[0]
    far = bearing(140.0, 60.0, 300.0, 300.0, 80.0, 60.0)[0]
    assert 0 < near < far


def test_bearing_rejects_missing_intrinsics():
    with pytest.raises(ValueError):
        bearing(10.0, 10.0, 0.0, 300.0, 80.0, 60.0)
