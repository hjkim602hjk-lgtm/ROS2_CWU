# 빨간 목표물 검출의 순수 계산 부분입니다. ROS도 카메라도 참조하지 않습니다.
# 색 마스크 → 덩어리 선택 → 광학 프레임 단위 방향까지를 함수로 나눠
# 하드웨어 없이 정지 이미지로 검증할 수 있게 했습니다.
#
# 깊이는 쓰지 않습니다. 카메라는 "목표물이 어느 쪽에 있는가"만 답하고,
# 거리 판단은 그리퍼 PSD가 맡습니다.

"""Pure image maths for the red cube detector; no ROS, no camera, no depth."""
import cv2
import numpy as np


def red_mask(bgr, low1, high1, low2, high2):
    """Binary mask of red pixels. Hue wraps at 0, so red needs two ranges."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, np.array(low1, np.uint8), np.array(high1, np.uint8)) | \
        cv2.inRange(hsv, np.array(low2, np.uint8), np.array(high2, np.uint8))


def largest_blob(mask, min_area_px, max_aspect, min_fill):
    """Return (u, v) of the biggest cube-shaped red region, else None.

    A 50 mm cube is convex and roughly square in the image, so a low fill ratio
    or a long thin box is reflection or clutter, not the target.
    """
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    best = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area_px:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if not h or not w:
            continue
        aspect = max(w / h, h / w)
        if aspect > max_aspect or area / float(w * h) < min_fill:
            continue
        if best is None or area > best[0]:
            best = (area, contour)
    if best is None:
        return None
    moments = cv2.moments(best[1])
    if moments['m00'] <= 0:
        return None
    return moments['m10'] / moments['m00'], moments['m01'] / moments['m00']


def bearing(u, v, fx, fy, cx, cy):
    """Unit direction to a pixel in the camera OPTICAL frame (REP-145: z forward,
    x right, y down).

    거리를 모르므로 위치가 아니라 방향만 돌려줍니다. 길이가 1이라는 것 외에
    어떤 거리도 주장하지 않습니다. 세로 중심선 정렬은 x 성분만 보면 됩니다.
    """
    if fx <= 0 or fy <= 0:
        raise ValueError('camera_info intrinsics must be positive')
    x, y, z = (u - cx) / fx, (v - cy) / fy, 1.0
    norm = (x * x + y * y + z * z) ** 0.5
    return x / norm, y / norm, z / norm
