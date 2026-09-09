# 가상 LiDAR에 필요한 2D ray casting 거리 계산 파일입니다.
# 센서에서 쏜 광선과 벽 선분의 교점을 찾아 가장 가까운 장애물까지의 거리를 반환합니다.
# 교점이 없거나 측정 범위를 벗어나면 inf를 반환하며, 실제 센서 실행에는 사용하지 않습니다.

"""Small 2D ray caster used only by the synthetic sensor demo."""
import math


def raycast(x, y, angle, walls, range_min, range_max):
    """Return distance to the nearest segment, or inf for an invalid return."""
    dx, dy = math.cos(angle), math.sin(angle)
    nearest = math.inf
    for x1, y1, x2, y2 in walls:
        sx, sy = x2 - x1, y2 - y1
        denominator = dx * sy - dy * sx
        if abs(denominator) < 1e-12:
            continue
        qx, qy = x1 - x, y1 - y
        distance = (qx * sy - qy * sx) / denominator
        fraction = (qx * dy - qy * dx) / denominator
        if distance >= 0 and -1e-12 <= fraction <= 1 + 1e-12:
            nearest = min(nearest, distance)
    return nearest if range_min <= nearest <= range_max else math.inf
