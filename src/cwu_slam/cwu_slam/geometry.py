# 가상 LiDAR에 필요한 2D ray casting 거리 계산 파일입니다.
# 센서에서 쏜 광선과 벽 선분의 교점을 찾아 가장 가까운 장애물까지의 거리를 반환합니다.
# 교점이 없거나 측정 범위를 벗어나면 inf를 반환하며, 실제 센서 실행에는 사용하지 않습니다.
#
# [공부 노트] 누가 쓰나?
#   demo_sensors.py(가짜 LiDAR)만 씁니다. 로봇 없이 SLAM 을 시험할 때
#   "이 위치에서 이 각도로 레이저를 쏘면 벽까지 몇 m?" 를 계산해 /scan 을 흉내냅니다.
#   실물에서는 진짜 G4 LiDAR 가 이 일을 하므로 이 파일은 안 쓰입니다.

"""Small 2D ray caster used only by the synthetic sensor demo."""
import math  # 표준 라이브러리: cos, sin, inf


# ─────────────────────────────────────────────────────────────────────────────
# raycast: 점 (x, y) 에서 angle 방향으로 광선을 쏴서 제일 가까운 벽까지 거리를 구합니다.
#   필요한 입력
#     x, y       : 센서 위치 (m, 지도 기준)
#     angle      : 광선 방향 (라디안, 지도 기준)
#     walls      : 벽 선분 목록 [(x1, y1, x2, y2), ...]  ← demo_sensors 가 만들어 넘김
#     range_min, range_max : 센서가 잴 수 있는 최소/최대 거리
#   돌려주는 값: 거리(m). 벽이 없거나 범위 밖이면 math.inf (= "측정 실패")
#   ※ LaserScan 규칙상 "못 쟀음"은 inf 로 표시합니다.
# ─────────────────────────────────────────────────────────────────────────────
def raycast(x, y, angle, walls, range_min, range_max):
    """Return distance to the nearest segment, or inf for an invalid return."""
    # 광선 방향 단위벡터 (길이 1)
    dx, dy = math.cos(angle), math.sin(angle)
    # 지금까지 찾은 가장 가까운 거리. 처음엔 "무한대"로 시작해서 더 작은 값이 나오면 갱신
    nearest = math.inf
    # 벽 하나씩 검사
    for x1, y1, x2, y2 in walls:
        # 벽 선분의 방향벡터 (끝점 - 시작점)
        sx, sy = x2 - x1, y2 - y1
        # 두 벡터의 2D 외적(cross product). 0 이면 광선과 벽이 평행 → 절대 안 만남
        denominator = dx * sy - dy * sx
        if abs(denominator) < 1e-12:
            continue  # 이 벽은 건너뛰고 다음 벽
        # 센서 위치 → 벽 시작점 벡터
        qx, qy = x1 - x, y1 - y
        # 두 직선의 교점을 푸는 공식 (크래머 공식)
        #   distance : 광선을 따라 몇 m 가서 만나는지  (음수면 센서 뒤쪽)
        #   fraction : 벽 선분 위 어느 지점인지 0(시작)~1(끝). 이 범위 밖이면 선분의 연장선
        distance = (qx * sy - qy * sx) / denominator
        fraction = (qx * dy - qy * dx) / denominator
        # 앞쪽이고(distance≥0) 선분 안(0~1)이면 진짜로 맞은 것. 1e-12 는 부동소수 오차 여유
        if distance >= 0 and -1e-12 <= fraction <= 1 + 1e-12:
            nearest = min(nearest, distance)
    # 센서 측정 범위 안이면 그 거리, 아니면 inf
    return nearest if range_min <= nearest <= range_max else math.inf
