# 엔코더 틱을 로봇 위치로 바꾸는 순수 계산 함수 모음입니다.
# 카운터 되감김 보정, 틱 → 이동거리 환산, 차동구동 호(arc) 적분을 담당합니다.
# ROS와 하드웨어에 의존하지 않으므로 로봇 없이 pytest로 검증합니다.

"""Pure differential-drive odometry maths, testable without ROS or hardware."""
import math


def wrap_delta(previous, current, counter_bits):
    """누적 카운터가 되감겨도 올바른 증분을 돌려줍니다.

    MCU의 틱 카운터는 보통 고정 비트 폭의 부호 있는/없는 정수라 최대값에서
    최소값으로 점프합니다. 그 점프를 실제 이동으로 착각하면 지도가 한순간에
    튀므로, 증분이 카운터 범위의 절반을 넘으면 되감김으로 보고 보정합니다.
    counter_bits가 0이면 되감김이 없는 것으로 보고 그대로 뺍니다.
    """
    delta = current - previous
    if counter_bits <= 0:
        return delta
    span = 1 << counter_bits
    # 절반을 넘는 증분은 반대 방향의 작은 증분으로 해석하는 쪽이 항상 옳습니다.
    return (delta + span // 2) % span - span // 2


def ticks_to_metres(ticks, ticks_per_rev, wheel_radius):
    """바퀴 틱 수를 바닥 위 이동 거리(m)로 바꿉니다."""
    return ticks / ticks_per_rev * (2 * math.pi * wheel_radius)


def differential_step(left, right, wheel_separation):
    """좌우 바퀴 이동거리로부터 로봇 기준(body frame) 변위를 구합니다.

    직선 근사가 아니라 호를 그대로 적분합니다. 제자리 회전과 급선회에서
    직선 근사는 오차가 빠르게 누적되기 때문입니다.
    """
    distance = (left + right) / 2.0
    heading = (right - left) / wheel_separation
    if abs(heading) < 1e-9:
        return distance, 0.0, heading
    radius = distance / heading
    return radius * math.sin(heading), radius * (1 - math.cos(heading)), heading


def integrate(pose, left, right, wheel_separation):
    """이전 자세 (x, y, theta)에 한 스텝을 더한 새 자세를 돌려줍니다."""
    x, y, theta = pose
    dx, dy, dtheta = differential_step(left, right, wheel_separation)
    return (x + dx * math.cos(theta) - dy * math.sin(theta),
            y + dx * math.sin(theta) + dy * math.cos(theta),
            normalise(theta + dtheta))


def normalise(angle):
    """각도를 -pi ~ pi 범위로 접습니다."""
    return math.atan2(math.sin(angle), math.cos(angle))
