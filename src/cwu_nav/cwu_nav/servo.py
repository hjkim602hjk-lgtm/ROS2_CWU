# 목표물 방향 하나로 바퀴 속도를 만드는 순수 계산입니다. ROS를 참조하지 않습니다.
# 깊이가 없으므로 "얼마나 남았는가"는 알 수 없습니다. 이 계산이 답하는 것은
# "지금 어느 쪽으로 얼마나 돌고, 전진해도 되는가" 두 가지뿐이며,
# 전진을 멈추는 판단은 그리퍼 PSD와 미션 상태머신의 몫입니다.

"""Pure visual-servo maths: a bearing in, a wheel command out."""
import math


def yaw_error(x, z):
    """Horizontal angle from the image centre line to the target, in radians.

    양수면 목표물이 오른쪽입니다(광학 프레임 +x가 오른쪽). 세로 중심선 위에 있으면 0입니다.
    """
    return math.atan2(x, z)


def follow(error, k_yaw, max_yaw, min_yaw, align_tol, approach_speed):
    """Return (linear_x, angular_z) for one bearing sample.

    오차가 임계보다 크면 제자리에서 돌고, 정렬된 뒤에만 전진합니다. 정렬 전에 전진하면
    목표물이 시야 가장자리로 밀려 상실 구간이 길어집니다.
    """
    angular = max(-max_yaw, min(max_yaw, -k_yaw * error))
    # min_yaw는 정지 마찰을 넘기는 최소 각속도입니다. 실측 전에는 0(끔)이며,
    # 작은 오차에서 모터가 울기만 하고 안 돌면 이 값을 올립니다.
    if 0.0 < abs(angular) < min_yaw:
        angular = math.copysign(min_yaw, angular)
    linear = approach_speed if abs(error) < align_tol else 0.0
    return linear, angular
