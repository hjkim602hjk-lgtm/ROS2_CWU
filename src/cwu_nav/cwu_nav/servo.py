# 목표물 방향 하나로 바퀴 속도를 만드는 순수 계산입니다. ROS를 참조하지 않습니다.
# 깊이가 없으므로 "얼마나 남았는가"는 알 수 없습니다. 이 계산이 답하는 것은
# "지금 어느 쪽으로 얼마나 돌고, 전진해도 되는가" 두 가지뿐이며,
# 전진을 멈추는 판단은 그리퍼 PSD와 미션 상태머신의 몫입니다.
#
# [공부 노트] 시각 서보(visual servo)
#   "카메라로 보면서 바로 몸을 움직이는" 제어. 여기서는 목표를 화면 세로 중심선에 맞추기만 합니다.
#   흐름: 방향 (x, z) ─yaw_error─▶ 각도 오차 ─follow─▶ (전진 속도, 회전 속도)
#   P 제어: 회전 속도 = -k × 오차. 오차가 클수록 빨리 돌고, 0 에 가까우면 천천히.

"""Pure visual-servo maths: a bearing in, a wheel command out."""
import math


# yaw_error: 방향 벡터 → 좌우 각도 오차(라디안)
#   필요한 입력: target_detector 가 낸 방향의 x(오른쪽 성분), z(앞 성분)
#   atan2(x, z): "앞으로 z 가고 오른쪽으로 x 가는 방향"의 각도. 정면이면 0.
def yaw_error(x, z):
    """Horizontal angle from the image centre line to the target, in radians.

    양수면 목표물이 오른쪽입니다(광학 프레임 +x가 오른쪽). 세로 중심선 위에 있으면 0입니다.
    """
    return math.atan2(x, z)


# follow: 오차 → (linear_x m/s, angular_z rad/s)
#   필요한 입력 (servo.yaml)
#     k_yaw          : 비례 이득. 클수록 빨리 돌지만 너무 크면 좌우로 흔들림
#     max_yaw        : 최대 회전 속도
#     min_yaw        : 최소 회전 속도(마찰 극복용, 0 = 끔)
#     align_tol      : 이 각도 안이면 "정렬됨" → 전진 허용
#     approach_speed : 정렬됐을 때 전진 속도
def follow(error, k_yaw, max_yaw, min_yaw, align_tol, approach_speed):
    """Return (linear_x, angular_z) for one bearing sample.

    오차가 임계보다 크면 제자리에서 돌고, 정렬된 뒤에만 전진합니다. 정렬 전에 전진하면
    목표물이 시야 가장자리로 밀려 상실 구간이 길어집니다.
    """
    # 부호가 - 인 이유: 목표가 오른쪽(error>0)이면 오른쪽으로 = 시계방향 = angular_z 음수(ROS 규칙)
    # max(-a, min(a, v)) : v 를 -a ~ +a 범위로 자르기(clamp)
    angular = max(-max_yaw, min(max_yaw, -k_yaw * error))
    # min_yaw는 정지 마찰을 넘기는 최소 각속도입니다. 실측 전에는 0(끔)이며,
    # 작은 오차에서 모터가 울기만 하고 안 돌면 이 값을 올립니다.
    # copysign(크기, 부호원본) : min_yaw 크기에 angular 의 부호를 붙임
    if 0.0 < abs(angular) < min_yaw:
        angular = math.copysign(min_yaw, angular)
    # 정렬됐을 때만 전진, 아니면 제자리 회전
    linear = approach_speed if abs(error) < align_tol else 0.0
    return linear, angular
