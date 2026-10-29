# 목표물 방향 벡터를 화면 세로 중앙선 기준 좌우 각도로 바꾸는 순수 계산입니다. ROS를 참조하지 않습니다.
# 깊이가 없으므로 "얼마나 남았는가"는 알 수 없습니다. 이 각도로 mission1이 얼마나 돌지([TL/TR]) 정하고,
# 전진을 멈추는 판단은 그리퍼 PSD(보드)의 몫입니다.

"""Pure visual-servo maths: a bearing in, a yaw error out."""
import math


# yaw_error: 방향 벡터 → 좌우 각도 오차(라디안)
#   필요한 입력: target_detector 가 낸 방향의 x(오른쪽 성분), z(앞 성분)
#   atan2(x, z): "앞으로 z 가고 오른쪽으로 x 가는 방향"의 각도. 정면이면 0.
def yaw_error(x, z):
    """Horizontal angle from the image centre line to the target, in radians.

    양수면 목표물이 오른쪽입니다(광학 프레임 +x가 오른쪽). 세로 중심선 위에 있으면 0입니다.
    """
    return math.atan2(x, z)
