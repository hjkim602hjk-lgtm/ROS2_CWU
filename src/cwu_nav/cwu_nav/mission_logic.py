# 미션 1 상태머신(mission1.py)이 쓰는 판단을 ROS 없이 계산하는 순수 함수 모음입니다.
# 보드가 멈춘 이유가 파지인지 장애물인지를 정합니다.
# 칸 단위 이동 계획은 grid.py에 있습니다. 로봇 없이 pytest로 검증합니다.

"""Pure mission-1 decisions: what a board stop means. No ROS."""

# /base/telemetry 필드 번호 (README_STM_PI_INTERFACE.md 6절, PSD는 raw ADC)
FRONT, REAR, LEFT, RIGHT, SERVO_L, SERVO_R = 6, 7, 8, 9, 10, 11


def is_grasp(event, telemetry, front_min, approaching, target_recent):
    """True when the board stopped on the front PSD while closing in on a seen target.

    PSD 근접은 정체를 증명하지 않으므로, 접근 중이고 카메라가 방금 목표물을 중앙에 봤을 때만
    파지로 봅니다. 그 외의 PSD 정지는 벽·장애물·상대 로봇입니다.
    front_min은 보드 임계(550)가 아니라 "정면에 무언가 가깝다"는 기준입니다. 보드는 표본 하나가
    550을 넘으면 멈추지만 20 Hz 텔레메트리에는 그 표본이 안 잡힐 수 있습니다(실물 3회차: 큐브 앞
    정지 때 텔레메트리 최대 461~520). 집게가 열리고 앞이 비면 정면은 185 이하입니다.
    2026-10-29: 정면 PSD 정지는 mission1이 텔레메트리로 판단해 q로 세우고 "# ABORT PSD_FRONT (Pi)"로 넘김 (이전 판 보드 PSD_BLOCKED).
    """
    return (approaching and target_recent and 'PSD_FRONT' in event
            and telemetry[FRONT] >= front_min)
