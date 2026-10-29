# 미션 1 상태머신(mission1.py)이 쓰는 판단을 ROS 없이 계산하는 순수 함수 모음입니다.
# 보드가 멈춘 이유가 파지인지 장애물인지, 제자리에서 어느 쪽으로 얼마나 둘러볼지를 정합니다.
# 칸 단위 이동 계획은 grid.py에 있습니다. 로봇 없이 pytest로 검증합니다.

"""Pure mission-1 decisions: what a board stop means, where to look. No ROS."""
import math

import numpy as np

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


def spin_plan(ranges, angle_min, angle_inc, yaw_offset, open_dist, half_fov_deg, min_gap_deg):
    """Rotation (ccw, cw) in degrees that sweeps the camera over every open direction.

    벽 높이 200 mm는 50 mm 큐브를 가리므로 벽을 향해 도는 것은 시간 낭비입니다. 라이다 한 번의 스캔에서
    open_dist보다 멀리 트인 방향(무효 0.0·inf는 먼 곳으로 봄)만 남기고, min_gap_deg보다 좁은 틈은 버립니다.
    카메라가 정면 ±half_fov_deg를 보므로 반시계로 θ 돌면 [-half, θ+half]가 화면에 들어옵니다.
    yaw_offset: base_link 기준 라이다 각도 (rad). 0도 = 로봇 정면. 트인 방향이 없으면 (0, 0).
    """
    bins = np.zeros(360, bool)
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0 or r > open_dist:
            bins[int(round(math.degrees(angle_min + i * angle_inc + yaw_offset))) % 360] = True
    if bins.all():
        open_deg = np.arange(360)
    else:
        start = int(np.argmin(bins))  # 닫힌 칸에서 시작하면 원형 구간이 끊기지 않음
        rolled, keep, run = np.roll(bins, -start), np.zeros(360, bool), []
        for k in range(361):
            if k < 360 and rolled[k]:
                run.append(k)
            else:
                if len(run) >= min_gap_deg:
                    keep[run] = True
                run = []
        open_deg = (np.nonzero(keep)[0] + start) % 360
    if not len(open_deg):
        return 0.0, 0.0
    half = half_fov_deg
    ccw = max(0.0, float(max((d + half) % 360 for d in open_deg)) - 2 * half)
    cw = max(0.0, float(max((-d + half) % 360 for d in open_deg)) - 2 * half)
    return ccw, cw
