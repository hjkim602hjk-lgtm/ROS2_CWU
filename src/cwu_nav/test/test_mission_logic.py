# 미션 1 판단 함수의 하드웨어 없는 검사입니다.
# PSD 정지를 파지와 장애물로 바르게 가르는지, 제자리 둘러보기가 벽 쪽을 건너뛰는지 확인합니다.

"""Checks for mission_logic.py; no ROS, no robot."""
import math

from cwu_nav.mission_logic import is_grasp, spin_plan


def telemetry(front=0, rear=0, left=0, right=0):
    return [0, 0, 0, 0, 0, 0, front, rear, left, right, 40, 140]


def test_grasp_only_on_front_psd_while_approaching_a_seen_target():
    near = telemetry(front=600)
    assert is_grasp('# ABORT PSD_FRONT', near, 550, True, True)
    assert not is_grasp('# ABORT PSD_FRONT', near, 550, False, True)   # exploring: wall
    assert not is_grasp('# ABORT PSD_FRONT', near, 550, True, False)   # target not seen
    assert not is_grasp('# ABORT PSD_FRONT', telemetry(left=600), 550, True, True)
    assert is_grasp('# ABORT PSD_FRONT', telemetry(front=461), 300, True, True)  # 보드만 550 표본을 봄
    assert not is_grasp('# ABORT PSD_FRONT', telemetry(front=185, left=600), 300, True, True)
    assert not is_grasp('# ABORT ENCODER_NO_PULSE', near, 550, True, True)


def scan(open_from=None, open_to=None, near=0.3, far=2.0):
    """360 rays at 1 deg, base frame (0 = ahead, +CCW); open (far) between open_from..open_to deg."""
    rays = []
    for d in range(360):
        a = (d + 180) % 360 - 180
        inside = open_from is not None and open_from <= a <= open_to
        rays.append(far if inside else near)
    return rays, math.radians(0), math.radians(1)


def test_spin_skips_walls():
    rays, a0, inc = scan()
    assert spin_plan(rays, a0, inc, 0.0, 0.6, 30, 5) == (0.0, 0.0)          # 사방이 벽: 돌지 않음
    rays, a0, inc = scan(-20, 20)
    assert spin_plan(rays, a0, inc, 0.0, 0.6, 30, 5) == (0.0, 0.0)          # 정면만 트임: 이미 보임
    rays, a0, inc = scan(80, 100)
    ccw, cw = spin_plan(rays, a0, inc, 0.0, 0.6, 30, 5)
    assert ccw == 70.0 and cw > ccw                                         # 왼쪽만 트임: 반시계 70도
    rays, a0, inc = scan(-180, 180)
    assert spin_plan(rays, a0, inc, 0.0, 0.6, 30, 5) == (299.0, 299.0)      # 다 트임: 한 바퀴 - 화각


def test_spin_ignores_narrow_slits_and_uses_lidar_mount():
    rays, a0, inc = scan(88, 90)                                            # 3도 틈
    assert spin_plan(rays, a0, inc, 0.0, 0.6, 30, 5) == (0.0, 0.0)
    rays, a0, inc = scan(80, 100)
    ccw, _ = spin_plan(rays, a0, inc, math.pi, 0.6, 30, 5)                  # 라이다가 거꾸로 달림
    assert ccw == 250.0                                                     # 트인 곳은 로봇 기준 오른쪽(−90도)
