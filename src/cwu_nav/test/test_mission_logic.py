# 미션 1 판단 함수의 하드웨어 없는 검사입니다.
# PSD 정지를 파지와 장애물로 바르게 가르는지 확인합니다.

"""Checks for mission_logic.py; no ROS, no robot."""
from cwu_nav.mission_logic import is_grasp


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
