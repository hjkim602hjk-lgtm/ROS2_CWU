# 시리얼 줄에서 엔코더 틱을 뽑는 파싱을 하드웨어 없이 검사하는 pytest 파일입니다.
# 밀려 들어온 옛 줄을 버리고 최신 줄만 쓰는지, 깨진 줄을 건너뛰는지 확인합니다.

"""Guard the line parser: newest sample only, malformed lines skipped."""
from cwu_base.encoder_odom import latest_sample


def test_takes_the_newest_line_not_the_backlog():
    # 한 번의 read에 밀린 줄이 여러 개 들어와도 최신 값 하나만 씁니다.
    # 옛 줄까지 처리하면 줄 사이 dt가 0에 가까워져 twist가 발산합니다.
    lines = [b'10,20', b'11,21', b'12,22']
    assert latest_sample(lines, 0, 1) == (12, 22)


def test_skips_lines_without_enough_fields():
    # 잘린 첫 줄과 펌웨어 로그 줄은 건너뛰고 그 앞의 온전한 줄을 씁니다.
    lines = [b'5,6', b'BOOT ok', b'3']
    assert latest_sample(lines, 0, 1) == (5, 6)


def test_negative_ticks_and_field_order():
    assert latest_sample([b'L=-7 R=8'], 0, 1) == (-7, 8)
    assert latest_sample([b'L=-7 R=8'], 1, 0) == (8, -7)


def test_nucleo_firmware_lines():
    # 2026-09-29 팀 펌웨어(changwon_robot_stm.ino)의 실제 출력입니다.
    # 명령 응답 줄에는 숫자가 없어 건너뛰어야 합니다.
    lines = [b'CHANGWON ROBOT READY\r', b'ENC,1499,1406\r',
             b'ENC,-2023,1893\r', b'CMD: FORWARD\r']
    assert latest_sample(lines, 0, 1) == (-2023, 1893)


def test_nothing_usable_returns_none():
    assert latest_sample([], 0, 1) is None
    assert latest_sample([b'', b'BOOT'], 0, 1) is None


def test_dropped_digit_does_not_teleport_the_robot(monkeypatch):
    # 모터 잡음으로 숫자 가운데 바이트가 빠진 줄도 형식은 맞습니다. 그대로 적분하면
    # /odom이 한 표본 동안 10 cm 튀므로, 속도 상한을 넘는 표본은 버려야 합니다.
    import rclpy

    from cwu_base import encoder_odom
    monkeypatch.setattr(encoder_odom.serial, 'Serial', lambda *a, **k: None)
    rclpy.init(args=['--ros-args', '-p', 'wheel_radius:=0.04265',
                     '-p', 'wheel_separation:=0.20686', '-p', 'ticks_per_rev:=3009.5',
                     '-p', 'left_field:=0', '-p', 'right_field:=1'])
    try:
        node = encoder_odom.EncoderOdometry()
        node.update(1234, 5678)
        node.update(124, 5678)  # 1234에서 '3'이 빠진 줄
        assert node.pose == (0.0, 0.0, 0.0)  # 적분했다면 10 cm 후진 + 27도 회전
        node.update(1250, 5694)
        assert node.pose[0] > 0.001  # 기준점을 지켰으니 정상 이동 1.4 mm를 되찾음
        node.destroy_node()
    finally:
        rclpy.shutdown()
