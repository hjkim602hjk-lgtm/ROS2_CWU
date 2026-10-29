# 시리얼 줄에서 엔코더 틱을 뽑는 파싱을 하드웨어 없이 검사하는 pytest 파일입니다.
# 밀려 들어온 옛 줄을 버리고 최신 줄만 쓰는지, 깨진 줄을 건너뛰는지 확인합니다.

"""Guard the line parser: newest sample only, malformed lines skipped."""
from cwu_base.encoder_odom import board_command, latest_fields


def test_takes_the_newest_line_not_the_backlog():
    # 한 번의 read에 밀린 줄이 여러 개 들어와도 최신 값 하나만 씁니다.
    # 옛 줄까지 처리하면 줄 사이 dt가 0에 가까워져 twist가 발산합니다.
    lines = [b'10,20', b'11,21', b'12,22']
    assert latest_fields(lines, 2) == [12, 22]


def test_skips_lines_without_enough_fields():
    # 잘린 첫 줄과 펌웨어 로그 줄은 건너뛰고 그 앞의 온전한 줄을 씁니다.
    lines = [b'5,6', b'BOOT ok', b'3']
    assert latest_fields(lines, 2) == [5, 6]


def test_negative_ticks():
    assert latest_fields([b'L=-7 R=8'], 2) == [-7, 8]


def test_nucleo_firmware_lines():
    # 2026-09-29 팀 펌웨어(changwon_robot_stm.ino)의 실제 출력입니다.
    # 명령 응답 줄에는 숫자가 없어 건너뛰어야 합니다.
    lines = [b'CHANGWON ROBOT READY\r', b'ENC,1499,1406\r',
             b'ENC,-2023,1893\r', b'CMD: FORWARD\r']
    assert latest_fields(lines, 2) == [-2023, 1893]


def test_nothing_usable_returns_none():
    assert latest_fields([], 2) is None
    assert latest_fields([b'', b'BOOT'], 2) is None


def test_dropped_digit_does_not_teleport_the_robot(monkeypatch):
    # 모터 잡음으로 숫자 가운데 바이트가 빠진 줄도 형식은 맞습니다. 그대로 적분하면
    # /odom이 한 표본 동안 10 cm 튀므로, 속도 상한을 넘는 표본은 버려야 합니다.
    import rclpy

    from cwu_base import encoder_odom
    monkeypatch.setattr(encoder_odom.serial, 'Serial', lambda *a, **k: FakeSerial())
    rclpy.init(args=['--ros-args', '-p', 'counts_per_mm_left:=11.23',
                     '-p', 'counts_per_mm_right:=11.23', '-p', 'wheel_separation:=0.20686',
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


def test_robot40_v3_telemetry_skips_status_lines():
    # ms,mode,L,R,... ; "# CAL,0.000000,0.000000" must not be read as ticks.
    lines = [b'1768,8,-75,-70,80,80,412,428,338,12,70,130\r', b'# CAL,0.000000,0.000000\r']
    assert latest_fields(lines, 4)[2:4] == [-75, -70]


def test_board_command_allow_list():
    # 2026-10-29 펌웨어: 괄호 명령은 [..], 한 글자 명령은 그대로. 그 밖(공백 = 보드 정지, V 등)은 버림.
    assert board_command('F400') == b'[F400]'
    assert board_command(' TL90 ') == b'[TL90]'
    assert board_command('TR180') == b'[TR180]'
    assert board_command('q') == b'q' and board_command('8') == b'8'
    for bad in ('V80,0.3', 'PF0', '!', 'F40', 'F400]q[F1000', 'TL', 'z', '', 'F-400', 'L105'):
        assert board_command(bad) is None, bad


class FakeSerial:
    def __init__(self):
        self.sent = b''

    def write(self, data):
        self.sent += data

    def read(self, size):
        return b''


def test_stream_heartbeat_and_counter_reset(monkeypatch):
    # 시작 때 't'로 스트림을 켜고, OFF 응답이면 다시 켜고, 재부팅이면 틱 기준점을 다시 잡음.
    import rclpy
    from std_msgs.msg import String

    from cwu_base import encoder_odom
    board = FakeSerial()
    monkeypatch.setattr(encoder_odom.serial, 'Serial', lambda *a, **k: board)
    rclpy.init(args=['--ros-args', '-p', 'counts_per_mm_left:=5.759',
                     '-p', 'counts_per_mm_right:=5.671', '-p', 'wheel_separation:=0.2016',
                     '-p', 'left_field:=2', '-p', 'right_field:=3'])
    try:
        node = encoder_odom.EncoderOdometry()
        published = []  # 미션 노드가 받을 텔레메트리·이벤트
        node.telemetry.publish = node.events.publish = published.append
        assert board.sent == b't'
        node.buffer += b'Telemetry = OFF\r\n'  # 이미 켜져 있던 스트림을 꺼 버린 경우
        node.poll()
        assert board.sent == b'tt'
        node.buffer += b'Telemetry = ON\r\n100,0,5000,5000,0,0,1,1,1,1,40,140\r\n'
        node.poll()
        assert board.sent == b'tt'
        assert list(published[0].data) == [100, 0, 5000, 5000, 0, 0, 1, 1, 1, 1, 40, 140]
        node.on_command(String(data='F400'))
        node.on_command(String(data='V80,0.0'))  # 버림
        assert board.sent.endswith(b'[F400]')
        node.buffer += b'# REJECT BUSY\r\n'
        node.poll()
        assert published[-1].data == '# REJECT BUSY' and board.sent.endswith(b'q')
        node.buffer += b'# ROBOT40_V4 READY\r\n150,0,0,0,0,0,1,1,1,1,40,140\r\n'
        node.poll()
        assert node.ticks is None and board.sent.endswith(b't')
        node.buffer += b'200,0,10,10,0,0,1,1,1,1,40,140\r\n'
        node.poll()
        assert node.ticks == (10, 10)  # 재부팅 뒤 0 근처 틱이 새 기준 (속도 상한으로 버려지지 않음)
        node.destroy_node()
    finally:
        rclpy.shutdown()
