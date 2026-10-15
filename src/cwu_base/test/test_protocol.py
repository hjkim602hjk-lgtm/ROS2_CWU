# 시리얼 줄에서 엔코더 틱을 뽑는 파싱을 하드웨어 없이 검사하는 pytest 파일입니다.
# 밀려 들어온 옛 줄을 버리고 최신 줄만 쓰는지, 깨진 줄을 건너뛰는지 확인합니다.

"""Guard the line parser: newest sample only, malformed lines skipped."""
from cwu_base.encoder_odom import drive_frame, latest_fields


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


def test_drive_frame():
    # BASE_NAV_V1: mm/s, rad/s. 한계(150 mm/s, 1.2 rad/s)를 넘으면 보드가 ABORT하므로 비율 유지 축소.
    assert drive_frame(0.0, 0.0) == b'[V0,0.000]\n'
    assert drive_frame(0.08, 0.3) == b'[V80,0.300]\n'
    assert drive_frame(-0.08, -0.3) == b'[V-80,-0.300]\n'
    assert drive_frame(0.3, 0.0) == b'[V150,0.000]\n'     # clamp
    assert drive_frame(0.3, 1.2) == b'[V150,0.600]\n'     # curvature kept
    assert drive_frame(0.0, 2.4) == b'[V0,1.200]\n'
    assert b' ' not in drive_frame(-0.0001, -0.0001)       # 공백은 보드 정지 명령


class FakeSerial:
    def __init__(self):
        self.sent = b''

    def write(self, data):
        self.sent += data

    def read(self, size):
        return b''


def test_arm_follows_board_and_counter_reset(monkeypatch):
    # 무장은 시작 때 한 번만, ABORT 뒤 자동 재무장 없음, 재부팅이면 틱 기준점을 다시 잡음.
    import rclpy

    from cwu_base import encoder_odom
    board = FakeSerial()
    monkeypatch.setattr(encoder_odom.serial, 'Serial', lambda *a, **k: board)
    rclpy.init(args=['--ros-args', '-p', 'counts_per_mm_left:=5.992624',
                     '-p', 'counts_per_mm_right:=5.900948', '-p', 'wheel_separation:=0.1904',
                     '-p', 'left_field:=2', '-p', 'right_field:=3', '-p', 'drive:=true'])
    try:
        node = encoder_odom.EncoderOdometry()
        published = []  # 미션 노드가 받을 텔레메트리·이벤트
        node.telemetry.publish = node.events.publish = published.append
        assert board.sent == b'[N1]\n'
        node.send_drive()
        assert board.sent == b'[N1]\n'  # 보드가 살아 있는지 모르면 무장 요청 안 함
        node.buffer += b'100,0,5000,5000,0,0,1,1,1,1,40,140\n'
        node.poll()
        node.send_drive()
        node.send_drive()
        assert board.sent.endswith(b'[ARM]\n') and board.sent.count(b'[ARM]') == 1
        node.buffer += b'# ARMED\n'
        node.poll()
        assert list(published[0].data) == [100, 0, 5000, 5000, 0, 0, 1, 1, 1, 1, 40, 140]
        assert published[1].data == '# ARMED'
        reply = node.on_arm(None, encoder_odom.Trigger.Response())
        assert reply.success and reply.message == '이미 무장됨.'
        node.cmd, node.cmd_stamp = (0.08, 0.0), node.get_clock().now()
        node.send_drive()
        assert board.sent.endswith(b'[V80,0.000]\n')
        node.buffer += b'# ABORT PSD_BLOCKED\n'
        node.poll()
        sent = board.sent
        node.send_drive()
        assert not node.armed and board.sent == sent  # 서비스 호출 전에는 아무것도 안 보냄
        node.buffer += b'# ROBOT40_V4 READY\n150,0,0,0,0,0,1,1,1,1,40,140\n'
        node.poll()
        assert node.ticks is None and board.sent.endswith(b'[N1]\n')
        node.buffer += b'200,0,10,10,0,0,1,1,1,1,40,140\n'
        node.poll()
        assert node.ticks == (10, 10)  # 재부팅 뒤 0 근처 틱이 새 기준 (속도 상한으로 버려지지 않음)
        assert not node.on_arm(None, encoder_odom.Trigger.Response()).success
        node.send_drive()
        assert board.sent.endswith(b'[ARM]\n')
        request = encoder_odom.SetBool.Request(data=True)
        node.on_carry(request, encoder_odom.SetBool.Response())
        assert board.sent.endswith(b'8[PF0]\n')  # 잡기 + 운반 모드
        request.data = False
        node.on_carry(request, encoder_odom.SetBool.Response())
        assert board.sent.endswith(b'7[PF1]\n')
        node.destroy_node()
    finally:
        rclpy.shutdown()
