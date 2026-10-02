# [공부 노트] 이 파일이 하는 일 — motor_bridge.py 의 "두뇌(순수 로직)" 부분
#   1) parse_frame     : Nucleo 가 보낸 한 줄을 엄격하게 검사해 튜플로 바꿈
#   2) wheel_ticks     : 로봇 속도(m/s, rad/s) → 좌·우 바퀴 속도(틱/초)
#   3) Safety          : "지금 모터에 명령을 보내도 안전한가?"를 판단하는 문지기
#   4) validate_config : motor.yaml 값이 실측·확인됐는지 검사, 아니면 실행 거부
#   ROS·시리얼을 전혀 쓰지 않아 test/test_motor_safety.py 로 로봇 없이 시험합니다.
#
# [공부 노트] 새 펌웨어 통신 규약 (115200 baud, 한 줄 = 한 메시지)
#   Nucleo → Pi :  READY,1                         (부팅 완료)
#                  ACK,<session>                   (설정 받았음)
#                  ENC,<millis>,<L틱>,<R틱>,<session>  (엔코더, 주기적)
#   Pi → Nucleo :  CFG,... / VEL,<session>,<L틱/s>,<R틱/s> / STOP
#   session = 연결할 때마다 새로 뽑는 번호. 옛 연결의 메시지를 새 것으로 착각하지 않게 함.
#   ※ 지금 보드의 legacy 펌웨어(ENC,<L>,<R>)와는 형식이 다릅니다 → 새 펌웨어 업로드 후에만 동작.

"""Strict MCU frames and monotonic safety gates; independent of ROS/hardware."""
import math  # isfinite, pi
import re    # 정수 형식 검사


# ─────────────────────────────────────────────────────────────────────────────
# parse_frame: 바이트 한 줄 → ('ACK', session) / ('ENC', ms, L, R, session) / ('READY', 1)
#   필요한 입력: 줄바꿈을 뗀 바이트 한 줄 (예: b"ENC,1200,500,498,77")
#   형식이 조금이라도 틀리면 ValueError. 호출자(motor_bridge)는 그 줄을 버리고 명령을 끊습니다.
#   ※ encoder_odom 의 "숫자만 대충 뽑기"보다 훨씬 엄격 — 모터를 움직이는 경로라서.
# ─────────────────────────────────────────────────────────────────────────────
def parse_frame(line):
    # 바이트 → 문자열(ASCII 아니면 UnicodeError) → 쉼표로 나누기
    fields = line.decode('ascii').split(',')
    if fields == ['READY', '1']:
        return ('READY', 1)
    # 첫 칸은 ACK 또는 ENC 여야 하고, 칸 수가 정확해야 함 (ACK=2칸, ENC=5칸)
    if fields[0] not in ('ACK', 'ENC') or len(fields) != (2 if fields[0] == 'ACK' else 5):
        raise ValueError('unknown frame')
    # 나머지 칸은 모두 "정수 모양"이어야 함. fullmatch = 문자열 전체가 패턴과 일치
    if not all(re.fullmatch(r'-?\d+', x) for x in fields[1:]):
        raise ValueError('invalid integer')
    # map(int, ...) : 각 칸을 int 로 변환, tuple 로 묶음
    values = tuple(map(int, fields[1:]))
    # 값 범위 검사 — MCU 의 자료형(32비트) 범위를 벗어나면 잡음으로 깨진 것
    if fields[0] == 'ACK':
        valid = 0 < values[0] <= 2147483647          # session: 양의 32비트 정수
    else:
        ms, left, right, session = values
        # millis: 부호 없는 32비트 / 틱: 부호 있는 32비트
        valid = (0 <= ms <= 0xffffffff and 0 <= session <= 2147483647
                 and all(-2147483648 <= x <= 2147483647 for x in (left, right)))
    if not valid:
        raise ValueError('out of range')
    # ('ENC', ms, left, right, session) — *values 는 튜플을 펼쳐 이어 붙임
    return (fields[0], *values)


# ─────────────────────────────────────────────────────────────────────────────
# wheel_ticks: 로봇 전체 속도 → 좌·우 바퀴 목표 속도(틱/초)
#   필요한 입력
#     linear  : 전진 속도 m/s (Nav2 의 /cmd_vel_safe linear.x)
#     angular : 회전 속도 rad/s (angular.z, 반시계 +)
#     separation, radius, ticks_per_rev : motor.yaml 실측값
#     limit   : 바퀴 최대 틱/초
#   kinematics.differential_step 의 "거꾸로" 계산입니다.
# ─────────────────────────────────────────────────────────────────────────────
def wheel_ticks(linear, angular, separation, radius, ticks_per_rev, limit):
    # NaN/inf 명령은 절대 모터로 보내지 않음
    if not all(math.isfinite(x) for x in (linear, angular)):
        raise ValueError('nonfinite command')
    # 1 m 가는 데 필요한 틱 수 = 회전당 틱 / 바퀴 둘레
    scale = ticks_per_rev / (2 * math.pi * radius)
    # 차동구동 역기구학: 왼쪽 = v - ω·(트레드/2), 오른쪽 = v + ω·(트레드/2)
    left, right = ((linear - angular * separation / 2) * scale,
                   (linear + angular * separation / 2) * scale)
    # 한쪽이 한계를 넘으면 두 바퀴를 "같은 비율로" 줄임 → 곡률(휘는 정도)은 유지
    ratio = max(1., abs(left) / limit, abs(right) / limit)
    return left / ratio, right / ratio


# ─────────────────────────────────────────────────────────────────────────────
# Safety: 모터 명령 문지기. 아래 조건이 "모두" 참일 때만 속도를 내보냅니다.
#   - Nucleo 가 설정을 확인(ACK)했다
#   - LiDAR 스캔이 scan_timeout 안에 들어왔다
#   - 엔코더가 encoder_timeout 안에 들어왔다
#   - TF(지도·센서 위치)가 살아 있다
#   - 마지막 속도 명령이 command_timeout 안에 들어왔다
#   하나라도 끊기면 velocity=None → motor_bridge 가 STOP 을 보냄.
#   시간은 모두 time.monotonic() (시스템 시계를 바꿔도 거꾸로 가지 않는 시계) 기준.
# ─────────────────────────────────────────────────────────────────────────────
class Safety:
    # 필요한 입력: 세 가지 제한 시간(초) — motor.yaml 의 command/scan/encoder_timeout
    def __init__(self, command_timeout, scan_timeout, encoder_timeout):
        self.command_timeout = command_timeout
        self.scan_timeout = scan_timeout
        self.encoder_timeout = encoder_timeout
        self.reset()

    # 연결을 새로 할 때: 모든 "마지막 수신 시각"을 -무한대(=한 번도 안 옴)로
    def reset(self):
        self.acknowledged = False
        self.scan = self.enc = -math.inf
        self.ms = None         # 마지막 ENC 의 MCU millis
        self.discard()

    # 지금 들고 있는 속도 명령을 버림 → 다음 출력은 STOP
    def discard(self):
        self.velocity = None
        self.cmd = -math.inf

    # ENC 수신 기록. MCU 시각(ms)이 앞으로 갔을 때만 인정(같거나 뒤로 가면 옛/깨진 줄)
    #   % (1 << 32) : millis 는 약 49일마다 0 으로 되감기므로 그 경우도 "앞으로"로 계산
    def encoder(self, ms, now):
        if self.ms is not None and not 0 < (ms-self.ms) % (1 << 32) < (1 << 31):
            return False
        self.ms, self.enc = ms, now
        return True

    # 명령 외의 모든 조건 검사. 0 <= now-시각 <= 제한 : "최근에, 그리고 미래가 아닌 시각에 왔다"
    def healthy(self, now, tf_ok):
        return (self.acknowledged and tf_ok and 0 <= now-self.scan <= self.scan_timeout
                and 0 <= now-self.enc <= self.encoder_timeout)

    # 새 속도 명령 접수. 건강하지 않으면 받지 않고 버림. 성공 여부를 True/False 로 돌려줌
    def command(self, left, right, now, tf_ok):
        if not self.healthy(now, tf_ok) or not all(math.isfinite(x) for x in (left, right)):
            self.discard()
            return False
        self.velocity, self.cmd = (left, right), now
        return True

    # 지금 보낼 속도. 명령이 오래됐거나 건강하지 않으면 None (→ STOP)
    def output(self, now, tf_ok):
        if not self.healthy(now, tf_ok) or not 0 <= now-self.cmd <= self.command_timeout:
            self.discard()
        return self.velocity


# ─────────────────────────────────────────────────────────────────────────────
# validate_config: motor.yaml 을 검사해 하나라도 문제면 ValueError → 노드가 시작하지 않음.
#   필요한 입력: 파라미터 사전 c (motor_bridge 가 선언한 모든 값)
#   통과 조건 (요약)
#     - calibration_confirmed, mount_calibration_confirmed 가 true (사람이 "쟀다"고 확인)
#     - measured 목록의 값이 전부 양수 (기본값 -1 = 안 잰 값)
#     - pwm_limit 1~255, mcu_timeout_ms 50~300, 부호는 ±1, 확장 반경 ≤ 0.20 m(대회 지름 400 mm)
#     - safety_latency ≥ 모든 제한 시간의 합 + 여유 — "끊김을 알아채고 멈추기까지 걸리는 최악 시간"
#   이 검사 때문에 지금 기본 motor.yaml 로는 drive:=true 실행이 거부됩니다(의도된 동작).
# ─────────────────────────────────────────────────────────────────────────────
def validate_config(c):
    measured = ('wheel_radius', 'wheel_separation', 'ticks_per_rev', 'max_wheel_speed',
                'kp_left', 'ki_left', 'kp_right', 'ki_right', 'max_accel_ticks',
                'robot_radius', 'braking_distance', 'safety_margin', 'max_linear_speed',
                'max_angular_speed', 'max_linear_accel', 'max_angular_accel', 'safety_latency')
    if (not c['calibration_confirmed'] or not c['mount_calibration_confirmed']
            or any(not math.isfinite(c[k]) or c[k] <= 0 for k in measured)
            or not 0 < c['pwm_limit'] <= 255 or not 50 <= c['mcu_timeout_ms'] <= 300
            or not c['publish_tf'] or c['robot_radius'] > .20 or c['left_sign'] not in (-1., 1.)
            or c['right_sign'] not in (-1., 1.)
            or any(not math.isfinite(c[k]) or c[k] <= 0 for k in
                   ('command_timeout', 'scan_timeout', 'encoder_timeout', 'tf_timeout', 'tf_future_tolerance'))
            or c['safety_latency'] < c['command_timeout'] + .05 + c['mcu_timeout_ms']/1000 + max(c['scan_timeout'], c['encoder_timeout'], c['tf_timeout']) + .2):
        raise ValueError('motor.yaml requires confirmed measured geometry, gains and safety limits')

    # 최고 바퀴 속도를 틱/초로 환산해 펌웨어가 받을 수 있는 범위인지 확인
    max_ticks = c['max_wheel_speed'] * c['ticks_per_rev'] / (2*math.pi*c['wheel_radius'])
    if (not 1 <= max_ticks <= 1000000 or c['max_accel_ticks'] > 10000000
            or any(c[k] > 1000000 for k in ('kp_left', 'ki_left', 'kp_right', 'ki_right'))):
        raise ValueError('motor.yaml exceeds MCU protocol limits')
