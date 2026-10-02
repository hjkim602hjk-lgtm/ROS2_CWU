# Nucleo가 시리얼로 보내는 엔코더 틱을 읽어 /odom과 odom → base_link TF를 발행합니다.
# SLAM Toolbox는 이 TF가 있어야 지도를 만들 수 있으므로 실물 SLAM의 필수 입력입니다.
# 프로토콜과 바퀴 제원은 encoder.yaml에서 읽으며, 미입력 값이 있으면 실행을 거부합니다.
#
# [공부 노트] 이 노드의 입력과 출력
#   입력  : 시리얼 포트 /dev/ttyAMA0 에서 0.1초마다 들어오는 "ENC,<왼쪽틱>,<오른쪽틱>\n"
#   설정  : config/encoder.yaml (포트, 바퀴 반지름, 트레드, 회전당 틱 …)
#   출력1 : /odom 토픽 (nav_msgs/Odometry) — 위치 + 속도 + 불확실성
#   출력2 : TF  odom → base_link — "전원 켠 자리(odom)에서 로봇 몸체(base_link)가 어디 있나"
#
# [공부 노트] TF(좌표 변환)가 왜 필요한가?
#   SLAM은 "LiDAR가 본 벽"을 지도에 찍어야 합니다. 그러려면
#     map → odom → base_link → laser
#   로 이어지는 좌표 사슬이 끊김 없이 있어야 합니다.
#     map → odom        : SLAM Toolbox 가 발행
#     odom → base_link  : ★ 이 노드가 발행
#     base_link → laser : cwu_slam/mounts.py 가 발행 (LiDAR 장착 위치)
#   하나라도 빠지면 SLAM은 조용히 아무것도 안 합니다.
#
# [공부 노트] 실행 방법
#   ros2 launch cwu_base encoder.launch.py          # 이 노드만
#   ros2 topic echo /odom --once                     # 결과 한 번 보기
#   ※ motor_bridge 와 같은 포트를 쓰므로 둘을 동시에 켜면 안 됩니다.

"""Serial encoder ticks to /odom and the odom -> base_link transform."""
# ── 표준 라이브러리 ──
import math  # sin, cos (각도 → 쿼터니언 변환)
import re    # 정규식: "ENC,1499,1406" 에서 숫자만 뽑기

# ── ROS 2 / 외부 패키지 ── (package.xml 에 의존성으로 적혀 있어야 합니다)
import rclpy                    # ROS 2 파이썬 클라이언트 라이브러리. 노드를 만들고 돌리는 뼈대
import serial                   # pyserial: 시리얼 포트 열기/읽기 (apt: python3-serial)
from geometry_msgs.msg import Quaternion, TransformStamped  # 회전(쿼터니언), TF 한 개
from nav_msgs.msg import Odometry                           # /odom 메시지 형식
from rclpy.node import Node                                 # 모든 노드의 부모 클래스
from tf2_ros import TransformBroadcaster                    # TF 를 발행하는 도구

# 우리가 만든 순수 계산 함수 (kinematics.py)
from cwu_base.kinematics import integrate, ticks_to_metres, wrap_delta

# rb'...' = 바이트(raw bytes) 정규식. 시리얼은 글자(str)가 아니라 바이트로 들어오므로 바이트용.
# -?\d+ = "마이너스 기호가 있을 수도 있는 연속된 숫자"  → b"ENC,-12,34" 에서 [b"-12", b"34"]
NUMBER = re.compile(rb'-?\d+')
# 값을 지어내지 않기 위한 표식입니다. 이 값이 남아 있으면 실측 전이라는 뜻입니다.
UNSET = -1.0
# 보드레이트가 틀리면 줄바꿈 없는 쓰레기가 무한히 쌓입니다. 오래된 쪽부터 버립니다.
MAX_BUFFER = 65536
# MCU가 USB 열거와 펌웨어 시작을 마칠 때까지 기다려 주는 시간입니다. 이보다
# 늦게까지 한 줄도 못 받으면 설정이 틀린 것으로 봅니다. 이 유예가 없으면
# 부팅 때마다 오류가 한 줄 뜨고, 사람은 곧 오류 로그를 무시하게 됩니다.
STARTUP_GRACE = 3.0
# 펌웨어 ENC 전송 주기(s). 밀린 줄이 두 poll에 나뉘어 들어와 dt가 0에 가까워도
# 한 주기 분량의 이동은 허용하도록 속도 상한 검사의 dt 하한으로 씁니다.
SAMPLE_PERIOD = 0.1


# ─────────────────────────────────────────────────────────────────────────────
# latest_sample: 받은 여러 줄 중 "제일 최근의 멀쩡한 줄"에서 (왼쪽틱, 오른쪽틱)을 꺼냅니다.
#   필요한 입력
#     lines       : 바이트 줄 목록 [b"ENC,10,12", b"ENC,20,25", ...]
#     left_field  : 숫자 중 몇 번째가 왼쪽인지 (0부터 셈). encoder.yaml 에서 0
#     right_field : 오른쪽 번호. encoder.yaml 에서 1
#   돌려주는 값: (왼쪽, 오른쪽) 정수 튜플, 쓸 만한 줄이 없으면 None
#   ROS 를 안 쓰는 함수라 테스트에서 따로 불러 시험할 수 있습니다.
# ─────────────────────────────────────────────────────────────────────────────
def latest_sample(lines, left_field, right_field):
    """완성된 줄들 중 가장 최신의 유효한 (왼쪽, 오른쪽) 틱을 돌려줍니다.

    틱은 누적값이라 최신 줄 하나만 봐도 자세 증분은 같습니다. 반대로 한 번의
    read에 밀려 들어온 옛 줄까지 차례로 처리하면 줄 사이 간격을 벽시계로 재게 되어
    dt가 0에 가까워지고 twist가 발산합니다(부팅 직후 밀린 버퍼에서 특히).
    형식이 어긋난 줄(잘린 첫 줄, 펌웨어 로그)은 건너뜁니다.
    """
    # reversed(): 뒤(최신)에서부터 봅니다. 멀쩡한 줄을 찾으면 바로 끝.
    for line in reversed(lines):
        # 줄에서 숫자만 전부 뽑기. b"ENC,1499,1406" → [b"1499", b"1406"]
        fields = NUMBER.findall(line)
        # 필요한 번호까지 숫자가 있어야 함. 잘린 줄 b"ENC,14" 은 숫자가 1개라 탈락.
        if len(fields) > max(left_field, right_field):
            # int(b"1499") → 1499. 바이트도 int()로 바로 바뀝니다.
            return int(fields[left_field]), int(fields[right_field])
    # for 가 끝까지 돌았다 = 쓸 만한 줄이 하나도 없음
    return None


# ─────────────────────────────────────────────────────────────────────────────
# EncoderOdometry: 실제 ROS 2 노드. Node 를 상속(class A(Node))해서 만듭니다.
#   노드 = ROS 안에서 돌아가는 하나의 프로그램. 이름·파라미터·토픽·타이머를 가집니다.
# ─────────────────────────────────────────────────────────────────────────────
class EncoderOdometry(Node):

    # __init__: 노드가 생성될 때 딱 한 번 실행. "준비 작업"을 여기서 다 합니다.
    def __init__(self):
        # 부모(Node) 초기화 + 노드 이름 지정. 이 이름이 encoder.yaml 맨 위 "encoder_odom:" 과
        # 같아야 YAML 파라미터가 이 노드에 들어옵니다.
        super().__init__('encoder_odom')
        # 파라미터 선언: (이름, 기본값) 목록. 선언해야 YAML/런치에서 값을 넣을 수 있습니다.
        # 기본값의 "자료형"이 곧 그 파라미터의 자료형입니다. (예: 115200 → int, -1.0 → float)
        # 첫 인자 '' 는 네임스페이스(앞에 붙는 이름) 없음이라는 뜻.
        p = self.declare_parameters('', [
            ('port', '/dev/ttyAMA0'), ('baud', 115200),         # 시리얼 포트, 속도
            ('wheel_radius', UNSET), ('wheel_separation', UNSET),  # 실측 필수 → 기본 -1
            ('ticks_per_rev', UNSET),                           # 실측 필수 → 기본 -1
            ('left_field', -1), ('right_field', -1),            # 줄에서 몇 번째 숫자인지
            ('left_sign', 1.0), ('right_sign', 1.0),            # 방향 뒤집기(+1 / -1)
            ('counter_bits', 32), ('timeout', 0.5), ('max_wheel_speed', 0.5),
            ('odom_frame', 'odom'), ('base_frame', 'base_link'),  # TF 좌표계 이름
            ('publish_tf', True),                               # TF 도 낼지
            ('pose_covariance', [0.01, 0.01, 0.05]),            # 위치 불확실성 (x, y, yaw)
            ('twist_covariance', [0.01, 0.01, 0.05]),           # 속도 불확실성
        ])
        # 파라미터 객체 목록 → {이름: 값} 사전으로 정리. 이후 self.cfg['port'] 처럼 꺼내 씁니다.
        self.cfg = {d.name: d.value for d in p}
        # 실측값이 비어 있으면 여기서 바로 에러로 멈춥니다(아래 함수 참고).
        self._require_measured_values()

        # ── 노드가 기억하고 있어야 하는 상태 ──
        self.pose = (0.0, 0.0, 0.0)          # 현재 위치 (x, y, theta). 켠 자리가 원점
        self.start = self.get_clock().now()  # 노드 시작 시각 (첫 데이터 대기 판단용)
        self.ticks = None                    # 지난번 (왼쪽, 오른쪽) 틱. None = 아직 없음
        self.stamp = None                    # 지난번 틱을 받은 시각
        self.buffer = bytearray()            # 아직 줄바꿈(\n)이 안 온 조각을 모아두는 통
        self.warned = False                  # 끊김 경고를 이미 띄웠는지 (도배 방지)

        # 퍼블리셔: 'odom' 토픽으로 Odometry 메시지를 내보냄. 10 = 큐 크기(밀리면 10개까지 보관)
        self.odom = self.create_publisher(Odometry, 'odom', 10)
        # TF 발행 도구. /tf 토픽으로 좌표 변환을 뿌립니다.
        self.tf = TransformBroadcaster(self)
        # 시리얼 포트 열기
        #   timeout=0      : 읽을 게 없으면 기다리지 않고 바로 빈 값 반환(논블로킹)
        #   exclusive=True : 다른 프로그램이 같은 포트를 못 열게 잠금 (motor_bridge 와 충돌 방지)
        #   포트가 없거나 권한이 없으면 여기서 예외가 나고 노드가 죽습니다.
        #   → Pi 에서 사용자가 dialout 그룹이어야 /dev/ttyAMA0 을 열 수 있습니다.
        self.serial = serial.Serial(
            self.cfg['port'], self.cfg['baud'], timeout=0, exclusive=True)
        # 시작 로그. f"..." 는 f-문자열: {} 안의 값이 문자열에 끼워집니다.
        self.get_logger().info(
            f"엔코더 {self.cfg['port']} @ {self.cfg['baud']} baud, "
            f"필드 L={self.cfg['left_field']} R={self.cfg['right_field']}")
        # 시리얼을 블로킹으로 읽지 않고 짧은 주기로 비웁니다. 스레드가 없어야
        # 종료와 파라미터 처리가 단순하고, MCU 전송 주기에 맞춰 발행됩니다.
        # create_timer(주기초, 함수): 그 주기마다 함수를 자동 호출. rclpy.spin() 이 돌려줍니다.
        self.create_timer(0.005, self.poll)                      # 5 ms 마다 시리얼 확인
        self.create_timer(self.cfg['timeout'], self.check_link)  # 0.5 s 마다 끊김 확인

    # 밑줄(_)로 시작하는 이름 = "이 클래스 안에서만 쓰는 함수"라는 파이썬 관례
    def _require_measured_values(self):
        """추측 대신 즉시 실패합니다. 틀린 오도메트리는 조용히 지도를 망칩니다."""
        # 리스트 컴프리헨션: 조건에 맞는 이름만 모음. 0 이하면 "안 채운 값"
        missing = [k for k in ('wheel_radius', 'wheel_separation', 'ticks_per_rev')
                   if self.cfg[k] <= 0]
        missing += [k for k in ('left_field', 'right_field') if self.cfg[k] < 0]
        # 하나라도 비었으면 ValueError 를 던져 노드 생성을 중단 → 어떤 값이 빠졌는지 알려줌
        if missing:
            raise ValueError(
                f"encoder.yaml 미입력 값: {', '.join(missing)}. "
                '바퀴 반지름·트레드·회전당 틱은 실측하고, 필드 번호는 '
                'python3 tools/sniff_encoder_serial.py 출력에서 확인하십시오.')

    # poll: 5 ms 마다 타이머가 부름. 시리얼에 쌓인 바이트를 가져와 줄 단위로 자릅니다.
    def poll(self):
        """수신 버퍼를 비우고 가장 최신 표본으로 자세를 갱신합니다."""
        # try/except: 에러가 나도 노드 전체가 죽지 않게 잡아서 로그만 남김
        # (예: 케이블이 흔들려 순간적으로 읽기 실패)
        try:
            # 최대 4096 바이트를 읽어 버퍼 뒤에 붙임. timeout=0 이라 없으면 b'' 가 붙음.
            self.buffer += self.serial.read(4096)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'엔코더 시리얼 오류: {e}')
            return
        # \n 으로 자르기. 마지막 조각은 아직 줄바꿈이 안 온 "미완성 줄"이라 버퍼에 남겨 둡니다.
        #   b"ENC,1,2\nENC,3,4\nENC,5" → lines=[b"ENC,1,2", b"ENC,3,4"], buffer=b"ENC,5"
        #   *lines 는 "앞의 나머지 전부를 리스트로" 받는 파이썬 문법
        *lines, self.buffer = self.buffer.split(b'\n')
        # 버퍼가 너무 커지면 앞(오래된 쪽)을 잘라냄. 뒤 MAX_BUFFER 바이트만 남김.
        del self.buffer[:-MAX_BUFFER]
        # 이번에 완성된 줄 중 최신 한 줄만 사용
        sample = latest_sample(lines, self.cfg['left_field'], self.cfg['right_field'])
        if sample is not None:
            # *sample 은 튜플 펼치기: update(왼쪽, 오른쪽) 과 같음
            self.update(*sample)

    # update: 새 틱 한 쌍으로 위치를 한 스텝 갱신하고 발행합니다.
    def update(self, left_ticks, right_ticks):
        # ROS 시계의 현재 시각 (시뮬레이션이면 가짜 시계를 따름: use_sim_time)
        now = self.get_clock().now()
        previous, previous_stamp = self.ticks, self.stamp
        if previous is None:
            self.ticks, self.stamp = (left_ticks, right_ticks), now
            return  # 첫 표본은 기준점으로만 씁니다.
        # 데이터가 다시 들어왔으니 "끊김 경고" 상태를 풀어줌
        self.warned = False
        bits = self.cfg['counter_bits']
        # 왼쪽·오른쪽을 같은 방식으로 처리해 (왼쪽 m, 오른쪽 m) 를 만듭니다.
        #   각 바퀴: 틱 증분(wrap_delta) → 미터(ticks_to_metres) → 방향 부호(sign) 곱하기
        #   ( ... for ... in ...) 는 제너레이터. 두 개가 나오니 left, right 로 바로 풀립니다.
        left, right = (
            sign * ticks_to_metres(wrap_delta(was, is_now, bits),
                                   self.cfg['ticks_per_rev'],
                                   self.cfg['wheel_radius'])
            for was, is_now, sign in (
                (previous[0], left_ticks, self.cfg['left_sign']),
                (previous[1], right_ticks, self.cfg['right_sign'])))
        # 지난 표본 이후 흐른 시간(초). nanoseconds 는 정수 나노초라 1e9 로 나눔
        dt = (now - previous_stamp).nanoseconds / 1e9
        # 말도 안 되게 빨리 움직였으면 = 잡음으로 숫자가 깨진 줄 → 버림
        #   허용 거리 = 최대속도(0.5 m/s) × 시간 (시간이 너무 짧으면 최소 0.1 s 로 봄)
        if max(abs(left), abs(right)) > self.cfg['max_wheel_speed'] * max(dt, SAMPLE_PERIOD):
            # 모터 잡음으로 숫자 중간 바이트가 빠진 줄(예: 1234 -> 124)도 형식은 맞아
            # 통과합니다. 적분하면 /odom과 TF가 한 표본 동안 튀고 SLAM이 그 순간을
            # 쓸 수 있습니다. 기준점을 그대로 두면 다음 정상 줄이 이동을 모두 되찾습니다.
            self.get_logger().warn(
                f'엔코더 틱이 속도 상한을 넘어 버렸습니다: {previous} -> {(left_ticks, right_ticks)}')
            return
        # 정상 → 기준점 갱신
        self.ticks, self.stamp = (left_ticks, right_ticks), now
        # kinematics.integrate 로 새 위치 계산
        self.pose = integrate(self.pose, left, right,
                              self.cfg['wheel_separation'])
        # 발행. 속도 = 거리 ÷ 시간
        #   선속도(m/s)  = 두 바퀴 평균 거리 / dt
        #   각속도(rad/s) = (오른쪽-왼쪽)/트레드 / dt
        #   A if 조건 else B : 조건이 참이면 A. dt 가 0 이면 0으로 나누기를 피합니다.
        self.publish(now, (left + right) / 2 / dt if dt > 0 else 0.0,
                     (right - left) / self.cfg['wheel_separation'] / dt if dt > 0 else 0.0)

    # publish: 현재 위치/속도를 /odom 메시지와 TF 로 만들어 내보냅니다.
    def publish(self, stamp, linear, angular):
        x, y, theta = self.pose
        # ROS 는 각도를 쿼터니언(x,y,z,w 4개 숫자)으로 표현합니다.
        # 평면(z축)에서만 도는 로봇은 z=sin(θ/2), w=cos(θ/2) 만 채우면 됩니다(x=y=0).
        rotation = Quaternion(z=math.sin(theta / 2), w=math.cos(theta / 2))
        message = Odometry()
        # header: 이 값이 "언제", "어느 좌표계 기준"인지. 모든 센서 메시지의 공통 머리말.
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = self.cfg['odom_frame']   # 위치의 기준 = odom
        message.child_frame_id = self.cfg['base_frame']    # 속도의 기준 = base_link(로봇 몸)
        message.pose.pose.position.x, message.pose.pose.position.y = x, y
        message.pose.pose.orientation = rotation
        # 차동구동은 옆으로 못 가므로 linear.x(앞) 와 angular.z(회전) 만 채움
        message.twist.twist.linear.x, message.twist.twist.angular.z = linear, angular
        px, py, pyaw = self.cfg['pose_covariance']
        tx, ty, tyaw = self.cfg['twist_covariance']
        # 6x6 대각 성분 중 x, y, yaw만 채우고 쓰지 않는 축은 큰 값으로 막습니다.
        #   공분산 = "이 값이 얼마나 불확실한지". 36칸짜리 1차원 배열로 6x6 행렬을 표현.
        #   축 순서 x,y,z,roll,pitch,yaw → 대각선 위치 0,7,14,21,28,35
        #   z/roll/pitch 는 안 쓰니 1e6(엄청 불확실) = "이 값은 믿지 마세요"
        for target, (vx, vy, vyaw) in ((message.pose.covariance, (px, py, pyaw)),
                                       (message.twist.covariance, (tx, ty, tyaw))):
            target[0], target[7], target[35] = vx, vy, vyaw
            target[14] = target[21] = target[28] = 1e6
        # /odom 토픽으로 전송
        self.odom.publish(message)

        # YAML 에서 publish_tf: false 면 TF 는 생략 (다른 노드가 대신 낼 때)
        if not self.cfg['publish_tf']:
            return
        # TF 한 개 = "부모 좌표계 → 자식 좌표계" 위치+회전
        transform = TransformStamped()
        transform.header = message.header                   # 시각 + 부모(odom) 재사용
        transform.child_frame_id = self.cfg['base_frame']   # 자식 = base_link
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation = rotation
        self.tf.sendTransform(transform)

    # check_link: 0.5 s 마다 타이머가 부름. 데이터가 끊겼는지 감시합니다.
    def check_link(self):
        """틱이 끊기거나 처음부터 오지 않으면 알립니다. 자세를 추정하지는 않습니다.

        포트·보드레이트·필드 번호가 틀리면 노드는 멀쩡히 살아 있고 로그도 조용한 채
        /odom만 나오지 않습니다. SLAM은 TF가 없어 지도를 못 만드는데 원인이 보이지
        않으므로, 한 번도 수신하지 못한 경우도 같은 오류로 알립니다.
        """
        # 이미 경고했으면 또 하지 않음 (로그 도배 방지)
        if self.warned:
            return
        # 받은 적 있음 → 마지막 수신 시각부터 0.5 s 기준
        # 받은 적 없음 → 노드 시작 시각부터 3 s 기준 (Nucleo 부팅 여유)
        if self.stamp is not None:
            since, limit = self.stamp, self.cfg['timeout']
        else:
            since, limit = self.start, STARTUP_GRACE
        # 아직 기준 시간 안이면 정상
        if (self.get_clock().now() - since).nanoseconds / 1e9 <= limit:
            return
        self.warned = True
        self.get_logger().error(
            '엔코더 데이터가 끊겼습니다. 이동 중이면 SLAM 지도가 어긋납니다.'
            if self.stamp is not None else
            '엔코더 데이터가 한 번도 오지 않았습니다. encoder.yaml의 port·baud와 '
            'left_field/right_field를 확인하십시오 (tools/sniff_encoder_serial.py).')


# ─────────────────────────────────────────────────────────────────────────────
# main: `ros2 run cwu_base encoder_odom` 하면 실행되는 시작점.
#   setup.py 의 'encoder_odom = cwu_base.encoder_odom:main' 이 이 함수를 가리킵니다.
#   모든 rclpy 노드의 main 은 거의 이 모양입니다: init → 노드 생성 → spin → 정리
# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()               # ROS 2 통신 시작 (반드시 노드 만들기 전에)
    node = EncoderOdometry()   # __init__ 실행 → 파라미터 읽고 포트 열고 타이머 등록
    try:
        rclpy.spin(node)       # 무한 루프: 타이머·구독 콜백을 때 맞춰 호출. Ctrl+C 까지 여기 머묾
    except KeyboardInterrupt:  # Ctrl+C 는 정상 종료로 취급
        pass
    finally:                   # 에러가 나든 안 나든 항상 실행되는 정리 구간
        node.serial.close()    # 포트 닫기 (잠금 해제 → 다른 노드가 열 수 있게)
        node.destroy_node()
        rclpy.try_shutdown()   # ROS 2 통신 종료
