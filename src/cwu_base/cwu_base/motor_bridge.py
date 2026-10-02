# [공부 노트] 이 노드가 하는 일 — "Nav2 의 속도 명령을 바퀴로, 바퀴 엔코더를 /odom 으로"
#   입력  : /cmd_vel_safe (geometry_msgs/Twist)  ← Nav2 → Collision Monitor 를 거친 안전한 속도
#           /scan (LaserScan)                    ← LiDAR 가 살아 있는지 감시용
#           TF  base_link→laser, map→odom        ← 좌표 사슬이 살아 있는지 감시용
#   시리얼: /dev/ttyAMA0 로 Nucleo 와 대화 (규약은 motor_safety.py 맨 위 참고)
#   출력  : /odom + TF odom→base_link  (encoder_odom 과 같은 역할을 이 노드가 대신 함)
#   설정  : config/motor.yaml — 실측 전(-1, false)이면 validate_config 가 실행을 거부
#
# [공부 노트] 왜 encoder_odom 을 "상속"했나?
#   class MotorBridge(EncoderOdometry) → encoder_odom 의 publish() (/odom·TF 만들기)를 그대로 재사용.
#   대신 __init__ 은 부모 것을 쓰지 않고 Node.__init__ 을 직접 불러 자기 설정으로 새로 만듭니다.
#   한 시리얼 포트는 한 프로그램만 열 수 있으므로 encoder_odom 과 동시에 켜면 안 됩니다.
#
# [공부 노트] 안전 설계 한 줄 요약
#   "의심스러우면 STOP". 스캔·엔코더·TF·명령 중 하나라도 늦거나 이상하면 Safety 가 속도를 버리고,
#   send() 가 0.05 s 마다 STOP 을 보냅니다. Pi 가 죽어도 Nucleo 가 300 ms 뒤 스스로 멈춥니다.

"""One serial owner for closed-loop wheel commands and encoder odometry."""
import math
import secrets  # 안전한 난수 — session 번호 뽑기
import time     # time.monotonic(): 거꾸로 가지 않는 시계 (안전 판단용)

import rclpy
import serial
from geometry_msgs.msg import Twist            # 속도 명령 메시지 (linear.x, angular.z)
from nav_msgs.msg import Odometry
from rclpy.node import Node
# 센서용 QoS (best effort) — LiDAR 드라이버와 맞춰야 수신됨
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
# Buffer + TransformListener : 다른 노드가 발행한 TF 를 받아 저장하고 조회하는 도구
from tf2_ros import Buffer, TransformBroadcaster, TransformListener, TransformException

from cwu_base.encoder_odom import EncoderOdometry  # 부모 클래스 (publish 재사용)
from cwu_base.kinematics import integrate, ticks_to_metres, wrap_delta
from cwu_base.motor_safety import Safety, parse_frame, wheel_ticks, validate_config


class MotorBridge(EncoderOdometry):
    def __init__(self):
        # 부모(EncoderOdometry)의 __init__ 은 건너뛰고 Node 로 직접 초기화 — 포트를 두 번 열지 않기 위해
        Node.__init__(self, 'motor_bridge')
        # 실측해야 하는 값 목록 (모두 기본 -1 → 안 채우면 validate_config 에서 거부)
        measured = ('wheel_radius', 'wheel_separation', 'ticks_per_rev', 'max_wheel_speed',
                    'kp_left', 'ki_left', 'kp_right', 'ki_right', 'max_accel_ticks',
                    'robot_radius', 'braking_distance', 'safety_margin', 'max_linear_speed',
                    'max_angular_speed', 'max_linear_accel', 'max_angular_accel', 'safety_latency')
        # (이름, 기본값) 목록 만들기. 리스트 컴프리헨션 + 나머지 고정 항목 이어 붙이기
        defaults = [(k, -1.0) for k in measured] + [
            ('calibration_confirmed', False), ('mount_calibration_confirmed', False),
            ('port', '/dev/ttyAMA0'), ('baud', 115200), ('pwm_limit', -1),
            ('mcu_timeout_ms', 300), ('command_timeout', .2), ('scan_timeout', .3),
            ('encoder_timeout', .3), ('tf_timeout', .5), ('tf_future_tolerance', .2),
            ('left_sign', 1.0), ('right_sign', 1.0), ('odom_frame', 'odom'),
            ('base_frame', 'base_link'), ('map_frame', 'map'), ('publish_tf', True),
            ('pose_covariance', [.01, .01, .05]), ('twist_covariance', [.01, .01, .05])]
        self.cfg = {p.name: p.value for p in self.declare_parameters('', defaults)}
        c = self.cfg
        # motor.yaml 검사 — 실패하면 여기서 예외로 종료
        validate_config(c)
        # 실물 모터를 가짜 시계로 돌리면 시간 판단이 전부 틀어지므로 금지
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Physical motor bridge cannot use simulated time')
        # Integer cap is exactly representable by the MCU float parser.
        # ↑ 최고 바퀴 속도(m/s)를 틱/초 정수로. floor = 내림
        self.max_ticks = math.floor(c['max_wheel_speed'] * c['ticks_per_rev'] / (2*math.pi*c['wheel_radius']))
        self.safety = Safety(c['command_timeout'], c['scan_timeout'], c['encoder_timeout'])
        self.pose, self.ticks, self.encoder_ms = (0., 0., 0.), None, None
        # 부모의 publish() 가 self.odom, self.tf 를 쓰므로 같은 이름으로 만들어 둠
        self.odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)
        # TF 조회 준비: listener 가 /tf 를 계속 받아 buffer 에 쌓음
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_frame = ''              # 스캔의 frame_id (예: laser_frame). 첫 정상 스캔에서 채움
        self.last_scan_stamp = -math.inf  # 같은 스캔이 두 번 오는 것 방지
        self.serial, self.buffer = None, bytearray()  # 포트는 poll() 에서 연결 (끊기면 재연결)
        self.next_connect = 0.
        # 구독: 토픽, 메시지 형식, 콜백 함수, QoS
        #   /cmd_vel_safe 큐 1 = 가장 최신 명령만 의미 있음
        self.create_subscription(Twist, '/cmd_vel_safe', self.command, 1)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_timer(.005, self.poll)  # 5 ms: 시리얼 수신
        self.create_timer(.05, self.send)   # 50 ms(20 Hz): 모터 명령 송신

    # write: 문자열 한 줄을 시리얼로 보냄. 일부만 써지면 예외 → 호출자가 연결을 끊음
    def write(self, line):
        data = (line+'\n').encode('ascii')
        if self.serial.write(data) != len(data):
            raise serial.SerialException('short serial write')

    # configure: 새 세션 시작. STOP → CFG(설정) 전송. Nucleo 가 ACK 를 보내야 구동 허용
    def configure(self):
        self.safety.reset()
        self.ticks = self.encoder_ms = None
        self.mcu_time = self.mcu_stamp = None
        # 1 ~ 2147483646 사이 무작위 세션 번호
        self.session = secrets.randbelow(2147483646)+1
        c = self.cfg
        self.write('STOP')
        # CFG,session,max_ticks,kpL,kiL,kpR,kiR,pwm_limit,accel_ticks,timeout_ms,left_sign,right_sign
        fields = (self.session, self.max_ticks, c['kp_left'], c['ki_left'], c['kp_right'],
                  c['ki_right'], c['pwm_limit'], c['max_accel_ticks'], c['mcu_timeout_ms'],
                  int(c['left_sign']), int(c['right_sign']))
        self.write('CFG,'+','.join(str(v) for v in fields))
        self.configured_at = time.monotonic()  # ACK 가 1초 안에 안 오면 다시 보내려고 기록

    # disconnect: STOP 시도 후 포트를 닫고, 1초 뒤 재연결 예약
    def disconnect(self):
        self.safety.reset()
        if self.serial is not None:
            try:
                self.write('STOP')
            except (serial.SerialException, OSError):
                pass  # 이미 끊겼으면 STOP 도 못 보냄 → Nucleo 의 300 ms 만료가 멈춰 줌
            self.serial.close()
        self.serial = None
        self.next_connect = time.monotonic()+1.

    # tf_ok: 필요한 TF 두 개가 "최근 것"으로 존재하는지
    #   ① base_link → 스캔 frame (LiDAR 장착. static 이면 stamp=0 이라 나이 검사 생략)
    #   ② map → odom (SLAM 이 살아 있다는 증거)
    def tf_ok(self):
        if not self.scan_frame:
            return False
        try:
            # Time() = "가장 최근 것 아무거나"
            laser = self.tf_buffer.lookup_transform(self.cfg['base_frame'], self.scan_frame, Time())
            stamp = Time.from_msg(laser.header.stamp).nanoseconds
            age = (self.get_clock().now().nanoseconds-stamp)/1e9
            if stamp and not -self.cfg['tf_future_tolerance'] <= age <= self.cfg['tf_timeout']:
                return False
            tf = self.tf_buffer.lookup_transform(self.cfg['map_frame'], self.cfg['odom_frame'], Time())
            age = (self.get_clock().now().nanoseconds - Time.from_msg(tf.header.stamp).nanoseconds)/1e9
            return -self.cfg['tf_future_tolerance'] <= age <= self.cfg['tf_timeout']
        except TransformException:  # 아직 한 번도 안 왔거나 사슬이 끊김
            return False

    # scan: /scan 이 올 때마다 호출. 스캔이 "정상이고 최신인지" 검사해 Safety 에 알려줌
    def scan(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        age = self.get_clock().now().nanoseconds/1e9-stamp
        # 정상 조건: frame 이름 있음, 최신, 새 스캔, 범위·각도 값이 유한, 유효 거리가 하나라도 있음
        valid = (bool(msg.header.frame_id) and 0 <= age <= self.cfg['scan_timeout']
                 and stamp > self.last_scan_stamp and math.isfinite(msg.range_min)
                 and math.isfinite(msg.range_max) and 0 < msg.range_min < msg.range_max
                 and math.isfinite(msg.angle_min) and math.isfinite(msg.angle_max)
                 and math.isfinite(msg.angle_increment) and msg.angle_increment != 0
                 and any(math.isfinite(r) and msg.range_min <= r <= msg.range_max for r in msg.ranges))
        if valid:
            self.scan_frame, self.last_scan_stamp = msg.header.frame_id, stamp
            # ROS 시각 대신 monotonic 기준으로 "스캔 찍힌 시각"을 환산해 저장
            self.safety.scan = time.monotonic()-age
        else:
            self.safety.scan = -math.inf
            self.safety.discard()

    # command: /cmd_vel_safe 가 올 때마다 호출. 속도 → 바퀴 틱/초 로 바꿔 Safety 에 맡김
    def command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
        # 거부 조건: NaN/inf, 후진(linear.x<0 — 복구 동작에서 후진을 뺀 설계와 맞춤),
        #            차동구동이 못 하는 축(옆·위·기울기) 명령
        if (not all(math.isfinite(v) for v in values)
                or msg.linear.x < 0 or any(v != 0 for v in values[1:5])):
            self.safety.discard()
            return
        c = self.cfg
        # 속도/회전 한계를 넘으면 같은 비율로 줄임 (경로 모양 유지)
        scale = max(1., abs(msg.linear.x)/c['max_linear_speed'], abs(msg.angular.z)/c['max_angular_speed'])
        left, right = wheel_ticks(msg.linear.x/scale, msg.angular.z/scale, c['wheel_separation'],
                                  c['wheel_radius'], c['ticks_per_rev'], self.max_ticks)
        self.safety.command(left, right, time.monotonic(), self.tf_ok())

    # encoder: 정상 ENC 프레임 하나 처리 → 위치 적분 → /odom 발행
    #   필요한 입력: ms(MCU 시각), left/right(누적 틱)
    def encoder(self, ms, left, right):
        now = time.monotonic()
        # MCU 시계와 Pi 시계를 맞춰 보며, 늦게 온(밀린) 프레임이면 명령을 끊음
        if self.mcu_stamp is None:
            self.mcu_time, self.mcu_stamp = now, ms
        else:
            elapsed = (ms-self.mcu_stamp) % (1 << 32)   # MCU 기준 흐른 시간 (되감김 처리)
            expected = self.mcu_time+elapsed/1000.       # 그러면 Pi 시계로 지금쯤이어야 함
            if elapsed >= (1 << 31) or not -.1 <= now-expected <= self.cfg['encoder_timeout']:
                self.safety.discard()
                return
            self.mcu_time, self.mcu_stamp = expected, ms
        if not self.safety.encoder(ms, now):
            self.safety.discard()
            return
        if self.ticks is not None:
            # dt 는 Pi 시계가 아니라 MCU 시각 차이로 계산 → 시리얼 지연에 흔들리지 않음
            dt = ((ms-self.encoder_ms) % (1 << 32))/1000.
            # 좌·우 이동 거리(m) 목록. zip 으로 (이전, 지금, 부호)를 짝지어 처리
            distances = [sign*ticks_to_metres(wrap_delta(old, new, 32), self.cfg['ticks_per_rev'], self.cfg['wheel_radius'])
                         for old, new, sign in zip(self.ticks, (left, right), (self.cfg['left_sign'], self.cfg['right_sign']))]
            # 물리적으로 불가능한 이동 = 깨진 데이터 → 엔코더를 "끊김" 처리해 정지
            if max(map(abs, distances)) > self.cfg['max_wheel_speed']*dt*1.5:
                self.safety.enc = -math.inf
                self.safety.discard()
                return
            self.pose = integrate(self.pose, *distances, self.cfg['wheel_separation'])
            # 부모(EncoderOdometry)의 publish 재사용
            self.publish(self.get_clock().now(), sum(distances)/2/dt,
                         (distances[1]-distances[0])/self.cfg['wheel_separation']/dt)
        self.ticks, self.encoder_ms = (left, right), ms

    # poll: 5 ms 마다. (필요하면 연결) → 수신 → 줄 단위로 프레임 처리
    def poll(self):
        try:
            if self.serial is None:
                if time.monotonic() < self.next_connect:
                    return  # 재연결 대기 중
                # write_timeout=.02 : 쓰기가 20 ms 넘게 막히면 예외 → 루프가 멈추지 않게
                self.serial = serial.Serial(self.cfg['port'], self.cfg['baud'], timeout=0,
                                            write_timeout=.02, exclusive=True)
                self.serial.reset_input_buffer()  # 연결 전에 쌓인 옛 데이터 버림
                self.buffer.clear()
                self.configure()
            self.buffer.extend(self.serial.read(4096))
            # 줄바꿈 없이 8 KB 가 쌓임 = 보드레이트 불일치 등 → 재연결
            if len(self.buffer) > 8192:
                raise serial.SerialException('serial frame overflow')
            # 완성된 줄이 있는 동안 하나씩 꺼내 처리. partition = 첫 \n 기준 앞/구분자/뒤
            while b'\n' in self.buffer:
                line, _, self.buffer = self.buffer.partition(b'\n')
                try:
                    frame = parse_frame(line.rstrip(b'\r'))
                except (ValueError, UnicodeError):
                    self.safety.discard()  # 깨진 줄 = 잡음 의심 → 명령 버림
                    continue
                if frame[0] == 'READY':                        # Nucleo 재부팅됨 → 다시 설정
                    self.configure()
                elif frame[0] == 'ACK' and frame[1] == self.session:  # 우리 설정 확인됨
                    self.safety.acknowledged = True
                elif frame[0] == 'ENC':
                    if frame[4] != self.session:               # 다른 세션 = 설정이 풀림 → 재설정
                        self.configure()
                    elif self.safety.acknowledged:
                        self.encoder(*frame[1:4])              # (ms, L, R)
            # CFG 보낸 지 1초가 지나도 ACK 가 없으면 다시 보냄
            if not self.safety.acknowledged and time.monotonic()-self.configured_at > 1.:
                self.configure()
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(str(e))
            self.disconnect()

    # send: 20 Hz. Safety 가 허락한 속도면 VEL, 아니면 STOP 을 매번 보냄
    #   (계속 보내는 이유: Nucleo 는 300 ms 동안 명령이 없으면 스스로 멈춤 — 살아있다는 신호 겸용)
    def send(self):
        if self.serial is None:
            return
        velocity = self.safety.output(time.monotonic(), self.tf_ok())
        try:
            self.write('STOP' if velocity is None or velocity == (0., 0.) else
                       f'VEL,{self.session},{int(velocity[0])},{int(velocity[1])}')
        except (serial.SerialException, OSError):
            self.disconnect()


# ros2 run cwu_base motor_bridge 시작점. 종료 시 반드시 STOP 을 보내고 포트를 닫음
def main():
    rclpy.init()
    node = None  # 생성 중 예외(설정 거부)가 나도 finally 에서 안전하게 처리하려고 먼저 None
    try:
        node = MotorBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.disconnect()
            node.destroy_node()
        rclpy.try_shutdown()
