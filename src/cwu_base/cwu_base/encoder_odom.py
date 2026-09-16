# Nucleo가 시리얼로 보내는 엔코더 틱을 읽어 /odom과 odom → base_link TF를 발행합니다.
# SLAM Toolbox는 이 TF가 있어야 지도를 만들 수 있으므로 실물 SLAM의 필수 입력입니다.
# 프로토콜과 바퀴 제원은 encoder.yaml에서 읽으며, 미입력 값이 있으면 실행을 거부합니다.

"""Serial encoder ticks to /odom and the odom -> base_link transform."""
import math
import re

import rclpy
import serial
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from cwu_base.kinematics import integrate, ticks_to_metres, wrap_delta

NUMBER = re.compile(rb'-?\d+')
# 값을 지어내지 않기 위한 표식입니다. 이 값이 남아 있으면 실측 전이라는 뜻입니다.
UNSET = -1.0
# 보드레이트가 틀리면 줄바꿈 없는 쓰레기가 무한히 쌓입니다. 오래된 쪽부터 버립니다.
MAX_BUFFER = 65536
# MCU가 USB 열거와 펌웨어 시작을 마칠 때까지 기다려 주는 시간입니다. 이보다
# 늦게까지 한 줄도 못 받으면 설정이 틀린 것으로 봅니다. 이 유예가 없으면
# 부팅 때마다 오류가 한 줄 뜨고, 사람은 곧 오류 로그를 무시하게 됩니다.
STARTUP_GRACE = 3.0


def latest_sample(lines, left_field, right_field):
    """완성된 줄들 중 가장 최신의 유효한 (왼쪽, 오른쪽) 틱을 돌려줍니다.

    틱은 누적값이라 최신 줄 하나만 봐도 자세 증분은 같습니다. 반대로 한 번의
    read에 밀려 들어온 옛 줄까지 차례로 처리하면 줄 사이 간격을 벽시계로 재게 되어
    dt가 0에 가까워지고 twist가 발산합니다(부팅 직후 밀린 버퍼에서 특히).
    형식이 어긋난 줄(잘린 첫 줄, 펌웨어 로그)은 건너뜁니다.
    """
    for line in reversed(lines):
        fields = NUMBER.findall(line)
        if len(fields) > max(left_field, right_field):
            return int(fields[left_field]), int(fields[right_field])
    return None


class EncoderOdometry(Node):

    def __init__(self):
        super().__init__('encoder_odom')
        p = self.declare_parameters('', [
            ('port', '/dev/ttyACM0'), ('baud', 115200),
            ('wheel_radius', UNSET), ('wheel_separation', UNSET),
            ('ticks_per_rev', UNSET),
            ('left_field', -1), ('right_field', -1),
            ('left_sign', 1.0), ('right_sign', 1.0),
            ('counter_bits', 32), ('timeout', 0.5),
            ('odom_frame', 'odom'), ('base_frame', 'base_link'),
            ('publish_tf', True),
            ('pose_covariance', [0.01, 0.01, 0.05]),
            ('twist_covariance', [0.01, 0.01, 0.05]),
        ])
        self.cfg = {d.name: d.value for d in p}
        self._require_measured_values()

        self.pose = (0.0, 0.0, 0.0)
        self.start = self.get_clock().now()
        self.ticks = None
        self.stamp = None
        self.buffer = bytearray()
        self.warned = False

        self.odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)
        self.serial = serial.Serial(
            self.cfg['port'], self.cfg['baud'], timeout=0)
        self.get_logger().info(
            f"엔코더 {self.cfg['port']} @ {self.cfg['baud']} baud, "
            f"필드 L={self.cfg['left_field']} R={self.cfg['right_field']}")
        # 시리얼을 블로킹으로 읽지 않고 짧은 주기로 비웁니다. 스레드가 없어야
        # 종료와 파라미터 처리가 단순하고, MCU 전송 주기에 맞춰 발행됩니다.
        self.create_timer(0.005, self.poll)
        self.create_timer(self.cfg['timeout'], self.check_link)

    def _require_measured_values(self):
        """추측 대신 즉시 실패합니다. 틀린 오도메트리는 조용히 지도를 망칩니다."""
        missing = [k for k in ('wheel_radius', 'wheel_separation', 'ticks_per_rev')
                   if self.cfg[k] <= 0]
        missing += [k for k in ('left_field', 'right_field') if self.cfg[k] < 0]
        if missing:
            raise ValueError(
                f"encoder.yaml 미입력 값: {', '.join(missing)}. "
                '바퀴 반지름·트레드·회전당 틱은 실측하고, 필드 번호는 '
                'python3 tools/sniff_encoder_serial.py 출력에서 확인하십시오.')

    def poll(self):
        """수신 버퍼를 비우고 가장 최신 표본으로 자세를 갱신합니다."""
        try:
            self.buffer += self.serial.read(4096)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'엔코더 시리얼 오류: {e}')
            return
        *lines, self.buffer = self.buffer.split(b'\n')
        del self.buffer[:-MAX_BUFFER]
        sample = latest_sample(lines, self.cfg['left_field'], self.cfg['right_field'])
        if sample is not None:
            self.update(*sample)

    def update(self, left_ticks, right_ticks):
        now = self.get_clock().now()
        previous, self.ticks, previous_stamp, self.stamp = (
            self.ticks, (left_ticks, right_ticks), self.stamp, now)
        if previous is None:
            return  # 첫 표본은 기준점으로만 씁니다.
        self.warned = False
        bits = self.cfg['counter_bits']
        left, right = (
            sign * ticks_to_metres(wrap_delta(was, is_now, bits),
                                   self.cfg['ticks_per_rev'],
                                   self.cfg['wheel_radius'])
            for was, is_now, sign in (
                (previous[0], left_ticks, self.cfg['left_sign']),
                (previous[1], right_ticks, self.cfg['right_sign'])))
        self.pose = integrate(self.pose, left, right,
                              self.cfg['wheel_separation'])
        dt = (now - previous_stamp).nanoseconds / 1e9
        self.publish(now, (left + right) / 2 / dt if dt > 0 else 0.0,
                     (right - left) / self.cfg['wheel_separation'] / dt if dt > 0 else 0.0)

    def publish(self, stamp, linear, angular):
        x, y, theta = self.pose
        rotation = Quaternion(z=math.sin(theta / 2), w=math.cos(theta / 2))
        message = Odometry()
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = self.cfg['odom_frame']
        message.child_frame_id = self.cfg['base_frame']
        message.pose.pose.position.x, message.pose.pose.position.y = x, y
        message.pose.pose.orientation = rotation
        message.twist.twist.linear.x, message.twist.twist.angular.z = linear, angular
        px, py, pyaw = self.cfg['pose_covariance']
        tx, ty, tyaw = self.cfg['twist_covariance']
        # 6x6 대각 성분 중 x, y, yaw만 채우고 쓰지 않는 축은 큰 값으로 막습니다.
        for target, (vx, vy, vyaw) in ((message.pose.covariance, (px, py, pyaw)),
                                       (message.twist.covariance, (tx, ty, tyaw))):
            target[0], target[7], target[35] = vx, vy, vyaw
            target[14] = target[21] = target[28] = 1e6
        self.odom.publish(message)

        if not self.cfg['publish_tf']:
            return
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = self.cfg['base_frame']
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation = rotation
        self.tf.sendTransform(transform)

    def check_link(self):
        """틱이 끊기거나 처음부터 오지 않으면 알립니다. 자세를 추정하지는 않습니다.

        포트·보드레이트·필드 번호가 틀리면 노드는 멀쩡히 살아 있고 로그도 조용한 채
        /odom만 나오지 않습니다. SLAM은 TF가 없어 지도를 못 만드는데 원인이 보이지
        않으므로, 한 번도 수신하지 못한 경우도 같은 오류로 알립니다.
        """
        if self.warned:
            return
        if self.stamp is not None:
            since, limit = self.stamp, self.cfg['timeout']
        else:
            since, limit = self.start, STARTUP_GRACE
        if (self.get_clock().now() - since).nanoseconds / 1e9 <= limit:
            return
        self.warned = True
        self.get_logger().error(
            '엔코더 데이터가 끊겼습니다. 이동 중이면 SLAM 지도가 어긋납니다.'
            if self.stamp is not None else
            '엔코더 데이터가 한 번도 오지 않았습니다. encoder.yaml의 port·baud와 '
            'left_field/right_field를 확인하십시오 (tools/sniff_encoder_serial.py).')


def main():
    rclpy.init()
    node = EncoderOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial.close()
        node.destroy_node()
        rclpy.try_shutdown()
