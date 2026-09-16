# /target/bearing을 받아 목표물이 화면 세로 중심선에 오도록 도는 시각 서보 노드입니다.
# 속도는 /cmd_vel_safe가 아니라 /cmd_vel_servo로 나갑니다. Nav2의 /cmd_vel과 이 토픽
# 중 무엇을 모터로 보낼지는 안전 감시자(cwu_safety, 미구현)가 미션 상태를 보고 고릅니다.
#
# 검출 콜백이 아니라 고정 주기로 발행합니다. 목표물이 안 보이게 됐을 때 마지막 명령이
# 남아 로봇이 계속 도는 것을 막으려면, 표본이 끊긴 것 자체가 0을 만들어야 합니다.

"""Visual servo: keep the target on the image centre line."""
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from cwu_nav.servo import follow, yaw_error


class TargetFollower(Node):
    def __init__(self):
        super().__init__('target_follower')
        defaults = {
            'bearing_topic': '/target/bearing',
            'cmd_topic': '/cmd_vel_servo',
            'rate_hz': 20.0,
            'k_yaw': 1.2, 'max_yaw': 0.8, 'min_yaw': 0.0,
            'align_tol_rad': 0.09, 'approach_speed': 0.08,
            'sample_timeout_s': 0.5,
        }
        self.declare_parameters('', list(defaults.items()))
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if self.p['rate_hz'] <= 0 or self.p['sample_timeout_s'] <= 0:
            raise ValueError('rate_hz and sample_timeout_s must be positive')
        if self.p['k_yaw'] <= 0 or self.p['max_yaw'] <= 0:
            raise ValueError('k_yaw and max_yaw must be positive')
        if not 0 <= self.p['min_yaw'] <= self.p['max_yaw']:
            raise ValueError('min_yaw must be between 0 and max_yaw')
        if self.p['align_tol_rad'] <= 0 or self.p['approach_speed'] <= 0:
            raise ValueError('align_tol_rad and approach_speed must be positive')

        self.sample = None
        self.publisher = self.create_publisher(Twist, self.p['cmd_topic'], 10)
        self.create_subscription(PointStamped, self.p['bearing_topic'],
                                 self.on_bearing, 10)
        self.create_timer(1.0 / self.p['rate_hz'], self.on_tick)
        self.get_logger().info(
            '시각 서보 시작. %s로 발행하며 모터 연결은 안전 감시자가 정합니다.'
            % self.p['cmd_topic'])

    def on_bearing(self, msg):
        self.sample = msg

    def age_s(self):
        stamp = self.sample.header.stamp
        return self.get_clock().now().nanoseconds * 1e-9 - (
            stamp.sec + stamp.nanosec * 1e-9)

    def on_tick(self):
        command = Twist()
        if self.sample is not None and self.age_s() <= self.p['sample_timeout_s']:
            p = self.p
            error = yaw_error(self.sample.point.x, self.sample.point.z)
            command.linear.x, command.angular.z = follow(
                error, p['k_yaw'], p['max_yaw'], p['min_yaw'],
                p['align_tol_rad'], p['approach_speed'])
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TargetFollower()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
