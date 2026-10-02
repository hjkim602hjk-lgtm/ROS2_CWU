# 장비 없이 SLAM을 검증하기 위한 가상 센서 노드입니다.
# 가상 로봇의 이동과 벽까지의 거리를 계산해 /scan, /odom, odom → base_link TF를 발행합니다.
# drive:=circle은 고정 원운동, drive:=cmd_vel은 /cmd_vel을 적분해 움직입니다.
# cmd_vel 모드는 Nav2가 보낸 속도가 실제 움직임으로 이어지는지 하드웨어 없이 보려고 있습니다.
# 이상적인 센서 데이터를 사용하며 실제 모터를 제어하지 않습니다.
#
# [공부 노트] 이 노드는 "LiDAR 드라이버 + encoder_odom" 두 개를 한 번에 흉내냅니다.
#   출력 형식이 실물과 똑같아서, SLAM·Nav2 쪽은 가짜인지 진짜인지 모르고 그대로 동작합니다.
#   → 로봇 없이 PC 에서 전체 흐름을 공부·시험할 수 있는 이유.
#   가상 방: 3.6 m x 3.6 m 정사각형 벽(경기장 크기) + 안쪽에 0.4 m 상자 하나 (walls 파라미터)

"""Development-only ideal scans and odometry; never commands real actuators."""
import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data  # 실물 LiDAR 드라이버와 같은 QoS 로 발행
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from cwu_slam.geometry import raycast  # 광선 → 벽까지 거리 (geometry.py)


class DemoSensors(Node):
    def __init__(self):
        super().__init__('demo_sensors')
        defaults = {
            'publish_rate': 10.0, 'beam_count': 360,   # 10 Hz, 1도 간격 360개 빔
            'range_min': 0.12, 'range_max': 12.0,     # 가짜 LiDAR 측정 범위 (m)
            # circle 모드: 반지름 0.55 m 원을 0.12 rad/s 로
            'path_radius': 0.55, 'angular_speed': 0.12,
            'drive': 'circle', 'cmd_vel_topic': '/cmd_vel', 'cmd_timeout': 0.5,
            # LiDAR 장착 위치 (bringup 이 mount.yaml 값으로 넘김)
            'laser_x': 0.0, 'laser_y': 0.0, 'laser_yaw': 0.0,
            # 벽 선분 목록: 4개씩 끊어서 (x1, y1, x2, y2). 앞 4개 선분 = 바깥 벽, 뒤 4개 = 상자 4변
            'walls': [-1.8, -1.8, 1.8, -1.8, 1.8, -1.8, 1.8, 1.8,
                      1.8, 1.8, -1.8, 1.8, -1.8, 1.8, -1.8, -1.8,
                      0.9, 0.9, 1.3, 0.9, 1.3, 0.9, 1.3, 1.3,
                      1.3, 1.3, 0.9, 1.3, 0.9, 1.3, 0.9, 0.9],
        }
        self.declare_parameters('', list(defaults.items()))
        self.p = {key: self.get_parameter(key).value for key in defaults}
        p = self.p
        # ── 설정 검사 ──
        # 숫자 파라미터가 inf/NaN 이면 거부 (문자열·리스트 항목은 제외하고 검사)
        if any(not math.isfinite(v) for k, v in p.items()
               if k not in ('walls', 'drive', 'cmd_vel_topic')):
            raise ValueError('Demo scalar parameters must be finite')
        if p['drive'] not in ('circle', 'cmd_vel'):
            raise ValueError("drive must be 'circle' or 'cmd_vel'")
        if p['cmd_timeout'] <= 0:
            raise ValueError('cmd_timeout must be positive')
        if p['publish_rate'] <= 0 or p['beam_count'] < 4:
            raise ValueError('publish_rate must be positive and beam_count >= 4')
        if not 0 < p['range_min'] < p['range_max'] or p['path_radius'] < 0:
            raise ValueError('Invalid scan range or trajectory radius')
        # len % 4 : 4의 배수가 아니면(선분이 덜 적혔으면) 0 이 아님 → 거부
        if not p['walls'] or len(p['walls']) % 4 or not all(
                math.isfinite(v) for v in p['walls']):
            raise ValueError('walls must contain finite x1,y1,x2,y2 segments')
        # 평평한 목록 → 4개씩 묶은 목록. range(0, n, 4) = 0, 4, 8, ...
        self.walls = [p['walls'][i:i + 4] for i in range(0, len(p['walls']), 4)]
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.start = self.get_clock().now()
        self.last_step = self.start
        self.pose = [0.0, 0.0, 0.0]   # cmd_vel 모드용 위치 (x, y, yaw)
        self.cmd = (0.0, 0.0)         # 마지막 속도 명령 (전진, 회전)
        self.cmd_time = None
        # cmd_vel 모드일 때만 속도 명령 구독
        if p['drive'] == 'cmd_vel':
            self.create_subscription(Twist, p['cmd_vel_topic'], self.on_cmd, 10)
        self.timer = self.create_timer(1.0 / p['publish_rate'], self.publish)
        self.get_logger().info('SYNTHETIC INPUTS ONLY: ideal odometry, no motor control')

    # 속도 명령 저장
    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_time = self.get_clock().now()

    def step(self, now):
        """Advance the synthetic pose and return (x, y, yaw, vx, wz)."""
        p = self.p
        # circle 모드: 시간만으로 위치를 바로 계산 (적분 오차 없음)
        if p['drive'] == 'circle':
            elapsed = (now - self.start).nanoseconds * 1e-9
            yaw = p['angular_speed'] * elapsed
            # A closed circle starts at the odom origin, facing +x.
            return (p['path_radius'] * math.sin(yaw),
                    p['path_radius'] * (1 - math.cos(yaw)), yaw,
                    p['path_radius'] * p['angular_speed'], p['angular_speed'])
        # cmd_vel 모드: 지난 스텝 이후 dt 동안 속도만큼 움직임 (단순 오일러 적분)
        dt = (now - self.last_step).nanoseconds * 1e-9
        self.last_step = now
        vx, wz = self.cmd
        # 명령이 끊기면 멈춥니다. 실물 MCU에도 같은 만료 정지가 필요합니다(계획 Phase 1b).
        if self.cmd_time is None or (
                now - self.cmd_time).nanoseconds * 1e-9 > p['cmd_timeout']:
            vx = wz = 0.0
        self.pose[2] += wz * dt
        self.pose[0] += vx * math.cos(self.pose[2]) * dt
        self.pose[1] += vx * math.sin(self.pose[2]) * dt
        return self.pose[0], self.pose[1], self.pose[2], vx, wz

    # 10 Hz: 위치 갱신 → /odom, TF, /scan 발행
    def publish(self):
        now = self.get_clock().now()
        p = self.p
        x, y, yaw, vx, wz = self.step(now)
        # ── /odom (encoder_odom.publish 와 같은 형식) ──
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2)   # yaw → 쿼터니언
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)
        # ── TF odom → base_link ──
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)

        # ── /scan (LaserScan) ──
        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = 'laser_frame'       # 장착 TF 의 child 이름과 같아야 함
        scan.angle_min = -math.pi                  # 첫 빔 방향: 뒤쪽(-180도)
        scan.angle_increment = 2 * math.pi / p['beam_count']  # 빔 사이 각도
        scan.angle_max = scan.angle_min + (p['beam_count'] - 1) * scan.angle_increment
        scan.scan_time = 1.0 / p['publish_rate']
        # Instantaneous synthetic scan: no per-beam time or motion distortion.
        scan.time_increment = 0.0
        scan.range_min = p['range_min']
        scan.range_max = p['range_max']
        # LiDAR 의 지도상 위치 = 로봇 위치 + (장착 위치를 로봇 방향만큼 회전한 것)
        sx = x + math.cos(yaw) * p['laser_x'] - math.sin(yaw) * p['laser_y']
        sy = y + math.sin(yaw) * p['laser_x'] + math.cos(yaw) * p['laser_y']
        # 빔 i 개마다 광선 쏘기. 방향 = 로봇 yaw + 장착 yaw + 빔 각도
        scan.ranges = [raycast(
            sx, sy, yaw + p['laser_yaw'] + scan.angle_min + i * scan.angle_increment,
            self.walls, scan.range_min, scan.range_max,
        ) for i in range(p['beam_count'])]
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DemoSensors()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
