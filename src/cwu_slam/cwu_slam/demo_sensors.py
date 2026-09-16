# 장비 없이 SLAM을 검증하기 위한 가상 센서 노드입니다.
# 가상 로봇의 이동과 벽까지의 거리를 계산해 /scan, /odom, odom → base_link TF를 발행합니다.
# drive:=circle은 고정 원운동, drive:=cmd_vel은 /cmd_vel을 적분해 움직입니다.
# cmd_vel 모드는 Nav2가 보낸 속도가 실제 움직임으로 이어지는지 하드웨어 없이 보려고 있습니다.
# 이상적인 센서 데이터를 사용하며 실제 모터를 제어하지 않습니다.

"""Development-only ideal scans and odometry; never commands real actuators."""
import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from cwu_slam.geometry import raycast


class DemoSensors(Node):
    def __init__(self):
        super().__init__('demo_sensors')
        defaults = {
            'publish_rate': 10.0, 'beam_count': 360,
            'range_min': 0.12, 'range_max': 12.0,
            'path_radius': 0.55, 'angular_speed': 0.12,
            'drive': 'circle', 'cmd_vel_topic': '/cmd_vel', 'cmd_timeout': 0.5,
            'laser_x': 0.0, 'laser_y': 0.0, 'laser_yaw': 0.0,
            'walls': [-1.8, -1.8, 1.8, -1.8, 1.8, -1.8, 1.8, 1.8,
                      1.8, 1.8, -1.8, 1.8, -1.8, 1.8, -1.8, -1.8,
                      0.9, 0.9, 1.3, 0.9, 1.3, 0.9, 1.3, 1.3,
                      1.3, 1.3, 0.9, 1.3, 0.9, 1.3, 0.9, 0.9],
        }
        self.declare_parameters('', list(defaults.items()))
        self.p = {key: self.get_parameter(key).value for key in defaults}
        p = self.p
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
        if not p['walls'] or len(p['walls']) % 4 or not all(
                math.isfinite(v) for v in p['walls']):
            raise ValueError('walls must contain finite x1,y1,x2,y2 segments')
        self.walls = [p['walls'][i:i + 4] for i in range(0, len(p['walls']), 4)]
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.start = self.get_clock().now()
        self.last_step = self.start
        self.pose = [0.0, 0.0, 0.0]
        self.cmd = (0.0, 0.0)
        self.cmd_time = None
        if p['drive'] == 'cmd_vel':
            self.create_subscription(Twist, p['cmd_vel_topic'], self.on_cmd, 10)
        self.timer = self.create_timer(1.0 / p['publish_rate'], self.publish)
        self.get_logger().info('SYNTHETIC INPUTS ONLY: ideal odometry, no motor control')

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_time = self.get_clock().now()

    def step(self, now):
        """Advance the synthetic pose and return (x, y, yaw, vx, wz)."""
        p = self.p
        if p['drive'] == 'circle':
            elapsed = (now - self.start).nanoseconds * 1e-9
            yaw = p['angular_speed'] * elapsed
            # A closed circle starts at the odom origin, facing +x.
            return (p['path_radius'] * math.sin(yaw),
                    p['path_radius'] * (1 - math.cos(yaw)), yaw,
                    p['path_radius'] * p['angular_speed'], p['angular_speed'])
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

    def publish(self):
        now = self.get_clock().now()
        p = self.p
        x, y, yaw, vx, wz = self.step(now)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)

        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / p['beam_count']
        scan.angle_max = scan.angle_min + (p['beam_count'] - 1) * scan.angle_increment
        scan.scan_time = 1.0 / p['publish_rate']
        # Instantaneous synthetic scan: no per-beam time or motion distortion.
        scan.time_increment = 0.0
        scan.range_min = p['range_min']
        scan.range_max = p['range_max']
        sx = x + math.cos(yaw) * p['laser_x'] - math.sin(yaw) * p['laser_y']
        sy = y + math.sin(yaw) * p['laser_x'] + math.cos(yaw) * p['laser_y']
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
