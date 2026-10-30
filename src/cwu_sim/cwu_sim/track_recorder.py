# 시뮬레이션 경로 기록 (PC 전용): /odom(시뮬레이션에서는 Gazebo 실제 자세, world 원점 = 출발 자세)을 0.5 s마다,
# mission1의 상태 전이 로그(/rosout)를 그때그때 파일에 남깁니다. tools/sim/plot_track.py가 이 파일로 경로 그림을 그림.
#   줄 형식: "t x y yaw" (시뮬레이션 s, m, m, rad) / "E t 내용" / "M t dx dy dyaw" (SLAM map→odom)
#   시뮬레이션 odom은 실제 자세라 map→odom이 곧 SLAM 위치 오차 (복귀 중 벽이 14 cm 가깝게 보인 것 확인용, 2026-10-31)
# Gazebo CLI(gz model -p)로 조회하던 방식은 큐브를 지운 뒤 응답이 끊겨 복귀 구간이 빠졌음 (2026-10-30)
"""Record the simulated robot path and mission state changes to a text file."""
import math

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class TrackRecorder(Node):
    def __init__(self):
        super().__init__('track_recorder')
        self.out = open(self.declare_parameter('out_file', '/tmp/track.txt').value, 'w', buffering=1)
        self.last = -math.inf
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Log, '/rosout', self.on_log, 100)

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t - self.last >= 0.5:
            self.last = t
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            # M 줄을 먼저: run_maze.sh는 E가 아닌 마지막 줄을 끝 자세로 읽음
            if self.tf.can_transform('map', 'odom', Time()):
                m = self.tf.lookup_transform('map', 'odom', Time()).transform
                yaw = 2 * math.atan2(m.rotation.z, m.rotation.w)
                self.out.write(f'M {t:.2f} {m.translation.x:.3f} {m.translation.y:.3f} {yaw:.3f}\n')
            self.out.write(f'{t:.2f} {p.x:.3f} {p.y:.3f} {2 * math.atan2(q.z, q.w):.3f}\n')

    def on_log(self, msg):
        if msg.name == 'mission1' and ('→' in msg.msg or '미션 종료' in msg.msg):
            # /rosout 시각 도장은 시스템 시계라 받은 순간의 시뮬레이션 시각을 씀 (경로 점과 같은 시간축)
            self.out.write(f'E {self.get_clock().now().nanoseconds * 1e-9:.2f} {msg.msg}\n')


def main(args=None):
    rclpy.init(args=args)
    node = TrackRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.out.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
