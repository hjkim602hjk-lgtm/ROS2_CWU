# 로봇 없이 target_follower 노드의 ROS 연결을 검증하는 스크립트입니다.
# 합성 /target/bearing을 넣고 /cmd_vel_servo가 기대한 부호로 나오는지,
# 그리고 표본이 끊기면 0으로 돌아오는지를 봅니다.
# 순수 계산은 test_servo.py가 보고, 여기서는 토픽·파라미터·주기 발행과
# "안 보이면 멈춘다"는 타이머 경로를 봅니다.

"""Feed the follower node synthetic bearings and verify the published twist."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node

TIMEOUT_S = 0.5      # servo.yaml의 sample_timeout_s와 같아야 합니다.


class Feeder(Node):
    def __init__(self):
        super().__init__('follower_check')
        self.last = None
        self.bearing = self.create_publisher(PointStamped, '/target/bearing', 10)
        self.create_subscription(Twist, '/cmd_vel_servo', self.on_cmd, 10)

    def on_cmd(self, msg):
        self.last = msg

    def send(self, x, y=0.0, z=1.0):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.point.x, msg.point.y, msg.point.z = x, y, z
        self.bearing.publish(msg)

    def drive(self, x, seconds=1.5):
        """Publish one bearing repeatedly and return the newest command."""
        self.last = None
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.send(x)
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.02)
        assert self.last is not None, '/cmd_vel_servo가 나오지 않았습니다'
        return self.last

    def coast(self, seconds=2.0):
        """Stop publishing bearings and return the newest command."""
        self.last = None
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        assert self.last is not None, '표본이 끊긴 뒤 발행이 멈췄습니다'
        return self.last


def verify():
    rclpy.init()
    node = Feeder()
    try:
        # 오른쪽 목표물(+x) → 시계방향(음의 각속도), 정렬 전이므로 전진 없음.
        right = node.drive(0.5)
        assert right.angular.z < 0, '오른쪽 목표물에 좌회전했습니다: %.3f' % right.angular.z
        assert right.linear.x == 0.0, '정렬 전에 전진했습니다: %.3f' % right.linear.x
        print('PASS: 우측 목표물 → angular.z=%.3f, linear.x=%.3f'
              % (right.angular.z, right.linear.x), flush=True)

        left = node.drive(-0.5)
        assert left.angular.z > 0, '왼쪽 목표물에 우회전했습니다: %.3f' % left.angular.z

        centred = node.drive(0.0)
        assert centred.linear.x > 0, '정렬됐는데 전진하지 않습니다: %.3f' % centred.linear.x
        assert abs(centred.angular.z) < 1e-6, '중심선에서 회전합니다: %.3f' % centred.angular.z
        print('PASS: 중심선 정렬 → angular.z=%.3f, linear.x=%.3f'
              % (centred.angular.z, centred.linear.x), flush=True)

        # 목표물 상실: 마지막 명령이 남아 계속 돌면 안 됩니다.
        stopped = node.coast()
        assert stopped.linear.x == 0.0 and stopped.angular.z == 0.0, \
            '표본이 끊겼는데 멈추지 않습니다: (%.3f, %.3f)' \
            % (stopped.linear.x, stopped.angular.z)
        print('PASS: 표본 만료(%.1fs) → 정지 명령' % TIMEOUT_S, flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    params = Path(get_package_share_directory('cwu_nav')) / 'config' / 'servo.yaml'
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1')
    env.setdefault('ROS_DOMAIN_ID', '71')
    # 검사 프로세스도 같은 도메인·전송 설정으로 붙어야 합니다.
    # 노드 쪽에만 걸면 서로 안 보여서 '발행이 없다'로 잘못 나옵니다.
    os.environ.update(env)
    directory = Path(tempfile.mkdtemp(prefix='cwu-follower-check-'))
    env['ROS_LOG_DIR'] = str(directory / 'ros-log')
    logfile = directory / 'follower.log'
    print('Check artifacts: %s' % directory, flush=True)
    with logfile.open('w') as output:
        node = subprocess.Popen(
            ['ros2', 'run', 'cwu_nav', 'target_follower',
             '--ros-args', '--params-file', str(params)],
            env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            verify()
            assert node.poll() is None, 'target_follower exited unexpectedly'
        finally:
            if node.poll() is None:
                node.send_signal(signal.SIGINT)
                try:
                    node.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(node.pid, signal.SIGKILL)
                    node.wait()
            print(logfile.read_text()[-4000:], flush=True)


if __name__ == '__main__':
    main()
