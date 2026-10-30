# 로봇 없이 mission1 상태머신의 전체 흐름을 검증하는 스크립트입니다 (2026-10-30 펌웨어 속도 모드).
# 가짜 보드: /base/command "ARM"에 # ARMED, "q"에 # STOP으로 답하고, 무장돼 있으면 /cmd_vel을 적분해 map → base_link
# TF를 옮깁니다. 7/8은 서보 각도만 바꿉니다.
# 흐름: 집게 열기 → 제자리 회전·힌트 없이 앞이 뚫렸으니 직진(직진 중 정면 PSD → 멈추고 돌아감)
# → 도착해도 돌지 않고 앞이 뚫렸으니 직진(CRUISE) → 목표물 보임 → 접근(오른쪽 오차 → 오른쪽으로 틀며 직진)
# → 화면에서 빠져도 PSD 가까우면 그대로 직진 → 정면 PSD → 파지(PF0 → 8) → 출발 칸 복귀 → 종료.
# 순수 판단은 test_grid.py·test_mission_logic.py가 보고, 여기서는 상태 전이와 토픽 연결을 봅니다.

"""Drive the mission1 node through a full fake mission on continuous velocity commands."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, TransformStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import TransformBroadcaster


class FakeBoard(Node):
    def __init__(self):
        super().__init__('mission1_check')
        self.x = self.y = self.yaw = 0.0
        self.psd = [0, 0, 0, 0]          # 앞, 뒤, 좌, 우 raw
        self.servo = [99, 78]            # 지난 실행이 닫힌 채 끝남 (실물 2회차)
        self.armed, self.cmd, self.last = False, (0.0, 0.0), time.monotonic()
        self.commands, self.first_move_servo, self.bearing_x = [], None, None
        self.tf = TransformBroadcaster(self)
        self.telemetry = self.create_publisher(Int32MultiArray, '/base/telemetry', 10)
        self.event = self.create_publisher(String, '/base/event', 10)
        self.bearing = self.create_publisher(PointStamped, '/target/bearing', 10)
        self.scan = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(String, '/base/command', self.on_command, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_timer(0.05, self.publish)

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        if self.first_move_servo is None and (abs(self.cmd[0]) > 1e-3 or abs(self.cmd[1]) > 1e-3):
            self.first_move_servo = list(self.servo)

    def publish(self):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        if self.armed:
            v, w = self.cmd
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
            self.yaw += w * dt
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = 'map', 'base_link'
        transform.transform.translation.x, transform.transform.translation.y = self.x, self.y
        transform.transform.rotation.z = math.sin(self.yaw / 2)
        transform.transform.rotation.w = math.cos(self.yaw / 2)
        self.tf.sendTransform(transform)
        self.telemetry.publish(Int32MultiArray(data=[0, 0, 0, 0, 0, 0] + self.psd + self.servo))
        scan = LaserScan()
        scan.header.frame_id = 'base_link'
        scan.angle_increment, scan.range_min, scan.range_max = math.radians(1.0), 0.12, 12.0
        scan.ranges = [0.0] * 360  # 사방이 트임 (무효 = 먼 곳)
        self.scan.publish(scan)
        if self.bearing_x is not None:
            msg = PointStamped()
            msg.point.x, msg.point.z = self.bearing_x, 1.0
            self.bearing.publish(msg)

    def on_command(self, msg):
        text = msg.data
        self.commands.append(text)
        if text == '7':
            self.servo = [40, 140]
        elif text == '8':
            self.servo = [99, 78]
        elif text == 'ARM':
            self.armed = True
            self.event.publish(String(data='# ARMED'))
        elif text == 'q':
            self.armed = False
            self.event.publish(String(data='# STOP'))

    def moving(self):
        return self.armed and abs(self.cmd[0]) > 1e-3

    def wait(self, what, condition, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if condition():
                print('PASS: ' + what, flush=True)
                return
        raise AssertionError('시간 초과: %s (x %.2f y %.2f yaw %.0f° cmd %s, 명령 %s)' % (
            what, self.x, self.y, math.degrees(self.yaw), self.cmd, self.commands[-8:]))


def verify():
    rclpy.init()
    board = FakeBoard()
    try:
        board.wait('RELEASE: 운반 모드 해제 → 집게 열기', lambda: board.commands[:2] == ['PF1', '7'], 15)
        board.wait('둘러보기 회전 없이 바로 출발', lambda: board.armed and board.cmd != (0.0, 0.0), 15)
        assert board.first_move_servo == [40, 140], '집게가 다 열리기 전에 움직였습니다: %s' % board.first_move_servo
        print('PASS: 첫 이동은 집게가 다 열린 뒤', flush=True)
        board.wait('앞이 뚫렸으니 직진', board.moving, 15)
        board.psd[0] = 600  # 낮은 장애물 앞 (탐색 중이라 파지 아님)
        board.wait('직진 중 정면 PSD → 멈춤', lambda: board.cmd == (0.0, 0.0), 5)
        board.psd[0] = 0
        x, y = board.x, board.y
        board.wait('막힌 칸을 피해 다시 움직임', lambda: math.hypot(board.x - x, board.y - y) > 0.2, 30)
        board.wait('도착 뒤 둘러보기(LOOK)', lambda: board.cmd == (0.0, 0.0), 30)
        x, y, yaw = board.x, board.y, board.yaw
        board.wait('제자리 회전 없이 앞이 뚫렸으니 직진 (CRUISE)',
                   lambda: math.hypot(board.x - x, board.y - y) > 0.2, 30)
        assert abs(math.remainder(board.yaw - yaw, 2 * math.pi)) < math.radians(30), '도착 뒤 제자리에서 돌았습니다'

        board.bearing_x = math.tan(math.radians(17))  # 오른쪽 17°
        board.wait('APPROACH: 오른쪽 오차 → 오른쪽으로 틀며 접근', lambda: board.cmd[1] < -0.1, 15)
        board.bearing_x = 0.0
        board.wait('APPROACH: 중앙 → 곧장 직진', lambda: board.moving() and abs(board.cmd[1]) < 0.1, 10)
        board.bearing_x, board.psd[0] = None, 300  # 화면 아래로 빠짐 + 정면 PSD 가까움
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(board, timeout_sec=0.02)
        assert board.moving() and abs(board.cmd[1]) < 0.05, '화면에서 빠진 뒤 직진하지 않음: %s' % (board.cmd,)
        print('PASS: 화면에서 빠져도 PSD 가까우면 그대로 직진 (회전 없음)', flush=True)
        board.psd[0] = 600  # 큐브가 정면 PSD 임계에 들어옴
        board.wait('GRASP: 멈춤 → 운반 모드 → 잡기', lambda: board.commands[-2:] == ['PF0', '8'], 10)
        board.psd[0] = 615  # 쥔 큐브가 정면 PSD 앞
        board.wait('RETURN: 출발 칸 (0,0) 도착', lambda: math.hypot(board.x, board.y) < 0.1, 90)
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            rclpy.spin_once(board, timeout_sec=0.05)
    finally:
        board.destroy_node()
        rclpy.shutdown()


def main():
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1')
    env.setdefault('ROS_DOMAIN_ID', '73')
    os.environ.update(env)  # 검사 쪽도 같은 도메인에 붙어야 서로 보입니다
    directory = Path(tempfile.mkdtemp(prefix='cwu-mission1-check-'))
    env['ROS_LOG_DIR'] = str(directory / 'ros-log')
    logfile = directory / 'mission1.log'
    params = Path(get_package_share_directory('cwu_nav')) / 'config' / 'mission1.yaml'
    print('Check artifacts: %s' % directory, flush=True)
    with logfile.open('w') as output:
        node = subprocess.Popen(
            ['ros2', 'run', 'cwu_nav', 'mission1', '--ros-args', '--params-file', str(params)],
            env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            verify()
            assert node.poll() is None, 'mission1 exited unexpectedly'
            log = logfile.read_text()
            assert 'LOOK → CRUISE' in log, 'CRUISE 상태에 들어가지 않았습니다'
            assert '미션 종료' in log, 'DONE 상태에 들어가지 않았습니다'
            print('PASS: DONE (목표물 보유)', flush=True)
        finally:
            if node.poll() is None:
                node.send_signal(signal.SIGINT)
                try:
                    node.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(node.pid, signal.SIGKILL)
                    node.wait()
            print(logfile.read_text()[-3000:], flush=True)


if __name__ == '__main__':
    main()
