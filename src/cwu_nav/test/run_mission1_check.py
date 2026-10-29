# 로봇 없이 mission1 상태머신의 전체 흐름을 검증하는 스크립트입니다 (2026-10-29 펌웨어 규약).
# 가짜 보드: /base/command의 [F…]/[TL…]/[TR…]를 받으면 MOVE_S 뒤 map → base_link TF를 그만큼 옮기고 # DONE을,
# 그 전에 q가 오면 움직이지 않고 # STOP을 돌려줍니다 (보드는 PSD로 멈추지 않음). 7/8은 서보 각도만 바꿉니다.
# 흐름: 집게 열기 → 첫 둘러보기(반시계 90°) → 힌트 칸으로 격자 이동(정면 PSD → mission이 q → 그 칸 막힘 → 돌아감)
# → 목표물 보임 → 접근(오른쪽 오차 → TR, 중앙 → F100) → 정면 PSD → q = 파지(8) → 출발 칸 복귀 → 종료.
# 순수 판단은 test_grid.py·test_mission_logic.py가 보고, 여기서는 상태 전이와 토픽 연결을 봅니다.

"""Drive the mission1 node through a full fake mission on discrete board moves."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import TransformBroadcaster

MOVE_S = 0.4  # 가짜 이동 하나가 걸리는 시간 (그 사이에 mission이 q로 세울 수 있게)


class FakeBoard(Node):
    def __init__(self):
        super().__init__('mission1_check')
        self.x = self.y = self.yaw = 0.0
        self.psd = [0, 0, 0, 0]          # 앞, 뒤, 좌, 우 raw
        self.servo = [99, 78]            # 지난 실행이 닫힌 채 끝남 (실물 2회차)
        self.commands, self.pending, self.first_move_servo, self.bearing_x = [], None, None, None
        self.tf = TransformBroadcaster(self)
        self.telemetry = self.create_publisher(Int32MultiArray, '/base/telemetry', 10)
        self.event = self.create_publisher(String, '/base/event', 10)
        self.bearing = self.create_publisher(PointStamped, '/target/bearing', 10)
        self.scan = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(String, '/base/command', self.on_command, 10)
        self.create_timer(0.05, self.publish)

    def publish(self):
        if self.pending is not None and time.monotonic() >= self.pending[1]:
            self.finish(self.pending[0])
            self.pending = None
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
        elif text == 'q':
            self.pending = None
            self.event.publish(String(data='# STOP'))
        elif text[0] in 'FT':
            if self.first_move_servo is None:
                self.first_move_servo = list(self.servo)
            self.pending = (text, time.monotonic() + MOVE_S)

    def finish(self, text):
        if text[0] == 'F':
            d = int(text[1:]) / 1000
            self.x, self.y = self.x + d * math.cos(self.yaw), self.y + d * math.sin(self.yaw)
            self.event.publish(String(data='# DONE DISTANCE'))
        else:
            self.yaw += math.radians(int(text[2:])) * (1 if text[1] == 'L' else -1)
            self.event.publish(String(data='# DONE TURN_LEFT' if text[1] == 'L' else '# DONE TURN_RIGHT'))

    def moves(self):
        return [c for c in self.commands if c[0] in 'FT']

    def wait(self, what, condition, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if condition():
                print('PASS: ' + what, flush=True)
                return
        raise AssertionError('시간 초과: ' + what + ' (명령 %s)' % self.commands[-12:])


def verify():
    rclpy.init()
    board = FakeBoard()
    try:
        board.wait('RELEASE: 집게 열기', lambda: board.commands[:1] == ['7'], 15)
        board.wait('LOOK: 첫 둘러보기 = 반시계 45° 두 번', lambda: board.moves()[:2] == ['TL45', 'TL45'], 10)
        assert board.first_move_servo == [40, 140], '집게가 다 열리기 전에 움직였습니다: %s' % board.first_move_servo
        print('PASS: 첫 이동은 집게가 다 열린 뒤', flush=True)
        # 힌트 칸 (4,4) 쪽으로: 지금 +y를 봄 → 첫 직진은 (0,1) 칸
        board.wait('GOTO: 힌트 쪽 첫 직진 F400', lambda: board.pending and board.pending[0] == 'F400'
                   and board.moves()[2:].count('F400') == 1, 10)
        n = len(board.moves())
        board.psd[0] = 600  # 낮은 장애물 앞 (탐색 중이라 파지 아님): 보드는 안 멈춤 → mission이 q
        board.wait('직진 중 정면 PSD → mission이 q로 정지', lambda: board.commands[-1] == 'q', 5)
        board.psd[0] = 0
        board.wait('막힌 칸을 피해 회전부터 (다른 칸으로 돌아감)',
                   lambda: len(board.moves()) > n and board.moves()[n][0] == 'T', 10)
        board.wait('힌트 옆 칸 도착 → 둘러보기', lambda: abs(board.x - 1.6) + abs(board.y - 1.6) <= 0.41
                   and board.moves()[-1].startswith('T'), 40)

        board.bearing_x = math.tan(math.radians(17))  # 오른쪽 17°
        board.wait('APPROACH: 오른쪽 오차 → 시계 회전 TR17', lambda: 'TR17' in board.moves(), 15)
        board.bearing_x = 0.0
        board.wait('APPROACH: 중앙 → F100 직진', lambda: board.pending and board.pending[0] == 'F100', 10)
        board.psd[0] = 600  # 큐브가 정면 PSD 임계에 들어옴 → mission이 q
        board.wait('GRASP: q로 정지 → 잡기', lambda: board.commands[-2:] == ['q', '8'], 10)
        board.bearing_x, board.psd[0] = None, 615  # 쥔 큐브가 정면 PSD 앞
        board.wait('RETURN: 출발 칸 (0,0) 도착', lambda: math.hypot(board.x, board.y) < 0.1, 60)
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
        # 가짜 보드는 바로 답하므로 자세 대기를 줄여 검사를 빠르게
        node = subprocess.Popen(
            ['ros2', 'run', 'cwu_nav', 'mission1', '--ros-args', '--params-file', str(params),
             '-p', 'settle_s:=0.2'],
            env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            verify()
            assert node.poll() is None, 'mission1 exited unexpectedly'
            assert '미션 종료' in logfile.read_text(), 'DONE 상태에 들어가지 않았습니다'
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
