# Nav2 주행 경로 전체를 하드웨어 없이 자동 검증하는 스크립트입니다.
# 가상 센서를 cmd_vel 모드로 띄워 SLAM → Nav2 → /cmd_vel → 실제 이동까지 한 바퀴를 확인합니다.
# 목표점 도달 후 시작점 복귀까지 시켜 미션1의 왕복 구조를 같은 방식으로 검사합니다.
# 마지막으로 상자 안을 목표로 주어 코스트맵이 장애물을 실제로 막는지도 확인합니다.
# 기존 원운동 데모는 /cmd_vel을 무시하므로 이 검사를 대신하지 못합니다.

"""Drive a Nav2 goal and a return trip on synthetic sensors; no hardware needed."""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

# 데모 방은 3.6 m이고 내부 상자가 x,y 0.9~1.3에 있습니다.
GOAL = (1.0, -0.8)        # 상자 밖의 빈 공간
HOME = (0.0, 0.0)
BLOCKED = (1.1, 1.1)      # 상자 한가운데. 코스트맵이 제대로 동작하면 갈 수 없습니다.
GOAL_TOLERANCE_M = 0.25


def pose(x, y):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    goal.pose.orientation.w = 1.0
    return goal


class Checker(Node):
    def __init__(self):
        super().__init__('nav2_check')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def spin_until(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            result = predicate()
            if result:
                return result
        raise AssertionError('시간 초과: %s (%.0f초)' % (what, timeout))

    def robot_xy(self):
        try:
            tf = self.buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), Duration(seconds=0.5))
        except Exception:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def send(self, x, y, timeout):
        """Send one goal and return its terminal status."""
        goal = self.client.send_goal_async(NavigateToPose.Goal(pose=pose(x, y)))
        handle = self.spin_until(
            lambda: goal.done() and goal.result(), 30.0, 'Nav2가 목표를 받지 않음')
        if not handle.accepted:
            return GoalStatus.STATUS_ABORTED
        outcome = handle.get_result_async()
        return self.spin_until(
            lambda: outcome.done() and outcome.result(), timeout,
            '(%.1f, %.1f) 주행이 끝나지 않음' % (x, y)).status

    def expect_blocked(self, x, y, timeout):
        """A goal inside the box must fail. If it succeeds the obstacle layer is empty,
        which would mean Nav2 is driving through walls it cannot see."""
        status = self.send(x, y, timeout)
        assert status != GoalStatus.STATUS_SUCCEEDED, \
            '상자 안 (%.2f, %.2f)에 도달했습니다. 코스트맵에 장애물이 안 찍힙니다' % (x, y)
        print('PASS: 상자 안 (%.2f, %.2f) 주행 거부됨 status=%d' % (x, y, status), flush=True)

    def drive_to(self, x, y, timeout):
        status = self.send(x, y, timeout)
        assert status == GoalStatus.STATUS_SUCCEEDED, 'Nav2 주행 실패 status=%d' % status
        at = self.robot_xy()
        assert at is not None, 'map → base_link TF를 읽지 못했습니다'
        error = math.hypot(at[0] - x, at[1] - y)
        assert error <= GOAL_TOLERANCE_M, \
            '목표 (%.2f, %.2f), 도착 (%.2f, %.2f), 오차 %.2f m' % (x, y, at[0], at[1], error)
        print('PASS: (%.2f, %.2f) 도달, 오차 %.2f m' % (at[0], at[1], error), flush=True)


def verify(timeout):
    rclpy.init()
    node = Checker()
    try:
        node.spin_until(lambda: node.robot_xy() is not None, 90.0,
                        'SLAM이 map → base_link TF를 내지 않음')
        node.spin_until(lambda: node.client.wait_for_server(timeout_sec=0.5), 90.0,
                        'navigate_to_pose 액션 서버가 뜨지 않음')
        node.drive_to(GOAL[0], GOAL[1], timeout)
        # 미션1의 복귀 구간과 같은 동작입니다. 좌표는 부팅 시점 원점입니다.
        node.drive_to(HOME[0], HOME[1], timeout)
        # 여기까지는 빈 공간만 지났습니다. 코스트맵이 장애물을 실제로 표시하는지 확인합니다.
        node.expect_blocked(BLOCKED[0], BLOCKED[1], timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='가상 입력으로 Nav2 주행·복귀 검사')
    parser.add_argument('--goal-timeout', type=float, default=120.0)
    args = parser.parse_args()
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1')
    env.setdefault('ROS_DOMAIN_ID', '69')
    directory = Path(tempfile.mkdtemp(prefix='cwu-nav2-check-'))
    env['ROS_LOG_DIR'] = str(directory / 'ros-log')
    logfile = directory / 'launch.log'
    print('Check artifacts: %s' % directory, flush=True)
    with logfile.open('w') as output:
        launch = subprocess.Popen(
            ['ros2', 'launch', 'cwu_nav', 'autonomy.launch.py', 'demo:=true',
             'demo_drive:=cmd_vel', 'camera:=false'],
            env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            verify(args.goal_timeout)
            assert launch.poll() is None, 'Launch process exited unexpectedly'
            print('PASS: SLAM → Nav2 → /cmd_vel → 이동 경로가 연결됩니다', flush=True)
        finally:
            if launch.poll() is None:
                # Signal the launcher only; ROS launch forwards SIGINT to its children.
                launch.send_signal(signal.SIGINT)
                try:
                    launch.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)
                    launch.wait()
            print(logfile.read_text()[-6000:], flush=True)


if __name__ == '__main__':
    main()
