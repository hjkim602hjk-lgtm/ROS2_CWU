# 로봇 없이 mission1 상태머신의 전체 흐름을 검증하는 스크립트입니다.
# 가짜 보드(텔레메트리·이벤트·arm/carry 서비스), 가짜 Nav2(바로 성공하는 액션 서버),
# 고정 TF를 붙이고 탐색 → 측면 PSD 후퇴 → 접근 → 정면 PSD 파지 → 출발점 복귀 → 종료를 확인합니다.
# 순수 판단은 test_mission_logic.py가 보고, 여기서는 상태 전이와 토픽·서비스·액션 연결을 봅니다.

"""Drive the mission1 node through a full fake mission and check each hand-off."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import StaticTransformBroadcaster


class FakeRobot(Node):
    def __init__(self):
        super().__init__('mission1_check')
        self.psd = [0, 0, 0, 0]          # 앞, 뒤, 좌, 우 raw
        self.servo = [105, 75]           # 지난 실행이 닫힌 채 끝남 (실물 2회차)
        self.carries, self.goals, self.cmd = [], [], None
        self.open_at = self.first_cmd_servo = None
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'base_link'
        transform.transform.rotation.w = 1.0
        self.tf = StaticTransformBroadcaster(self)
        self.tf.sendTransform(transform)
        self.telemetry = self.create_publisher(Int32MultiArray, '/base/telemetry', 10)
        self.event = self.create_publisher(String, '/base/event', 10)
        self.bearing = self.create_publisher(PointStamped, '/target/bearing', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_service(Trigger, '/encoder_odom/arm', self.on_arm)
        self.create_service(SetBool, '/encoder_odom/carry', self.on_carry)
        self.server = ActionServer(self, NavigateToPose, 'navigate_to_pose', self.on_goal)
        self.create_timer(0.05, self.send_telemetry)

    def send_telemetry(self):
        if self.open_at is not None and time.monotonic() >= self.open_at:
            self.servo, self.open_at = [40, 140], None  # 서보는 1도/30 ms라 열리는 데 시간이 걸림
        self.telemetry.publish(Int32MultiArray(data=[0, 0, 0, 0, 0, 0] + self.psd + self.servo))

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        if self.first_cmd_servo is None:
            self.first_cmd_servo = list(self.servo)

    def on_arm(self, request, response):
        response.success = True
        return response

    def on_carry(self, request, response):
        self.carries.append(request.data)
        if request.data:
            self.servo = [99, 78]
        else:
            self.open_at = time.monotonic() + 1.0
        response.success = True
        return response

    def on_goal(self, handle):
        p = handle.request.pose.pose.position
        self.goals.append((round(p.x, 3), round(p.y, 3)))
        if len(self.goals) == 1:
            handle.abort()  # 첫 목표: 장애물 모서리에 붙어 RPP가 거부 (모의 경기장 1·4회차)
        else:
            handle.succeed()
        return NavigateToPose.Result()

    def wait(self, what, condition, seconds, see=False):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if see:
                msg = PointStamped()
                msg.point.z = 1.0  # 화면 정중앙
                self.bearing.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            if condition():
                print('PASS: ' + what, flush=True)
                return
        raise AssertionError('시간 초과: ' + what)


def verify():
    rclpy.init()
    robot = FakeRobot()
    try:
        robot.wait('INIT: 집게 열기·정면 PSD 켜기 요청', lambda: robot.carries == [False], 15)
        robot.wait('SPIN: 제자리 회전 명령', lambda: robot.cmd and robot.cmd[1] != 0, 5)
        assert robot.first_cmd_servo == [40, 140], '집게가 다 열리기 전에 움직였습니다: %s' % robot.first_cmd_servo
        print('PASS: 첫 주행 명령은 집게가 다 열린 뒤', flush=True)
        robot.wait('GOTO: 첫 목표 = 중앙 힌트 앞 (1.176, 1.176)',
                   lambda: robot.goals[:1] == [(1.176, 1.176)], 15)
        robot.wait('Nav2 실패 → 다음 목표', lambda: len(robot.goals) >= 2, 20)
        robot.wait('도착 뒤 다시 SPIN', lambda: robot.cmd and robot.cmd[1] != 0, 5)

        robot.psd[3] = 600  # 오른쪽 벽: 보드가 회전 중 멈춤
        robot.event.publish(String(data='# ABORT PSD_BLOCKED'))
        robot.wait('BACKOFF: 측면 막힘 → 후진', lambda: robot.cmd and robot.cmd[0] < 0, 5)
        robot.psd[3] = 0
        robot.wait('후퇴 뒤 SPIN 재개', lambda: robot.cmd and robot.cmd[1] != 0, 5)

        robot.wait('APPROACH: 중앙에 보이면 직진',
                   lambda: robot.cmd and robot.cmd[0] > 0 and robot.cmd[1] == 0, 5, see=True)
        robot.psd[0] = 300  # 큐브가 집게 앞: 화면 아래로 잘려 검출이 끊겨도
        end, turned = time.monotonic() + 1.5, False
        while time.monotonic() < end:
            rclpy.spin_once(robot, timeout_sec=0.02)
            turned |= bool(robot.cmd and robot.cmd[1] != 0)
        assert robot.cmd and robot.cmd[0] > 0 and not turned, 'PSD 직진 단계에서 회전했습니다: %s' % (robot.cmd,)
        print('PASS: 정면 PSD 근접 → 카메라 없이 직진', flush=True)
        robot.psd[0] = 600  # 큐브가 정면 PSD 임계에 들어옴 → 보드 정지
        end = time.monotonic() + 0.2
        while time.monotonic() < end:
            rclpy.spin_once(robot, timeout_sec=0.02)
        robot.psd[0] = 540  # 잡음: 정지 직후 줄은 임계 아래일 수 있음 (실물 1회차 32 s)
        robot.event.publish(String(data='# ABORT PSD_BLOCKED'))
        robot.wait('GRASP: 잡기 + 운반 모드 요청', lambda: robot.carries[-1:] == [True], 5)
        robot.wait('RETURN: 출발점 (0, 0)으로 복귀 목표', lambda: robot.goals[-1:] == [(0.0, 0.0)], 10)
        end = time.monotonic() + 2.0  # 복귀 결과가 mission1에 전달될 때까지 계속 돌림
        while time.monotonic() < end:
            rclpy.spin_once(robot, timeout_sec=0.05)
    finally:
        robot.destroy_node()
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
        # 고정 TF라 실제로 돌지 않으므로, 회전 시간 제한(2·2π/spin_speed + 5 s)을 줄이려고 빠르게
        node = subprocess.Popen(
            ['ros2', 'run', 'cwu_nav', 'mission1', '--ros-args', '--params-file', str(params),
             '-p', 'spin_speed:=20.0'],
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
