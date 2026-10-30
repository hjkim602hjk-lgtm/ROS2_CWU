# 시뮬레이션에서 encoder_odom + Nucleo 보드 자리를 대신합니다 (실물 경로에는 없음).
#   /base/command "ARM" → # ARMED, "q" → # STOP, "7"/"8" → 집게 열기/잡기(서보 각도가 천천히 따라감), "PF0/1" 무시
#   /cmd_vel → (무장 중일 때만, 펌웨어 한계로 자름) /sim/cmd_vel. 0.5 s 끊기면 정지
#   /base/telemetry 20 Hz: 12필드, PSD는 Gazebo 거리 → raw ADC 근사
#   잡기(8)가 끝났을 때 정면 PSD 앞에 큐브가 있으면 Gazebo에서 큐브를 지우고 "쥔 것"으로 둠 (정면 ADC 600 고정)
#   odom → base_link TF: Gazebo 실제 자세(/odom)의 움직임에 실물 바퀴 odom 오차(회전 배율 반시계 0.995·시계 0.967,
#   slam.yaml 주석의 실측)와 직진 중 방향 흐름(odom_drift_deg_per_m, 실물 7차: 1 m에 4~5°)을 넣어 적분.
#   완벽한 odom으로는 실물 2차의 방향 오차 누적(→ 복귀 실패)이 안 드러났음 (2026-10-31)
#   /sim/scan → scan_delay_s 늦게 /scan: 실물 G4(한 바퀴 135 ms + 전달 지연)처럼 스캔이 자세보다 늦게 옴. 지연 없는
#   시뮬레이션에서는 돈 직후 스캔을 지금 자세에 붙이는 잘못이 안 드러났음 (2026-10-31 실물 격자 보정 -31°)
"""Simulated motor board: arming, velocity pass-through, gripper servos, PSD telemetry."""
import math
import struct
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from gazebo_msgs.srv import DeleteEntity
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import TransformBroadcaster

OPEN, GRIP = [40, 140], [99, 78]
MAX_V, MAX_W = 0.15, 1.2           # 펌웨어 [V] 한계
SERVO_STEP = 2                     # 도/틱 (20 Hz) ≈ 실물 1도/30 ms
GRAB_RANGE = 0.10                  # m, 잡기 끝났을 때 정면 PSD 거리가 이 안이면 쥠
SIDES = ('front', 'rear', 'left', 'right')


def adc(distance):
    """PSD distance (m) → raw ADC like the real sensor: 550 ≈ 9 cm, open air ≈ 100."""
    if not math.isfinite(distance) or distance >= 0.8:
        return 100
    return int(min(900, 50 / max(distance, 0.01)))


class FakeBoard(Node):
    def __init__(self):
        super().__init__('fake_board')
        self.events = self.create_publisher(String, '/base/event', 10)
        self.telemetry = self.create_publisher(Int32MultiArray, '/base/telemetry', 10)
        self.out = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        self.create_subscription(String, '/base/command', self.on_command, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.range = {s: math.inf for s in SIDES}
        # 안 맞으면 Gazebo가 range = FLT_MAX를 보내는데 rclpy float32 검사가 그 값을 거부해 노드가 죽음 → 원시 바이트로
        # 받아 마지막 필드(range, float32 LE)만 읽음
        for s in SIDES:
            self.create_subscription(Range, f'/sim/psd_{s}', self.store_range(s), qos_profile_sensor_data, raw=True)
        self.delete = self.create_client(DeleteEntity, '/delete_entity')
        self.armed, self.holding = False, False
        self.servo, self.servo_goal = [70, 130], [70, 130]
        self.cmd, self.cmd_at = Twist(), -math.inf
        self.create_timer(0.05, self.tick)
        self.scan_delay = self.declare_parameter('scan_delay_s', 0.2).value
        self.scans = deque()
        self.scan_out = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/sim/scan', lambda m: self.scans.append((self.now(), m)),
                                 qos_profile_sensor_data)
        self.create_timer(0.01, self.relay_scans)
        self.turn_scale = (self.declare_parameter('odom_turn_scale_ccw', 0.995).value,
                           self.declare_parameter('odom_turn_scale_cw', 0.967).value)
        self.drift = math.radians(self.declare_parameter('odom_drift_deg_per_m', 4.0).value)
        self.true_last, self.est = None, [0.0, 0.0, 0.0]
        self.tf_out = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)

    def on_odom(self, msg):
        """Integrate the true motion with wheel-odometry turn errors and publish odom → base_link."""
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        true = (p.x, p.y, 2 * math.atan2(q.z, q.w))
        last, self.true_last = self.true_last, true
        if last is not None:
            dyaw = math.remainder(true[2] - last[2], 2 * math.pi)
            ahead = (true[0] - last[0]) * math.cos(last[2]) + (true[1] - last[1]) * math.sin(last[2])
            dyaw *= self.turn_scale[0] if dyaw > 0 else self.turn_scale[1]
            e = self.est
            e[0] += ahead * math.cos(e[2] + dyaw / 2)
            e[1] += ahead * math.sin(e[2] + dyaw / 2)
            e[2] += dyaw + self.drift * ahead
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = msg.header.stamp, 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.est[0], self.est[1]
        t.transform.rotation.z, t.transform.rotation.w = math.sin(self.est[2] / 2), math.cos(self.est[2] / 2)
        self.tf_out.sendTransform(t)

    def relay_scans(self):
        while self.scans and self.now() - self.scans[0][0] >= self.scan_delay:
            self.scan_out.publish(self.scans.popleft()[1])

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def store_range(self, side):
        def store(raw):
            self.range[side] = struct.unpack('<f', raw[-4:])[0]
        return store

    def event(self, text):
        self.events.publish(String(data=text))

    def busy(self):
        return self.servo != self.servo_goal

    def on_command(self, msg):
        text = msg.data
        if text == 'ARM':
            if self.busy():
                return self.event('# REJECT BUSY')
            self.armed = True
            self.event('# ARMED')
        elif text == 'q':
            self.armed = False
            self.event('# STOP')
        elif text in ('7', '8'):
            self.servo_goal = list(OPEN if text == '7' else GRIP)
            if text == '7':
                self.holding = False

    def on_cmd_vel(self, msg):
        self.cmd, self.cmd_at = msg, self.now()

    def tick(self):
        out = Twist()
        if self.armed and self.now() - self.cmd_at < 0.5:
            out.linear.x = max(-MAX_V, min(MAX_V, self.cmd.linear.x))
            out.angular.z = max(-MAX_W, min(MAX_W, self.cmd.angular.z))
        self.out.publish(out)
        was_busy = self.busy()
        self.servo = [s + max(-SERVO_STEP, min(SERVO_STEP, g - s)) for s, g in zip(self.servo, self.servo_goal)]
        if was_busy and not self.busy() and self.servo_goal == GRIP and self.range['front'] < GRAB_RANGE:
            self.holding = True
            self.delete.call_async(DeleteEntity.Request(name='target'))
            self.get_logger().info(f'큐브 쥠 (정면 {self.range["front"]:.3f} m)')
        psd = [adc(self.range[s]) for s in SIDES]
        if self.holding:
            psd[0] = 600
        ms = int(self.now() * 1000) & 0x7fffffff
        self.telemetry.publish(Int32MultiArray(data=[ms, int(self.armed), 0, 0, 0, 0] + psd + self.servo))


def main(args=None):
    rclpy.init(args=args)
    node = FakeBoard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
