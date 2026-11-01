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
# 상대 로봇·실물 흉내 시험 (환경 변수, 좌표는 출발 칸 기준 미션 칸 (i 앞, j 왼쪽) 또는 m):
#   SIM_STEAL_AFTER=10      쥔 지 10 s 뒤 빼앗김: 쥔 것 풀림, 큐브를 (5,5) 칸 중심에 다시 놓음
#   SIM_GATE=1,0,60[,20]    쥐고 20 s(기본 0) 뒤 칸 (1,0)에 상자를 놓고 60 s 뒤 치움 (상대 로봇이 길을 잠깐 막음)
#   SIM_BOX_AT=3,0,40,600   시뮬 40 s에 칸 (3,0)에 상자를 놓고 600 s 뒤 치움 (이미 빈 칸으로 본 길에 나중에 놓인 장애물)
#   SIM_BLOCKER=60,3        시뮬 60 s에 로봇 바로 앞 0.45 m에 상자(상대 로봇 흉내)를 놓고 3 s 뒤 치움
#   SIM_LOWBOX=0.8,0.12,0.12,0.12,0.08  처음부터 낮은 상자 (x, y, 크기 m): 라이다 아래라 몸이 걸림 (끼임)
#   SIM_ODOM_CRAB=3         바퀴 odom이 실제보다 3° 오른쪽으로 간다고 셈 = 실물처럼 몸 방향보다 왼쪽으로 비스듬히 감
"""Simulated motor board: arming, velocity pass-through, gripper servos, PSD telemetry."""
import math
import os
import struct
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import TransformBroadcaster

from cwu_sim.maze import box

OPEN, GRIP = [40, 140], [99, 78]
MAX_V, MAX_W = 0.35, 6.0           # 실물 PWM 255에서 0.35 m/s (2026-10-31 1 m 시험) — 펌웨어 [V] 상한은 없앰
ACCEL, TRACK = 0.1, 0.2            # 펌웨어 바퀴 가속 제한 ACCEL_MM_S2 100 (실물은 0.25 m/s까지 약 3 s), 윤거
SERVO_STEP = 2                     # 도/틱 (20 Hz) ≈ 실물 1도/30 ms
GRAB_RANGE = 0.10                  # m, 잡기 끝났을 때 정면 PSD 거리가 이 안이면 쥠
CUBE = (1.6, 1.6)                  # 큐브 (5,5) 칸 중심, 출발 자세 기준 (maze.world: 출발이 원점, 경기장이 왼쪽)
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
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
        env = os.environ.get
        self.steal_after = float(env('SIM_STEAL_AFTER') or 0)
        self.gate = [float(v) for v in env('SIM_GATE').split(',')] if env('SIM_GATE') else None
        self.lowbox = [float(v) for v in env('SIM_LOWBOX').split(',')] if env('SIM_LOWBOX') else None
        self.crab = math.radians(float(env('SIM_ODOM_CRAB') or 0))
        self.target, self.held_at, self.gate_off = 'target', None, None
        self.box_at = [float(v) for v in env('SIM_BOX_AT').split(',')] if env('SIM_BOX_AT') else None
        self.blocker = [float(v) for v in env('SIM_BLOCKER').split(',')] if env('SIM_BLOCKER') else None
        self.armed, self.holding = False, False
        self.servo, self.servo_goal = [70, 130], [70, 130]
        self.cmd, self.cmd_at = Twist(), -math.inf
        self.wheels = [0.0, 0.0]  # 지금 바퀴 속도 (가속 제한)
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
            e[0] += ahead * math.cos(e[2] + dyaw / 2 - self.crab)
            e[1] += ahead * math.sin(e[2] + dyaw / 2 - self.crab)
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

    def put(self, name, x, y, sx, sy, sz, rgb, static=True):
        req = SpawnEntity.Request(name=name, xml=f'<sdf version="1.6">{box(name, 0, 0, sx, sy, sz, rgb, static)}</sdf>')
        req.initial_pose.position.x, req.initial_pose.position.y, req.initial_pose.position.z = x, y, sz / 2
        self.spawn.call_async(req)
        self.get_logger().info(f'시험: {name} 놓음 ({x:.2f}, {y:.2f})')

    def scenario(self, now):
        """SIM_LOWBOX / SIM_STEAL_AFTER / SIM_GATE test events (see header)."""
        if self.lowbox and now > 2.0 and self.spawn.service_is_ready():
            x, y, sx, sy, sz = self.lowbox
            self.put('lowbox', x, y, sx, sy, sz, (0.3, 0.3, 0.3))
            self.lowbox = None
        if self.blocker and now >= self.blocker[0] and self.true_last and self.spawn.service_is_ready():
            x, y, yaw = self.true_last
            self.put('gate', x + 0.45 * math.cos(yaw), y + 0.45 * math.sin(yaw), 0.25, 0.25, 0.25, (0.2, 0.2, 0.8))
            self.gate_off, self.blocker = now + self.blocker[1], None
        if self.box_at and now >= self.box_at[2] and self.spawn.service_is_ready():
            i, j, _, hold = self.box_at
            self.put('gate', 0.4 * i, 0.4 * j, 0.38, 0.38, 0.2, (0.5, 0.5, 0.5))
            self.gate_off, self.box_at = now + hold, None
        if self.holding and self.held_at is None:
            self.held_at = now
        if self.gate and self.held_at is not None and now - self.held_at >= (self.gate[3] if len(self.gate) > 3 else 0):
            i, j, hold = self.gate[:3]
            self.put('gate', 0.4 * i, 0.4 * j, 0.38, 0.38, 0.2, (0.5, 0.5, 0.5))
            self.gate_off, self.gate = now + hold, None
        if self.gate_off is not None and now > self.gate_off:
            self.delete.call_async(DeleteEntity.Request(name='gate'))
            self.get_logger().info('시험: gate 치움')
            self.gate_off = None
        if self.holding and self.steal_after > 0 and now - self.held_at > self.steal_after:
            self.holding, self.steal_after, self.target = False, 0.0, 'target2'
            self.put('target2', *CUBE, 0.05, 0.05, 0.05, (1, 0, 0), static=False)
            self.get_logger().info('시험: 큐브를 빼앗아 (5,5)에 다시 놓음')

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
        v = w = 0.0
        if self.armed and self.now() - self.cmd_at < 0.5:
            v = max(-MAX_V, min(MAX_V, self.cmd.linear.x))
            w = max(-MAX_W, min(MAX_W, self.cmd.angular.z))
        # 바퀴마다 가속 제한 (펌웨어와 같게): 시뮬레이션이 즉시 속도를 내면 빠른 직진의 이득이 부풀려짐
        step = ACCEL * 0.05
        target = (v - w * TRACK / 2, v + w * TRACK / 2)
        self.wheels = [c + max(-step, min(step, t - c)) for c, t in zip(self.wheels, target)]
        out.linear.x = (self.wheels[0] + self.wheels[1]) / 2
        out.angular.z = (self.wheels[1] - self.wheels[0]) / TRACK
        self.out.publish(out)
        was_busy = self.busy()
        self.servo = [s + max(-SERVO_STEP, min(SERVO_STEP, g - s)) for s, g in zip(self.servo, self.servo_goal)]
        if was_busy and not self.busy() and self.servo_goal == GRIP and self.range['front'] < GRAB_RANGE:
            # 큐브가 실제로 집게 앞에 있을 때만 (몸 중심 0.45 m 안): 벽에 PSD로 닫아도 큐브를 지워 "쥠"으로 쳐서 오인 파지가
            # 성공으로 숨었음 (2026-11-01 Gazebo m4: 큐브 1.8 m 앞 벽에서 쥠). 빈 채로 닫으면 정면 PSD가 실제 값 → hold_lost
            if self.true_last and math.dist(self.true_last[:2], CUBE) > 0.45:
                self.get_logger().warn(f'벽을 쥠 (큐브까지 {math.dist(self.true_last[:2], CUBE):.2f} m): 큐브 아님')
            else:
                self.holding = True
                self.delete.call_async(DeleteEntity.Request(name=self.target))
                self.get_logger().info(f'큐브 쥠 (정면 {self.range["front"]:.3f} m)')
        self.scenario(self.now())
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
