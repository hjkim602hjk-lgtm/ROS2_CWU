# 미션 2 배틀 시뮬레이션의 심판 + 상대 로봇 3대 두뇌입니다 (PC 전용, 실물 경로에는 없음).
#   경기 시작 = mission2가 보드에 첫 명령(PF0·7)을 보낸 때. 그때부터 상대도 움직이고 match_s 뒤 판정 한 줄.
#   목표물 'victim'(동적 상자): 바닥에서는 Gazebo 위치를 따르고, 쥔 로봇 앞 0.2 m에는 심판이 매 틱 옮겨 붙임.
#     우리 쥠 = fake_board 텔레메트리 서보 [99,78] + 정면 600(쥠 표시) + 큐브가 집게 앞 0.3 m 안
#     상대 쥠 = 상대 중심이 바닥 큐브 0.22 m 안에 1.5 s (집게 닫는 시간)
#   탈락: 로봇 중심이 모서리 칸(|x|,|y| > 1.4)에 들어감 → 상대는 경기장 밖으로, 쥐고 있었으면 큐브를 중앙에 다시 놓음(규정 23조)
#   상대: 큐브가 바닥이면 큐브로, 자기가 쥐었으면 안쪽(|x|,|y| < 1.0)을 배회, 남이 쥐었으면 그 로봇을 쫓아감. 속도는 시드별
"""Mission-2 battle referee and opponent brains."""
import math
import random

import rclpy
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String

OPPS = ('opp_1', 'opp_2', 'opp_3')
GRIP = [99, 78]


def in_corner(x, y):
    return abs(x) > 1.4 and abs(y) > 1.4


class Referee(Node):
    def __init__(self):
        super().__init__('referee')
        self.start_pose = [self.declare_parameter(k, 0.0).value for k in ('start_x', 'start_y', 'start_yaw')]
        self.match_s = self.declare_parameter('match_s', 300.0).value
        rng = random.Random(self.declare_parameter('seed', 0).value)
        self.speed = {o: rng.uniform(0.07, 0.14) for o in OPPS}
        self.delay = {o: rng.uniform(0.0, 2.0) for o in OPPS}
        self.rng = rng
        self.pose = {}                       # 이름 → 경기장 좌표 (x, y, yaw)
        self.out = {o: self.create_publisher(Twist, f'/{o}/cmd_vel', 10) for o in OPPS}
        self.create_subscription(Odometry, '/odom', lambda m: self.on_odom('us', m), 50)
        for o in OPPS:
            self.create_subscription(Odometry, f'/{o}/odom', lambda m, o=o: self.on_odom(o, m), 50)
        self.create_subscription(Int32MultiArray, '/base/telemetry', self.on_telemetry, 10)
        self.create_subscription(String, '/base/command', self.on_command, 10)
        self.set_state = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_models, 10)
        self.telemetry = None
        self.holder, self.cube = None, (0.0, 0.0)
        self.out_of_play, self.waypoint, self.near_since = set(), {}, {}
        self.start = None
        self.first_grab, self.held_total, self.held_since, self.corner_min = None, 0.0, None, math.inf
        self.last_tick, self.done = None, False
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def t(self):
        return 0.0 if self.start is None else self.now() - self.start

    def on_odom(self, name, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        sx, sy, syaw = self.start_pose
        c, s = math.cos(syaw), math.sin(syaw)  # world(= 우리 출발 자세) → 경기장
        self.pose[name] = (sx + c * p.x - s * p.y, sy + s * p.x + c * p.y, syaw + 2 * math.atan2(q.z, q.w))

    def on_models(self, msg):
        """Floor cube position from Gazebo (로봇 바퀴에 밀릴 수 있음)."""
        if self.holder is not None or 'victim' not in msg.name:
            return
        p = msg.pose[msg.name.index('victim')].position
        sx, sy, syaw = self.start_pose
        c, s = math.cos(syaw), math.sin(syaw)
        self.cube = (sx + c * p.x - s * p.y, sy + s * p.x + c * p.y)

    def on_telemetry(self, msg):
        self.telemetry = list(msg.data)

    def on_command(self, msg):
        if self.start is None:
            self.start = self.now()
            self.get_logger().info(f'심판: 경기 시작 (상대 속도 {[round(self.speed[o], 2) for o in OPPS]})')

    def put_cube(self, xy):
        """Teleport the cube to arena (x, y)."""
        self.cube = xy
        sx, sy, syaw = self.start_pose
        dx, dy = xy[0] - sx, xy[1] - sy
        c, s = math.cos(syaw), math.sin(syaw)
        req = SetEntityState.Request()
        req.state = EntityState(name='victim')
        req.state.pose.position.x, req.state.pose.position.y = dx * c + dy * s, -dx * s + dy * c
        req.state.pose.position.z = 0.025
        req.state.pose.orientation.w = 1.0
        if self.set_state.service_is_ready():
            self.set_state.call_async(req)

    def front(self, name, d=0.2):
        x, y, yaw = self.pose[name]
        return x + d * math.cos(yaw), y + d * math.sin(yaw)

    def we_hold(self):
        t = self.telemetry
        return bool(t) and t[10:12] == GRIP and t[6] == 600

    def tick(self):
        if self.start is None or self.done or 'us' not in self.pose:
            return
        now = self.now()
        dt = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        # ── 쥠 ──
        if self.holder is None:
            if self.we_hold() and math.dist(self.front('us', 0.17), self.cube) < 0.3:
                self.grab('us')
            else:
                for o in OPPS:
                    # 상대도 집게를 닫는 시간(1.5 s, 우리 서보와 비슷) 동안 큐브 곁에 있어야 쥠
                    close = (o not in self.out_of_play and o in self.pose
                             and math.dist(self.pose[o][:2], self.cube) < 0.22)
                    self.near_since[o] = (self.near_since.get(o) or now) if close else None
                    if close and now - self.near_since[o] >= 1.5:
                        self.grab(o)
                        break
        elif self.holder == 'us' and not self.we_hold():
            self.get_logger().info(f'심판: 우리가 놓침 ({self.t():.1f} s)')
            self.drop(self.front('us'))
        if self.holder is not None:
            self.put_cube(self.front(self.holder))
        if self.holder == 'us':
            self.held_total += dt
        x, y, yaw = self.pose['us']
        if now - getattr(self, 'status_at', -math.inf) >= 5.0:
            self.status_at = now  # mission2 "상태" 줄의 추정 위치와 비교할 실제 위치
            q = round(self.start_pose[2] / (math.pi / 2)) * math.pi / 2  # 우리 출발 칸을 (2,2)로 돌린 좌표 (mission2와 같은 틀)
            cx, cy = math.cos(q) * x + math.sin(q) * y, -math.sin(q) * x + math.cos(q) * y
            self.get_logger().info(f'심판: 실제 위치 (우리 틀) ({cx:+.2f}, {cy:+.2f}) {math.degrees(yaw - q):+.0f}° | 큐브 '
                                   f'({self.cube[0]:+.2f}, {self.cube[1]:+.2f}) {self.holder or "바닥"}')
        # ── 탈락 ──
        self.corner_min = min(self.corner_min, max(1.4 - abs(x), 1.4 - abs(y)))
        if in_corner(x, y):
            self.get_logger().error(f'심판: 우리 로봇 모서리 진입 ({x:+.2f}, {y:+.2f}) {self.t():.1f} s')
            return self.finish('우리 탈락')
        for o in OPPS:
            if o in self.out_of_play or o not in self.pose:
                continue
            if in_corner(*self.pose[o][:2]):
                self.eliminate(o)
        # ── 상대 두뇌 ──
        for o in OPPS:
            self.drive(o)
        if self.t() >= self.match_s:
            self.finish('시간 종료')

    def grab(self, name):
        self.holder = name
        if name == 'us':
            self.held_since = self.t()
            if self.first_grab is None:
                self.first_grab = self.t()
        self.get_logger().info(f'심판: {name} 쥠 ({self.t():.1f} s)')

    def drop(self, xy):
        self.holder, self.held_since = None, None
        self.put_cube(xy)

    def eliminate(self, o):
        self.get_logger().info(f'심판: {o} 탈락 ({self.t():.1f} s){" — 쥐고 있었음, 큐브 중앙으로" if self.holder == o else ""}')
        self.out_of_play.add(o)
        self.out[o].publish(Twist())
        req = SetEntityState.Request()
        req.state = EntityState(name=o)
        req.state.pose.position.x, req.state.pose.position.y = 10.0 + 2 * OPPS.index(o), 10.0
        req.state.pose.orientation.w = 1.0
        self.set_state.call_async(req)
        if self.holder == o:
            spot = next((s for s in ((0.0, 0.0), (0.3, 0.0), (-0.3, 0.0), (0.0, 0.3), (0.0, -0.3))
                         if all(math.dist(self.pose[n][:2], s) > 0.25 for n in self.pose if n not in self.out_of_play)),
                        (0.0, 0.0))
            self.drop(spot)

    def drive(self, o):
        if o in self.out_of_play or o not in self.pose:
            return
        cmd = Twist()
        if self.t() >= self.delay[o]:
            x, y, yaw = self.pose[o]
            if self.holder is None:
                goal = self.cube
            elif self.holder == o:
                goal = self.waypoint.get(o)
                if goal is None or math.dist(goal, (x, y)) < 0.15:
                    goal = self.waypoint[o] = (self.rng.uniform(-1.0, 1.0), self.rng.uniform(-1.0, 1.0))
            else:
                goal = self.pose[self.holder][:2]
            err = math.remainder(math.atan2(goal[1] - y, goal[0] - x) - yaw, 2 * math.pi)
            cmd.angular.z = max(-1.5, min(1.5, 2.0 * err))
            cmd.linear.x = self.speed[o] if abs(err) < 0.6 else 0.0
        self.out[o].publish(cmd)

    def finish(self, why):
        self.done = True
        for o in OPPS:
            self.out[o].publish(Twist())
        last = 0.0 if self.held_since is None else self.t() - self.held_since
        first = '-' if self.first_grab is None else f'{self.first_grab:.1f}'
        self.get_logger().info(
            f'심판 종료: {why} | 보유 {self.holder or "없음"} | 처음 잡음 {first} s | 누적 보유 {self.held_total:.0f} s | '
            f'마지막 연속 {last:.0f} s | 탈락 {sorted(self.out_of_play) or "-"} | 모서리까지 최소 {self.corner_min:.2f} m '
            f'| {self.t():.0f} s')


def main(args=None):
    rclpy.init(args=args)
    node = Referee()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
