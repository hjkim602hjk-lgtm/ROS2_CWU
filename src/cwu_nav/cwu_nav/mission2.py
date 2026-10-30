# 미션 2(배틀럼블) 상태머신 노드입니다. 미션 1(mission1.py)과 따로 돕니다. 설계: docs/superpowers/specs/2026-10-30-mission2-design.md
#   SPIN    출발하자마자 제자리 반시계 회전(중앙이 왼쪽) → 빨간색이 보이면 바로 APPROACH (사용자 결정)
#   RUSH    못 찾았거나 놓쳤으면 중앙으로 최고속, 중앙에서도 안 보이면 제자리 회전
#   APPROACH 빨간색을 화면 세로 중앙선에 맞추며 전진, 정면 PSD 임계 지속 → 정지 → 잡기. 집게 앞에 상대가 붙어 있으면 안 닫음
#   GRASP   정지 → "8" → 서보 닫힘 + 정면 PSD 임계 이상이면 HOLD
#   HOLD    (3,3)~(7,7) 테두리를 반시계로 계속 돎, 상대가 다가오면 반대쪽으로 비킨 뒤 다시 합류. 정면 PSD가 떨어지면 놓침
#   PUSH    상대가 쥐었으면 빼앗지 않고 그 상대를 가장 가까운 모서리 칸으로 밀어 넣음 (탈락 → 목표물 중앙 재배치, 규정 23조)
#   FINISH  경기 시간(match_s) 끝: 정지, 집게는 닫힌 채
# 모든 상태의 속도는 마지막에 모서리 금지(corner_guard)를 거칩니다. 위치 = odom + 벽 맞춤 (SLAM 없음).
#   ros2 launch cwu_nav mission2.launch.py       # 경기에서는 tools/field/m2_start.sh

"""Mission 2 state machine: spin-and-grab, ring hold, push the holder into a corner."""
import math

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

from cwu_nav.grid import compose, scan_points, wall_heading
from cwu_nav.mission2_logic import (START, corner_guard, flee_target, held_by, in_corner, push_plan, pursue,
                                    ring_target, robot_ahead, robot_on_bearing, robots, wall_fix)
from cwu_nav.mission_logic import FRONT, SERVO_L, SERVO_R
from cwu_nav.servo import yaw_error

STOP_EVENTS = ('# ABORT', '# REJECT', '# STOP')


class Mission2(Node):

    def __init__(self):
        super().__init__('mission2')
        defaults = {
            'bearing_topic': '/target/bearing', 'rate_hz': 20.0, 'match_s': 300.0, 'arm_retry_s': 1.0,
            'psd_threshold': 550, 'psd_hold_samples': 2, 'near_front_min': 250, 'drop_s': 0.6,
            'grip_angles': [99, 78], 'open_angles': [40, 140], 'grip_timeout_s': 4.0, 'grip_settle_s': 0.15,
            'spin_speed': 0.9, 'spin_max_deg': 150.0, 'confirm_count': 2, 'confirm_window_s': 0.6, 'lost_s': 1.5,
            'drive_speed': 0.15, 'approach_speed': 0.15, 'near_speed': 0.05, 'k_heading': 1.5, 'turn_only_deg': 35.0,
            'k_servo': 2.0, 'align_deg': 8.0, 'centre_search_m': 0.35,
            'ring_half': 0.8, 'ring_lead': 0.35, 'avoid_m': 0.6, 'flee_step': 0.5,
            'push_speed': 0.15, 'push_lost_s': 3.0, 'search_spin_speed': 0.6, 'blind_push_s': 4.0,
            'push_behind': 0.4, 'push_lateral_tol': 0.15, 'push_timeout_s': 30.0,
            'held_centre_min': 0.5, 'bearing_tol_deg': 6.0, 'block_reach': 0.35, 'block_half_width': 0.12,
            'corner_margin': 0.25, 'corner_lookahead': 0.25,
            'fix_period_s': 0.2, 'fix_gain': 0.5, 'fix_tol': 0.3, 'fix_min_points': 15, 'heading_fix_max_deg': 10.0,
            'robot_wall_margin': 0.15, 'robot_gap': 0.1, 'robot_min_points': 3, 'robot_max_extent': 0.45,
            'robot_track_m': 0.4,
        }
        self.declare_parameters('', list(defaults.items()))
        self.p = {key: self.get_parameter(key).value for key in defaults}

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.command = self.create_publisher(String, '/base/command', 10)
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PointStamped, self.p['bearing_topic'], self.on_bearing, 10)
        self.create_subscription(Int32MultiArray, '/base/telemetry', self.on_telemetry, 10)
        self.create_subscription(String, '/base/event', self.on_event, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.telemetry, self.scan_base = None, None   # 최근 텔레메트리, 최근 스캔 점(base_link)
        self.psd_log, self.sightings = [], []         # (t, 정면 PSD), (t, 화면 중앙선 기준 오차 rad, 양수 = 오른쪽)
        self.robot_xys, self.robots_at = [], -math.inf  # 상대 로봇 위치(경기장 좌표)
        self.fix, self.fixed_at = START, -math.inf    # odom → 경기장 (x, y, yaw): 출발 자세 = (2,2) 중심, +x
        self.armed, self.arm_at = False, -math.inf
        self.after_release, self.push_target, self.push_seen = 'SPIN', None, -math.inf
        self.last_red = None  # 마지막으로 빨간색을 본 로봇 위치 (보유자 추정)
        self.state, self.since, self.start = 'INIT', self.now(), None
        self.create_timer(1.0 / self.p['rate_hz'], self.tick)
        self.get_logger().info('미션 2 대기: 텔레메트리·TF·스캔을 기다립니다.')

    # ── 입력 ──
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_bearing(self, msg):
        t = self.now()
        self.sightings = [s for s in self.sightings if t - s[0] < 3.0] + [(t, yaw_error(msg.point.x, msg.point.z))]

    def on_telemetry(self, msg):
        if len(msg.data) >= 12:
            self.telemetry = list(msg.data)
            t = self.now()
            self.psd_log = [h for h in self.psd_log if t - h[0] < 2.0] + [(t, msg.data[FRONT])]

    def on_event(self, msg):
        if msg.data.startswith('# ARMED'):
            self.armed = True
        elif msg.data.startswith(STOP_EVENTS) and not msg.data.startswith('# REJECT BUSY'):
            self.armed = False
            self.get_logger().warn(f'보드: {msg.data} ({self.state})')

    def on_scan(self, msg):
        """Scan points → base_link; every fix_period_s, fit the walls (pose at the scan's own stamp) and find robots.

        스캔은 자세보다 늦게 옴(G4 한 바퀴 135 ms + 전달 지연). 멈추지 않고 보정하려고 스캔 시각의 odom 자세에 붙임 (설계 4.1).
        """
        try:
            q = self.tf.lookup_transform('base_link', msg.header.frame_id, Time()).transform.rotation
        except TransformException:
            return
        lo, hi = msg.range_min, msg.range_max
        pts = [(x, y) for x, y in scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                              2 * math.atan2(q.z, q.w)) if lo <= math.hypot(x, y) <= hi]
        self.scan_base = pts
        if self.now() - self.fixed_at < self.p['fix_period_s'] or not pts:
            return
        odom = self.tf2d('odom', 'base_link', Time.from_msg(msg.header.stamp))
        if odom is None:
            return
        self.fixed_at = self.now()
        self.update_fix(odom, pts)
        pose = compose(self.fix, odom)
        p = self.p
        self.robot_xys = robots(self.to_arena(pose, pts), p['robot_wall_margin'], p['robot_gap'],
                                p['robot_min_points'], p['robot_max_extent'])
        self.robots_at = self.now()

    def update_fix(self, odom, pts):
        """Turn, then shift, the odom → arena transform so the walls sit on x, y = ±1.8."""
        p, g = self.p, self.p['fix_gain']
        pose = compose(self.fix, odom)
        heading = wall_heading(pts)
        if heading is not None:
            da = math.remainder(-(pose[2] + heading), math.pi / 2)
            if abs(math.degrees(da)) <= p['heading_fix_max_deg']:
                self.rotate_fix(pose, g * da)
                pose = compose(self.fix, odom)
        d = wall_fix(self.to_arena(pose, pts), p['fix_tol'], p['fix_min_points'])
        if d is not None:
            x, y, a = self.fix
            self.fix = (x + g * d[0], y + g * d[1], a)

    def rotate_fix(self, pose, da):
        """Turn the odom → arena transform (x, y, yaw, as compose() reads it) by da about the robot's position."""
        x, y, a = self.fix
        c, s = math.cos(da), math.sin(da)
        px, py = pose[0], pose[1]
        self.fix = (c * (x - px) - s * (y - py) + px, s * (x - px) + c * (y - py) + py, a + da)

    @staticmethod
    def to_arena(pose, pts):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return [(pose[0] + c * x - s * y, pose[1] + s * x + c * y) for x, y in pts]

    def tf2d(self, parent, child, at=None):
        try:
            t = self.tf.lookup_transform(parent, child, at or Time()).transform
        except TransformException:
            return None
        return t.translation.x, t.translation.y, 2 * math.atan2(t.rotation.z, t.rotation.w)

    def pose(self):
        odom = self.tf2d('odom', 'base_link')
        return None if odom is None else compose(self.fix, odom)

    def seen(self, window):
        t = self.now()
        return [e for s, e in self.sightings if t - s <= window]

    def target_confirmed(self):
        p = self.p
        return bool(self.seen(0.3)) and len(self.seen(p['confirm_window_s'])) >= p['confirm_count']

    def front(self):
        return self.telemetry[FRONT] if self.telemetry else 0

    def front_near(self, window=0.5):
        """The last psd_hold_samples front PSD readings are all at or above psd_threshold."""
        p, t = self.p, self.now()
        recent = [v for s, v in self.psd_log if t - s < window][-p['psd_hold_samples']:]
        return len(recent) == p['psd_hold_samples'] and all(v >= p['psd_threshold'] for v in recent)

    def gripper_at(self, angles):
        t = self.telemetry
        return bool(t) and [t[SERVO_L], t[SERVO_R]] == list(angles)

    def fresh_robots(self):
        return self.robot_xys if self.now() - self.robots_at < 0.5 else []

    # ── 출력 ──
    def board(self, text):
        self.command.publish(String(data=text))
        self.get_logger().info(f'{self.state}: 보드 {text}')

    def request_arm(self):
        if self.now() - self.arm_at >= self.p['arm_retry_s']:
            self.arm_at = self.now()
            self.command.publish(String(data='ARM'))

    def drive(self, v=0.0, w=0.0):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd_vel.publish(msg)

    # ── 상태머신 ──
    def enter(self, state):
        t = 0.0 if self.start is None else self.now() - self.start
        self.get_logger().info(f'{self.state} → {state} ({t:.1f} s)')
        self.state, self.since = state, self.now()
        getattr(self, 'start_' + state.lower(), lambda: None)()

    def tick(self):
        pose = self.pose()
        if pose is not None and self.now() - getattr(self, 'status_at', -math.inf) >= 5.0:
            self.status_at = self.now()  # 현장·시뮬레이션 분석용 상태 줄 (위치 = 경기장 좌표, 원점 중앙)
            t = self.telemetry or [0] * 12
            self.get_logger().info(
                f'상태 {self.state}: 위치 ({pose[0]:+.2f}, {pose[1]:+.2f}) {math.degrees(pose[2]):+.0f}° '
                f'| 상대 {len(self.fresh_robots())} | 검출 5 s {len(self.seen(5.0))} '
                f'| PSD 앞·뒤·좌·우 {t[FRONT:FRONT + 4]} | 무장 {int(self.armed)}')
        if self.start is not None and self.state != 'FINISH' and self.now() - self.start >= self.p['match_s']:
            self.enter('FINISH')
        if self.state in ('INIT', 'RELEASE', 'GRASP', 'FINISH'):
            getattr(self, 'do_' + self.state.lower())()
            return self.drive()
        if pose is None:
            return self.drive()
        if not self.armed:
            self.request_arm()
        v, w = getattr(self, 'do_' + self.state.lower())(pose) or (0.0, 0.0)
        if self.state in ('GRASP', 'RELEASE', 'FINISH'):  # do_가 방금 멈춰야 하는 상태로 바꿈
            return self.drive()
        self.drive(*corner_guard(pose, v, w, self.p['corner_margin'], self.p['corner_lookahead']))

    def do_init(self):
        if self.telemetry is not None and self.pose() is not None and self.scan_base is not None:
            self.start = self.now()  # 경기 시간 = 노드가 센서를 다 받은 때부터 (온보드 시계)
            self.enter('RELEASE')

    def start_release(self):
        self.sent_release = False

    def do_release(self):
        """Open the gripper and turn the board's front-PSD stop off (밀기 중 상대에 걸려 서면 안 됨; 잡기는 이 노드가 판단)."""
        if not self.sent_release:
            if self.now() - self.since < 0.3:  # [V0,0]으로 보드가 정지 모드가 된 뒤 (괄호·7·8은 정지 중에만 받음)
                return
            self.board('PF0')
            self.board('7')
            self.sent_release, self.since = True, self.now()
        elif self.gripper_at(self.p['open_angles']):
            self.enter(self.after_release)
        elif self.now() - self.since > self.p['grip_timeout_s']:
            self.sent_release = False

    def start_spin(self):
        self.spun, self.last_yaw = 0.0, None

    def do_spin(self, pose):
        """Counter-clockwise on the spot until the red target is seen (출발 자세에서 중앙이 왼쪽)."""
        if self.last_yaw is not None:
            self.spun += abs(math.remainder(pose[2] - self.last_yaw, 2 * math.pi))
        self.last_yaw = pose[2]
        if self.target_confirmed():
            return self.decide(pose)
        if math.degrees(self.spun) > self.p['spin_max_deg']:
            self.get_logger().info('돌아도 목표물이 안 보임: 중앙으로')
            return self.enter('RUSH')
        return 0.0, self.p['spin_speed']

    def decide(self, pose):
        """Red target confirmed: grab it, or push its holder."""
        p = self.p
        bearing = pose[2] - self.seen(0.3)[-1]
        holder = held_by(pose[:2], bearing, self.fresh_robots(), math.radians(p['bearing_tol_deg']),
                         p['held_centre_min'])
        self.last_red = holder
        if holder is not None:
            self.push_target = holder
            self.get_logger().info(f'상대가 목표물을 쥠 ({holder[0]:+.2f}, {holder[1]:+.2f}): 모서리로 밉니다')
            return self.enter('PUSH')
        self.enter('APPROACH')

    def start_rush(self):
        self.searching_since = None

    def do_rush(self, pose):
        p = self.p
        if self.target_confirmed():
            return self.decide(pose)
        if math.hypot(pose[0], pose[1]) < p['centre_search_m']:
            self.searching_since = self.searching_since or self.now()
            robots_now = self.fresh_robots()
            if robots_now and self.now() - self.searching_since > p['blind_push_s']:
                # 쥔 큐브는 그 로봇 앞에서만 보여 중앙에서 돌아도 못 봄 (2026-10-30 Gazebo 배틀 1: 100 s 넘게 검출 0).
                # 마지막으로 빨간색을 본 곳(없으면 중앙)에 가장 가까운 로봇을 보유자로 보고 밂. 틀려도 경쟁자 하나를 미는 것
                guess = self.last_red or (0.0, 0.0)
                self.push_target = min(robots_now, key=lambda r: math.dist(r, guess))
                self.get_logger().info(f'{p["blind_push_s"]:.0f} s 동안 목표물이 안 보임: 마지막 위치 근처 로봇 '
                                       f'({self.push_target[0]:+.2f}, {self.push_target[1]:+.2f})을 보유자로 보고 밉니다')
                return self.enter('PUSH')
            return 0.0, p['search_spin_speed']  # 중앙인데 안 보임: 둘러봄 (재배치는 중앙 "주변"일 수 있음, 규정 23조)
        return pursue(pose, (0.0, 0.0), p['drive_speed'], p['k_heading'], math.radians(p['turn_only_deg']))

    def start_approach(self):
        self.last_sight = self.now()

    def do_approach(self, pose):
        p = self.p
        fresh = self.seen(0.3)
        if fresh:
            self.last_sight = self.now()
        near = self.front() >= p['near_front_min']
        blocked = self.scan_base is not None and robot_ahead(self.scan_base, p['block_half_width'], p['block_reach'])
        if blocked and near:
            # 집게 앞에 상대 로봇이 붙어 있음: 상대가 쥔 목표물을 같이 쥐지 않음 (동시 확보 금지, 사용자 결정)
            ahead = [r for r in self.fresh_robots() if math.dist(r, pose[:2]) < p['block_reach'] + 0.25]
            self.get_logger().info('집게 앞에 상대 로봇: 닫지 않습니다')
            if ahead:
                self.push_target = min(ahead, key=lambda r: math.dist(r, pose[:2]))
                return self.enter('PUSH')
            return 0.0, 0.0
        if self.front_near() and self.gripper_at(p['open_angles']):
            self.get_logger().info(f'정면 PSD {self.front()}: 잡기')
            return self.enter('GRASP')
        if fresh and not near:
            holder = held_by(pose[:2], pose[2] - fresh[-1], self.fresh_robots(),
                             math.radians(p['bearing_tol_deg']), p['held_centre_min'])
            if holder is not None:
                self.push_target = holder
                self.get_logger().info('다가가 보니 상대가 쥐고 있음: 밉니다')
                return self.enter('PUSH')
        if self.now() - self.last_sight > p['lost_s'] and not near:
            self.get_logger().info('목표물 놓침: 중앙으로')
            return self.enter('RUSH')
        if near:
            return p['near_speed'], 0.0  # 집게 바로 앞: 화면 아래로 잘려 검출이 흔들리므로 직진만
        if not fresh:
            return p['approach_speed'] * 0.5, 0.0
        err = fresh[-1]
        w = max(-1.2, min(1.2, -p['k_servo'] * err))
        v = p['approach_speed'] if abs(math.degrees(err)) < 3 * p['align_deg'] else 0.0
        return v, w

    def start_grasp(self):
        self.gripped_at = None

    def do_grasp(self):
        p = self.p
        if self.gripped_at is None:
            if self.now() - self.since >= p['grip_settle_s']:  # [V0,0]으로 보드가 정지 모드가 된 뒤 (7·8은 정지 중에만 받음)
                self.board('8')
                self.gripped_at = self.now()
            return
        if self.gripper_at(p['grip_angles']):
            if self.front() >= p['psd_threshold']:
                self.get_logger().info(f'확보 (정면 PSD {self.front()})')
                return self.enter('HOLD')
            self.get_logger().info(f'닫았지만 정면 PSD {self.front()}: 못 잡음')
            self.after_release = 'RUSH'
            return self.enter('RELEASE')
        if self.now() - self.gripped_at > p['grip_timeout_s']:
            self.gripped_at = None  # 다시 보냄 (서보가 돌던 중 REJECT BUSY)

    def start_hold(self):
        self.low_since = None

    def do_hold(self, pose):
        p = self.p
        if self.front() < p['psd_threshold']:
            self.low_since = self.low_since or self.now()
            if self.now() - self.low_since > p['drop_s']:
                self.get_logger().warn(f'놓침 (정면 PSD {self.front()})')
                self.after_release = 'RUSH'
                return self.enter('RELEASE')
        else:
            self.low_since = None
        near = [r for r in self.fresh_robots() if math.dist(r, pose[:2]) < p['avoid_m']]
        if near:
            opp = min(near, key=lambda r: math.dist(r, pose[:2]))
            target = flee_target(pose, opp, p['flee_step'], p['corner_margin'])
        else:
            target = ring_target(pose, p['ring_half'], p['ring_lead'])
        return pursue(pose, target, p['drive_speed'], p['k_heading'], math.radians(p['turn_only_deg']))

    def start_push(self):
        self.push_seen, self.free_since, self.pushing = self.now(), None, False

    def do_push(self, pose):
        p = self.p
        # 빨간색이 보이는데 그 방향에 로봇이 1 s 넘게 없으면 바닥에 떨어진 것 (한 번 판정으로 바꾸면 APPROACH ↔ PUSH를 0.1 s마다
        # 오갔음, 2026-10-30 Gazebo 배틀 1)
        free = self.target_confirmed() and robot_on_bearing(
            pose[:2], pose[2] - self.seen(0.3)[-1], self.fresh_robots(), math.radians(p['bearing_tol_deg'])) is None
        self.free_since = (self.free_since or self.now()) if free else None
        if self.free_since is not None and self.now() - self.free_since > 1.0:
            self.get_logger().info('목표물이 상대에게서 떨어짐: 잡으러 갑니다')
            return self.enter('APPROACH')
        cands = [r for r in self.fresh_robots() if math.dist(r, self.push_target) < p['robot_track_m']]
        if cands:
            self.push_target, self.push_seen = min(cands, key=lambda r: math.dist(r, self.push_target)), self.now()
            self.last_red = self.push_target
        if (self.now() - self.push_seen > p['push_lost_s'] or in_corner(*self.push_target)
                or self.now() - self.since > p['push_timeout_s']):
            self.get_logger().info('밀기 끝 (상대 탈락·소실·시간): 다시 찾습니다')
            return self.enter('RUSH')
        target, self.pushing = push_plan(pose, self.push_target, p['push_behind'], p['push_lateral_tol'], self.pushing)
        pushing = self.pushing
        return pursue(pose, target, p['push_speed'] if pushing else p['drive_speed'], p['k_heading'],
                      math.radians(p['turn_only_deg']))

    def do_finish(self):
        if not getattr(self, 'finished', False):
            self.finished = True
            self.get_logger().info(f'미션 2 종료 ({self.p["match_s"]:.0f} s): 정지, 정면 PSD {self.front()}')


def main(args=None):
    rclpy.init(args=args)
    node = Mission2()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.drive()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
