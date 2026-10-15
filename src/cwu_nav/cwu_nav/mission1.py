# 미션 1 상태머신 노드입니다.
#   탐색: 제자리 한 바퀴 → 중앙 힌트 → 미탐색 경계(frontier)를 Nav2로 차례로 방문
#   접근: 빨간 목표물이 보이면 Nav2를 취소하고 화면 세로 중앙선에 맞춘 뒤 직진(servo.py)
#   파지: 보드가 정면 PSD 임계(STM의 PSD_THRESHOLD)에서 멈추면 집게를 닫고 운반 모드로
#   복귀: 출발점(map 원점 = SLAM 시작 자세)으로 Nav2 주행
# 좌·우·뒤 PSD 정지는 보드가 하고, 이 노드는 막힌 쪽 반대로 직진해 빠진 뒤 하던 일을 이어 갑니다.
# 미션 중에는 사람이 없으므로 보드 재무장도 이 노드가 정지 원인을 판단한 뒤에 요청합니다.
#   ros2 launch cwu_nav mission1.launch.py       # 전체 스택 + 이 노드

"""Mission 1 state machine: search, align, approach, PSD grasp, return home."""
import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Int32MultiArray, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from cwu_nav.go_home import home_goal
from cwu_nav.mission_logic import (FRONT, SERVO_L, SERVO_R, backoff_direction, frontier_goal,
                                   hint_goal, is_grasp, side_block, spin_plan, wrap)
from cwu_nav.servo import follow, yaw_error

# 보드 정지(ABORT 등)를 받으면 BACKOFF로 처리하는 상태
# 정지 원인을 볼 PSD 기록 길이 (텔레메트리 20 Hz → 20표본)
PSD_WINDOW_S = 1.0
# 보드 정지 이벤트 뒤 판단까지 기다리는 시간 (텔레메트리 3줄)
STOP_SETTLE_S = 0.15
MOVING = ('SPIN', 'GOTO', 'APPROACH', 'RETURN', 'BACKOFF')


class Mission(Node):

    def __init__(self):
        super().__init__('mission1')
        defaults = {
            'bearing_topic': '/target/bearing', 'cmd_topic': '/cmd_vel', 'rate_hz': 20.0,
            'psd_threshold': 550, 'grasp_front_min': 300, 'near_front_min': 250, 'final_timeout_s': 4.0,
            'search_hint': [1.6, 1.6], 'hint_standoff': 0.6, 'spin_speed': 0.4,
            'spin_open_dist': 0.6, 'camera_half_fov_deg': 30.0, 'spin_min_gap_deg': 5,
            'k_yaw': 1.2, 'max_yaw': 0.8, 'min_yaw': 0.0,
            'align_tol_rad': 0.09, 'approach_speed': 0.06, 'side_guard_reach': 0.165, 'side_guard_width': 0.18,
            'sample_timeout_s': 0.5, 'confirm_count': 3, 'confirm_window_s': 1.0,
            'blind_forward_s': 1.0, 'grasp_target_age_s': 2.0,
            'grip_angles': [99, 78], 'open_angles': [40, 140], 'grip_timeout_s': 4.0,
            'arm_timeout_s': 2.0, 'backoff_speed': 0.05, 'backoff_s': 1.5,
            'hold_min_raw': 550, 'hold_lost_s': 1.0, 'retry_wait_s': 2.0,
            'frontier_clearance': 0.15, 'frontier_min_cells': 8, 'frontier_min_dist': 0.5,
            'frontier_passage': 0.075,
            'blacklist_radius': 0.4, 'max_aborts': 8, 'abort_window_s': 60.0,
        }
        self.declare_parameters('', list(defaults.items()))
        self.p = {key: self.get_parameter(key).value for key in defaults}

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.cmd = self.create_publisher(Twist, self.p['cmd_topic'], 10)
        self.create_subscription(PointStamped, self.p['bearing_topic'], self.on_bearing, 10)
        self.create_subscription(Int32MultiArray, '/base/telemetry', self.on_telemetry, 10)
        self.create_subscription(String, '/base/event', self.on_event, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.arm_client = self.create_client(Trigger, '/encoder_odom/arm')
        self.carry_client = self.create_client(SetBool, '/encoder_odom/carry')

        self.telemetry = self.map = self.scan = self.pending = self.goal_handle = self.goal_result = None
        self.psd_log = []            # (수신 시각 s, [앞, 뒤, 좌, 우])
        self.final_since = None      # APPROACH 마지막 PSD 직진 단계 시작 시각
        self.sightings = []          # (수신 시각 s, 화면 중앙선 기준 방향 오차 rad)
        self.armed, self.arm_at, self.aborts = False, -math.inf, []
        self.goal_id, self.goal, self.hint_done, self.blacklist = 0, None, False, []
        self.released = self.holding = self.side_hit = False
        self.resume = 'SPIN'
        self.state, self.since, self.start = 'INIT', self.now(), self.now()
        self.create_timer(1.0 / self.p['rate_hz'], self.tick)
        self.get_logger().info('미션 1 대기: 센서·TF·Nav2·보드 무장을 기다립니다.')

    # ── 입력 ──
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_bearing(self, msg):
        t = self.now()
        self.sightings = [s for s in self.sightings if t - s[0] < 3.0]
        self.sightings.append((t, yaw_error(msg.point.x, msg.point.z)))

    def on_telemetry(self, msg):
        if len(msg.data) >= 12:
            self.telemetry = list(msg.data)
            t = self.now()
            self.psd_log = [h for h in self.psd_log if t - h[0] < PSD_WINDOW_S] + [(t, msg.data[6:10])]

    def psd_peak(self):
        """Telemetry with each PSD replaced by its maximum over the last PSD_WINDOW_S.

        보드는 PSD 한 표본이 임계를 넘는 순간 멈추는데, 다음 텔레메트리 줄은 잡음으로 임계 아래일 수
        있습니다(정지 상태에서도 581~654로 흔들림). 최신 한 줄로 판단하면 파지를 장애물로 오판하고
        후퇴 방향도 '없음'이 나옵니다 (2026-10-14 실물 1회차 32 s).
        """
        t = list(self.telemetry or [0] * 12)
        for i in range(4):
            t[FRONT + i] = max([h[1][i] for h in self.psd_log] or [t[FRONT + i]])
        return t

    def on_map(self, msg):
        self.map = msg

    def on_scan(self, msg):
        self.scan = msg

    def on_event(self, msg):
        if msg.data.startswith('# ARMED'):
            self.armed = True
        elif msg.data.startswith(('# ABORT', '# REJECT', '# STOP')):
            self.armed = False
            if self.pending is None:  # 첫 원인을 남김 (REJECT 뒤 q의 STOP이 덮지 않게)
                self.pending = (msg.data, self.now())

    def seen(self, window):
        t = self.now()
        return [e for s, e in self.sightings if t - s <= window]

    def target_confirmed(self):
        """Enough recent detections, the newest still fresh.

        신선도 없이 개수만 보면 0.5~1 s 전 검출로 확정 → APPROACH가 곧바로 '상실'로 SPIN →
        다시 확정을 0.05 s마다 반복합니다 (2026-10-14 실물 1회차).
        """
        p = self.p
        return (bool(self.seen(p['sample_timeout_s']))
                and len(self.seen(p['confirm_window_s'])) >= p['confirm_count'])

    def pose(self):
        try:
            tf = self.tf.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return None
        q = tf.transform.rotation
        return tf.transform.translation.x, tf.transform.translation.y, 2 * math.atan2(q.z, q.w)

    # ── 출력 ──
    def drive(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.cmd.publish(msg)

    def call(self, client, request, done=None):
        if not client.service_is_ready():
            self.get_logger().warn(f'{client.srv_name} 서비스가 없습니다.', throttle_duration_sec=2.0)
            return
        future = client.call_async(request)
        if done is not None:
            future.add_done_callback(done)

    def request_arm(self):
        self.arm_at = self.now()

        def done(future):
            if future.result() is not None and future.result().success:
                self.armed = True  # 이미 무장돼 있었음 (# ARMED 를 놓친 경우)
        self.call(self.arm_client, Trigger.Request(), done)

    def armed_or_retry(self):
        if not self.armed and self.now() - self.arm_at > self.p['arm_timeout_s']:
            self.request_arm()
        return self.armed

    def carry(self, on):
        self.call(self.carry_client, SetBool.Request(data=on))

    def pose_goal(self, x, y, yaw):
        goal = home_goal(self.get_clock().now().to_msg())
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        return goal

    def navigate(self, goal):
        """Send a NavigateToPose goal; its outcome lands in self.goal_result ('ok'/'fail')."""
        self.cancel()
        gid = self.goal_id

        def accepted(future):
            handle = future.result()
            if gid != self.goal_id:  # 그사이 취소·교체됨
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.goal_result = 'fail'
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(finished)

        def finished(future):
            if gid == self.goal_id:
                ok = future.result().status == GoalStatus.STATUS_SUCCEEDED
                self.goal_result = 'ok' if ok else 'fail'
        self.nav.send_goal_async(goal).add_done_callback(accepted)

    def cancel(self):
        self.goal_id += 1
        self.goal_result = None
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    # ── 상태머신 ──
    def enter(self, state):
        self.get_logger().info(f'{self.state} → {state} ({self.now() - self.start:.0f} s)')
        self.state, self.since = state, self.now()
        getattr(self, 'start_' + state.lower(), lambda: None)()

    def tick(self):
        # 정지 줄 뒤에 오는 텔레메트리(높은 PSD 값)까지 받고 판단합니다
        if self.pending is not None and self.now() - self.pending[1] >= STOP_SETTLE_S:
            event, self.pending = self.pending[0], None
            if self.state in MOVING:
                self.on_stop(event)
        getattr(self, 'do_' + self.state.lower())()

    def on_stop(self, event):
        """The board stopped and disarmed: grasp, or back away and carry on."""
        p = self.p
        # PSD 직진 단계면 카메라가 큐브를 놓쳤어도 직전까지 중앙에 보고 들어온 것
        centred = self.target_centred() or (self.state == 'APPROACH' and self.final_since is not None)
        psd = self.psd_peak()
        # 집게가 다 열려 있을 때만: 닫히는 중·반쯤 닫힌 날도 정면 PSD를 가립니다
        approaching = self.state == 'APPROACH' and self.gripper_at(p['open_angles'])
        if is_grasp(event, psd, p['grasp_front_min'], approaching, centred):
            self.get_logger().info(f'정면 PSD 최대 {psd[FRONT]}: 목표물 파지')
            return self.enter('GRASP')
        t = self.now()
        # 보드 정지(ABORT)만, 1 s 안에 이어진 것은 한 번으로 셉니다. 뒤따르는 REJECT·STOP과
        # 같은 장애물 앞의 연속 정지는 한 번의 상황입니다 (실물 3회차 9회 중 다수가 이것).
        if event.startswith('# ABORT') and not (self.aborts and t - self.aborts[-1] < 1.0):
            self.aborts.append(t)
        self.aborts = [a for a in self.aborts if t - a < p['abort_window_s']]
        if len(self.aborts) > p['max_aborts']:
            self.get_logger().error(
                f'{p["abort_window_s"]:.0f} s 안에 정지 {len(self.aborts)}회: 미션 중단')
            return self.enter('DONE')
        self.get_logger().warn(f'{event} ({self.state}, PSD 앞·뒤·좌·우 최대 {psd[FRONT:FRONT + 4]}, '
                               f'목표물 {"중앙" if centred else "최근 중앙 아님"}) → 후퇴 후 재개')
        if self.state != 'BACKOFF':
            self.resume = self.state
        self.enter('BACKOFF')

    def do_init(self):
        if self.telemetry is None or self.pose() is None or not self.nav.server_is_ready():
            return
        if self.carry_client.service_is_ready() and self.armed_or_retry():
            self.enter('RELEASE')  # 집게부터 끝까지 엶 (지난 실행이 닫힌 채·운반 모드로 끝났을 수 있음)

    def gripper_at(self, angles):
        t = self.telemetry
        return bool(t) and [t[SERVO_L], t[SERVO_R]] == list(angles)

    def start_spin(self):
        self.cancel()
        self.turned, self.last_yaw = 0.0, None
        # 마지막으로 본 쪽부터 돎. 오차 > 0 이면 목표물이 오른쪽 → 시계 방향
        last_right = bool(self.sightings) and self.sightings[-1][1] > 0
        self.spin_dir, self.spin_target = (-1.0 if last_right else 1.0), 2 * math.pi
        plan = self.open_spin()
        if plan is None:
            return  # 스캔·TF가 없으면 한 바퀴
        ccw, cw = plan
        if not self.sightings:
            self.spin_dir = 1.0 if ccw <= cw else -1.0
        self.spin_target = math.radians(ccw if self.spin_dir > 0 else cw)
        self.get_logger().info(
            f'트인 방향만 둘러봄: {"반시계" if self.spin_dir > 0 else "시계"} {math.degrees(self.spin_target):.0f}도')

    def laser_yaw(self):
        """Laser yaw in base_link (rad), None without scan or TF."""
        if self.scan is None:
            return None
        try:
            q = self.tf.lookup_transform('base_link', self.scan.header.frame_id, Time()).transform.rotation
        except TransformException:
            return None
        return 2 * math.atan2(q.z, q.w)

    def open_spin(self):
        """(ccw, cw) degrees covering every direction open beyond spin_open_dist, else None."""
        yaw, p = self.laser_yaw(), self.p
        if yaw is None:
            return None
        return spin_plan(self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                         yaw, p['spin_open_dist'], p['camera_half_fov_deg'], p['spin_min_gap_deg'])

    def approach_drive(self, linear, angular=0.0):
        """Approach command, but never push forward with an obstacle beside the body (LiDAR)."""
        yaw, p = self.laser_yaw(), self.p
        side = 0 if linear <= 0 or yaw is None else side_block(
            self.scan.ranges, self.scan.angle_min, self.scan.angle_increment, yaw,
            p['side_guard_reach'], p['side_guard_width'])
        if not side:
            return self.drive(linear, angular)
        self.drive()
        self.side_hit = True  # 보드 정지처럼 처리하되 측면 PSD 값과 상관없이 뒤로 뺌
        self.on_stop(f'# ABORT SIDE_LIDAR {"LEFT" if side > 0 else "RIGHT"}')

    def do_spin(self):
        p = self.p
        if self.target_confirmed():
            return self.enter('APPROACH')
        pose = self.pose()
        if pose is not None:
            if self.last_yaw is not None:
                self.turned += abs(wrap(pose[2] - self.last_yaw))
            self.last_yaw = pose[2]
        limit = 2 * (2 * math.pi / p['spin_speed']) + 5.0  # 못 돌고 있으면 그냥 다음으로
        if self.turned >= self.spin_target or self.now() - self.since > limit:
            self.drive()
            return self.next_goal()
        self.drive(angular=self.spin_dir * p['spin_speed'])

    def next_goal(self):
        p = self.p
        pose = self.pose()
        if not self.hint_done:
            self.hint_done = True
            x, y = hint_goal(p['search_hint'], p['hint_standoff'])
        else:
            found = None
            if self.map is not None and pose is not None:
                info = self.map.info
                grid = np.array(self.map.data, np.int8).reshape(info.height, info.width)
                found = frontier_goal(
                    grid, info.resolution, (info.origin.position.x, info.origin.position.y),
                    pose[:2], p['frontier_clearance'], p['frontier_min_cells'],
                    p['frontier_min_dist'], self.blacklist, p['blacklist_radius'],
                    p['frontier_passage'])
            if found is None:
                self.get_logger().warn('갈 곳이 없습니다. 실패 목록을 비우고 중앙 힌트부터 다시 봅니다.')
                self.blacklist.clear()
                self.hint_done = False
                return self.enter('SPIN')
            x, y = found
        yaw = math.atan2(y - pose[1], x - pose[0]) if pose is not None else 0.0
        self.goal = (x, y, yaw)
        self.enter('GOTO')

    def start_goto(self):
        self.get_logger().info(f'탐색 목표 ({self.goal[0]:.2f}, {self.goal[1]:.2f})')
        self.navigate(self.pose_goal(*self.goal))

    def do_goto(self):
        if self.target_confirmed():
            return self.enter('APPROACH')
        if self.goal_result == 'ok':
            return self.enter('SPIN')
        if self.goal_result == 'fail':
            self.blacklist.append(self.goal[:2])
            self.next_goal()

    def target_centred(self):
        recent = self.seen(self.p['grasp_target_age_s'])
        return bool(recent) and abs(recent[-1]) < 2 * self.p['align_tol_rad']

    def start_approach(self):
        self.cancel()
        self.final_since = None

    def do_approach(self):
        p, t = self.p, self.telemetry
        # 마지막 구간: 큐브가 화면 아래로 잘려 들어가면 검출 중심이 좌우로 흔들려(±11°, 4회차) 제자리
        # 회전을 반복합니다. 중앙에 보고 들어왔고 집게의 정면 PSD가 가까움을 보면 카메라 대신 직진하고
        # 보드의 PSD 정지(550, 60 ms)를 기다립니다. 집게 열림·앞 빔이면 정면은 185 이하입니다.
        if self.final_since is None and t and t[FRONT] >= p['near_front_min'] and self.target_centred():
            self.final_since = self.now()
            self.get_logger().info(f'정면 PSD {t[FRONT]}: 카메라 대신 PSD로 직진')
        if self.final_since is not None:
            if self.now() - self.final_since > p['final_timeout_s']:
                self.get_logger().info('PSD 직진 중 보드 정지 없음: 다시 둘러봅니다.')
                return self.enter('SPIN')
            return self.approach_drive(p['approach_speed'])
        if self.seen(p['sample_timeout_s']):
            return self.approach_drive(*follow(self.sightings[-1][1], p['k_yaw'], p['max_yaw'],
                                               p['min_yaw'], p['align_tol_rad'], p['approach_speed']))
        # 가까워지면 큐브가 화면 아래로 빠집니다. 정렬된 채 놓쳤으면 잠깐 그대로 직진(PSD가 멈춤).
        if (self.sightings and abs(self.sightings[-1][1]) < p['align_tol_rad']
                and self.now() - self.sightings[-1][0] <= p['sample_timeout_s'] + p['blind_forward_s']):
            return self.approach_drive(p['approach_speed'])
        self.get_logger().info('목표물 상실: 다시 둘러봅니다.')
        self.enter('SPIN')

    def start_grasp(self):
        self.carry(True)  # 잡기 + 운반 모드 (정면 PSD 정지 끔). 보드는 PSD 정지로 이미 멈춤
        self.grip_tries, self.gripped = 1, False

    def do_grasp(self):
        p = self.p
        if not self.gripped:
            if self.gripper_at(p['grip_angles']):
                self.gripped = True
            elif self.now() - self.since > p['grip_timeout_s']:
                if self.grip_tries < 2:
                    self.grip_tries, self.since = self.grip_tries + 1, self.now()
                    return self.carry(True)
                self.get_logger().warn('집게 각도를 확인하지 못했지만 복귀합니다.')
                self.gripped = True
            return
        if self.armed_or_retry():  # 서보가 다 움직인 뒤에만 보드가 무장을 받음
            self.holding = True
            self.enter('RETURN')

    def start_return(self):
        self.navigate(home_goal(self.get_clock().now().to_msg()))
        self.lost_since = self.retry_at = None

    def do_return(self):
        p, t, now = self.p, self.telemetry, self.now()
        if p['hold_min_raw'] > 0 and t and t[FRONT] < p['hold_min_raw']:
            self.lost_since = self.lost_since or now
            if now - self.lost_since > p['hold_lost_s']:
                self.get_logger().warn(f'정면 PSD {t[FRONT]}: 목표물을 놓쳤습니다.')
                return self.enter('RELEASE')
        else:
            self.lost_since = None
        if self.goal_result == 'ok':
            return self.enter('DONE')
        if self.goal_result == 'fail':
            self.goal_result, self.retry_at = None, now + p['retry_wait_s']
            self.get_logger().warn('복귀 경로 실패: 잠시 뒤 다시 계획합니다.')
        if self.retry_at is not None and now > self.retry_at:
            self.enter('RETURN')

    def start_release(self):
        self.cancel()
        self.holding = self.released = False

    def do_release(self):
        """Open the gripper fully before anything moves.

        서보가 움직이는 동안 주행 명령은 보드가 거절(REJECT BUSY)하고, 그 뒤 정지(q)가 서보를 그 자리에
        멈춰 집게가 반쯤 닫힌 채 남습니다. 반쯤 닫힌 날은 정면 PSD를 가려 가짜 파지가 됩니다 (실물 2회차).
        """
        # encoder_odom이 0.25 s 뒤 [V0,0]을 보내 보드가 정지 모드가 된 뒤에야 집게 명령을 받음
        if not self.released and self.now() - self.since > 0.6:
            self.carry(False)
            self.released, self.since = True, self.now()
        elif self.released and self.gripper_at(self.p['open_angles']):
            self.enter('SPIN')
        elif self.released and self.now() - self.since > self.p['grip_timeout_s']:
            self.released = False  # 다시 보냄

    def start_backoff(self):
        self.cancel()
        t = self.psd_peak()
        if self.holding:
            t[FRONT] = 0  # 쥔 큐브가 정면 PSD 앞에 있음. 운반 중 정면은 보드도 검사하지 않음
        self.back_dir = backoff_direction(t, self.p['psd_threshold']) or (-1 if self.side_hit else 0)
        self.side_hit = False
        self.request_arm()
        self.back_since = None

    def do_backoff(self):
        p = self.p
        if not self.armed_or_retry():
            return
        if self.back_since is None:
            self.back_since = self.now()
        if self.back_dir and self.now() - self.back_since < p['backoff_s']:
            return self.drive(self.back_dir * p['backoff_speed'])
        self.drive()
        self.enter(self.resume if self.resume in ('GOTO', 'RETURN') else 'SPIN')

    def start_done(self):
        self.cancel()
        self.drive()
        self.get_logger().info(
            f'미션 종료 ({self.now() - self.start:.0f} s, 목표물 {"보유" if self.holding else "없음"})')

    def do_done(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Mission()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
