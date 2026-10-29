# 미션 1 상태머신 노드입니다 (2026-10-29 펌웨어: 연속 속도 없음, 이산 이동만).
#   탐색: 트인 방향만 제자리 45°씩 돌며 카메라로 둘러봄 → 중앙 힌트 옆 칸 → 미탐색 경계 칸 (grid.py BFS)
#   이동: "돌고([TL/TR]) → 한 칸 직진([F])"을 하나씩 보내고 # DONE 뒤 SLAM 자세로 다음 명령을 다시 계산
#   접근: 빨간 목표물이 화면 세로 중앙선에서 align_deg 넘게 벗어나면 그만큼 돌고, 아니면 F100 직진
#   파지: 직진 중 정면 PSD가 임계 이상으로 이어지면 이 노드가 q로 세우고(보드는 PSD로 멈추지 않음), 접근 중이면 집게 닫기(8)
#   복귀: 출발 칸(0, 0)으로 같은 방식. 경로가 없을 때 같은 요청을 무한 반복하지 않음
# 명령은 /base/command(encoder_odom이 보드로 전달), 결과는 /base/event(# DONE / # ABORT / # REJECT)입니다.
#   ros2 launch cwu_nav mission1.launch.py       # 전체 스택 + 이 노드

"""Mission 1 state machine on discrete board moves: look, grid moves, approach, PSD grasp, return home."""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

from cwu_nav.grid import (BLOCKED, NEIGHBOURS, UNKNOWN, bfs, cell_of, cell_state, command_text, corridor_clear,
                          frontier, next_move, scan_points)
from cwu_nav.mission_logic import FRONT, SERVO_L, SERVO_R, is_grasp, spin_plan
from cwu_nav.servo import yaw_error

# 정지 원인을 볼 PSD 기록 길이 (텔레메트리 20 Hz → 20표본)
PSD_WINDOW_S = 1.0
# 보드 정지 이벤트 뒤 판단까지 기다리는 시간 (정지 뒤 텔레메트리 한 줄)
STOP_SETTLE_S = 0.3
STOP_EVENTS = ('# ABORT', '# REJECT', '# STOP')


class Mission(Node):

    def __init__(self):
        super().__init__('mission1')
        defaults = {
            'bearing_topic': '/target/bearing', 'rate_hz': 20.0,
            'psd_threshold': 550, 'psd_hold_samples': 2, 'grasp_front_min': 300, 'near_front_min': 250,
            'search_hint': [1.6, 1.6], 'initial_spin_deg': 90.0, 'look_step_deg': 45,
            'spin_open_dist': 0.6, 'camera_half_fov_deg': 30.0, 'spin_min_gap_deg': 5,
            'align_deg': 5.0, 'approach_step_mm': 100, 'max_blind_steps': 3, 'max_approach_steps': 40,
            'sample_timeout_s': 0.5, 'confirm_count': 3, 'confirm_window_s': 1.0, 'grasp_target_age_s': 2.0,
            'grip_angles': [99, 78], 'open_angles': [40, 140], 'grip_timeout_s': 4.0,
            'cell_size': 0.4, 'grid_span': 8, 'turn_min_deg': 5.0, 'max_cells': 1, 'body_radius': 0.165,
            'settle_s': 0.5, 'move_timeout_s': 20.0, 'max_goal_steps': 30, 'max_return_retries': 3,
            'max_aborts': 8, 'abort_window_s': 60.0,
        }
        self.declare_parameters('', list(defaults.items()))
        self.p = {key: self.get_parameter(key).value for key in defaults}

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.command = self.create_publisher(String, '/base/command', 10)
        self.create_subscription(PointStamped, self.p['bearing_topic'], self.on_bearing, 10)
        self.create_subscription(Int32MultiArray, '/base/telemetry', self.on_telemetry, 10)
        self.create_subscription(String, '/base/event', self.on_event, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.telemetry = self.map = self.scan = None
        self.psd_log = []            # (수신 시각 s, [앞, 뒤, 좌, 우])
        self.sightings = []          # (수신 시각 s, 화면 중앙선 기준 방향 오차 rad, 양수 = 오른쪽)
        self.aborts = []
        # 이동 명령 하나: move = (명령 문자열, 보낸 시각, 직진 목표 칸 또는 None),
        # result = None(진행 중) / ('done', t) / (정지 줄, t). idle_since = 마지막 이동이 끝난 시각
        self.move, self.result, self.idle_since = None, None, self.now()
        # 라이다·보드가 막았다고 본 칸, 둘러본 칸, 못 간 목표
        self.blocked, self.visited, self.failed = set(), set(), set()
        self.goal_cells, self.goal_steps, self.hint_done = set(), 0, False
        self.holding, self.initial_look = False, True
        self.state, self.since, self.start = 'INIT', self.now(), self.now()
        self.create_timer(1.0 / self.p['rate_hz'], self.tick)
        self.get_logger().info('미션 1 대기: 텔레메트리·TF·스캔을 기다립니다.')

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
        있습니다(정지 상태에서도 581~654로 흔들림, 2026-10-14 실물 1회차).
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
        if self.move is None or self.result is not None:
            return
        if msg.data.startswith('# DONE'):
            self.result = ('done', self.now())
        elif msg.data.startswith(STOP_EVENTS):
            self.result = (msg.data, self.now())  # 첫 원인만 (REJECT 뒤 q의 STOP이 덮지 않게)

    def seen(self, window, after=-math.inf):
        t = self.now()
        return [e for s, e in self.sightings if t - s <= window and s >= after]

    def target_confirmed(self):
        """Enough recent detections since the robot stopped, the newest still fresh."""
        p, after = self.p, self.idle_since + 0.1  # 회전 중 흐린 프레임은 빼고
        return (bool(self.seen(p['sample_timeout_s'], after))
                and len(self.seen(p['confirm_window_s'], after)) >= p['confirm_count'])

    def target_centred(self):
        recent = self.seen(self.p['grasp_target_age_s'])
        return bool(recent) and abs(math.degrees(recent[-1])) < 2 * self.p['align_deg']

    def pose(self):
        try:
            tf = self.tf.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return None
        q = tf.transform.rotation
        return tf.transform.translation.x, tf.transform.translation.y, 2 * math.atan2(q.z, q.w)

    def laser_yaw(self):
        """Laser yaw in base_link (rad), None without scan or TF."""
        if self.scan is None:
            return None
        try:
            q = self.tf.lookup_transform('base_link', self.scan.header.frame_id, Time()).transform.rotation
        except TransformException:
            return None
        return 2 * math.atan2(q.z, q.w)

    def points(self):
        yaw = self.laser_yaw()
        if yaw is None:
            return None
        return scan_points(self.scan.ranges, self.scan.angle_min, self.scan.angle_increment, yaw)

    def gripper_at(self, angles):
        t = self.telemetry
        return bool(t) and [t[SERVO_L], t[SERVO_R]] == list(angles)

    def states(self):
        """{cell: FREE/UNKNOWN/BLOCKED} for every cell within grid_span of the start (지도 없으면 모두 미탐색)."""
        p, span = self.p, self.p['grid_span']
        cells = [(i, j) for i in range(-span, span + 1) for j in range(-span, span + 1)]
        if self.map is None:
            return {c: UNKNOWN for c in cells}
        info = self.map.info
        grid = np.array(self.map.data, np.int8).reshape(info.height, info.width)
        origin = (info.origin.position.x, info.origin.position.y)
        return {c: cell_state(grid, info.resolution, origin, c, p['cell_size']) for c in cells}

    def here(self):
        pose = self.pose()
        return cell_of(pose[0], pose[1], self.p['cell_size']) if pose is not None else (0, 0)

    def passable(self, states):
        span = self.p['grid_span']
        return lambda c: (abs(c[0]) <= span and abs(c[1]) <= span and c not in self.blocked
                          and states.get(c) != BLOCKED)

    # ── 출력 ──
    def board(self, text, target_cell=None):
        """One board command. Moves (F/T) wait for # DONE / # ABORT in tick; gripper commands do not."""
        self.command.publish(String(data=text))
        if text[0] in 'FT':
            self.move, self.result = (text, self.now(), target_cell), None
            self.get_logger().info(f'{self.state}: {text}')

    def ready(self):
        """No move in flight and the SLAM pose has had settle_s to catch up."""
        return self.move is None and self.now() - self.idle_since >= self.p['settle_s']

    # ── 상태머신 ──
    def enter(self, state):
        self.get_logger().info(f'{self.state} → {state} ({self.now() - self.start:.0f} s)')
        self.state, self.since = state, self.now()
        getattr(self, 'start_' + state.lower(), lambda: None)()

    def tick(self):
        if self.move is not None:
            self.watch_move()
            if self.move is not None:
                return
        getattr(self, 'do_' + self.state.lower())()

    def watch_move(self):
        text, sent, _ = self.move
        now = self.now()
        if self.result is None:
            if text[0] == 'F' and not self.holding and self.front_blocked(now):
                # 보드는 정면 PSD로 멈추지 않으므로 여기서 세웁니다. 쥔 큐브는 정면 PSD 앞에 있어 운반 중에는 안 봄
                self.command.publish(String(data='q'))
                self.result = ('# ABORT PSD_FRONT (Pi)', now)
            elif now - sent > self.p['move_timeout_s']:
                self.get_logger().error(f'{text}: {self.p["move_timeout_s"]:.0f} s 안에 결과 없음, 정지합니다.')
                self.command.publish(String(data='q'))
                self.result = ('# ABORT PI_TIMEOUT', now - STOP_SETTLE_S)
            return
        event, at = self.result
        if event != 'done' and now - at < STOP_SETTLE_S:
            return  # 정지 줄 뒤에 오는 텔레메트리(높은 PSD 값)까지 받고 판단
        move, self.move, self.result, self.idle_since = self.move, None, None, now
        if event != 'done':
            self.on_stop(event, move)

    def front_blocked(self, now):
        """The last psd_hold_samples front PSD readings (all within 0.2 s) are at or above psd_threshold.

        표본 하나는 잡음으로 70~150 튀어 큐브 앞에서 일찍 멈췄음(2026-10-14) → 20 Hz 두 표본(50 ms) 연속.
        """
        p = self.p
        recent = [h for h in self.psd_log if now - h[0] < 0.2][-p['psd_hold_samples']:]
        return len(recent) == p['psd_hold_samples'] and all(h[1][0] >= p['psd_threshold'] for h in recent)

    def on_stop(self, event, move):
        """The board refused or stopped a move: grasp, or mark the way blocked and carry on."""
        p, (text, _, cell) = self.p, move
        centred = self.target_centred()
        psd = self.psd_peak()
        # 집게가 다 열려 있을 때만: 닫히는 중·반쯤 닫힌 날도 정면 PSD를 가립니다
        approaching = self.state == 'APPROACH' and text[0] == 'F' and self.gripper_at(p['open_angles'])
        if is_grasp(event, psd, p['grasp_front_min'], approaching, centred):
            self.get_logger().info(f'정면 PSD 최대 {psd[FRONT]}: 목표물 파지')
            return self.enter('GRASP')
        t = self.now()
        if event.startswith('# ABORT') and not (self.aborts and t - self.aborts[-1] < 1.0):
            self.aborts.append(t)
        self.aborts = [a for a in self.aborts if t - a < p['abort_window_s']]
        self.get_logger().warn(f'{text} → {event} ({self.state}, PSD 앞·뒤·좌·우 최대 {psd[FRONT:FRONT + 4]})')
        if len(self.aborts) > p['max_aborts']:
            self.get_logger().error(f'{p["abort_window_s"]:.0f} s 안에 정지 {len(self.aborts)}회: 미션 중단')
            return self.enter('DONE')
        if cell is not None:
            self.blocked.add(cell)  # 그 칸으로 가는 직진이 막힘 → 다음 계획에서 돌아감
        elif text[0] == 'T':
            # 회전 중 PSD 정지(PSD_PIVOT: 네 방향 중 하나라도 550 이상): 앞이 비었으면 짧게 빠져나와 다시
            points = self.points()
            if points is not None and corridor_clear(points, p['approach_step_mm'] / 1000, p['body_radius'],
                                                     p['body_radius']):
                return self.board(f'F{p["approach_step_mm"]}')
            if self.state == 'LOOK':
                return self.next_goal()
        if self.state == 'APPROACH':
            self.enter('LOOK')

    def do_init(self):
        if self.telemetry is not None and self.pose() is not None and self.scan is not None:
            self.enter('RELEASE')  # 집게부터 끝까지 엶 (지난 실행이 닫힌 채·운반 모드로 끝났을 수 있음)

    def start_release(self):
        self.holding, self.released = False, False

    def do_release(self):
        """Open the gripper fully before anything moves (반쯤 닫힌 날은 정면 PSD를 가려 가짜 파지, 실물 2회차)."""
        if not self.released:
            self.board('7')
            self.released, self.since = True, self.now()
        elif self.gripper_at(self.p['open_angles']):
            self.enter('LOOK')
        elif self.now() - self.since > self.p['grip_timeout_s']:
            self.released = False  # 다시 보냄

    def start_look(self):
        """Degrees to sweep: the first look only toward the arena, later only the open directions (spin_plan)."""
        p = self.p
        self.look_dir, self.look_left = 1, 0.0
        if self.initial_look:
            # 출발 자세는 경기장이 왼쪽 (힌트 가정) → 반시계로 그쪽만 보고 바로 힌트로 (2026-10-17 사용자 결정)
            self.initial_look, self.look_left = False, p['initial_spin_deg']
            return
        yaw = self.laser_yaw()
        if yaw is None:
            self.look_left = 360.0 - p['look_step_deg']
            return
        ccw, cw = spin_plan(self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                            yaw, p['spin_open_dist'], p['camera_half_fov_deg'], p['spin_min_gap_deg'])
        # 마지막으로 본 쪽부터. 오차 > 0 이면 목표물이 오른쪽 → 시계 방향
        last_right = bool(self.sightings) and self.sightings[-1][1] > 0
        self.look_dir = -1 if (last_right if self.sightings else cw < ccw) else 1
        self.look_left = cw if self.look_dir < 0 else ccw
        self.get_logger().info(f'트인 방향만 둘러봄: {"반시계" if self.look_dir > 0 else "시계"} {self.look_left:.0f}도')

    def do_look(self):
        if not self.ready():
            return
        if self.target_confirmed():
            return self.enter('APPROACH')
        if self.look_left <= 0:
            self.visited.add(self.here())
            return self.next_goal()
        step = min(self.p['look_step_deg'], max(10, round(self.look_left)))
        self.look_left -= step
        self.board(command_text(('T', self.look_dir * step)))

    def next_goal(self):
        """Hint first (the hint cell and its neighbours), then the nearest frontier cell not yet looked from."""
        p, states = self.p, self.states()
        if not self.hint_done:
            self.hint_done = True
            hx, hy = cell_of(*p['search_hint'], p['cell_size'])
            self.goal_cells = {(hx + di, hy + dj) for di, dj in NEIGHBOURS} | {(hx, hy)}
        else:
            self.goal_cells = frontier(states) - self.visited - self.failed - {self.here()}
        if not self.goal_cells or bfs(self.here(), self.passable(states), self.goal_cells.__contains__) is None:
            self.get_logger().warn('갈 곳이 없습니다. 막힘·실패·둘러본 칸을 비우고 중앙 힌트부터 다시 봅니다.')
            self.blocked.clear()
            self.failed.clear()
            self.visited.clear()
            self.hint_done = False
            return self.enter('LOOK')
        self.enter('GOTO')

    def start_goto(self):
        self.goal_steps = 0

    def step_toward(self, goal_cells):
        """Send the next move toward the nearest goal cell. Returns 'arrived', 'moving' or 'no_path'."""
        p, pose = self.p, self.pose()
        if pose is None:
            return 'moving'  # TF를 기다림
        states = self.states()
        path = bfs(cell_of(pose[0], pose[1], p['cell_size']), self.passable(states), goal_cells.__contains__)
        if path is None:
            return 'no_path'
        move = next_move(pose, path, p['cell_size'], p['turn_min_deg'], max_cells=p['max_cells'])
        if move is None:
            return 'arrived'
        target = path[min(len(path) - 1, 1)] if move[0] == 'F' else None
        if move[0] == 'F':
            points = self.points()
            if points is not None and not corridor_clear(points, move[1] / 1000, p['body_radius'], p['body_radius']):
                self.get_logger().info(f'라이다: {target} 칸으로 가는 직선이 막힘 → 돌아갑니다.')
                self.blocked.add(target)
                return 'moving'
        self.goal_steps += 1
        self.board(command_text(move), target)
        return 'moving'

    def do_goto(self):
        if not self.ready():
            return
        if self.target_confirmed():
            return self.enter('APPROACH')
        if self.goal_steps > self.p['max_goal_steps']:
            self.get_logger().warn(f'{self.p["max_goal_steps"]}번 움직여도 목표에 못 감: 다른 목표로.')
            self.failed |= self.goal_cells
            return self.next_goal()
        result = self.step_toward(self.goal_cells)
        if result == 'arrived':
            self.enter('LOOK')
        elif result == 'no_path':
            self.failed |= self.goal_cells
            self.next_goal()

    def start_approach(self):
        self.approach_steps, self.blind_steps = 0, 0

    def do_approach(self):
        """Turn to null the bearing, then F100 until the board's front PSD stops it (on_stop → GRASP)."""
        p, t = self.p, self.telemetry
        if not self.ready():
            return
        if self.approach_steps >= p['max_approach_steps']:
            self.get_logger().info('접근이 끝나지 않음: 다시 둘러봅니다.')
            return self.enter('LOOK')
        fresh = self.seen(p['sample_timeout_s'], self.idle_since + 0.1)
        if fresh:
            err = math.degrees(fresh[-1])
            self.blind_steps = 0
            self.approach_steps += 1
            if abs(err) >= p['align_deg']:
                # 양수 = 오른쪽 = 시계(TR). 펌웨어 최소 10°
                return self.board(command_text(('T', -math.copysign(max(10, round(abs(err))), err))))
            return self.board(f'F{p["approach_step_mm"]}')
        # 가까워지면 큐브가 화면 아래로 빠집니다. 중앙에 보고 들어왔으면(또는 정면 PSD가 가까움) 몇 걸음 그대로 직진
        near = t is not None and t[FRONT] >= p['near_front_min']
        if (self.target_centred() or near) and self.blind_steps < p['max_blind_steps']:
            self.blind_steps += 1
            self.approach_steps += 1
            return self.board(f'F{p["approach_step_mm"]}')
        self.get_logger().info('목표물 상실: 다시 둘러봅니다.')
        self.enter('LOOK')

    def start_grasp(self):
        self.grip_tries = 0
        self.grip()

    def grip(self):
        self.board('8')
        self.grip_tries, self.since = self.grip_tries + 1, self.now()

    def do_grasp(self):
        p = self.p
        if self.gripper_at(p['grip_angles']):
            self.holding = True
            return self.enter('RETURN')
        if self.now() - self.since > p['grip_timeout_s']:
            if self.grip_tries < 2:
                return self.grip()
            self.get_logger().warn('집게 각도를 확인하지 못했지만 복귀합니다.')
            self.holding = True
            self.enter('RETURN')

    def start_return(self):
        self.return_retries, self.goal_steps = 0, 0

    def do_return(self):
        """Grid moves home.

        경로가 없으면 막힘 표시를 비우고 옆 칸으로 한 칸 옮겨 다시 계획합니다 (16회차: 같은 자리에서 같은
        요청만 157번 반복). max_return_retries 넘으면 멈춥니다.
        """
        if not self.ready():
            return
        result = self.step_toward({(0, 0)})
        if result == 'arrived':
            return self.enter('DONE')
        if result != 'no_path':
            return
        self.return_retries += 1
        if self.return_retries > self.p['max_return_retries']:
            self.get_logger().error('복귀 경로를 찾지 못했습니다: 미션 종료.')
            return self.enter('DONE')
        self.get_logger().warn(f'복귀 경로 없음 ({self.return_retries}회): 막힘 표시를 비우고 옆 칸으로 옮겨 다시 계획.')
        self.blocked.clear()
        here, states = self.here(), self.states()
        sideways = {(here[0] + di, here[1] + dj) for di, dj in NEIGHBOURS}
        self.step_toward({c for c in sideways if self.passable(states)(c)} or sideways)

    def start_done(self):
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
