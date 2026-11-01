# 미션 1 상태머신 노드입니다 (2026-10-30 펌웨어 속도 모드: /cmd_vel → encoder_odom → [V mm/s,rad/s]).
#   탐색: 트인 방향만 제자리 45°씩 돌며 카메라로 둘러봄 → 중앙 힌트 옆 칸 → 앞이 뚫렸으면 쭉 직진(CRUISE)
#         → 막히면 둘러보고 미탐색 경계 칸 (grid.py BFS). 1순위: 탐색 중 지나간 칸은 다시 안 들어감
#   이동: 판단은 "회전(T) / 칸 중심까지 직진(F)" 단위 그대로, 실행은 멈추지 않는 속도 명령 — 직진은 목표 칸 중심을
#         향해 방향을 계속 고치며 달리고, 끝나면 바로 다음 이동으로 이어 감(할 일이 없을 때만 섬)
#   접근: 보이면 방향을 기록하고 그쪽으로 직진하면서 새 검출마다 방향을 고침(연속 시각 서보). 정면 PSD가 가까우면 직진만
#   파지: 보드가 정면 PSD 60 ms로 멈추거나(# ABORT PSD_FRONT) 이 노드가 텔레메트리로 세우면, 접근 중이면 PF0 → 8
#   복귀: 출발 칸(0, 0)으로 같은 방식. 경로가 없을 때 같은 요청을 무한 반복하지 않음
# 속도는 /cmd_vel, 무장·집게는 /base/command("ARM", "7", "8", "PF0", "PF1"), 보드 결과는 /base/event입니다.
#   ros2 launch cwu_nav mission1.launch.py       # 전체 스택 + 이 노드

"""Mission 1 state machine on discrete board moves: look, grid moves, approach, PSD grasp, return home."""
import math
import time
from collections import Counter
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

from cwu_nav.grid import (BLOCKED, FREE, NEIGHBOURS, UNKNOWN, aim_cell, axis, bfs, bfs_avoiding, camera_view, cell_of,
                          cell_state, center, command_text, corridor_clear, free_distance, frontier, next_move,
                          scan_points, to_line, wall_push, blocking_points, ray_cells, passage,
                          pocket_entries, viewpoints, best_heading, seen_faces, lattice_fix, compose, invert,
                          return_route, wall_heading, mark_scan, occupancy,
                          wall_faces, walled_off, sweep_hits, open_frontier, clear_pixels, trail_route)
from cwu_nav.mission_logic import FRONT, REAR, SERVO_L, SERVO_R, is_grasp
from cwu_nav.servo import yaw_error

# 정지 원인을 볼 PSD 기록 길이 (텔레메트리 5 Hz → 5표본)
PSD_WINDOW_S = 1.0
# 보드 정지 이벤트 뒤 판단까지 기다리는 시간 (정지 뒤 텔레메트리 한 줄)
STOP_SETTLE_S = 0.3
STOP_EVENTS = ('# ABORT', '# REJECT', '# STOP')
# 무장·집게 요청에 대한 거절 — 이동이 막힌 게 아니므로 이동 결과로 쓰지 않음
BENIGN_REJECTS = ('# REJECT BUSY', '# REJECT NOT_ARMED')


class Mission(Node):

    def __init__(self):
        super().__init__('mission1')
        defaults = {
            'bearing_topic': '/target/bearing', 'rate_hz': 20.0,
            'psd_threshold': 550, 'psd_hold_samples': 2, 'grasp_front_min': 300, 'near_front_min': 250,
            'look_pause_s': 0.3,
            'align_deg': 5.0, 'approach_step_mm': 100, 'approach_max_mm': 600, 'max_blind_moves': 2,
            'max_approach_steps': 40,
            'sample_timeout_s': 0.5, 'confirm_count': 3, 'confirm_window_s': 1.0, 'grasp_target_age_s': 2.0,
            'grip_angles': [99, 78], 'open_angles': [40, 140], 'grip_timeout_s': 4.0,
            'camera_half_fov_deg': 35.0, 'camera_range': 1.3, 'lattice_gain': 0.7, 'lattice_period_s': 0.5,
            'lattice_still_s': 0.4, 'lattice_every_s': 1.0, 'heading_jump_deg': 15.0,
            'drive_fix_period_s': 0.5, 'drive_fix_gain': 0.5, 'drive_fix_max_deg': 5.0, 'drive_fix_shift_m': 0.03,
            'crab_init_deg': 0.0, 'crab_gain': 0.3, 'crab_min_m': 0.6, 'crab_max_deg': 6.0,
            'dyn_wait_s': 4.0, 'hold_lost_psd': 350, 'hold_lost_s': 0.5, 'block_ttl_s': 30.0, 'stall_s': 2.0, 'stall_m': 0.03,
            'look_min_free_m': 0.45, 'look_half_width': 0.12,
            'map_still_turn_s': 0.4,
            'edge_jump_m': 0.05,
            'arena_cells': 9,
            'target_hold_s': 8.0,
            'center_tol_m': 0.07, 'corridor_tol_m': 0.03, 'grasp_push_m': 0.015, 'grasp_push_speed': 0.03, 'grasp_psd': 600,
            'passage_min_m': 0.35, 'k_center': 4.0, 'squeeze_back_mm': 150,
            'cube_start_cell': [4, 4],
            'trail_clearance_m': 0.17, 'trail_arrive_m': 0.15, 'trail_short_m': 0.45, 'trail_turn_deg': 10.0,
            'trail_diag_clearance_m': 0.22,
            'map_resolution': 0.05, 'map_min_misses': 2, 'map_size_m': 16.0, 'map_range_m': 4.0, 'map_period_s': 0.25,
            'mission_map_hz': 3.0,
            'cell_size': 0.4, 'turn_min_deg': 12.0, 'max_cells': 2,
            'body_radius': 0.165, 'front_extent': 0.255, 'front_extent_holding': 0.28, 'front_half_width': 0.12,
            'path_half_width': 0.03,
            'wall_keep': 0.175, 'k_wall': 5.0, 'max_wall_push': 0.25, 'near_wall_m': 0.25,
            'settle_s': 0.0, 'move_timeout_s': 20.0, 'max_goal_steps': 30, 'max_return_retries': 3,
            'block_confirm': 3,
            'drive_speed': 0.12, 'approach_speed': 0.08, 'turn_speed': 0.6,
            'min_turn_speed': 0.4, 'k_heading': 1.5, 'max_steer': 1.0,
            'turn_tol_deg': 4.0, 'turn_extra_deg': 1.0,
            'fast_speed': 0.22, 'fast_min_m': 0.75, 'fast_clear_m': 0.6, 'scan_latency_s': 0.2,
            'arrive_m': 0.04, 'approach_turn_deg': 25.0, 'arm_retry_s': 1.0,
            'max_aborts': 8, 'abort_window_s': 60.0,
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
        # 미션이 판단에 쓰는 자체 지도를 RViz로 보기 위함 (SLAM /map과 따로). Transient Local: RViz 지도 표시 기본 QoS
        self.map_pub = self.create_publisher(OccupancyGrid, '/mission_map', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.telemetry = self.scan = None
        self.psd_log = []            # (수신 시각 s, [앞, 뒤, 좌, 우])
        self.sightings = []          # (수신 시각 s, 화면 중앙선 기준 방향 오차 rad, 양수 = 오른쪽)
        self.aborts = []
        # 이동 명령 하나: move = (명령 문자열, 보낸 시각, 직진 목표 칸 또는 None),
        # result = None(진행 중) / ('done', t) / (정지 줄, t). idle_since = 마지막 이동이 끝난 시각
        self.move, self.result, self.idle_since = None, None, self.now()
        self.motion, self.armed, self.arm_at = None, False, -math.inf  # 실행 중인 연속 이동, 보드 무장 상태
        # 라이다·보드가 막았다고 본 칸, 둘러본 칸, 못 간 목표, 탐색 중 지나간 칸(목표에서 빼고 경로에서도 피함)
        self.blocked, self.visited, self.failed, self.passed = set(), set(), set(), set()
        self.fix, self.fix_at = (0.0, 0.0, 0.0), -math.inf  # 격자 보정 (회전 rad, 이동 x, y): odom → 칸 좌표
        self.jump_pending = None  # 아직 확인 못 한 큰 방향 보정 (update_fix)
        self.drive_fix_at = -math.inf  # 직진 중 방향 보정 (drive_fix)
        self.crab = math.radians(self.p['crab_init_deg'])  # 몸 방향보다 실제로 가는 쪽 (rad, +왼쪽), 직진 중 배움
        self.crab_sum = [0.0, 0.0]  # 배우는 중인 (옆 보정 합 m, 간 거리 m)
        self.walls, self.entries = frozenset(), set()  # 빈 칸 사이 바깥벽·새 길 입구 (states가 갱신)
        self.plan = []  # 지금 따라가는 칸 경로 (step_toward)
        # 복귀 지름길 경유점 (follow_trail), 마지막으로 직진·회전을 보낸 경유점
        self.trail, self.trail_sent, self.trail_turned, self.trail_prev = [], None, None, None
        self.trail_centred = None  # 구간 중심선 점을 이미 넣은 경유점 (follow_trail, 한 번만)
        self.target_hold_until = -math.inf  # 이때까지 큐브를 안 본 것으로 (go_detour 길 없음)
        self.squeeze = {}  # 칸마다 옆 모서리로 물러나 다시 간 시각들 (on_stop LIDAR_SIDE)
        self.final_pending = None  # 큐브 앞까지 가는 마지막 직진 거리 (m): 끝나면 정면 PSD로 파지/물러남 판단
        self.entry_turns = 0  # 출입구에서 큐브를 정면에 두려고 돈 횟수 (approach_start_cell, 세 번까지)
        self.center_tries = {}  # 칸마다 한가운데·통로 가운데로 옮기기를 한 시각들 (drive_path, 1분에 두 번까지)
        self.pocket_shift = None  # 큐브 칸 입구에서 라이다로 잰 실제 입구 가운데 (격자 중심에서 옮긴 만큼, map m)
        self.pocket = None  # on_stop이 APPROACH 전에도 봄: 없으면 첫 라이다 정지에서 노드가 죽었음 (Gazebo 18r real)
        self.centered = None  # 한가운데로 옮기기를 이미 한 칸 (drive_path)
        self.center_goal = None  # (칸, 점): 한가운데로 가려고 돈 뒤 다음 틱에 그 점으로 직진 (drive_path)
        self.ever_held = False  # 큐브를 한 번이라도 쥐었음: 그 전에는 큐브가 (5,5)에 그대로 있음 (cube_cells)
        self.sight_stop_at = -math.inf  # 직진 중 빨강을 보고 멈춘 시각 (watch_move)
        self.start_looked = False  # 큐브 시작 칸 (5,5)를 둘러보기로 똑바로 봤음 (look_around)
        self.sight_turn = None  # 직진 중 빨강 한 번 보고 멈춘 뒤 그쪽으로 돌 각도 (°, +왼쪽)
        self.recheck_start = False  # 갈 곳이 다 떨어져 큐브 시작 칸을 다시 보러 가는 중 (next_goal)
        self.lost_since = None  # 운반 중 정면 PSD가 낮아진 때 (hold_lost)
        self.block_at = {}  # 칸마다 마지막으로 막힘 판정한 시각 (block_ttl_s 지나면 다시 시도)
        self.map_reset_at = -math.inf  # 갈 곳이 없어 지도를 지운 때 (next_goal)
        self.backed_at = -math.inf  # 돌기 전에 집게를 벽에서 떼려고 물러난 때 (board)
        self.stall_at = {}  # 끼임으로 막은 칸과 시각: "갈 곳 없음" 비우기에도 남고 block_ttl_s의 3배 뒤 풀림 (passable)
        self.wait_until, self.dyn_waited = -math.inf, {}  # 지나온 칸이 막혀 상대 로봇으로 보고 기다리는 중 (moving_thing)
        self.ticks, self.states_at, self.states_cache = 0, -1, {}  # states()를 틱마다 한 번만 계산
        self.goal_view = None  # 마지막 GOTO의 목적지 칸 (look_around: 거기서는 등 뒤 칸 하나도 돌아봄)
        self.track, self.saved = [], False  # 0.5 s마다 (시각, 칸 좌표 자세, 상태): 끝날 때 지도와 함께 저장 (save_record)
        # 마지막으로 움직인·돈 시각 (track_motion): 스캔은 늦게 오므로 움직이는 중 스캔을 지금 자세에 붙이면 틀림
        self.last_odom, self.moved_at, self.turned_at = None, self.now(), self.now()
        # 직접 쌓는 지도 (states): 화소마다 빔 끝이 맞은 수·빔이 지나간 수. 원점 = 출발 칸 중심, 사방 map_size_m/2
        n = int(round(self.p['map_size_m'] / self.p['map_resolution']))
        self.hits, self.misses = np.zeros((n, n), np.uint16), np.zeros((n, n), np.uint16)
        self.map_origin, self.mapped_at = (-self.p['map_size_m'] / 2, -self.p['map_size_m'] / 2), -math.inf
        self.block_hits = Counter()  # 칸마다 막힘 판정 횟수: block_confirm번 넘으면 비울 때도 남김 (unblock)
        self.camera_seen, self.camera_at = set(), -math.inf  # 카메라로 본 칸 (탐색 완료 판단)
        self.view_targets = {}  # GOTO 목적지(보는 자리) → 거기서 보이는 못 본 칸들
        self.resume_axis = None  # 직진 중 옆을 돌아본 뒤 이어 갈 진행 축
        self.goal_cells, self.goal_steps, self.cleared = set(), 0, False
        self.look_tried = set()  # 가는 길에 (5,5)를 보려고 멈춰 본 칸 (do_goto, 칸마다 한 번)
        self.goal_strict = True      # 지금 목표가 지나간 칸을 안 거치고 닿는 목표인지
        self.holding = False
        self.state, self.since, self.start = 'INIT', self.now(), self.now()
        self.create_timer(1.0 / self.p['rate_hz'], self.tick)
        if self.p['mission_map_hz'] > 0:
            self.create_timer(1.0 / self.p['mission_map_hz'], self.publish_map)
        self.get_logger().info('미션 1 대기: 텔레메트리·TF·스캔을 기다립니다.')

    # ── 입력 ──
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_bearing(self, msg):
        t = self.now()
        self.sightings = [s for s in self.sightings if t - s[0] < 3.0]
        err = yaw_error(msg.point.x, msg.point.z)
        if not self.sightings or t - self.sightings[-1][0] > 1.0:
            # 검출이 로그에 없어 "카메라가 못 봤는지, 봤는데 안 갔는지"를 가릴 수 없었음 (2026-10-31 실물: 구멍 블록 속 큐브)
            self.get_logger().info(f'카메라: 빨강 보임 (방향 {math.degrees(err):+.0f}°, {self.state}, 칸 {self.here()})')
        self.sightings.append((t, err))

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

    def publish_map(self):
        """Own map → /mission_map (frame odom), cropped to what has been seen; mission_map_hz times a second."""
        grid = occupancy(self.hits, self.misses, self.p['map_min_misses'])
        rows, cols = np.nonzero(grid >= 0)
        if not rows.size:
            return
        r0, r1, c0, c1 = rows.min(), rows.max() + 1, cols.min(), cols.max() + 1
        res, (x0, y0) = self.p['map_resolution'], self.map_origin
        a, tx, ty = self.fix
        # 지도는 칸 좌표(격자 보정 뒤)로 쌓였으므로 보정을 거꾸로 적용해 odom 좌표로 내보냄
        ox, oy, oyaw = compose(invert((tx, ty, a)), (x0 + c0 * res, y0 + r0 * res, 0.0))
        msg = OccupancyGrid()
        msg.header.frame_id, msg.header.stamp = 'odom', self.get_clock().now().to_msg()
        msg.info.resolution, msg.info.width, msg.info.height = res, int(c1 - c0), int(r1 - r0)
        msg.info.origin.position.x, msg.info.origin.position.y = ox, oy
        msg.info.origin.orientation.z, msg.info.origin.orientation.w = math.sin(oyaw / 2), math.cos(oyaw / 2)
        msg.data = grid[r0:r1, c0:c1].ravel().tolist()
        self.map_pub.publish(msg)

    def on_scan(self, msg):
        """Keep the scan; every map_period_s add it to the own map, except while or just after turning."""
        self.scan = msg
        p = self.p
        # 도는 중·막 돈 뒤(map_still_turn_s)는 빼고 쌓음: 늦게 온 스캔이 돌아간 방향으로 벽을 그림 (track_motion)
        if self.now() - self.mapped_at < p['map_period_s'] or self.now() - self.turned_at < p['map_still_turn_s']:
            return
        pose, points = self.scan_pose(), self.points()  # 스캔을 찍은 때의 자세: 직진 중에도 0.2 s 지연만큼 안 어긋남
        if pose is None or not points:
            return
        self.mapped_at = self.now()
        mark_scan(self.hits, self.misses, self.map_origin, p['map_resolution'], pose, points, p['map_range_m'])

    def on_event(self, msg):
        if msg.data.startswith('# ARMED'):
            self.armed = True
        elif msg.data.startswith(STOP_EVENTS):
            self.armed = False  # 보드는 ABORT·STOP에 무장을 풂
            if self.move is not None and self.result is None and not msg.data.startswith(BENIGN_REJECTS):
                self.drive()
                self.result = (msg.data, self.now())  # 첫 원인만 (REJECT 뒤 q의 STOP이 덮지 않게)

    def seen(self, window, after=-math.inf):
        t = self.now()
        return [e for s, e in self.sightings if t - s <= window and s >= after]

    def target_confirmed(self, after=None):
        """Enough recent detections since `after` (default: the last stop), the newest still fresh."""
        p = self.p
        after = self.idle_since + 0.1 if after is None else after
        return (bool(self.seen(p['sample_timeout_s'], after))
                and len(self.seen(p['confirm_window_s'], after)) >= p['confirm_count'])

    def target_seen(self):
        """'yes' when confirmed, 'wait' while the camera has seen it since the stop but not confirm_count times yet, else 'no'.

        멈춘 뒤 settle_s 안에 검출이 confirm_count번 와야 인정하면, 검출이 느릴 때 화면에 있는 큐브를 두고 다음
        이동을 함 (2026-10-30 검사). 한 번이라도 보였으면 confirm_window_s까지 더 기다립니다.
        """
        p = self.p
        if self.now() < self.target_hold_until:
            return 'no'  # 큐브 쪽 길이 없어 다른 곳으로 옮기는 중 (go_detour)
        if self.target_confirmed():
            return 'yes'
        if (self.seen(p['sample_timeout_s'], self.idle_since + 0.1)
                and self.now() - self.idle_since < p['settle_s'] + p['confirm_window_s']):
            return 'wait'
        return 'no'

    def target_centred(self):
        recent = self.seen(self.p['grasp_target_age_s'])
        return bool(recent) and abs(math.degrees(recent[-1])) < 2 * self.p['align_deg']

    def tf2d(self, parent, child):
        try:
            t = self.tf.lookup_transform(parent, child, Time()).transform
        except TransformException:
            return None
        return t.translation.x, t.translation.y, 2 * math.atan2(t.rotation.z, t.rotation.w)

    def pose(self):
        """Odometry pose moved onto the cell lattice (self.fix, update_fix); all planning uses this.

        SLAM 위치·지도는 쓰지 않음: 긴 통로에서 SLAM이 1~2.6 m 튀어 맵 2·3을 거의 매번 실패했고, 튐을 막아도 SLAM 지도가
        튄 자세로 계속 쌓여 망가졌음 (2026-10-31 Gazebo 11차). odom의 작은 어긋남은 멈출 때마다 격자 보정이 잡음.
        """
        odom = self.tf2d('odom', 'base_link')
        if odom is None:
            return None
        a, tx, ty = self.fix
        return compose((tx, ty, a), odom)

    def track_motion(self):
        """Remember when the odometry last moved (3 mm or 0.3° between ticks) and last turned (0.3°).

        실물 G4는 한 바퀴 135 ms + 전달 지연이라, 돌고 난 직후 스캔은 그 전 방향에서 찍힌 것. 그 스캔을 지금 자세에 붙여
        격자 보정이 TL 뒤에는 -4°, TR 뒤에는 +4°씩 쌓여 -31°까지 틀어졌고(위치를 잃고 벽 쪽을 돌아보거나 열린 길을 막힘으로
        봄), 지도에도 돌아간 벽이 쌓였음 (2026-10-31 실물 시험). 시뮬레이션 스캔은 지연이 없어 안 드러났음.
        """
        odom = self.tf2d('odom', 'base_link')
        if odom is None:
            return
        last, self.last_odom = self.last_odom, odom
        if last is None:
            return
        # 제자리 회전만 돎으로 침: 직진하며 방향을 고치는 것(0.1 rad/s 넘음)까지 세어 직진이 끝날 때마다 "돈 뒤 보정"으로
        # 0.4~1 s씩 섰고(사용자: "이동하면서 중간중간 멈춤"), 직진 중에는 지도도 안 쌓였음 (2026-10-31 실물 10차)
        moved = math.dist(odom[:2], last[:2])
        turn = abs(math.remainder(odom[2] - last[2], 2 * math.pi)) > math.radians(0.3) and moved < 0.002
        if turn:
            self.turned_at = self.now()
        if turn or moved > 0.003:
            self.moved_at = self.now()

    def fix_due(self):
        """True while waiting to stand still for a lattice fix (one every lattice_every_s, at LOOK, GOTO and RETURN steps).

        settle_s 0으로 이동을 쉬지 않고 이어 가 lattice_still_s만큼 멈추는 때가 거의 없었음 → 보정이 한 번도 안 됨
        (2026-10-31 지연 넣은 Gazebo). 실물 odom 방향 오차는 이 보정으로만 잡힘.
        """
        # 마지막 보정 뒤 돈 적이 있을 때만 일부러 섬: 직진만 했으면 방향 오차가 거의 안 쌓이는데 GOTO가 한 칸 갈 때마다
        # 0.4 s씩 서서 칸마다 끊겼음 (2026-10-31 사용자, Gazebo 맵 4). 멈춘 김에 하는 보정(update_fix)은 그대로
        p = self.p
        if p['lattice_gain'] > 0 and self.jump_pending is not None:
            return True  # 큰 방향 보정을 다시 재는 중: 움직이지 않음
        return (p['lattice_gain'] > 0 and self.turned_at >= self.fix_at
                and self.now() - self.fix_at >= p['lattice_every_s'] + p['lattice_still_s'])

    def update_fix(self):
        """While stopped: set the heading from the wall direction, then nudge the position onto the cell boundary lines.

        위치가 실제보다 10~20 cm·4° 어긋나 벽이 칸 경로 띠에 들어와 빈 칸을 막힘으로 보고 먼 길로 돌았음
        (2026-10-31 Gazebo 사용자 맵 4). 규정상 벽·장애물 면은 서로 평행·직각이고 400 mm 칸 경계선 위라, 방향은 벽 방향으로
        한 번에(grid.wall_heading), 위치는 경계선 쪽으로 lattice_gain만큼(grid.lattice_fix). lattice_gain 0이면 끔.
        """
        p = self.p
        if (p['lattice_gain'] <= 0 or self.now() - self.fix_at < p['lattice_period_s']
                or self.now() - self.moved_at < p['lattice_still_s']):  # lattice_still_s 넘게 완전히 멈췄을 때만
            return
        self.fix_at = self.now()
        pose, points = self.pose(), self.points()
        if pose is None or not points:
            return
        # 방향: 벽 방향으로 한 번에 대강(wall_heading, 90° 단위 안에서 절대값) → 그 둘레 ±3°를 칸 경계선 맞춤으로 정밀하게
        # (lattice_fix). 위치: 같은 맞춤의 이동을 lattice_gain만큼
        heading = wall_heading(points)
        coarse = 0.0 if heading is None else math.remainder(-(pose[2] + heading), math.pi / 2)
        delta = lattice_fix(points, (pose[0], pose[1], pose[2] + coarse), p['cell_size'], max_yaw=math.radians(5))
        g = p['lattice_gain']
        dx, dy, da = (0.0, 0.0, coarse) if delta is None else (g * delta[0], g * delta[1], coarse + delta[2])
        # 한 번에 heading_jump_deg 넘게 바뀌면 다음 측정이 5° 안으로 같을 때만 받음: 보정 사이 odom 방향 오차는 회전 몇 번의
        # 몇 %라 그렇게 클 수 없음. 한 번 잘못 읽은 +35°를 받아 6분 내내 위치를 잃고 큐브 자리를 집으로 알았음 (실물 3차)
        # 확인할 때까지 서 있음(fix_due): 보류만 하고 움직여 회전 때 미끄러진 -35°를 안은 채 직진해 오른쪽 벽을 뚫고 나갔음
        # (2026-10-31 실물 14차). 세 번 재도 서로 안 맞으면 버리고 감
        if abs(math.degrees(da)) > p['heading_jump_deg']:
            pending, tries = self.jump_pending or (None, 0)
            if pending is None or abs(math.degrees(math.remainder(da - pending, math.pi / 2))) > 5:
                self.jump_pending = (da, tries + 1) if tries < 2 else None
                self.get_logger().warn(f'방향 보정 {math.degrees(da):+.1f}°는 너무 커서 서서 다시 잽니다 ({tries + 1}번째).')
                return
            self.get_logger().warn(f'방향 보정 {math.degrees(da):+.1f}°: 다시 재도 같아 받아들임 (회전 중 미끄러짐)')
        self.jump_pending = None
        delta = (dx / g, dy / g, da)
        # 지금 자리를 중심으로 da 돌리고 (dx, dy) 옮기는 것을 지금 보정 뒤에 덧붙임
        a, tx, ty = self.fix
        c, s = math.cos(da), math.sin(da)
        px, py = pose[0], pose[1]
        # 이동 상한(odom과 15 cm)은 뺌: odom 방향이 몇 도만 틀어져도 원점에서 먼 곳에서는 정상 보정이 15 cm를 넘어
        # 매번 버렸고 위치가 어긋나 복귀가 막혔음 (2026-10-31 실물 14:18·15:11, Gazebo m4 400 s 제자리). 칸이 400 mm가
        # 아닌 연습장에서는 한 칸씩 밀릴 수 있음 (12:28 판) — 대회장은 규정상 400 mm
        tx, ty = c * (tx - px) - s * (ty - py) + px + dx, s * (tx - px) + c * (ty - py) + py + dy
        old, self.fix = self.fix, (a + da, tx, ty)
        if math.hypot(tx - old[1], ty - old[2]) > 0.02 or abs(da) > math.radians(1):
            self.get_logger().info(f'격자 보정: {100 * tx:+.0f}, {100 * ty:+.0f} cm, {math.degrees(a + da):+.1f}° '
                                   f'(벽 점 기준 이번 {100 * delta[0]:+.0f}, {100 * delta[1]:+.0f} cm, '
                                   f'{math.degrees(delta[2]):+.1f}°)')

    def drive_fix(self, now, m):
        """While driving straight: heading only, from the wall direction, with the odometry at the scan's own stamp.

        실물 odom은 직진 1 m에 방향이 4~5°씩 같은 쪽으로 틀어짐 (2026-10-31 실물 7차: 4칸 직진 뒤 -9.3°, 복귀 5칸 직진
        -7.8°, (9,6)에서 벽에 닿음). 멈췄을 때만 보정하면 긴 직진 동안 그대로 쌓임. 스캔은 0.2 s 늦게 오므로 그 시각의 odom과
        짝지음. 위치는 그 사이 2~3 cm 가므로 멈췄을 때만(update_fix). 돈 직후 lattice_still_s는 번진 스캔이라 안 씀
        """
        p, scan = self.p, self.scan
        if (p['drive_fix_gain'] <= 0 or scan is None or now - self.drive_fix_at < p['drive_fix_period_s']
                or now - max(self.move[1], self.turned_at) < p['lattice_still_s']):
            return False
        self.drive_fix_at = now
        points, pose, at_scan = self.points(), self.pose(), self.scan_pose()
        heading = wall_heading(points) if points and pose and at_scan else None
        if heading is None:
            return False
        a, tx, ty = self.fix
        da = math.remainder(-(at_scan[2] + heading), math.pi / 2)
        if abs(da) > math.radians(p['drive_fix_max_deg']):
            return False  # 잘못 읽었거나 크게 틀어짐: 멈췄을 때 update_fix가 확인해서 고침
        da *= p['drive_fix_gain']
        c, s, (px, py) = math.cos(da), math.sin(da), pose[:2]
        a, tx, ty = a + da, c * (tx - px) - s * (ty - py) + px, s * (tx - px) + c * (ty - py) + py
        # 옆(진행 방향과 직각) 위치도 칸 경계선에 맞춤: 큐브를 쥐고 2.45 m 직진하는 동안 옆으로 13 cm 밀려 왼쪽 벽을 긁었음
        # (2026-10-31 실물 19:49 판 RETURN F2448, 끝난 뒤 보정 "이번 -13 cm"). 스캔 지연은 진행 방향 위치만 틀리게 하므로 직각
        # 성분만 씀. 한 번에 lattice_gain만큼, 최대 drive_fix_shift_m
        # 몸 방향과 실제 진행 방향의 차(self.crab)만큼 옆으로 간 것을 미리 넣고, 남는 옆 보정으로 그 각을 배움 (적분 항).
        # 실물은 몸 방향을 벽에 맞춰도 3.5° 왼쪽으로 비스듬히 가 2 m에 12 cm 밀렸음 (2026-10-31: 10-11 녹화 +3.1·+3.8°,
        # 2 m 직진 시험, 10회차 bag +3.59°, 사용자: 캐스터 쪽 의심). 바닥·속도에 따라 바뀌어도 따라가게 상수 대신 배움
        nx, ny = -math.sin(m['h']), math.cos(m['h'])
        last = m.get('fix_pt')
        ds = 0.0 if last is None else max(0.0, (pose[0] - last[0]) * ny - (pose[1] - last[1]) * nx)  # 진행 방향 성분
        m['fix_pt'] = pose[:2]
        pred = self.crab * ds
        tx, ty = tx + pred * nx, ty + pred * ny
        delta = lattice_fix(points, (at_scan[0], at_scan[1], at_scan[2] + da), p['cell_size'], max_yaw=0.0)
        if delta is not None:
            side = p['lattice_gain'] * (delta[0] * nx + delta[1] * ny - pred)
            side = max(-p['drive_fix_shift_m'], min(p['drive_fix_shift_m'], side))
            tx, ty = tx + side * nx, ty + side * ny
            m['side_fix'] = m.get('side_fix', 0.0) + side
            self.crab_sum = [self.crab_sum[0] + side, self.crab_sum[1] + ds]
            if self.crab_sum[1] >= p['crab_min_m']:
                lim = math.radians(p['crab_max_deg'])
                step = max(-math.radians(1.0), min(math.radians(1.0), p['crab_gain'] * self.crab_sum[0] / self.crab_sum[1]))
                self.crab = max(-lim, min(lim, self.crab + step))  # 한 번에 1° 넘게 안 바꿈 (실물 잡음으로 0.6~5.3° 출렁임)
                self.crab_sum = [0.0, 0.0]
        self.fix = (a, tx, ty)
        m['yaw_fix'] = m.get('yaw_fix', 0.0) + da
        return True

    def front_extent(self):
        return self.p['front_extent_holding'] if self.holding else self.p['front_extent']

    def front_clear(self, points, distance):
        """corridor_clear for the body plus the gripper (front_extent over the middle ±front_half_width)."""
        p = self.p
        return (corridor_clear(points, distance, p['body_radius'], p['body_radius'])
                and corridor_clear(points, distance, p['front_half_width'], self.front_extent()))

    def scan_pose(self):
        """Pose (cell lattice) when the latest scan was taken: the scan arrives about 0.2 s late. None without TF."""
        if self.scan is None:
            return None
        try:
            t = self.tf.lookup_transform('odom', 'base_link', Time.from_msg(self.scan.header.stamp)).transform
        except TransformException:
            return None
        a, tx, ty = self.fix
        return compose((tx, ty, a), (t.translation.x, t.translation.y, 2 * math.atan2(t.rotation.z, t.rotation.w)))

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
        # ydlidar 드라이버는 range_min~range_max 밖 값을 그대로 둠 (ydlidar_g4.yaml 주석: 소비자가 버려야 함)
        lo, hi = self.scan.range_min, self.scan.range_max
        return [(x, y) for x, y in scan_points(self.scan.ranges, self.scan.angle_min, self.scan.angle_increment, yaw,
                                               self.p['edge_jump_m'])
                if lo <= math.hypot(x, y) <= hi]

    def gripper_at(self, angles):
        t = self.telemetry
        return bool(t) and [t[SERVO_L], t[SERVO_R]] == list(angles)

    def states(self):
        """{cell: FREE/UNKNOWN/BLOCKED}, computed once per tick (cell_states). 한 번 판단에 여러 번 다시 계산해 Pi에서 이동과
        이동 사이에 0.7~1.6 s씩 섰음 (2026-10-31 실물 14차 텔레메트리: 103 s 중 18번·16 s)"""
        if self.states_at != self.ticks:
            self.states_at, self.states_cache = self.ticks, self.cell_states()
        return self.states_cache

    def cell_states(self):
        """{cell: FREE/UNKNOWN/BLOCKED} for every cell the own map has seen, plus max_cells cells around it.

        경기장 크기는 위 한도(arena_cells, 규정 9×9)만 씀 — 실제 크기는 몰라도 됨 (2026-10-30 사용자 요청). 출발 칸·지금 칸
        둘레는 지도 밖이어도 넣습니다. 미탐색 칸은 지나갈 수 있다고 보므로 범위를 지도에 묶어야 BFS가 끝납니다.
        """
        p, size, res, (x0, y0) = self.p, self.p['cell_size'], self.p['map_resolution'], self.map_origin
        grid = occupancy(self.hits, self.misses, self.p['map_min_misses'])
        rows, cols = np.nonzero(grid >= 0)
        anchors = [(0, 0), self.here()]
        if rows.size:
            anchors += [cell_of(x0 + cols.min() * res, y0 + rows.min() * res, size),
                        cell_of(x0 + (cols.max() + 1) * res, y0 + (rows.max() + 1) * res, size)]
        m = p['max_cells']  # 지도 가장자리에서도 한 번에 max_cells칸 직진할 자리
        i0, i1 = min(a[0] for a in anchors) - m, max(a[0] for a in anchors) + m
        j0, j1 = min(a[1] for a in anchors) - m, max(a[1] for a in anchors) + m
        cells = [(i, j) for i in range(i0, i1 + 1) for j in range(j0, j1 + 1)]
        states = seen_faces(grid, res, (x0, y0), {c: cell_state(grid, res, (x0, y0), c, size) for c in cells}, size)
        states[(0, 0)] = FREE  # 출발 칸은 로봇이 서 있던 곳 (모서리 벽 두 면이 칸 안으로 보여 집이 막힘이 된 적 있음)
        # 경기장은 출발 모서리(뒤·오른쪽이 바깥벽, home_walls)에서 앞(+i)·왼쪽(+j)으로 최대 arena_cells칸 (규정 9×9).
        # 더 작은 연습장(5×9)은 이 안이라 그대로. 바깥벽 너머가 반사·흐린 벽으로 빈 칸이 되어 그 칸을 보러 다니며 카메라로
        # 다 본 칸을 다시 지났음 (2026-10-31 실물 16:30 판 10행·10열 목표, 사용자 "9×9로 제한")
        n = p['arena_cells']
        for c in states:
            if not (0 <= c[0] < n and 0 <= c[1] < n):
                states[c] = BLOCKED
        self.walls = wall_faces(grid, res, (x0, y0), states, size)  # 빈 칸 사이 바깥벽: 카메라 시선이 못 지나감
        self.entries = open_frontier(grid, res, (x0, y0), states, size)  # 빔이 지나간 새 길 입구 (next_goal)
        return walled_off(states, [(0, 0), self.here()], self.walls)  # 벽 너머(경기장 밖) 칸은 막힘

    def here(self):
        pose = self.pose()
        return cell_of(pose[0], pose[1], self.p['cell_size']) if pose is not None else (0, 0)

    def passable(self, states):
        # 막힘 표시는 block_ttl_s 뒤 다시 시도: 움직이는 상대 로봇이 잠깐 막은 칸(집 앞 등)이 영원히 막혀 복귀 길이 없어짐
        # (규정 9.1: 두 팀이 같이 뜀). 거듭 막힌 칸(block_confirm)은 그 세 배 뒤
        now, ttl = self.now(), self.p['block_ttl_s']
        for c in [c for c in self.blocked
                  if now - self.block_at.get(c, now) > ttl * (3 if self.block_hits[c] >= self.p['block_confirm'] else 1)]:
            self.blocked.discard(c)
        stalls = {c for c, at in self.stall_at.items() if now - at < 3 * ttl}
        return lambda c: c in states and c not in self.blocked and c not in stalls and states[c] != BLOCKED

    # ── 출력 ──
    def block(self, cell):
        self.blocked.add(cell)
        self.block_at[cell] = self.now()
        self.plan = []  # 막혔으니 경로를 새로
        self.block_hits[cell] += 1

    def unblock(self):
        """Forget blocked cells except those blocked block_confirm times or more; True if any were forgotten.

        라이다가 바로 옆에서 거듭 막혔다고 본 칸은 진짜 장애물: 지도에서 면을 못 잡아 지나갈 수 있다고 나오는 장애물 칸을
        비울 때마다 다시 경로에 넣어 같은 칸 앞에서 220 s를 멈춰 있었음 (2026-10-31 Gazebo 사용자 맵 2, 복귀 중 (2,4)).
        """
        # 방금(10 s 안) 라이다로 막힌 칸도 남김: 비우자마자 같은 직선을 다시 골라 막히기를 두 사이클 되풀이했음
        # (2026-10-31 실물 15회차 복귀 (5,2)→(4,2), 집 앞 (0,1)→(0,0))
        now = self.now()
        keep = {c for c in self.blocked if self.block_hits[c] >= self.p['block_confirm']
                or now - self.block_at.get(c, -math.inf) < 10.0}
        forgotten = self.blocked - keep
        self.blocked = keep
        return bool(forgotten)

    def drive(self, linear=0.0, angular=0.0):
        if not rclpy.ok():
            return  # 종료 중(SIGINT 뒤) 타이머가 한 번 더 돌면 발행이 RCLError로 노드를 exit 1로 끝냈음 (실물 16:30 판)
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.cmd_vel.publish(msg)

    def request_arm(self):
        if self.now() - self.arm_at >= self.p['arm_retry_s']:
            self.arm_at = self.now()
            self.command.publish(String(data='ARM'))

    def board(self, text, target_cell=None, point=None, stop=False):
        """A move (F mm / T deg, run here as continuous velocity until done) or a board command (7, 8, PF0, PF1).

        F는 point(map m, 없으면 지금 방향으로 그 거리)까지 방향을 고치며 직진, T는 그 각도만큼 제자리 회전.
        """
        if text[0] not in 'FTBC':
            return self.command.publish(String(data=text))
        pose = self.pose() or (0.0, 0.0, 0.0)
        if text[0] == 'F':
            if point is None:
                d = int(text[1:]) / 1000
                point = (pose[0] + d * math.cos(pose[2]), pose[1] + d * math.sin(pose[2]))
            # 시간 제한은 거리에 맞춤: 복귀는 한 번에 여러 칸 직진 (drive_path)
            self.motion = {'kind': 'F', 'point': point, 'stop': stop,
                           'long': math.dist(pose[:2], point) >= self.p['fast_min_m'],
                           'h': math.atan2(point[1] - pose[1], point[0] - pose[0]), 'sight': self.now(),
                           'timeout': self.p['move_timeout_s'] + math.dist(pose[:2], point) / self.p['drive_speed']}
        elif text[0] == 'C':  # 파지 직전 조금 더 들어감 (보드 명령 아님, PF0로 보드 정면 PSD 정지를 끈 뒤)
            self.motion = {'kind': 'C', 'start': pose[:2], 'dist': int(text[1:]) / 1000, 't0': self.now()}
        elif text[0] == 'B':  # 미션 안에서만 쓰는 짧은 후진 (보드 명령 아님, cmd_vel 음수)
            self.motion = {'kind': 'B', 'start': pose[:2], 'dist': int(text[1:]) / 1000}
        else:
            deg = int(text[2:]) * (1 if text[1] == 'L' else -1)
            # 크게 돌 때 집게가 쓸고 지나갈 쪽에 벽이 있고 반대로 돌면 비어 있으면 반대로 (집게가 통로 옆벽보다 김, 13차)
            points = self.points()
            if points and abs(deg) >= 60:
                # 집게 모서리(앞끝·옆 front_half_width)까지 + 3 cm: 앞끝 + 2 cm(30 cm)는 모서리(30.5 cm)보다 짧아 32 cm 통로에서
                # 돌며 집게로 장애물을 밀었음 (2026-10-31 실물 16:04 판 171 s TL77)
                reach = math.hypot(self.front_extent(), self.p['front_half_width']) + 0.03
                other = deg - math.copysign(360, deg)
                hits = sweep_hits(points, math.radians(deg), reach)
                if hits and abs(other) - abs(deg) > 60 and self.now() - self.backed_at > 5.0:
                    # 반대로 돌면 60° 넘게 더 돎: 집게가 안 닿게 되는 가장 짧은 거리(3~8 cm)만 물러난 뒤 짧은 쪽으로 (다음 판단에서
                    # 다시 잼). TR92 → TL268, TR174 → TL186으로 돌다 큐브를 화면에서 놓쳤음 (Gazebo 18l m4, seed 821). 400 mm 칸이라
                    # 고정 8 cm는 많음 (사용자) → 필요한 만큼만
                    behind = [(-x, -y) for x, y in points]
                    back = next((d for d in (0.03, 0.04, 0.05, 0.06, 0.08)
                                 if not sweep_hits([(x + d, y) for x, y in points], math.radians(deg), reach)), None)
                    if (back is not None and self.psd_peak()[REAR] < self.p['psd_threshold']
                            and corridor_clear(behind, back, self.p['front_half_width'], self.p['body_radius'])):
                        self.backed_at = self.now()
                        self.get_logger().info(f'{text}: 집게가 그쪽 벽에 닿음 → {100 * back:.0f} cm 물러난 뒤 다시 돎')
                        return self.board(f'B{round(back * 1000)}')
                if hits and not sweep_hits(points, math.radians(other), reach):
                    near = ' '.join(f'({100 * x:.0f},{100 * y:.0f})' for x, y in sorted(hits, key=lambda q: math.hypot(*q))[:4])
                    self.get_logger().info(f'{text}: 집게가 그쪽 벽에 닿아 반대로 {other:+.0f}° 돎 (닿는 점 cm {near}, '
                                           f'뒤 PSD {self.psd_peak()[REAR]})')
                    deg = other
                    text = f'T{"L" if deg > 0 else "R"}{abs(round(deg))}'  # 아래 명령 로그도 실제로 도는 쪽으로
            # 회전마다 turn_extra_deg 더: 실물 90° 회전이 평균 0.8° 모자람 (2026-10-31 회전 시험, 기어 유격·정지 미끄럼)
            self.motion = {'kind': 'T', 'dir': math.copysign(1.0, deg),
                           'total': math.radians(abs(deg) + self.p['turn_extra_deg']),
                           'turned': 0.0, 'last': pose[2], 'progress': (self.now(), 0.0)}
        self.move, self.result = (text, self.now(), target_cell), None
        self.get_logger().info(f'{self.state}: {text}')

    def ready(self):
        """No move in flight and the pose has had settle_s to catch up."""
        return self.move is None and self.now() - self.idle_since >= self.p['settle_s']

    # ── 상태머신 ──
    def enter(self, state):
        self.get_logger().info(f'{self.state} → {state} ({self.now() - self.start:.0f} s)')
        self.state, self.since = state, self.now()
        if state not in ('GOTO', 'CRUISE'):
            self.sight_turn = None  # 확인돼 접근으로 갔거나 둘러보기로 바뀜: 남은 확인 회전은 버림
        getattr(self, 'start_' + state.lower(), lambda: None)()

    def tick(self):
        self.ticks += 1
        self.track_motion()
        if not self.track or self.now() - self.track[-1][0] >= 0.5:
            pose = self.pose()
            if pose is not None:
                self.track.append((self.now(),) + tuple(pose) + (self.state,))
        if self.state in ('LOOK', 'GOTO', 'CRUISE') and self.now() - self.camera_at >= 0.5:
            self.update_camera()  # 2 Hz: 칸 상태 계산이 무거움
        if self.holding and self.state == 'RETURN' and self.hold_lost():
            return
        if self.move is not None:
            self.watch_move()
            if self.move is not None:
                return
        self.update_fix()  # 멈춰 있을 때만 (돌거나 가는 중에는 스캔과 자세 시각이 어긋남)
        if self.state in ('LOOK', 'GOTO', 'CRUISE'):
            self.passed.add(self.here())  # 큐브를 찾기 전 탐색 중에 들어간 칸
        if self.now() < self.wait_until:
            return self.drive()  # 앞을 막은 상대 로봇이 비키기를 기다림
        getattr(self, 'do_' + self.state.lower())()
        if self.move is None:
            self.drive()  # 할 일이 없을 때만 섬 (encoder_odom이 [V0,0] → 보드 정지·무장 유지)

    def hold_lost(self):
        """Carrying, but the front PSD stayed low for hold_lost_s: the cube is gone. Release and search again.

        규정 13절: 상대가 확보한 관심 대상도 탈취할 수 있음. 놓친 걸 모르면 빈 집게로 집에 가 "목표물 보유"로 끝남
        (CLAUDE.md: 실제로 쥐고 있는지 따로 확인). 실물 쥔 정면 PSD 약 560~680 (2026-10-31 9회차). 빈 채로 닫힌 집게 값은 아직
        안 잼 — 그 값이 hold_lost_psd보다 높으면 이 검사는 못 알아챔 (오탐은 없음)
        """
        p, now = self.p, self.now()
        front = (self.telemetry or [0] * 12)[FRONT]
        if p['hold_lost_psd'] <= 0 or front >= p['hold_lost_psd']:
            self.lost_since = None
            return False
        if self.lost_since is None:
            self.lost_since = now
        if now - self.lost_since < p['hold_lost_s']:
            return False
        self.get_logger().warn(f'운반 중 정면 PSD {front}: 큐브를 놓쳤습니다(탈취). 집게를 열고 다시 찾습니다.')
        self.drive()
        if self.move is not None:
            self.result, self.move = ('done', now), None
        self.idle_since = now
        self.holding, self.lost_since, self.trail = False, None, []
        if now - self.since < 6.0:
            # 쥔 직후(복귀 시작 6 s 안) 놓침 = 처음부터 못 쥔 것: 큐브는 그대로 (5,5) → 출입구 방식(approach_start_cell) 유지.
            # 집게 각도만으로 "쥐었음"을 세워 두면 예전 카메라 추적 접근으로 돌아갔음 (2026-11-01 실물 23:14 판 164 s)
            self.ever_held = False
        # 큐브는 이제 어디든 있을 수 있음: 본 칸 기록을 비워 가까운 데부터 다시 봄, (5,5)도 다시 봄. 그대로 두면 이미 본 칸을
        # 건너뛰고 먼 미탐색 칸부터 가 다시 쥐기까지 232 s 걸렸음 (Gazebo 18h 탈취 시험: 큐브는 (5,5)로 돌아가 있었음)
        self.camera_seen.clear()
        self.passed.clear()
        self.recheck_start, self.start_looked = True, False
        self.board('PF1')  # 운반 모드 해제 먼저 (정면 PSD 정지 다시 켬)
        self.board('7')
        self.enter('LOOK')
        return True

    def watch_move(self):
        text, sent, _ = self.move
        now = self.now()
        # 큐브를 잠시 무시하는 중(target_hold: 큐브 쪽 길 없음)에는 보여도 끊지 않음: 끊고 나면 target_seen이 'no'라 같은 회전을
        # 다시 내고 또 끊겨, 몇 도씩만 돌며 TL27·TL27·TL27·TL26…을 되풀이했음 (Gazebo 18l l5 99~109 s, 자세 그대로 -115°)
        hold = now < self.target_hold_until
        if self.result is None:
            if self.state in ('LOOK', 'GOTO', 'CRUISE') and not hold and self.target_confirmed(after=sent):
                # 움직이는 중 큐브 확인: 바로 멈추고 접근으로 (멈춘 뒤 검출을 다시 기다리지 않게 기준 시각을 당김)
                self.drive()
                self.result, self.idle_since = ('done', now), sent
                self.move = None
                return
            if (self.state in ('GOTO', 'CRUISE') and text[0] == 'F' and not hold and self.seen(0.3, sent)
                    and now - self.sight_stop_at > 5.0):
                # 직진 중 빨강이 한 번만 보여도 멈춰서 확인 (멈추면 target_seen이 더 기다려 확정): 계속 가면 화면 밖으로
                # 지나가 버림 (2026-10-31 실물 17:45 판 173 s: (7,4)에서 +31°로 보고도 그대로 가 돌아서 놓침)
                self.get_logger().info('직진 중 빨강이 보임: 멈춰서 확인합니다.')
                self.drive()
                self.sight_stop_at = now
                self.sight_turn = -math.degrees(self.seen(0.3, sent)[-1])  # 검출 오차 +오른쪽 → 회전 +왼쪽
                self.result, self.move = ('done', now), None
                self.idle_since = now
                return
            if text[0] == 'F' and not self.holding and self.front_blocked(now):
                # 보드도 60 ms 유지로 멈추지만 텔레메트리로 먼저 보이면 여기서 세움. 운반 중에는 안 봄
                self.drive()
                self.result = ('# ABORT PSD_FRONT (Pi)', now)
                pose, m = self.pose(), self.motion
                if self.state != 'APPROACH' and pose is not None and m['kind'] == 'F' and (
                        (m['point'][0] - pose[0]) * math.cos(m['h']) + (m['point'][1] - pose[1]) * math.sin(m['h'])
                        < 0.15):
                    # 목표 중심 15 cm 앞에서 벽: 더 못 갈 만큼 왔음 → 도착. 막힘으로 보면 같은 직진을 되풀이해 "정지 9회"로
                    # 미션이 끝났음 (2026-10-30 Gazebo 시드 1: SLAM이 10 cm 어긋나 칸 중심이 벽 너머). 접근 중은 파지 판단으로
                    self.result = ('done', now)
            elif now - sent > self.motion.get('timeout', self.p['move_timeout_s']):
                self.get_logger().error(f'{text}: {now - sent:.0f} s 안에 끝나지 않음, 정지합니다.')
                self.drive()
                self.result = ('# ABORT PI_TIMEOUT', now - STOP_SETTLE_S)
            else:
                self.run_motion(now)
            if self.result is None:
                return
        event, at = self.result
        if event != 'done' and now - at < STOP_SETTLE_S:
            return  # 정지 줄 뒤에 오는 텔레메트리(높은 PSD 값)까지 받고 판단
        move, self.move, self.result, self.idle_since = self.move, None, None, now
        if event != 'done':
            self.on_stop(event, move)

    def run_motion(self, now):
        """One velocity command for the move in flight; sets self.result when it is done."""
        p, m, pose = self.p, self.motion, self.pose()
        if pose is None:
            return self.drive()
        if not self.armed:  # ABORT·STOP 뒤: 무장부터 (서보가 도는 중이면 보드가 거절 → 다시)
            self.drive()
            return self.request_arm()
        if m['kind'] == 'C':
            # 정해진 거리 또는 1 s(큐브·벽에 밀려 못 가도)에서 끝: 실패로 보지 않고 바로 집게를 닫음
            # 천천히(grasp_push_speed): 접근 속도 0.08 m/s로 가면 바퀴 감속 제한(0.1 m/s²)으로 3 cm를 더 미끄러져 큐브를
            # 밀어내 빈 집게를 세 번 닫았음 (Gazebo 18y m4 990 s). 0.03 m/s면 감속 중 0.5 cm
            # 정면 PSD가 grasp_psd(600)에 닿으면 바로 닫음: 보드는 550에서 서지만(펌웨어) 집는 기준은 600 (2026-11-01 사용자)
            front = (self.telemetry or [0] * 12)[FRONT]
            if front >= p['grasp_psd'] or math.dist(pose[:2], m['start']) >= m['dist'] or now - m['t0'] > 1.5:
                self.result = ('done', now)
                self.get_logger().info(f'집게 닫기 전 더 들어감 {100 * math.dist(pose[:2], m["start"]):.1f} cm, 정면 PSD {front}')
                return self.drive()
            return self.drive(p['grasp_push_speed'], 0.0)
        if m['kind'] == 'B':
            # 뒤에 라이다 점이 몸 3 cm 안이면 거기서 멈춤: 큐브 앞에서 비스듬히 40 cm 후진하다 뒤 장애물에 부딪혀 밀었음
            # (2026-11-02 실물 00:00 판 203 s, 사용자 "장애물 사선 = 밀어서 생긴 현상")
            behind = [(-x, -y) for x, y in (self.points() or [])]
            if math.dist(pose[:2], m['start']) >= m['dist'] or (
                    behind and not corridor_clear(behind, 0.03, p['body_radius'], p['body_radius'])):
                self.result = ('done', now)
                return self.drive()
            return self.drive(-p['approach_speed'], 0.0)
        if m['kind'] == 'T':
            m['turned'] += abs(math.remainder(pose[2] - m['last'], 2 * math.pi))
            m['last'] = pose[2]
            left = m['total'] - m['turned']
            if left <= math.radians(p['turn_tol_deg']):
                self.result = ('done', now)
                return
            # 몸이 벽 모서리에 걸려 못 돎: 3 s 동안 2° 안 돌면 세움. 20 s 시간 초과를 같은 자리에서 되풀이했음 (2026-10-31
            # 실물 조건 Gazebo 맵 3: 뒤 PSD 857). on_stop이 앞·뒤 빈 쪽으로 조금 빠져나옴
            if m['turned'] - m['progress'][1] >= math.radians(2):
                m['progress'] = (now, m['turned'])
            elif now - m['progress'][0] > 3.0:
                self.drive()
                self.result = ('# ABORT PI_STALL', now)
                return
            return self.drive(0.0, m['dir'] * max(p['min_turn_speed'], min(p['turn_speed'], 2.0 * left)))
        if self.drive_fix(now, m):
            pose = self.pose()
        if self.state == 'APPROACH' and m.get('final'):
            self.follow_final(m, pose)
        if self.state == 'APPROACH' and m.get('cube'):
            # 큐브를 향한 직진만 카메라를 따라감: 입구 칸으로 가는 길 직진까지 검출마다 큐브 쪽으로 꺾어 비스듬히 가다 모서리에
            # 걸리고 왔다 갔다 했음 (2026-11-01 실물 22:30 판, 사용자 "이동 중에도 계속 큐브 감지되니까 정신 못 차리고")
            self.follow_cube(m, pose)
        dx, dy = m['point'][0] - pose[0], m['point'][1] - pose[1]
        left = dx * math.cos(m['h']) + dy * math.sin(m['h'])  # 진행 방향으로 남은 거리
        if left <= p['arrive_m']:
            self.result = ('done', now)  # 멈추지 않음: 다음 이동이 바로 이어 감
            if abs(m.get('yaw_fix', 0.0)) > math.radians(0.5) or abs(m.get('side_fix', 0.0)) > 0.01:
                self.get_logger().info(f'직진 중 방향 보정 합계 {math.degrees(m.get("yaw_fix", 0.0)):+.1f}°, '
                                       f'옆 {100 * m.get("side_fix", 0.0):+.0f} cm, 쏠림 {math.degrees(self.crab):+.1f}°')
            return
        # 몸이 실제로 쓸고 지나가는 곳 = 로봇이 향한 쪽 띠 (전진은 목표와 0.5 rad 안일 때만). 목표점 쪽 띠로 보면 제자리에서
        # 돌아도 띠가 그대로라 모서리를 비켜 돌기가 소용없었고 20 s 시간 초과를 냈음 (2026-10-30 Gazebo 사용자 미로 2차)
        points = self.points()
        # 큐브 바로 앞(따라오던 중 정면 PSD 가까움)에서는 라이다로 세우지 않음: 큐브는 라이다보다 낮아 안 보이고, 그 뒤 벽에
        # 걸려 PSD 402에서 섰음 (2026-10-30 실물). 여기서는 정면 PSD 정지가 큐브와 벽을 둘 다 막음
        near_cube = self.state == 'APPROACH' and self.cube_at_gripper()
        # 몸 옆(앞 6 cm 미만) 점은 정지 판정에서 빼고 아래 wall_push로 벽에서 떨어짐
        front = [] if points is None or near_cube else [q for q in points if q[0] >= 0.06]
        # 몸 둘레 + 집게(가운데 ±front_half_width만 front_extent까지). 앞끝 호를 몸 폭 끝까지 넓혔더니 벽에 몇 cm 붙어 달릴 때
        # 옆벽(앞 13~15 cm, 옆 16 cm)을 앞 막힘으로 보고 칸을 막아 "갈 곳 없음"을 되풀이했음 (2026-10-31 실물 11차)
        # 큐브를 쥐면 닫힌 집게가 라이다 높이(앞 19 cm, 옆 11 cm)에 들어옴: 앞끝을 그만큼(front_extent_holding) 늘려 자기 집게는
        # 몸 안(inside)으로 빼고 벽은 집게 끝 앞에서 섬. 집게 띠를 끄면 집게·큐브가 벽에 먼저 닿았고(13차 복귀 충돌),
        # 그대로 두면 자기 집게를 막힘으로 보고 같은 직진을 끝없이 되풀이했음 (12차)
        # 쥐면 닫힌 집게 손가락(앞 18~19 cm, 옆 10~12 cm)이 라이다에 잡힘: 그 자리만 뺌. 앞 28 cm·옆 12 cm 안을 통째로
        # 빼 정면 15 cm 앞 벽을 못 보고 부딪혔음 (2026-10-31 실물 16차 225 s). 가운데 띠는 늘 봄
        if self.holding:
            front = [(x, y) for x, y in front if not (0.08 <= abs(y) <= 0.14 and x <= 0.23)]
        # 앞이 길게 비고 남은 거리도 길면 빠르게(fast_speed). 빠를수록 스캔 지연(scan_latency_s) 동안 더 가므로 정지 띠를
        # 그만큼 넓힘 (2026-10-31 사용자: 펌웨어 PWM 상한 255, "속도는 미션에서 조절")
        clear = min(free_distance(front, p['body_radius'], p['body_radius'], 1.0),
                    free_distance(front, p['front_half_width'], self.front_extent(), 1.0))
        # 두 칸 넘게 가는 직진(long)에서 마지막 한 칸 전까지만: 한 칸짜리·접근·남은 한 칸은 보통 속도
        # 접근 속도(approach_speed)는 큐브로 곧게 들어가는 직진(final·cube)만. 출입구까지 돌아가는 길은 보통 이동: APPROACH
        # 전체를 0.08 m/s로 가 출입구까지 75 cm에 13 s를 썼음 (2026-11-02 실물 01:13 판 159~172 s, 사용자 "유독 느린데")
        toward = self.state == 'APPROACH' and (m.get('final') or m.get('cube'))
        fast = (not toward and m.get('long') and left > p['cell_size']
                and clear > p['fast_clear_m'])
        within = 0.02 + (p['scan_latency_s'] * (p['fast_speed'] - p['drive_speed']) if fast else 0.0)
        blockers = (blocking_points(front, p['body_radius'], p['body_radius'], within, inside=0.01)
                    + blocking_points(front, p['front_half_width'], self.front_extent(), within, inside=0.01))
        # 몸이 지나갈 폭(passage_min_m)의 길: 옆 모서리는 막힘이 아니라 가운데로 비켜 지나갈 것 (사용자: 모든 길은 지나가게
        # 되어 있으니 길 가운데에만 있으면 됨). 2026-11-01 실물 22:00 판 154 s: 오른쪽 8 cm 모서리를 막힘으로 보고 그 입구를
        # 버려 큐브 앞에서 164 s를 돌았음
        gap = None if not front or near_cube else passage(front)
        wide = gap is not None and gap[1] >= p['passage_min_m']
        if len(blockers) >= 3:  # 잡음 두 점까지는 무시 (free_distance와 같음)
            # 목표 중심 10 cm(앞끝이 길면 앞끝 - 15 cm) 앞: 더 못 갈 만큼 왔음 → 도착. 막다른 칸·집 칸은 중심 20 cm 뒤가 벽이라
            # 앞끝이 길수록 일찍 섬 (2026-10-31 사용자: 앞 +4 cm → 쥐면 중심 10 cm 앞에서 섬)
            if left < max(0.10, self.front_extent() - 0.15):
                self.drive()
                self.result = ('done', now)
                return
            side = sum(y for _, y in blockers) / len(blockers)
            # 띠 앞끝(22~26 cm)의 모서리는 옆 6 cm까지도 비켜 돎: 실물 오른쪽 앞 (23, -9) cm 벽 모서리에 스캔 10%에서 걸려
            # 목표 0.4 m 앞에서 섰음 (2026-10-31 실물 13회차 bag 4059 스캔 분석, Pi 모니터 세션)
            near_y = min(abs(y) for _, y in blockers)
            if ((near_y >= 0.08 or (near_y >= 0.05 and min(x for x, _ in blockers) >= 0.2))
                    and now - m.setdefault('pivot', now) < 3.0):
                # 옆쪽 모서리만 걸림: 서지 않고 반대쪽으로 제자리 회전 → 모서리가 띠에서 빠지면 그대로 출발. 서면 같은 자리에서
                # 같은 직진을 1초마다 60번 넘게 되풀이했음 (2026-10-30 Gazebo 사용자 미로: 장애물 모서리 앞 12 cm).
                # 이동 한 번에 처음 비켜 돈 때부터 3 s까지만 (비키기 ↔ 목표 쪽 되돌기 반복 방지), 넘으면 정지
                return self.drive(0.0, -math.copysign(p['min_turn_speed'], side))
            self.drive()
            if wide and near_y >= 0.04:
                # 비켜 돌아도 옆 모서리가 남음: 길은 몸이 지나갈 폭이라 막힘으로 두지 않고 물러나 다시 (on_stop LIDAR_SIDE).
                # 먼저 물러나면 몸 바로 옆 벽(앞 6 cm)에서도 물러나기만 22번 되풀이했음 (Gazebo 19a l5) → 비켜 돌기 다음에
                self.result = ('# ABORT LIDAR_SIDE', now)
                self.get_logger().info(f'라이다: 옆 모서리 ({100 * blockers[0][0]:.0f},{100 * blockers[0][1]:.0f}) cm, 길 폭 '
                                       f'{100 * gap[1]:.0f} cm → 막힘 아님, 물러나 길 가운데({100 * gap[0]:+.0f} cm)로 다시')
                return
            self.result = ('# ABORT LIDAR', now)  # 몸 앞 띠에 라이다 점: 그 칸은 막힘으로
            near = sorted(blockers)[:3]
            self.get_logger().info(f'라이다 정지: 남은 {left:.2f} m, 띠 안 점 (cm) '
                                   + ' '.join(f'({100 * x:.0f},{100 * y:.0f})' for x, y in near))
            return
        if 'pivot' in m:
            # 비켜 돈 방향 그대로 10 cm 빠져나온 뒤 목표를 향함. 바로 목표 쪽으로 되돌면 모서리가 다시 걸려 비키기 ↔ 되돌기만
            # 하다 3 s 뒤 정지 → 같은 직진을 되풀이했음 (2026-10-30 Gazebo 에이전트 분석: 사용자 미로 408 s)
            if math.dist(pose[:2], m.setdefault('escape_from', pose[:2])) < 0.10:
                return self.drive(p['approach_speed'], 0.0)
            del m['pivot'], m['escape_from']  # 빠져나왔으면 다음 모서리에 다시 비켜 돌 수 있게
        # 몸은 진행 방향보다 crab만큼 오른쪽을 보게 해야 목표로 곧게 감
        err = 0.0 if math.hypot(dx, dy) < 0.08 else math.remainder(math.atan2(dy, dx) - pose[2] - self.crab, 2 * math.pi)
        speed = p['approach_speed'] if toward else p['fast_speed'] if fast else p['drive_speed']
        if m.get('stop'):
            speed = min(speed, max(0.03, 0.8 * left))  # 꺾이는 칸: 끝에서 늦춰 중심에 섬
        push = 0.0 if points is None or near_cube else wall_push(points, p['wall_keep'], p['k_wall'])
        if wide:
            # 길 안·입구 앞: 양옆 벽 사이 가운데로 조향 (벽 1 cm 앞에서야 미는 wall_push보다 일찍, 입구 0.45 m 앞부터)
            push = p['k_center'] * gap[0]
        # 상한: 크면 목표에서 0.5 rad 넘게 돌아가 전진 0으로 버팀 → 20 s 시간 초과 (2026-10-30 Gazebo v10 시드 6)
        push = max(-p['max_wall_push'], min(p['max_wall_push'], push))
        steer = max(-p['max_steer'], min(p['max_steer'], p['k_heading'] * err + push))
        if self.stalled(m, now, points, abs(err) < 0.5):
            return
        self.drive(speed if abs(err) < 0.5 else 0.0, steer)

    def stalled(self, m, now, points, forward):
        """Driving forward but the LiDAR distance straight ahead has not shrunk by stall_m for stall_s: stuck.

        상대 로봇에 밀리거나 라이다 아래 낮은 것에 몸이 걸리면 바퀴가 헛돌아 엔코더(odom)로는 가는 것처럼 보임 (규정 13절
        접촉). 앞 벽까지 라이다 거리로 봄. 앞이 안 보이거나, 제자리에서 돌거나, 잠깐 쉬었으면 처음부터 다시 잼
        """
        p = self.p
        ahead = sorted(x for x, y in (points or []) if x > 0.3 and abs(y) < 0.1)
        ahead = ahead[len(ahead) // 2] if len(ahead) >= 3 else None
        t0, a0, last = m.get('fprog', (now, None, now))
        # 기준보다 멀어지면 기준을 다시 잡음: 막 돈 뒤 첫 스캔이 앞 거리를 짧게 잡아, 실제로 2 s에 18 cm 가는데도 기준보다
        # 3 cm 안 줄었다고 "끼임"을 판마다 4~5번 냈음 (Gazebo 18차 연습장 세 판, 첫 F800)
        if (not forward or ahead is None or a0 is None or now - last > 0.3 or a0 - ahead >= p['stall_m']
                or ahead - a0 > p['stall_m']):
            m['fprog'] = (now, ahead, now)
            return False
        m['fprog'] = (t0, a0, now)
        if now - t0 < p['stall_s']:
            return False
        self.drive()
        self.result = ('# ABORT PI_STALL', now)
        self.get_logger().warn(f'앞으로 가는데 {now - t0:.1f} s 동안 앞까지 거리가 그대로({100 * a0:.0f} → {100 * ahead:.0f} cm, '
                               f'점 {len([1 for x, y in points if x > 0.3 and abs(y) < 0.1])}개): 끼임')
        return True

    def cube_at_gripper(self):
        """Front PSD is close and it is the tracked cube, not a wall.

        정면 PSD 값만으로는 큐브와 벽을 못 가름: 주머니 벽을 큐브로 보고 10 cm 직진만 38번 되풀이했음 (2026-10-31 Gazebo
        에이전트 분석: 시드 6). 카메라가 최근 중앙에 봤거나(큐브가 벽에 붙은 실물 10-30 경우), 라이다가 정면 near_wall_m 안에서
        아무것도 못 보면(큐브는 라이다보다 낮음, 벽은 보임) 큐브로 봄.
        """
        p, t = self.p, self.telemetry
        if not (self.tracked and t is not None and t[FRONT] >= p['near_front_min']):
            return False
        # 앞끝 5 cm·집게 폭 안 벽이면 카메라가 가운데에 봐도 아님 (on_stop과 같은 검사): on_stop이 "파지하지 않음"으로 막은
        # 바로 다음 틱에 do_approach가 이 판정으로 같은 벽을 쥐었음 (Gazebo 18s m4 55 s, 큐브 1.5 m 앞)
        if self.wall_ahead(p['front_half_width'], 0.05):
            return False
        return self.target_centred() or not self.wall_ahead()

    def wall_ahead(self, half=0.08, reach=None):
        """LiDAR points in the band (±half) within reach (near_wall_m) of the front end: a wall, not the cube (큐브는 라이다보다 낮음)."""
        points, reach = self.points(), reach or self.p['near_wall_m']
        return points is not None and free_distance(points, half, self.front_extent(), reach) < reach

    def search_cells(self, states):
        """FREE cells the camera still has to look at for the cube.

        쥐기 전 큐브는 (5,5)뿐: 다른 칸은 보러 가지 않음 ((6,4)·(7,5)·(9,6)까지 보러 다니며 (5,5) 확인이 165 s,
        2026-11-02 실물 00:00 판, 사용자 지적). 새 길 찾기(라이다 미탐색 칸)는 그대로. 같은 속도 0.27로 모든 칸 보기와
        비교하면 큐브 접근이 l5 109 → 80 s, real2 130·137 → 121 s로 빨랐음 (Gazebo 20i·20j; seed 305는 끼임 하나로 느렸음)
        """
        unseen = {c for c, state in states.items() if state == FREE} - self.camera_seen - self.passed
        return unseen & {tuple(self.p['cube_start_cell'])} if not self.ever_held else unseen

    def start_entries(self, states):
        """Open (FREE or not yet seen) side cells of the cube start cell (5,5), no wall face between.

        아직 못 본 칸도 출입구 후보 (가 보면 지도가 정함): 출발 직후 근처 빨간 물체를 봤을 때 (5,5) 둘레가 지도에 없어 예전
        방식(빨강 쪽으로 직진)으로 34 cm 달려갔음 (2026-11-01 실물 23:44 판 1 s, 출발 칸에서 -13°)
        """
        start, ok = tuple(self.p['cube_start_cell']), self.passable(states)
        return {n for n in ((start[0] + di, start[1] + dj) for di, dj in NEIGHBOURS)
                if states.get(n) in (FREE, UNKNOWN) and ok(n) and frozenset((n, start)) not in self.walls}

    def approach_start_cell(self, pose):
        """Cube still at the rule's start cell (5,5): go to its nearest open side cell ignoring the camera, then face it.

        2026-11-01 사용자 제안: 큐브는 (5,5)에 있다고 확정이니 들어갈 수 있는 곳은 상하좌우 네 칸뿐. 빨강에 반응하는 동작은
        끄고 벽·장애물이 아닌 가까운 출입구로 간 뒤, 거기서 카메라를 켜 (5,5)를 보고 큐브가 정면에 오게 돌고 곧게 들어가 잡음.
        입구 칸이 하나도 없으면 False (예전 방식). 이 함수가 움직였으면 True.
        """
        p, size, states, here = self.p, self.p['cell_size'], self.states(), self.here()
        start = tuple(p['cube_start_cell'])
        entries = self.start_entries(states)
        if not entries:
            return False
        self.pocket = start
        # 출입구 칸에서 카메라로 큐브를 정가운데에 맞춘 뒤 들어감 (사용자: 한 칸 뒤 80 cm에서 출발은 안 됨)
        goals = entries
        if self.final_pending:
            # 큐브 앞까지만 간 직진이 PSD 정지 없이 끝남: 정면 PSD에 큐브가 걸려 있으면 잡고, 없으면 그만큼 물러나 다시 맞춤.
            # 큐브 칸 너머까지 가게 했더니 큐브를 밀고 장애물까지 밀었음 (2026-11-01 실물 23:00 판, 사용자 "뚫고 지나가잖아")
            back, self.final_pending = self.final_pending, None
            # 정면 PSD 임계(psd_threshold)에서만 잡음: 300으로 잡게 했더니 끼임으로 선 자리에서 PSD 369(큐브가 집게 한쪽)로 닫아
            # 빈 집게였음 (2026-11-01 실물 23:14 판 159 s, 사용자 "임계값에 오지도 않았는데 지멋대로 파지")
            if self.psd_peak()[FRONT] >= p['grasp_psd'] and self.gripper_at(p['open_angles']):
                self.get_logger().info(f'큐브 앞까지 왔고 정면 PSD {self.psd_peak()[FRONT]}: 파지')
                self.enter('GRASP')
                return True
            # 출입구 칸 한가운데로 돌아갈 만큼만(최대 15 cm): 간 거리 40 cm를 그대로 비스듬히 물러나다 뒤 장애물을 밀었음
            # (2026-11-02 실물 00:00 판 203 s). 남은 비킴은 출입구 칸 한가운데로 옮기기·정렬이 맞춤
            here_c = center(self.here(), size)
            back = min(0.15, back, math.dist(pose[:2], here_c) + 0.05)
            self.get_logger().info(f'큐브 앞까지 왔는데 정면 PSD에 안 걸림: {100 * back:.0f} cm 물러나 다시 맞춤')
            self.entry_turns = 0
            self.board(f'B{max(30, round(1000 * back))}')
            return True
        if here not in goals:
            if self.detour is None or self.detour[1] != goals:
                self.get_logger().info(f'큐브 (5,5) 확정: 출입구 칸 {sorted(goals)} 중 가까운 곳으로 (가는 동안 빨강에 반응 안 함)')
                self.entry_turns = 0
            self.detour = (here, goals)
            self.go_detour()
            return True
        self.detour = None
        c = center(here, size)
        if math.dist(pose[:2], c) > 0.15:  # 출입구 칸 한가운데로 (15 cm 안이면 그대로: 짧은 직진이 지나침)
            b = math.degrees(math.remainder(math.atan2(c[1] - pose[1], c[0] - pose[0]) - pose[2], 2 * math.pi))
            if abs(b) >= p['approach_turn_deg']:
                self.board(command_text(('T', math.copysign(max(10, round(abs(b))), b))))
            else:
                self.board(f'F{max(100, round(math.dist(pose[:2], c) * 1000))}', point=c)
            return True
        # (5,5) 쪽을 보고, 선 뒤 새 검출을 1.5 s까지 기다려 큐브를 정면에 두도록 돎: 0.9 s 지난 검출을 버리고 칸 중심으로 가
        # 위치 추정이 틀어진 만큼 큐브 옆을 지나쳤음 (2026-11-01 실물 22:58 판 159 s, 정면 PSD 338)
        sc = center(start, size)
        h = math.atan2(sc[1] - pose[1], sc[0] - pose[0])
        fresh = self.seen(1.0, self.idle_since)
        if not fresh and self.now() - self.idle_since < 3.0:
            return True
        cam = bool(fresh) and abs(math.remainder(pose[2] - fresh[-1] - h, 2 * math.pi)) < math.radians(40)
        if cam:
            h = pose[2] - fresh[-1]  # 카메라가 본 큐브 방향 (칸 중심 쪽 ±40° 안일 때만)
        turn = math.degrees(math.remainder(h - pose[2], 2 * math.pi))
        # 카메라가 안 보이면 위치 추정의 칸 중심 쪽으로는 크게(30° 넘게) 어긋날 때만 돎: 4번 돈 뒤 카메라 없이 칸 중심 쪽(-17°)으로
        # 곧게 가 큐브 칸 입구 모서리에 끼었음 (2026-11-02 실물 00:00 판, 이 구역 위치 추정은 10~20 cm 틀어짐). 작게 어긋나면 지금
        # 방향 그대로 가며 카메라 보정(follow_final)에 맡김
        if (abs(turn) >= (p['align_deg'] if cam else 30.0)) and self.entry_turns < 6:
            self.entry_turns += 1
            self.board(command_text(('T', math.copysign(max(5, round(abs(turn))), turn))))
            return True
        if not cam:
            h = pose[2]
        # 큐브를 정가운데에 두고(follow_final) 정면 PSD 임계에서 섬 → on_stop이 파지 (사용자: 들어가는 거리를 정하지 말고 정렬을
        # 유지하며 직진하다 PSD 임계에서 잡음). 상한은 큐브 칸 중심까지: PSD에 안 걸려도 큐브를 칸 밖으로 밀지 않게
        d = max(0.1, math.dist(pose[:2], sc))
        self.get_logger().info(f'출입구 {here}에서 (5,5)로 곧게 {100 * d:.0f} cm (방향 {turn:+.0f}°, 들어가며 큐브를 가운데에)')
        self.entry_turns = 0
        self.approach_moves += 1
        self.board(f'F{round(1000 * d)}', point=(pose[0] + d * math.cos(h), pose[1] + d * math.sin(h)))
        self.motion['final'] = True  # 들어가는 동안 큐브를 화면 가운데에 둠 (follow_final)
        self.final_pending = d
        return True

    def follow_final(self, m, pose):
        """Final run into (5,5): keep the cube on the image centre line with small, damped corrections.

        사용자: 출입구에서 카메라를 다시 켜 큐브를 정가운데에 맞춰 접근. 검출마다 방향을 통째로 바꾸면(화면 끝 검출 포함)
        S자로 흔들렸으므로 화면 가운데 ±20° 검출만, 오차의 70%만 고침. 집게 바로 앞(정면 PSD 가까움)에서는 안 고침
        """
        p = self.p
        if self.psd_peak()[FRONT] >= p['near_front_min']:
            return
        fresh = self.seen(0.3, m['sight'])
        if not fresh or abs(fresh[-1]) > math.radians(20):
            return
        m['sight'] = self.now()
        m['h'] = pose[2] - 0.7 * fresh[-1]
        left = math.hypot(m['point'][0] - pose[0], m['point'][1] - pose[1])
        m['point'] = (pose[0] + left * math.cos(m['h']), pose[1] + left * math.sin(m['h']))

    def pocket_ahead(self, pose):
        """Map heading to the cube cell (pocket) centre when it is within 0.7 m and ±20° ahead, else None."""
        if self.pocket is None or pose is None:
            return None
        c = center(self.pocket, self.p['cell_size'])
        if self.pocket_shift is not None:  # 실제 입구 가운데 선을 따라 (enter·do_approach가 잰 만큼)
            c = (c[0] + self.pocket_shift[0], c[1] + self.pocket_shift[1])
        h = math.atan2(c[1] - pose[1], c[0] - pose[0])
        ok = math.dist(pose[:2], c) < 0.7 and abs(math.remainder(h - pose[2], 2 * math.pi)) < math.radians(20)
        return h if ok else None

    def follow_cube(self, m, pose):
        """Approach run: every new detection re-aims the remaining distance at the cube (연속 시각 서보)."""
        p = self.p
        if self.cube_at_gripper():
            return  # 집게 바로 앞: 방향을 더 바꾸지 않고 직진 (큐브가 화면 아래로 잘려 검출이 흔들림)
        h = self.pocket_ahead(pose)
        if h is not None:
            # 큐브 칸 입구에서 칸 쪽을 보고 있으면 카메라로 다시 조준하지 않고 칸 중심으로 곧게: 검출마다 방향을 바꿔 S자로
            # 들어갔음 (2026-11-01 실물 22:00 판 사용자 "왜 S자로 접근") — 화면 끝·바로 앞 검출은 늦고 흔들림
            left = math.hypot(m['point'][0] - pose[0], m['point'][1] - pose[1])
            m['h'], m['point'] = h, (pose[0] + left * math.cos(h), pose[1] + left * math.sin(h))
            return
        fresh = self.seen(p['sample_timeout_s'], m['sight'])
        if not fresh:
            return
        m['sight'] = self.now()
        err = fresh[-1]
        self.tracked |= abs(math.degrees(err)) < 2 * p['align_deg']
        self.cube_heading = pose[2] - err
        left = math.hypot(m['point'][0] - pose[0], m['point'][1] - pose[1])
        m['h'] = self.cube_heading
        m['point'] = (pose[0] + left * math.cos(m['h']), pose[1] + left * math.sin(m['h']))

    def front_blocked(self, now):
        """The last psd_hold_samples front PSD readings (all within 0.2 s) are at or above psd_threshold.

        표본 하나는 잡음으로 70~150 튀어 큐브 앞에서 일찍 멈췄음(2026-10-14) → 두 표본 연속. 보드도 60 ms 유지로
        먼저 멈추므로(펌웨어 10-16 판정) 이건 보조 정지입니다 (텔레메트리 5 Hz).
        """
        p = self.p
        recent = [h for h in self.psd_log if now - h[0] < 0.5][-p['psd_hold_samples']:]
        return len(recent) == p['psd_hold_samples'] and all(h[1][0] >= p['psd_threshold'] for h in recent)

    def on_stop(self, event, move):
        """The board refused or stopped a move: grasp, or mark the way blocked and carry on."""
        p, (text, _, cell) = self.p, move
        # 가까워지면 큐브가 화면 아래로 빠지므로 "이번 접근에서 중앙에 두고 따라왔음"도 인정 (2026-10-30 실물:
        # 집게 앞 PSD 600~770인데 마지막 중앙 검출이 2 s 넘어 장애물로 보고 놓침)
        # 단 정면에 라이다 벽이 보이면 따라왔어도 아님: 큐브 1.8 m 앞에서 장애물을 돌다 벽에 PSD 700으로 "파지"했음
        # (2026-11-01 Gazebo m4 50 s, 2026-10-31 실물 12:28 판 88 s). 지금 중앙에 보이면(벽에 붙은 큐브) 그대로 파지
        # 집게 폭(±front_half_width) 앞끝 5 cm 안 벽도: 옆 9~11 cm 벽 모서리는 ±8 cm 띠 밖이라 1.5 m 앞 큐브를 두고 그 모서리를
        # 쥐었음 (2026-11-01 Gazebo 18o m4 두 판). 벽에 붙은 큐브면 벽은 큐브(5 cm) 너머라 5 cm 안에 안 들어옴
        # 큐브 칸(주머니)을 정면 0.7 m·±20° 안에 두고 섬: 입구 칸으로 가는 중이라 카메라로 따라오지 않았어도(tracked 아님,
        # 바로 앞 큐브는 화면 아래로 빠짐) 큐브. 입구로 곧게 들어가다 정면 PSD 783으로 큐브 앞에 섰는데 파지 안 하고 입구
        # 칸으로 되돌아가기를 되풀이했음 (2026-11-01 실물 17:12 판 127 s, 자세는 큐브 칸 안 15 cm로 잡힘)
        facing = self.pocket_ahead(self.pose()) is not None
        near_wall = self.wall_ahead(p['front_half_width'], 0.05)
        # 앞끝 5 cm 벽은 카메라가 지금 가운데에 봐도 아님: 1.5 m 밖 큐브가 -12°로 보이는 순간 정면 PSD가 앞 벽에 걸려 두 번
        # 벽을 쥐었음 (Gazebo 18r m4 57·67 s). 벽에 붙은 큐브는 벽이 큐브(5 cm) 너머라 그대로 파지
        centred = not near_wall and (self.target_centred() or (
            self.state == 'APPROACH' and (facing or (self.tracked and not self.wall_ahead()))))
        psd = self.psd_peak()
        # 집게가 다 열려 있을 때만: 닫히는 중·반쯤 닫힌 날도 정면 PSD를 가립니다
        approaching = self.state == 'APPROACH' and text[0] == 'F' and self.gripper_at(p['open_angles'])
        if (self.state == 'APPROACH' and 'LIDAR' in event and facing and psd[FRONT] >= p['grasp_psd']
                and self.gripper_at(p['open_angles'])):
            # 큐브 칸을 정면에 두고 들어가다 옆벽에 라이다로 섬 + 정면 PSD가 큐브를 봄 = 큐브가 집게 안(한쪽): 잡음. 정면 PSD
            # 509~536으로 큐브가 걸렸는데 라이다 정지라 파지 판단을 안 하고 물러나기를 되풀이했음 (2026-11-01 실물 22:58 판,
            # 사용자 "큐브 걸렸는데 왜 안 잡지?"). 벽이었으면 운반 중 정면 PSD 확인(hold_lost)이 놓침으로 알아챔
            self.get_logger().info(f'라이다로 섰지만 큐브 칸 정면에 정면 PSD {psd[FRONT]}: 큐브가 집게 안 → 파지')
            return self.enter('GRASP')
        if approaching and not centred and psd[FRONT] >= p['grasp_front_min']:
            self.get_logger().info(f'정면 PSD {psd[FRONT]}이지만 파지하지 않음: 따라옴 {self.tracked}, 큐브 칸 정면 {facing}, '
                                   f'앞끝 5 cm 벽 {near_wall}, 0.25 m 벽 {self.wall_ahead()}')
        if is_grasp(event, psd, p['grasp_front_min'], approaching, centred):
            self.get_logger().info(f'정면 PSD 최대 {psd[FRONT]}: 목표물 파지')
            return self.enter('GRASP')
        t = self.now()
        # 라이다 예방 정지(Pi)는 부딪힘이 아니므로 세지 않음: 좁은 통로에서 몇 번 나면 미션 전체가 끝났음
        # (2026-10-30 Gazebo 시드 6·7: "60 s 안에 정지 9회"로 큐브를 못 찾은 채 종료)
        # 끼임(PI_STALL, Pi가 세움)도 세지 않음: 낮은 장애물 앞 끼임 9번으로 "60 s 안에 정지 9회" 미션이 끝났음 (Gazebo 18c 저상자)
        if (event.startswith('# ABORT') and 'LIDAR' not in event and 'PI_STALL' not in event
                and not (self.aborts and t - self.aborts[-1] < 1.0)):
            self.aborts.append(t)
        self.aborts = [a for a in self.aborts if t - a < p['abort_window_s']]
        self.get_logger().warn(f'{text} → {event} ({self.state}, PSD 앞·뒤·좌·우 최대 {psd[FRONT:FRONT + 4]})')
        if len(self.aborts) > p['max_aborts']:
            self.get_logger().error(f'{p["abort_window_s"]:.0f} s 안에 정지 {len(self.aborts)}회: 미션 중단')
            return self.enter('DONE')
        if 'LIDAR_SIDE' in event:
            # 넓은 길의 옆 모서리: 칸을 막힘으로 두지 않고 squeeze_back_mm 물러남 → 같은 길을 다시 (가운데 조향). 칸마다 두 번까지,
            # 넘으면 아래 보통 라이다 정지처럼 막힘으로
            tries = [a for a in self.squeeze.get(cell, []) if t - a < 60.0] + [t]  # 1분에 두 번까지
            self.squeeze[cell] = tries
            behind = [(-x, -y) for x, y in (self.points() or [])]
            if len(tries) <= 2 and self.psd_peak()[REAR] < p['psd_threshold'] and corridor_clear(
                    behind, p['squeeze_back_mm'] / 1000, p['front_half_width'], p['body_radius']):
                self.centered = None
                return self.board(f'B{p["squeeze_back_mm"]}')
        if 'LIDAR' in event:
            # 라이다에 막혔으면 이 칸에서 한가운데로 옮기기를 다시 허락: 칸마다 한 번 제한 때문에 오른쪽으로 8 cm 비킨 채
            # 같은 모서리(앞 23 cm·옆 9 cm)에 계속 걸려 큐브 앞에서 못 들어갔음 (2026-10-31 실물 17:28 판 148~164 s)
            self.centered = None
        pose = self.pose()
        if cell is None and 'LIDAR' in event and pose is not None:
            # 복귀 경유점처럼 목표 칸 없이 간 직진도: 바로 앞 칸이 지나온 칸이면 기다림
            size = p['cell_size']
            if self.moving_thing(cell_of(pose[0] + 0.3 * math.cos(pose[2]), pose[1] + 0.3 * math.sin(pose[2]), size), t):
                return
        if cell is not None and any(k in event for k in ('PSD', 'LIDAR', 'STALL')):
            # 장애물 때문에 선 것만 그 칸을 막힘으로 둠. 엔코더·하트비트·시간 초과 정지까지 막힘으로 두면 열린 칸이
            # 쌓여 "갈 곳 없음"이 반복됨 (2026-10-30 실물: ENCODER_DIRECTION마다 칸이 막힘으로 남음)
            if 'STALL' not in event and self.moving_thing(cell, t):
                return
            self.block(cell)
            if 'STALL' in event:
                # 라이다에 안 보이는 낮은 장애물: 지도로는 빈 칸이라 "갈 곳 없음" 비우기(두 번째는 막힘을 모두 비움)에서 지워져
                # 같은 직진을 95번 되풀이하며 시간 초과 (Gazebo 18c·18h 저상자). 따로 기억 (block_ttl_s의 3배 뒤 다시 시도)
                self.stall_at[cell] = t
        elif text[0] == 'T':
            # 회전 중 PSD 정지(PSD_PIVOT: 네 방향 중 하나라도 550 이상): 앞이 비었으면 짧게 빠져나와 다시
            points = self.points()
            if points is not None and self.front_clear(points, p['approach_step_mm'] / 1000):
                return self.board(f'F{p["approach_step_mm"]}')
            # 앞이 막혔으면 뒤가 비었을 때 짧게 후진 (뒤 PSD가 멀고 라이다 뒤쪽 띠가 빔)
            behind = None if points is None else [(-x, -y) for x, y in points]
            if (psd[REAR] < p['psd_threshold'] and behind is not None
                    and corridor_clear(behind, p['approach_step_mm'] / 1000, p['body_radius'], p['body_radius'])):
                return self.board(f'B{p["approach_step_mm"]}')
            if self.state == 'LOOK':
                return self.next_goal()
        if 'PI_STALL' in event and text[0] == 'F':
            # 끼임: 뒤가 비었으면 조금 물러난 뒤 다른 길로 (그 칸은 위에서 막힘). 몸 폭 띠로 보면 옆에 붙은 벽(옆 PSD 900)이 늘
            # 띠 안이라 한 번도 못 물러났음 (Gazebo 18h 저상자 95번) → 가운데 ±front_half_width만 봄
            points = self.points()
            behind = None if points is None else [(-x, -y) for x, y in points]
            if (psd[REAR] < p['psd_threshold'] and behind is not None
                    and corridor_clear(behind, p['approach_step_mm'] / 1000, p['front_half_width'], p['body_radius'])):
                return self.board(f'B{p["approach_step_mm"]}')
        if self.state == 'APPROACH':
            # 큐브 쪽으로 장애물을 돌아가던 중(detour)이면 접근을 이어 감(do_approach가 다른 길로 go_detour). LOOK으로
            # 가면 큐브가 화면에 없어 먼 탐색 목표로 떠나 처음부터 다시 찾았음 (2026-10-31 실물 16:30 판 174 s:
            # 앞 길·오른쪽 대각선 길이 있는데 모서리에 한 번 걸린 뒤 8칸 밖으로 감, 사용자 지적)
            # 주머니 칸 입구에서 들어가다 모서리에 걸린 것도 같음: 칸 한가운데로 다시 맞춘 뒤(위 centered) 곧게 다시 들어감.
            # LOOK으로 가면 큐브가 화면 아래라 못 보고 떠나 다시 못 찾았음 (2026-10-31 Gazebo 17차 m4 161 s). max_approach_steps로 끝남
            if self.detour is not None or self.pocket is not None:
                return
            self.enter('LOOK')

    def moving_thing(self, cell, t):
        """A cell we already drove through is now blocked straight ahead: likely the other robot. Wait dyn_wait_s once.

        경기장 장애물은 고정이라 지나온 칸이 나중에 막히면 움직이는 것(상대 로봇)일 가능성이 큼 (2026-10-31 사용자 제안).
        막힘으로 두면 30 s 동안 멀리 돌아감. 정면 가운데(±8 cm)에 점이 있을 때만: 칸에서 몇 cm 비켜 옆 모서리가 띠 끝에
        걸린 것(23, -9 cm 같은 점)은 고정 장애물이라 빼고, 같은 칸은 30 s에 한 번만 기다림 (계속 서 있는 상대는 막힘으로)
        """
        size = self.p['cell_size']
        if cell not in {cell_of(x, y, size) for _, x, y, _, _ in self.track} or t - self.dyn_waited.get(cell, -math.inf) < 30:
            return False
        # 몸 앞끝 바로 앞(35 cm 안) 가운데 점 중 칸 경계선에서 6 cm 넘게 떨어진(칸 안쪽) 것만: 벽·장애물 면은 규정상 칸
        # 경계선 위라 그 점은 고정된 것. 경계선을 안 따지니 벽 모서리를 상대 로봇으로 보고 판마다 4 s씩 섰음 (Gazebo 18c·18g)
        pose = self.pose()
        if pose is None:
            return False
        c, s = math.cos(pose[2]), math.sin(pose[2])

        def off_line(v):
            r = (v - size / 2) % size
            return min(r, size - r)
        inside = [1 for x, y in (self.points() or []) if 0.06 < x < 0.35 and abs(y) < 0.08
                  and off_line(pose[0] + x * c - y * s) > 0.06 and off_line(pose[1] + x * s + y * c) > 0.06]
        if len(inside) < 3:
            return False
        self.dyn_waited[cell], self.wait_until = t, t + self.p['dyn_wait_s']
        self.get_logger().info(f'지나온 칸 {cell}이 정면에서 막힘: 움직이는 것(상대 로봇)으로 보고 '
                               f'{self.p["dyn_wait_s"]:.0f} s 기다린 뒤 다시 갑니다.')
        return True

    def do_init(self):
        if self.telemetry is not None and self.pose() is not None and self.scan is not None:
            self.enter('RELEASE')  # 집게부터 끝까지 엶 (지난 실행이 닫힌 채·운반 모드로 끝났을 수 있음)

    def start_release(self):
        self.holding, self.released = False, False

    def do_release(self):
        """Open the gripper fully before anything moves (반쯤 닫힌 날은 정면 PSD를 가려 가짜 파지, 실물 2회차)."""
        if not self.released:
            self.board('PF1')  # 운반 모드 해제 먼저: 서보가 도는 중에는 보드가 괄호 명령을 거절(REJECT BUSY)
            self.board('7')
            self.released, self.since = True, self.now()
        elif self.gripper_at(self.p['open_angles']):
            self.enter('LOOK')
        elif self.now() - self.since > self.p['grip_timeout_s']:
            self.released = False  # 다시 보냄

    def do_look(self):
        """Pause briefly for the camera (no turning), then pick the next goal.

        2026-10-30 사용자 요청: 웹캠 화각이 넓으므로 도착할 때마다 제자리에서 돌며 둘러보지 않음. 이동 중에도 카메라를
        계속 보고 있어 큐브가 확인되면 watch_move가 바로 멈추고 접근합니다.
        """
        if not self.ready() or self.now() - self.since < self.p['look_pause_s'] or self.fix_due():
            return
        seen = self.target_seen()
        if seen == 'yes':
            return self.enter('APPROACH')
        if seen == 'wait':
            return
        self.visited.add(self.here())
        # 돌아본 뒤 2 Hz 기록을 기다리지 않고 지금(멈춘 뒤) 스캔으로 본 칸부터 기록: 돌아서 다 본 막다른 옆길을 "더 이어진다"고
        # 보고 들어갔다 나왔음 (2026-10-31 Gazebo 맵 1)
        self.update_camera()
        self.next_goal()

    def update_camera(self):
        """Add the cells the camera sees now (live LiDAR line of sight, grid.camera_view) to camera_seen."""
        self.camera_at, pose, points = self.now(), self.scan_pose(), self.points()
        if pose is not None and points is not None:
            self.camera_seen |= camera_view(pose, points, self.states(), self.p['cell_size'],
                                            math.radians(self.p['camera_half_fov_deg']), self.p['camera_range'])

    def next_goal(self):
        """Straight on while open, else the nearest frontier cell; never a cell already passed.

        큐브 처음 위치 (5,5) 힌트는 쓰지 않음: 1단계로 넣어 봤지만 매번 일찍 포기하고 효과가 없어 뺌 (2026-10-30 사용자
        결정). 지나간 칸은 이미 카메라가 지나간 곳이라 목표에서 뺍니다. 더 갈 곳이 없으면 먼저 막힘·실패·둘러본 칸만 비우고(지나간 기록은 유지), 그래도 없을 때
        지나간 기록까지 비웁니다.
        """
        states, here, pose, p = self.states(), self.here(), self.pose(), self.p
        done = self.camera_seen | self.visited | self.failed | self.passed | {here}
        # ① 라이다로 빈 바닥인데 카메라가 아직 못 본 칸: 그 칸으로 가지 않고 화각(±camera_half_fov_deg, camera_range)에
        #    들어오는 자리로 감. 지금 자리에서 보이면 이동하지 않고 그쪽으로 돌아봄 (2026-10-30 사용자 요청, 화각 70°·1.3 m)
        # ② 라이다 미탐색 칸(①이 없을 때만). 외벽 너머 미탐색 칸은 영원히 못 가는데 ①과 섞어 가까운 순으로 고르면 그쪽으로
        #    GOTO 한 번에 100 s 넘게 썼음 (2026-10-30 Gazebo 에이전트 분석: 시드 1·6). 경기장 안 칸은 결국 빈 바닥이 됨
        # 칸을 떠나기 전에 직진으로는 못 볼 옆·뒤 칸을 돌아봄: 안 보고 지나가면 나중에 그걸 보러 지나간 길로 되돌아왔음
        # (2026-10-30 사용자 지적). 출발 칸에서는 앞이 뚫렸으면 바로 직진 (출발 직후 둘러보기 없앰, 2026-10-30 사용자 결정)
        start = here == (0, 0)
        cruise = self.ahead(states)
        # 출발 칸에서도 옆에 못 본 칸이 2개 이상이면 한 번 돌아봄: 안 보고 직진했다가 그 칸들을 보러 한 바퀴 돌아 나왔음
        # (2026-10-31 실물 조건 Gazebo 맵 3: (2,9)·(3,9)). 1개뿐이면 전처럼 바로 직진 (2026-10-30 사용자 결정)
        if self.look_around(states, pose, dry=True, min_cells=2 if start and cruise else 1):
            if start and cruise and pose is not None:
                self.resume_axis = axis(pose[2])  # 돌아본 뒤 처음 방향으로 직진을 이어 감 (옆으로 직진하면 한 바퀴 돌았음, 맵 4)
            self.look_around(states, pose, min_cells=2 if start and cruise else 1)
            return
        # 보는 자리에 왔는데 볼 칸 쪽으로 못 돎(라이다가 그 앞에서 막힘 = 지도에만 빈 칸, 예: 경기장 밖): 이 자리는 다시 안 고름.
        # 경기장 밖 (9,2) 같은 칸을 보러 두 자리 사이를 TR175·TL178로 오가며 위쪽에서 50 s 넘게 썼음 (2026-10-31 실물 8차)
        if self.view_targets.get(here, set()) - self.camera_seen:
            self.failed.add(here)
        if cruise:
            self.cleared = False
            return self.enter('CRUISE')
        unseen = self.search_cells(states)
        start_cell = tuple(p['cube_start_cell'])
        if self.recheck_start and not self.start_looked and states.get(start_cell) == FREE:
            unseen.add(start_cell)  # 다 돌고도 못 찾음: 큐브 시작 칸을 다시 보러 감 (아래 갈 곳 없음 처리)
        # 막다른 칸(사방 중 세 면이 막힘)은 다른 곳을 다 본 뒤에: 칸 하나 보러 막다른 통로까지 들어갔다 나왔음 (2026-10-31
        # 실물 조건 Gazebo 맵 2: (1,7) 보러 (2,7)까지). 큐브가 거기 있을 수도 있으니 빼지는 않고 마지막 순서
        ok = self.passable(states)
        dead = {c for c in unseen if sum(ok((c[0] + di, c[1] + dj)) for di, dj in NEIGHBOURS) <= 1}
        views = viewpoints(states, unseen - dead, p['cell_size'], p['camera_range'], walls=self.walls)
        later = viewpoints(states, dead, p['cell_size'], p['camera_range'], walls=self.walls)
        self.view_targets = {v: t for v, t in list(later.items()) + list(views.items())
                             if v not in self.failed and v != here}
        # 새 길 입구(라이다로 아직 모르는 칸·그 옆 칸)도 볼 칸과 같은 순위로 가까운 곳부터: 볼 칸이 하나라도 남으면 입구를 미뤄
        # 바로 옆 대각선 길 대신 출발 근처 칸을 보러 6칸 되돌아갔음 (2026-10-31 실물 14차). 벽 면이 보인 바깥 칸은 seen_faces가
        # 막힘으로 빼 옛날처럼 경기장 밖 미탐색 칸으로 헤매지 않음
        tiers = [(set(views) | self.entries) - done - self.failed - {here}, frontier(states) - done,
                 set(later) - self.failed - {here}]
        # 지나간 칸을 한 칸도 안 거치고 닿는 목표가 먼저. 그런 목표가 하나도 없을 때만(막다른 곳 등) 지나간 칸을 거침
        for strict in (True, False):
            if strict and cruise:
                self.cleared = False
                return self.enter('CRUISE')  # 경계 칸을 찾기 전에 앞이 뚫렸으면 그대로 직진
            for goals in tiers:
                path = self.route(here, states, goals, strict) if goals else None
                if path is not None:
                    # 지나간 칸을 피하는 길이 아직 모르는 칸을 지나고 지나간 칸으로 가는 최단 길보다 길면 최단 길로: 피하려다
                    # 모르는 위쪽 길((6,3)·(5,3) 장애물)로 돌아 60 s를 썼음 (2026-10-31 실물 10차, 사용자 결정)
                    # 같은 목적지로만 비교 (가까운 다른 목적지로 바뀌면 1칸 뒤로 돌아가 옆을 보느라 더 걸렸음, 맵 2)
                    short = bfs(here, self.passable(states), path[-1].__eq__) if strict else None
                    if short is not None and len(short) < len(path) and any(states.get(c) == UNKNOWN for c in path):
                        self.get_logger().info(f'지나간 칸을 피하는 길({len(path) - 1}칸)이 모르는 칸을 지나 '
                                               f'지나간 칸으로 가는 길({len(short) - 1}칸)로 갑니다.')
                        path, strict, goals = short, None, {short[-1]}
                    self.goal_cells, self.goal_strict, self.cleared = goals, strict, False
                    self.resume_axis = None
                    if strict is False:
                        self.get_logger().info('지나가지 않은 길로 닿는 곳이 없어 지나간 칸을 거쳐 갑니다.')
                    self.goal_view = path[-1]
                    look = sorted(self.view_targets.get(path[-1], set()) - self.camera_seen)
                    self.get_logger().info(f'목표 {path[-1]} ({len(path) - 1}칸)' + (f': {look} 보러' if look else ''))
                    return self.enter('GOTO')
        # 갈 곳은 없는데 방금 빨강이 보였으면 그쪽으로: 띄엄띄엄 검출(1 s에 3번 못 채움)이라 확인을 못 하고 제자리에서 갈 곳
        # 없음만 98번 되풀이했음 (2026-10-31 실물 12회차 235~306 s, (5,4)에서 -14°로 보임). 거의 정면이면 접근, 아니면 돌아서 봄
        # 접근 중(APPROACH) 길 없음으로 여기 온 것이면 빼고, 큐브를 잠시 무시하는 중(target_hold)도 뺌: 접근 → 길 없음 →
        # "정면에 빨강 → 접근"을 움직이지 않고 1초에 20번씩 88 s 되풀이했음 (Gazebo 18차 l2 176~264 s)
        recent = self.seen(3.0) if self.state != 'APPROACH' and self.now() >= self.target_hold_until else []
        if recent:
            turn = -math.degrees(recent[-1])
            if abs(turn) < p['camera_half_fov_deg'] / 4:
                self.get_logger().info(f'갈 곳은 없고 정면({turn:+.0f}°)에 빨강이 보임: 접근합니다.')
                return self.enter('APPROACH')
            self.get_logger().info(f'갈 곳은 없고 {turn:+.0f}° 쪽에 빨강이 보임: 그쪽으로 돌아 확인')
            self.enter('LOOK')
            return self.board(command_text(('T', math.copysign(max(10, round(abs(turn))), turn))))
        # 둘러보기가 끝날 때마다 지금 칸이 visited에 들어가므로 "비울 게 남았나"로 고르면 첫 단계만 반복함 (실물)
        if not self.cleared:
            self.cleared = True
            self.get_logger().warn('갈 곳이 없습니다. 막힘·실패·둘러본 칸을 비우고 다시 찾습니다 (지나간 칸 기록은 유지).')
            self.unblock()
            self.failed.clear()
            self.visited.clear()
        else:
            ok = self.passable(states)
            near = ' '.join(f'{c}:{states.get(c, "?")[0]}{"" if ok(c) else "x"}'
                            for c in [(here[0] + di, here[1] + dj) for di, dj in NEIGHBOURS])
            self.get_logger().warn('지나가지 않은 칸이 남지 않았습니다. 지나간 칸 기록을 비우고 다시 찾습니다. '
                                   f'(지금 {here}, 이웃 {near}, 끼임 칸 {sorted(self.stall_at)})')
            self.passed.clear()
            # 큐브 시작 칸을 다시 못 본 칸으로: 접근 중 라이다에 끊겨 놓친 뒤 그 칸을 "본 칸"으로 두고 갈 곳 없음만 900 s 되풀이하며
            # 멈췄음 (2026-10-31 Gazebo 17차 m4, 141 s에 (5,5) 큐브를 보고 입구에서 놓침)
            self.recheck_start, self.start_looked = True, False
            self.camera_seen.discard(tuple(p['cube_start_cell']))
            # 본 기록은 두고 막힘 표시를 모두(거듭 막힌 칸까지) 비움: 갈 곳이 없어지는 건 대개 가짜 막힘 때문인데, 본 기록을 지우면
            # 출발 줄·(1,1)만 끝없이 다시 돌아봤음 (2026-10-31 실물 11차: 앞끝 판정이 옆벽을 막힘으로 봄)
            self.blocked.clear()
            self.block_hits.clear()
            self.cleared = False
            # 다 돌고도 못 찾았으면 큐브 시작 칸 바로 옆 칸으로 가서 봄 (도착하면 look_around가 그 칸으로 돎). 지도에 볼 칸이
            # 안 남아 (5,5) 쪽으로 다시 안 가고 제자리 90° 돌기만 되풀이했음 (Gazebo 18k 저상자: 끼임 뒤 (4,0)에서 멈춤)
            start = tuple(p['cube_start_cell'])
            beside = {c for c in [(start[0] + di, start[1] + dj) for di, dj in NEIGHBOURS] if ok(c)} - {here}
            path = self.route(here, states, beside, False) if beside else None
            if path is None and self.now() - self.map_reset_at > 60.0:
                # 갈 곳도 없고 (5,5) 옆으로 가는 길도 없음 = 지도가 틀어져 가짜 벽이 길을 막음: 지도를 지우고 지금부터 다시 그림.
                # 낮은 장애물에 부딪힌 뒤 방향 보정이 6°씩 튀고 (5,5) 둘레가 모두 "모름"인 채 갈 길이 없어 제자리 90° 돌기만
                # 되풀이했음 (Gazebo 18k 저상자, 60 s에 한 번만)
                self.get_logger().warn('갈 곳도 큐브 시작 칸 쪽 길도 없음: 지도가 틀어졌다고 보고 지우고 다시 그립니다.')
                self.map_reset_at = self.now()
                self.hits[:] = 0
                self.misses[:] = 0
            if path is not None:
                self.get_logger().info(f'다 돌았는데 못 찾음: 큐브 시작 칸 {start} 옆 {path[-1]}로 가서 봅니다.')
                self.view_targets[path[-1]] = {start}
                self.goal_cells, self.goal_strict, self.goal_view, self.cleared = {path[-1]}, False, path[-1], False
                return self.enter('GOTO')
            # 그래도 갈 곳이 없으면 제자리에서 90°씩 돌며 카메라로 훑음: 안 움직이고 두 줄만 900 s 되풀이했음 (Gazebo 17차 m4)
            self.enter('LOOK')
            return self.board(command_text(('T', 90)))
        self.enter('LOOK')

    def look_around(self, states, pose, dry=False, min_cells=1):
        """Turn toward the unseen cells visible from here; True when a turn was sent (or would be, with dry).

        정면 쪽(±화각/4)이면 직진하며 보이므로 기록만 하고 돌지 않음. 못 본 칸 하나 때문에 90° 넘게 돌지 않음: 둘러보기가
        한 판에 10~17번, 그중 다수가 1~2칸에 60~144°라 시간을 잃었음 (2026-10-30 Gazebo 에이전트 분석: 시드 2·8, 5·6).
        """
        p, here = self.p, self.here()
        unseen = self.search_cells(states)
        # 규정 큐브 시작 칸 (5,5)는 가까이서 한 번 똑바로 볼 때까지 못 본 칸으로: 멀리서 비스듬히 "본" 것으로 쳐서 바로 옆
        # (5,4)에 와서도 오른쪽으로 안 돌고 반대쪽 벽으로 돌았음 (2026-10-31 실물 17:45 판 182~195 s, 사용자)
        start = tuple(p['cube_start_cell'])
        if not self.start_looked and states.get(start) == FREE:
            unseen.add(start)
            if pose is not None and abs(start[0] - here[0]) + abs(start[1] - here[1]) == 1:
                # 바로 옆 칸이면 격자 축으로 똑바로 돎 (사용자 '격자 축 맞추고 돌기'). 칸 중심 쪽 라이다 띠 검사는 로봇이 칸
                # 가장자리에 있으면 옆 장애물 모서리에 걸려, (5,4)에 와서도 안 돌고 (6,4)로 되돌아갔음 (2026-10-31 실물 18:19 판 147 s)
                turn = math.degrees(math.remainder(
                    math.atan2(start[1] - here[1], start[0] - here[0]) - pose[2], 2 * math.pi))
                if not dry or abs(turn) < p['camera_half_fov_deg'] / 2:  # 이미 그쪽을 보고 있으면 본 것
                    self.start_looked = True
                    self.camera_seen.add(start)
                    if abs(turn) >= p['camera_half_fov_deg'] / 2:
                        self.get_logger().info(f'둘러보기: 바로 옆 큐브 시작 칸 {start} 쪽으로 {turn:+.0f}°')
                        self.board(command_text(('T', math.copysign(max(10, round(abs(turn))), turn))))
                return abs(turn) >= p['camera_half_fov_deg'] / 2
        targets = viewpoints(states, unseen, p['cell_size'], p['camera_range'], walls=self.walls).get(here)
        if not targets or pose is None:
            return False
        heading, looked = best_heading(pose, targets, p['cell_size'], math.radians(p['camera_half_fov_deg']))
        if len(looked) < min_cells:
            return False
        turn = math.degrees(math.remainder(heading - pose[2], 2 * math.pi))
        # 일부러 찾아온 보는 자리(goal_view)에서는 하나라도 돎: 등 뒤 칸 하나를 안 돌아보고 그 칸의 다른 보는 자리 여섯 곳을
        # 돌아다녔음 (2026-10-31 Gazebo 맵 4: 출발 옆 (0,1), 125 → 239 s)
        if abs(turn) < p['camera_half_fov_deg'] / 2 or (abs(turn) > 90 and len(looked) < 2 and here != self.goal_view):
            if not dry and abs(turn) < p['camera_half_fov_deg'] / 2:
                self.camera_seen |= looked  # 이미 그쪽을 보고 있음
            return False
        # 지금 라이다로 그쪽 화각 가운데가 look_min_free_m 안에서 막혔으면(바로 옆 벽) 돌아봐도 벽만 보임: 돌지 않고 본 것으로.
        # 지도·위치가 틀리면 벽 너머 칸을 "못 본 빈 칸"으로 보고 벽 쪽으로 카메라를 돌렸음 (2026-10-31 실물 2차: (9,4)에서
        # 벽을 바라봄). 지도와 상관없이 지금 스캔으로 막음
        # 못 본 칸이 시작하는 거리보다 앞에서 벽이 보이면 그 칸들은 벽 너머: 돌아도 벽만 봄 (2026-10-31 실물 5차: (9,6)~(6,9)
        # 구석에서 벽 쪽으로 정면을 돌림)
        points = self.points()
        if points is not None:
            c, s = math.cos(pose[2] - heading), math.sin(pose[2] - heading)  # +x = 돌아볼 방향
            view = [(x * c - y * s, x * s + y * c) for x, y in points]
            nearest = min(math.dist(pose[:2], center(cell, p['cell_size'])) for cell in looked)
            # 칸 중심(큐브 자리)까지 보여야 봄: 가까운 경계까지만 요구하니 벽 너머 칸을 보려고 벽 쪽으로 148° 돌았음
            # (2026-10-31 실물 17:45 판 112 s, (6,9) 칸, 사용자 "벽·장애물 쪽으로 돌아보지 말라 했는데")
            need = max(p['look_min_free_m'], nearest - 0.05)
            free = free_distance(view, p['look_half_width'], 0.0, need)
            if free < need:
                if not dry:
                    self.camera_seen |= looked
                    self.get_logger().info(f'둘러보기 안 함: {turn:+.0f}° 쪽 {free:.2f} m 앞이 벽')
                return False
        if not dry:
            self.camera_seen |= looked  # 돌고 나면 화각 안 (큐브가 있으면 도는 중에 watch_move가 잡음)
            self.start_looked |= tuple(p['cube_start_cell']) in looked
            self.get_logger().info(f'둘러보기: 못 본 칸 {len(looked)}개 {sorted(looked)} 쪽으로 {turn:+.0f}°')
            self.board(command_text(('T', math.copysign(max(10, round(abs(turn))), turn))))
        return True

    def route(self, start, states, goals, strict, heading=None):
        """Cell path to the nearest goal: strict = never through a passed cell, else off them if possible."""
        ok = self.passable(states)
        if strict:
            return bfs(start, lambda c: c not in self.passed and ok(c), goals.__contains__, heading)
        return bfs_avoiding(start, ok, goals.__contains__, self.passed, heading)

    def ahead(self, states):
        """Cells straight ahead along the nearest grid axis that are open and not yet passed (up to max_cells).

        옆을 돌아본 뒤에는 원래 가던 축(resume_axis)으로 이어 가되, 돌아본 옆길이 카메라로 본 데보다 더 이어지면 그 길로
        바로 들어감: (9,2) 왼쪽 길을 보고도 앞으로 직진했다가 큐브가 있던 그 길로 227 s에야 돌아왔음 (2026-10-31 실물 9차
        사용자). 출발 칸은 원래 방향 (옆으로 직진하면 맵 4에서 한 바퀴 돌았음)
        """
        pose = self.pose()
        if pose is None:
            return []
        if self.resume_axis and self.resume_axis != axis(pose[2]) and self.here() != (0, 0):
            side = self.run_cells(states, axis(pose[2]))
            if side:
                self.resume_axis = None
                return side
        return self.run_cells(states, self.resume_axis or axis(pose[2]))

    def run_cells(self, states, along):
        """ahead() along one axis."""
        (di, dj), here, ok, cells = along, self.here(), self.passable(states), []
        for k in range(1, 10):
            cell = (here[0] + k * di, here[1] + k * dj)
            if not ok(cell) or cell in self.passed:
                break
            cells.append(cell)
        # 지금 라이다로 막힌 데까지만: 지도는 예전에 빈 걸 본 기록이 많으면 새로 놓인 장애물을 늦게 막힘으로 바꿈. 웹캠으로
        # 빈 걸 다 보고 라이다로 앞이 막힌 게 보이는데 막다른 칸까지 들어갔다 돌아 나오며 15 s를 씀 (2026-10-31 실물 13회차
        # 27 s, 사용자 "굳이 들어가야 하나"). 몸 앞끝이 닿기 전까지 갈 수 있는 칸만 남기고, 아래에서 본 칸을 다시 뺌
        pose, points = self.pose(), self.points()
        if cells and pose is not None and points is not None:
            yaw, size = math.atan2(dj, di), self.p['cell_size']
            c, s = math.cos(pose[2] - yaw), math.sin(pose[2] - yaw)
            free = free_distance([(x * c - y * s, x * s + y * c) for x, y in points],
                                 self.p['body_radius'], self.front_extent(), 4.0)
            hx, hy = center(here, size)
            start = (pose[0] - hx) * math.cos(yaw) + (pose[1] - hy) * math.sin(yaw)  # 지금 칸 중심에서 이미 간 거리
            cells = [cell for k, cell in enumerate(cells, 1) if k * size - start <= free + 0.05]  # k가 늘수록 멀어 앞쪽만 남음
        # 직진 끝쪽의 카메라로 이미 본 칸은 갈 이유가 없음(다른 곳으로 가는 길이면 GOTO가 최단으로 감). 다 본 막다른
        # 모서리까지 들어갔다 꺾어 나오느라 10 s를 썼음 (2026-10-31 Gazebo 사용자 미로 (1,1))
        while cells and cells[-1] in self.camera_seen:
            cells.pop()
        # 남은 못 본 칸이 카메라 거리(여유 0.1 m) 안에 드는 칸까지만: 막다른 통로 끝 칸 하나를 보려고 끝까지 들어갔다
        # 되돌아 나왔음 (2026-10-31 Gazebo m4 7행: 2칸째에서 끝 칸이 1.2 m라 보였는데 4칸 들어감, 사용자 "불필요한 움직임")
        if cells:
            reach = int((self.p['camera_range'] - 0.1) / self.p['cell_size'])
            cells = cells[:max(1, len(cells) - reach)]
        cells = cells[:self.p['max_cells']]
        # 두 칸을 한 번에 가면 가운데 칸에서 옆을 못 돌아봐, 그 칸 옆을 보러 180° 돌아 되돌아왔음 (2026-10-31 실물 조건
        # Gazebo 맵 1·2·3). 옆을 돌아봐야 할 칸이 중간에 있으면 거기까지만 감 (look_around와 같은 기준)
        p, yaw = self.p, math.atan2(dj, di)
        unseen = self.search_cells(states)
        views = (viewpoints(states, unseen, p['cell_size'], p['camera_range'], walls=self.walls)
                 if len(cells) > 1 else {})
        for k, cell in enumerate(cells[:-1]):
            # 옆 칸이 아직 미탐색(앞 장애물에 가려 라이다가 덜 봄)인데 경계에 벽 면도 안 보임 = 열렸을 수 있음 → 거기서 멈춰
            # 라이다로 봄. 출발 (9,2) 왼쪽이 (8,1) 장애물에 가려 미탐색이라 그냥 지나쳤다가 180° 돌아 되돌아왔음
            # (2026-10-31 사용자 질문, Gazebo 맵 1 TL180). 외벽·장애물 면이 보인 칸은 seen_faces가 이미 막힘으로 바꿈
            if any(states.get(c) == UNKNOWN for c in {(cell[0] + dj, cell[1] + di), (cell[0] - dj, cell[1] - di)}):
                return cells[:k + 1]
            if not views.get(cell):
                continue
            cx, cy = center(cell, p['cell_size'])
            heading, looked = best_heading((cx, cy, yaw), views[cell], p['cell_size'],
                                           math.radians(p['camera_half_fov_deg']))
            turn = abs(math.degrees(math.remainder(heading - yaw, 2 * math.pi)))
            if turn >= p['camera_half_fov_deg'] / 2 and (turn <= 90 or len(looked) >= 2):
                return cells[:k + 1]
        return cells

    def start_goto(self):
        self.goal_steps, self.plan = 0, []

    def step_toward(self, goal_cells, explore=None):
        """Next move toward the nearest goal cell: 'arrived', 'moving' or 'no_path'.

        explore=None: 최단 경로. True: 지나간 칸을 절대 안 거침 (길이 그것뿐이면 no_path → 다른 목표).
        False: 지나간 칸은 피할 수 있으면 피함. 'return': 쌓은 지도로 새 최단 경로 (grid.return_route: 막힘 칸·벽 면은
        안 지나고 모르는 칸은 비싸게). 지나온 칸을 따라가면 붙은 두 칸 사이 벽으로 곧장 가 장애물에 세 번 부딪혔음
        (2026-10-31 실물 13차, 사용자: "지나온 칸 기준이 아니라 기록한 장애물·벽 매핑으로 새로운 최단 복귀 계획").
        """
        p, pose = self.p, self.pose()
        if pose is None:
            return 'moving'  # TF를 기다림
        start, states, heading = cell_of(pose[0], pose[1], p['cell_size']), self.states(), axis(pose[2])
        # 짠 경로를 막히거나 목표가 사라질 때까지 그대로 따라감: 매 걸음 새로 짜면 같은 거리 길 중 지금 향한 쪽을 먼저 골라
        # 돌 때마다 반대쪽 길로 바뀌어 반 바퀴씩 두 번 돌았음 (2026-10-31 실물 16차 69~89 s)
        ok = self.passable(states)
        plan = self.plan[self.plan.index(start):] if start in self.plan else []
        if plan and plan[-1] in goal_cells and all(ok(c) for c in plan[1:]):
            path = plan
        elif explore == 'return':
            path = return_route(start, states, ok, goal_cells.__contains__, self.walls)
        elif explore is None:
            path = bfs(start, self.passable(states), goal_cells.__contains__, heading)
        else:
            path = self.route(start, states, goal_cells, explore, heading)
        if path is None:
            return 'no_path'
        self.plan = path
        return self.drive_path(pose, path)

    def corridor_mid(self, pose, a, u):
        """Side offset (m, left of u through a) of the middle of the one-cell corridor ahead, from LiDAR; None if not one.

        다음 칸(경로 방향 0.25~0.55 m) 양옆 벽 점을 각각 3개 넘게 보고, 그 사이가 0.5 m 안일 때만 통로로 봄.
        """
        points = self.points()
        if not points:
            return None
        c, s = math.cos(pose[2]), math.sin(pose[2])
        left, right = [], []
        for x, y in points:
            wx, wy = pose[0] + x * c - y * s - a[0], pose[1] + x * s + y * c - a[1]
            along, side = wx * u[0] + wy * u[1], -wx * u[1] + wy * u[0]
            if 0.25 <= along <= 0.55 and 0.08 < abs(side) < 0.35:  # 다음 칸 끝 벽(0.6)은 뺌
                (left if side > 0 else right).append(side)
        if len(left) < 3 or len(right) < 3:
            return None
        lo, hi = sorted(right)[-3], sorted(left)[2]  # 잡음 두 점 무시
        return (lo + hi) / 2 if hi - lo < 0.5 else None

    def drive_path(self, pose, path):
        """Send the next turn or straight run (up to max_cells, shorter if the LiDAR sees it blocked) along path."""
        p = self.p
        # 경로가 한 칸(이미 목표 칸 안, 중심에서 arrive 넘게 떨어짐)이면 next_move가 그 칸 중심으로 직진을 냄 → path[1]이 없어
        # IndexError로 노드가 죽었음 (2026-10-30 Gazebo 시드 3·5·6)
        cell = path[1] if len(path) > 1 else path[0]
        shift = (0.0, 0.0)  # 조준점을 칸 중심에서 이만큼 옮김 (라이다로 잰 통로 가운데)
        if self.center_goal is not None:
            (at, goal), self.center_goal = self.center_goal, None
            if at == path[0]:  # 한가운데로 가려고 돈 뒤: 그 점까지 직진으로 마무리 (돌기만 하고 끝나 비킨 채 들어갔음)
                return self.board(f'F{max(50, round(1000 * math.dist(pose[:2], goal)))}', None, goal, stop=True) or 'moving'
        if len(path) > 1:
            # 칸 경로 직선에서 옆으로 center_tol_m 넘게 비켜 있으면 먼저 지금 칸 한가운데로: 10 cm 비킨 채 다음 칸으로
            # 가 통로 모서리(앞 23 cm·옆 10 cm)에 두 번 걸리고 큐브를 눈앞에 두고 못 잡았음 (2026-10-31 실물 17:07 판
            # 155·160 s, 사용자 "장애물에 안 걸리게 우회해서 잡아야지")
            (ox, oy), (tx, ty) = center(path[0], p['cell_size']), center(path[1], p['cell_size'])
            ux, uy = (tx - ox) / p['cell_size'], (ty - oy) / p['cell_size']
            # 칸마다 한 번만: 벽 가까운 칸에서 중심 직진이 몇 mm씩만 가고 끝나 F258·F256·F255…를 되풀이했음 (Gazebo l5 10차)
            # 다음 칸이 한 칸 통로(양옆 벽이 라이다에 보임)면 칸 중심이 아니라 실제 통로 가운데에 corridor_tol_m 안으로 맞춤:
            # 몸 33 cm·통로 40 cm라 3.5 cm만 비켜도 닿음. 격자 중심에 맞췄는데 실제 통로가 8~12 cm 비켜 있어 입구 모서리
            # (앞 25 cm·옆 11 cm)에 라이다로 서고, 그 칸을 막힘으로 봐 "복귀 경로 없음" 좌우 회전을 50 s 되풀이했음
            # (2026-11-01 실물 21:01 판 290~344 s, 사용자 "들어가기 전에 앞길 중앙에 맞춰서")
            side = -(pose[0] - ox) * uy + (pose[1] - oy) * ux
            mid = self.corridor_mid(pose, (ox, oy), (ux, uy))
            want, tol = (0.0, p['center_tol_m']) if mid is None else (mid, p['corridor_tol_m'])
            # 맞춘 뒤 돌고 갈 때도 칸 중심이 아니라 그 통로 가운데를 조준: 통로 가운데에 맞춘 뒤 격자 칸 중심을 보고 돌아
            # TL99로 비스듬히 들어갔음 (2026-11-01 실물 21:27 판 99 s, 사용자 "회전할 때 딱 90도가 안 맞던데")
            shift = (-uy * want, ux * want)
            # 칸마다 1분에 두 번까지: 옆으로 14 cm 비키려 돌면 정면 벽(23 cm)에 집게가 걸려 라이다로 서고, 그 정지가 다시
            # 맞추기를 허락해 같은 칸에서 끝없이 되풀이했음 (2026-11-01 실물 22:30 판 (6,3) 380 s~). 넘으면 그대로 가고 직진 중
            # 길 가운데 조향(passage)에 맡김
            tries = [a for a in self.center_tries.get(path[0], []) if self.now() - a < 60.0]
            # 출발 칸은 빼고: 손으로 칸 가운데에 놓으므로 3 cm는 잰 값의 흔들림인데, 맞추느라 TL125·F50·TR125로 11 s를 썼음
            # (2026-11-02 실물 00:56 판, 사용자). 조준점(shift)은 그대로 통로 가운데라 첫 직진이 맞춰 들어감
            if abs(side - want) > tol and self.centered != path[0] and len(tries) < 2 and path[0] != (0, 0):
                self.centered = path[0]
                self.center_tries[path[0]] = tries + [self.now()]
                gx, gy = ox - uy * want, oy + ux * want
                err = math.degrees(math.remainder(math.atan2(gy - pose[1], gx - pose[0]) - pose[2], 2 * math.pi))
                if abs(err) >= p['turn_min_deg']:
                    self.board(f'T{"L" if err > 0 else "R"}{abs(round(err))}')
                    self.center_goal = (path[0], (gx, gy))
                else:
                    self.board(f'F{max(50, round(1000 * math.dist(pose[:2], (gx, gy))))}', None, (gx, gy), stop=True)
                where = '한가운데' if mid is None else f'앞 통로 가운데({100 * mid:+.0f} cm)'
                self.get_logger().info(f'칸 {path[0]} {where}로 먼저: 옆으로 {100 * (side - want):+.0f} cm 비킴')
                return 'moving'
        top = p['max_cells']
        if self.state == 'RETURN':
            # 복귀는 경로가 꺾이는 칸까지 한 번에: 2칸마다 끊고 끊을 때마다 격자 보정으로 1초씩 서서 한 칸씩 멈추며
            # 왔음 (2026-10-31 실물 6차 사용자: "직진 하는 부분이면 쭉 직진"). 보정은 꺾일 때 회전 앞뒤에서
            top = max(1, path.index(aim_cell(path, len(path))))
        for cells in range(top, 0, -1):
            # 도착 15 cm: 목표 15 cm 앞 근접 정지를 도착으로 보는 기준과 같게. 10 cm면 벽에 막혀 12 cm 앞에서 "도착"한 뒤
            # 같은 F122를 26번 되풀이했음 (2026-10-30 Gazebo 에이전트 분석: 시드 6)
            move = next_move((pose[0] - shift[0], pose[1] - shift[1], pose[2]), path, p['cell_size'], p['turn_min_deg'],
                             arrive=0.15, max_cells=cells)
            if move is None:
                return 'arrived'
            if move[0] == 'T' and len(path) > 1:
                # 칸 경로는 늘 격자 축 방향: 다음 칸 중심을 보고 돌면 칸 중심 몇 cm 앞에 선 만큼 80°처럼 덜 돌고 비스듬히
                # 들어갔음 (2026-11-01 실물 21:27 판 TR72·TR79·TL82, 사용자 "딱 90도가 안 맞던데"). 옆 비킴은 위에서 이미
                # 맞췄으니 축 방향(쏠림 crab만큼 몸을 틀어)으로 돎. 그 각도가 turn_min_deg보다 작으면 원래대로
                deg = math.degrees(math.remainder(math.atan2(uy, ux) - self.crab - pose[2], 2 * math.pi))
                if abs(deg) >= p['turn_min_deg']:
                    move = ('T', math.copysign(min(max(round(abs(deg)), 10), 180), deg))
            if move[0] == 'T':
                # 돌기 전에 그 칸 쪽이 막혔는지 먼저 봄 (라이다는 360°): 돌아서 벽을 마주한 뒤에야 막힘을 알고 다시 돌았음
                # (2026-10-31 실물 5차: 벽 쪽으로 정면을 돌림)
                points = self.points() if len(path) > 1 else None
                origin, target = center(path[0], p['cell_size']), center(path[min(1, len(path) - 1)], p['cell_size'])
                if points is not None and not corridor_clear(to_line(points, pose, origin, target),
                                                             math.dist(origin, target), p['path_half_width'], 0.0):
                    self.get_logger().info(f'라이다: {path[1]} 칸 쪽이 막혀 그쪽으로 돌지 않음')
                    self.block(path[1])
                    return 'moving'
                break
            # 칸 경로가 열렸는지: 로봇 위치가 아니라 지금 칸 중심 → 조준 칸 중심 직선에서 (grid.to_line)
            origin, target = center(path[0], p['cell_size']), center(aim_cell(path, cells), p['cell_size'])
            if len(path) == 1:
                origin = pose[:2]  # 이미 목표 칸 안: 칸 중심끼리는 길이 0이라 로봇 → 중심 직선을 봄
            points = self.points()
            # 몸 앞끝(0.165)까지 목표 중심 너머에 둘 자리를 요구하면 막다른 칸·집 칸(중심 0.2 m 뒤가 벽)은 여유가 35 mm뿐이라
            # SLAM이 몇 cm만 어긋나도 막힘이 됐음 (2026-10-30 Gazebo 시드 1: 집 칸 앞에서 복귀 실패). 칸을 막는 장애물 면은
            # 목표 중심보다 0.2 m 앞, 빈 칸 뒤 벽은 0.2 m 뒤 → 목표 중심까지만 봄 (front 0)
            if points is None or corridor_clear(to_line(points, pose, origin, target), math.dist(origin, target),
                                                p['path_half_width'], 0.0):
                break
            if cells == 1:
                near = sorted((x, y) for x, y in to_line(points, pose, origin, target)
                              if x > 0 and abs(y) < p['path_half_width'])[:3]
                self.get_logger().info(f'라이다: {cell} 칸으로 가는 직선이 막힘 → 돌아갑니다. 띠 안 점 (cm) '
                                       + ' '.join(f'({100 * x:.0f},{100 * y:.0f})' for x, y in near))
                self.block(cell)
                return 'moving'
        self.goal_steps += 1
        if move[0] == 'F':
            aim = aim_cell(path, cells)
            k = path.index(aim)
            # 경로가 꺾이는 칸이면 중심에 서도록 끝에서 늦춤: 안 늦추고 바로 돌아 13 cm 지나쳐 다음 직진이 벽 끝으로 들어갔음
            # (2026-10-31 실물 16차 217~225 s). 곧게 이어지면 그대로 (중간에 안 섬)
            turn = 0 < k < len(path) - 1 and (path[k + 1][0] - aim[0], path[k + 1][1] - aim[1]) != (
                aim[0] - path[k - 1][0], aim[1] - path[k - 1][1])
            c = center(aim, p['cell_size'])
            self.board(command_text(move), cell, (c[0] + shift[0], c[1] + shift[1]), stop=turn)
        else:
            self.board(command_text(move))
        return 'moving'

    def do_cruise(self):
        """Keep driving straight while the cells ahead are open and not yet passed; look around once blocked."""
        # 돈 뒤에는 격자 보정을 하고 출발: 출발 칸에서 TL90·TR91로 돌아본 뒤 보정 없이 1.4 m 직진해 실제로는 칸 중심보다
        # 7~10 cm 왼쪽으로 가 (8,4) 장애물에 왼쪽이 닿았음 (2026-10-31 실물 18:19 판 17~20 s, 왼쪽 PSD 979~1000)
        if not self.ready() or self.fix_due():
            return
        seen = self.target_seen()
        if seen == 'yes':
            return self.enter('APPROACH')
        if seen == 'wait':
            return
        if self.sight_turn_once():
            return
        states, pose, here = self.states(), self.pose(), self.here()
        # 바로 옆 칸이 아직 지도에 없고(모르는 칸) 경계에 벽도 안 보이면 지도가 채워지게 잠깐(1 s) 기다렸다 판단: 안 기다리고
        # 지나쳤다가 그 칸이 빈 칸이 되자 180° 돌아 보러 왔음 (2026-10-31 Gazebo 맵 1, 출발 한 칸 앞 (9,2))
        if pose is not None and self.now() - self.idle_since < 1.0:
            di, dj = axis(pose[2])
            if any(states.get(c) == UNKNOWN for c in ((here[0] - dj, here[1] + di), (here[0] + dj, here[1] - di))):
                return
        # 출발 칸에서도 옆을 돌아봄: 출발 칸 왼쪽 길(1열 위쪽)을 한 번도 안 보고 끝까지 탐색 안 했음 (2026-10-31 실물 16:30 판)
        if pose is not None and self.look_around(states, pose, dry=True):
            self.resume_axis = axis(pose[2])  # 돌아본 뒤 원래 가던 축으로 직진을 이어 감
            return self.enter('LOOK')
        cells = self.ahead(states)
        if not cells or pose is None:
            return self.enter('LOOK')
        self.drive_path(pose, [self.here()] + cells)
        self.resume_axis = None

    def do_goto(self):
        if not self.ready() or self.fix_due():
            return
        seen = self.target_seen()
        if seen == 'yes':
            return self.enter('APPROACH')
        if seen == 'wait':
            return
        if self.sight_turn_once():
            return
        # 쥐기 전: 가는 길에 (5,5)가 보이는 칸에 들어서면 목적지까지 가지 않고 거기서 돌아봄. 목적지는 출발 때 지도로 골라서
        # (5,5)가 보이는 칸을 몇 번 지나치고도 140 s 동안 미로를 돌았음 (Gazebo 20k seed 305: (5,3)·(3,2) 통과)
        here, start_cell = self.here(), tuple(self.p['cube_start_cell'])
        if (not self.ever_held and start_cell not in self.camera_seen and here not in self.look_tried
                and here in viewpoints(self.states(), {start_cell}, self.p['cell_size'], self.p['camera_range'],
                                       walls=self.walls)):
            self.look_tried.add(here)
            self.get_logger().info(f'가는 길 {here}에서 (5,5)가 보임: 돌아봅니다.')
            return self.next_goal()
        # 가는 도중 카메라로 다 본 목적지는 빼고 남은 가장 가까운 곳으로 (보는 자리는 그 자리에서 볼 칸이 남았는지로)
        self.goal_cells = {c for c in self.goal_cells
                           if (self.view_targets[c] - self.camera_seen if c in self.view_targets
                               else c not in self.camera_seen)}
        if not self.goal_cells:
            return self.arrive()
        if self.goal_steps > self.p['max_goal_steps']:
            self.get_logger().warn(f'{self.p["max_goal_steps"]}번 움직여도 목표에 못 감: 다른 목표로.')
            self.failed |= self.goal_cells
            return self.next_goal()
        result = self.step_toward(self.goal_cells, self.goal_strict)
        if result == 'arrived':
            self.arrive()
        elif result == 'no_path':
            # 지나간 칸을 안 거치는 길만 끊긴 것이면 목표만 다시 고름 (실패로 두면 경계 칸 전체가 빠짐)
            if not self.goal_strict:
                self.failed |= self.goal_cells
            self.next_goal()

    def sight_turn_once(self):
        """After a stop for a single red sighting that did not confirm, turn once toward where it was seen.

        멈춘 사이 큐브가 화면 끝(+32°) 밖으로 밀려 확인을 못 하고 그대로 가던 길을 갔음 (2026-10-31 실물 18:19 판 ~140 s)
        """
        turn, self.sight_turn = self.sight_turn, None
        if turn is None:
            return False
        self.get_logger().info(f'빨강이 보였던 쪽으로 {turn:+.0f}° 돌아 확인')
        self.board(command_text(('T', math.copysign(max(10, round(abs(turn))), turn))))
        return True

    def arrive(self):
        """GOTO goal done: LOOK if there is a side to turn to, else straight on to the next goal without the LOOK pause.

        보는 자리마다 LOOK에서 쉬었다 다음 목표(대개 한 칸 앞)로 가 통로를 한 칸씩 끊어 갔음 (2026-10-31 Gazebo 맵 4).
        이동 중에도 카메라는 보고 있어(watch_move) 쉬는 동안만 큐브를 볼 일은 없음
        """
        self.update_camera()
        if self.look_around(self.states(), self.pose(), dry=True):
            return self.enter('LOOK')
        self.visited.add(self.here())
        self.next_goal()

    def start_approach(self):
        self.approach_moves, self.blind_moves, self.tracked, self.cube_heading = 0, 0, False, None
        self.detour, self.final_pending = None, None  # (우회를 시작한 칸, 큐브 방향 칸들): 한 칸 옮길 때까지 큐브 쪽으로 다시 돌지 않음
        self.pocket, self.pocket_shift = None, None  # 주머니 속 큐브라고 본 칸: 입구로 도는 동안 화면에서 빠져도 이 칸 쪽으로 계속

    def do_approach(self):
        """Head for the cube's recorded direction in long LiDAR-clear runs; the PSD stop on the way is the grasp.

        2026-10-30 사용자 요청: 10 cm씩 끊어 가지 말고, 보이면 그 방향(지도 기준)을 기록해 장애물에 안 부딪히게
        그쪽으로 감. 한 번에 approach_max_mm까지(남는 방향 오차 5°에 큐브 50 mm를 안 놓치게 그만큼마다 다시 조준).
        직진 중 정면 PSD 550 이상이면 watch_move가 q로 세우고 on_stop이 파지로 봅니다.
        """
        p, pose = self.p, self.pose()
        if self.move is not None or pose is None:
            return
        # 회전 중에도 큐브가 집게 앞으로 들어옴(실물: TR16 중 PSD 534→700) → 움직이기 전에 PSD부터 봄
        # 정면 PSD만 보면 우회 뒤 돌아선 벽(PSD 900)도 큐브로 보고 빈 집게로 복귀했음 (2026-10-31 Gazebo 사용자 맵 2):
        # 큐브인지는 cube_at_gripper(카메라 중앙 또는 라이다 정면 빔)로
        front = self.psd_peak()[FRONT]
        if front >= p['psd_threshold'] and self.gripper_at(p['open_angles']) and self.cube_at_gripper():
            self.get_logger().info(f'정면 PSD 최대 {front}: 따라오던 목표물이 집게 앞 → 파지')
            return self.enter('GRASP')
        if self.approach_moves >= p['max_approach_steps']:
            self.get_logger().info('접근이 끝나지 않음: 다시 둘러봅니다.')
            return self.enter('LOOK')
        if not self.ever_held and self.approach_start_cell(pose):
            return
        if self.detour is not None:
            # 우회 중 큐브가 보일 때마다 큐브 쪽으로 돌았다가 막혀 다시 우회 쪽으로 돌기를 100 s 반복함
            # (2026-10-30 Gazebo 시드 9: TL50/TR50). 한 칸 옮긴 뒤에 다시 조준
            if self.here() == self.detour[0]:
                self.approach_moves += 1
                return self.go_detour()
            self.detour = None
        # 완료 직후 이어 가므로 이동 끝 무렵 받은 최신 검출도 사용합니다.
        fresh = self.seen(p['sample_timeout_s'])
        if fresh:
            err = math.degrees(fresh[-1])  # 양수 = 오른쪽
            self.tracked |= abs(err) < 2 * p['align_deg']
            self.cube_heading, self.blind_moves = pose[2] - math.radians(err), 0
        elif self.pocket is not None:
            # 주머니 입구로 돌아가는 동안 큐브가 화면 밖: 상실로 보지 않고 기억한 칸 쪽으로. 상실로 보면 처음부터 다시 찾느라
            # 100 s를 잃었음 (2026-10-30 Gazebo 에이전트 분석: 시드 6, 403 s)
            c = center(self.pocket, p['cell_size'])
            self.cube_heading = math.atan2(c[1] - pose[1], c[0] - pose[0])
        elif self.cube_heading is not None and self.now() - self.idle_since < p['confirm_window_s']:
            return  # 큐브 쪽으로 돈 직후 새 검출이 아직 없음: 상실로 보지 않고 기다림 (2026-10-30 Gazebo 시드 6: 네 번 다 상실)
        elif self.cube_heading is None or self.blind_moves >= p['max_blind_moves']:
            # 아직 중앙에 못 맞춘(먼) 큐브도 기록한 방향으로 max_blind_moves번은 감: 1.6 m 앞 큐브를 보고 돈 뒤 검출이
            # 끊기자 바로 상실로 보고 200 s를 다른 곳에서 썼음 (2026-10-31 Gazebo 사용자 맵 2). 파지는 cube_at_gripper가 막음
            self.get_logger().info('목표물 상실: 다시 둘러봅니다.')
            # 본 방향 칸들을 "카메라로 봄"에서 지워 다시 보러 가게 함. 안 지우면 큐브를 한 번 놓친 쪽은 탐색 목표에서 빠져
            # 끝까지 다시 안 갔음 (2026-10-30 Gazebo 시드 8)
            heading = self.cube_heading if self.cube_heading is not None else (
                pose[2] - self.sightings[-1][1] if self.sightings else None)
            if heading is not None:
                size = p['cell_size']
                self.camera_seen -= {cell_of(pose[0] + k * size * math.cos(heading),
                                             pose[1] + k * size * math.sin(heading), size) for k in range(1, 8)}
            return self.enter('LOOK')
        else:
            self.blind_moves += 1  # 가까워져 화면 아래로 빠짐: 기록한 방향으로 계속
        self.approach_moves += 1
        near = self.cube_at_gripper()
        if not near:
            # 옆이 막힌 칸(주머니) 속 큐브: 입구 칸 한가운데에 선 뒤 곧게 들어감 (grid.pocket_entries)
            size, states, here = p['cell_size'], self.states(), self.here()
            cells = self.cube_cells(pose, states)
            if self.pocket in cells:
                cells = [self.pocket]  # 정한 큐브 칸을 새 검출 방향도 지나면 유지: 검출마다 바뀌어 두 입구 칸 사이를 230 s
                #                        오갔음 (2026-10-30 Gazebo 에이전트 분석: 시드 12)
            # 큐브 칸과 사이에 벽(얇은 벽)이 보이는 칸은 입구 아님: 연습장 얇은 벽 너머 칸을 입구로 잡아 곧게 들어가려다 정면 벽
            # (25, 2) cm에 막히고 다른 입구로 3 m씩 돌기를 되풀이했음 (2026-11-01 실물 22:30 판 159 s)
            entries = [e for e in (pocket_entries(cells[0], states) if cells else [])
                       if frozenset((e, cells[0])) not in self.walls]
            if entries and here not in entries:
                self.get_logger().info(f'목표물이 {cells[0]} 칸(옆이 막힘) 안: 입구 칸 {sorted(entries)} 중 가까운 곳으로.')
                self.detour, self.pocket = (here, set(entries)), cells[0]
                return self.go_detour()
            # 15 cm 안이면 한가운데로 옮기지 않음: 6~10 cm 비킨 데서 그 점 쪽으로 돌면 45°쯤 돌아 큐브를 놓치고 LOOK ↔
            # APPROACH를 127번 되풀이했음 (Gazebo 18c 길 막힘 시험 161 s~)
            # 입구 칸에서는 라이다로 잰 큐브 칸 입구 가운데(corridor_mid)에 맞춤: 격자 칸 중심에 서도 실제 입구가 10 cm 넘게
            # 비켜 있어 옆 모서리(앞 24 cm·옆 6 cm)에 두 번 걸리고 다른 입구로 3 m씩 돌았음 (2026-11-01 실물 22:30 판)
            shift = (0.0, 0.0)
            if here in entries:
                c, pc = center(here, size), center(cells[0], size)
                u = ((pc[0] - c[0]) / size, (pc[1] - c[1]) / size)
                mid = self.corridor_mid(pose, c, u)
                if mid is not None:
                    shift = (-u[1] * mid, u[0] * mid)
                    side = -(pose[0] - c[0]) * u[1] + (pose[1] - c[1]) * u[0]
                    if abs(side - mid) > p['corridor_tol_m'] and self.pocket_shift is None:
                        self.get_logger().info(f'큐브 칸 입구 가운데({100 * mid:+.0f} cm)로 먼저: 옆으로 '
                                               f'{100 * (side - mid):+.0f} cm 비킴')
                        self.pocket_shift = shift  # 입구에서 한 번만 (맞춘 뒤 다시 재면 흔들림)
                    else:
                        shift = self.pocket_shift or shift
            # 15 cm 안이면 옮기지 않음 (위 18c 주석): 입구 가운데 3 cm 안으로 맞추려 F100(최소 직진)을 내 매번 지나쳐 벽에 닿기를
            # 28번 되풀이하고 "60 s 안에 정지 9회"로 끝났음 (Gazebo 19d m4). 입구 가운데는 들어가는 방향(pocket_ahead)에만 씀
            if here in entries and math.dist(pose[:2], center(here, size)) > 0.15:
                c = center(here, size)  # 입구 칸 한가운데로 (칸 축 위에서 큐브를 정면에 둠)
                c = (c[0] + shift[0], c[1] + shift[1])
                # 그 점 쪽으로 먼저 돎: 옆·뒤에 있는 점으로 바로 F를 보내면 남은 거리가 0이라 곧바로 "도착"으로 끝나 F100만
                # 35번 되풀이하고 접근을 포기했음 (2026-10-31 Gazebo 18차 탈취 시험 274~297 s)
                b = math.degrees(math.remainder(math.atan2(c[1] - pose[1], c[0] - pose[0]) - pose[2], 2 * math.pi))
                if abs(b) >= p['approach_turn_deg']:
                    return self.board(command_text(('T', math.copysign(max(10, round(abs(b))), b))))
                return self.board(f'F{max(100, round(math.dist(pose[:2], c) * 1000))}', point=c)
        turn = math.degrees(math.remainder(self.cube_heading - pose[2], 2 * math.pi))
        # 크게 벗어났을 때만 제자리 회전, 나머지는 직진하면서 방향을 고침. 집게 바로 앞에서는 안 돎
        if abs(turn) >= p['approach_turn_deg'] and not near:
            return self.board(command_text(('T', math.copysign(max(10, round(abs(turn))), turn))))
        points, limit = self.points(), p['approach_max_mm'] / 1000
        if points is not None:  # 큐브 방향 띠
            points = to_line(points, pose, pose[:2], (pose[0] + math.cos(self.cube_heading),
                                                      pose[1] + math.sin(self.cube_heading)))
        free = limit if points is None else min(free_distance(points, p['body_radius'], p['body_radius'], limit),
                                                free_distance(points, p['front_half_width'], p['front_extent'], limit))
        if free >= 0.1 or near:
            d = max(0.1, free)
            goal = (pose[0] + d * math.cos(self.cube_heading), pose[1] + d * math.sin(self.cube_heading))
            self.board(f'F{round(d * 1000)}', point=goal)
            self.motion['cube'] = True  # 이 직진만 카메라로 다시 조준 (follow_cube)
            return
        # 그 방향 바로 앞이 막힘(라이다): 큐브 칸(방향선이 처음 지나는 칸 중심)의 상하좌우 빈 칸 중 가까운 곳에 칸
        # 한가운데로 가서 큐브를 칸 축 정면에 두고 곧게 들어감 (2026-10-31 사용자: "격자니까 x·y축을 맞춘 뒤 그 방향으로
        # 돌면 되지 않나" — 실물 17:07 판은 비스듬히 돌아가다 통로 모서리에 두 번 걸려 눈앞 큐브를 못 잡음)
        size = p['cell_size']
        states, here = self.states(), self.here()
        cells = self.cube_cells(pose, states)
        if cells and cells[0] != here:
            entries = {(cells[0][0] + di, cells[0][1] + dj) for di, dj in NEIGHBOURS} & {
                c for c, s in states.items() if s == FREE} - {
                c for c in states if frozenset((c, cells[0])) in self.walls}
            if entries and here not in entries:
                self.get_logger().info(f'목표물 {cells[0]} 칸 앞이 막힘: 옆 칸 {sorted(entries)} 중 가까운 곳 한가운데로 가서 곧게.')
                self.detour, self.pocket = (here, entries), cells[0]
                return self.go_detour()
        # 방향선이 중심 근처를 지나는 빈 칸들로 감. 선 위 칸을 아무거나(막힌 칸·경기장 밖 포함) 잡으면 길이 없어 보이는
        # 큐브를 버리고 탐색으로 돌아갔음 (2026-10-30 Gazebo: 사용자 미로 86 s에 보고도 238 s에야 접근)
        ray = set(ray_cells(pose, self.cube_heading, self.states(), size)) - {self.here()} or {
            cell_of(pose[0] + k * size * math.cos(self.cube_heading),
                    pose[1] + k * size * math.sin(self.cube_heading), size) for k in range(1, 6)} - {self.here()}
        self.get_logger().info('목표물 방향 앞이 막힘: 장애물을 돌아 그 방향 칸으로 갑니다.')
        self.detour = (self.here(), ray)
        self.go_detour()

    def cube_cells(self, pose, states):
        """Candidate cube cells along the camera ray, nearest first; the rule's start cell (5,5) first if the ray passes it.

        카메라는 방향만 줌. 방향선 위 첫 빈 칸을 큐브 칸으로 잡으면 자리를 옮길 때마다 바뀌어 입구 칸을 바꿔 가며 맴돌았음
        (2026-10-31 실물 17:28 판: (4,4) → (5,3)). 규정상 큐브는 (5,5) 칸 중심에서 시작하므로 방향선이 그 칸을 지나면 그 칸
        """
        size = self.p['cell_size']
        cells = ray_cells(pose, self.cube_heading, states, size)
        start = tuple(self.p['cube_start_cell'])
        # 막힌 칸 가려짐은 보지 않음: 카메라가 실제로 봤으니 시선은 열려 있음. 장애물 모서리 틈으로 1.8 m 앞 (5,5)를 봤는데
        # 방향선이 모서리 칸을 스쳐 "가려짐"으로 빼고 앞의 빈 주머니 칸을 큐브 칸으로 잡아, 벽에 파지·150 s 맴돌다 자세를
        # 잃었음 (2026-11-01 Gazebo m4 두 판). ponytail: 큐브가 옮겨진 뒤(운반 중 놓침)에도 (5,5)를 먼저 봄
        sx, sy = center(start, size)
        c, s = math.cos(self.cube_heading), math.sin(self.cube_heading)
        # 한 번도 쥐기 전이면 큐브는 규정대로 (5,5)에 그대로 (미션 1은 로봇 한 대): 방향선이 비켜도 (5,5). 화면 끝(-31°)
        # 검출로 방향이 18° 틀어져 옆 칸 (5,4)를 큐브 칸으로 잡고 입구를 찾아 69 s 맴돌았음 (2026-11-01 실물 21:01 판)
        if states.get(start) != BLOCKED and (not self.ever_held or (
                (sx - pose[0]) * c + (sy - pose[1]) * s > 0 and abs(-(sx - pose[0]) * s + (sy - pose[1]) * c) < 0.2)):
            cells = [start] + [cell for cell in cells if cell != start]
        return cells

    def go_detour(self):
        if self.step_toward(self.detour[1]) == 'no_path':
            self.detour = None
            # 잠깐 큐브를 못 본 것으로: 안 그러면 GOTO가 바로 큐브를 보고 APPROACH로 돌아와 길 없음 → GOTO → APPROACH를
            # 움직이지 않고 690 s 되풀이했음 (2026-10-31 Gazebo l5 8차)
            self.target_hold_until = self.now() + self.p['target_hold_s']
            # LOOK은 큐브가 보이면 바로 APPROACH로 돌아와 한 번도 안 움직이고 0.3 s마다 반복함 (2026-10-30 Gazebo 시드 7:
            # 400 s). 큐브 칸은 라이다 지도에 안 잡히므로 길이 없으면 쌓인 막힘 표시 탓 → 비우고 다시. 비울 것도 없으면 탐색으로
            if self.unblock():
                self.get_logger().warn('목표물 쪽 길 없음: 막힘 표시를 비우고 다시 찾습니다.')
            else:
                self.next_goal()

    def start_grasp(self):
        self.grip_tries = 0
        if self.p['grasp_push_m'] > 0:
            # 정면 PSD로 선 자리에서 grasp_push_m만 더 들어간 뒤 닫음: 큐브가 집게 끝에 걸쳐 잡혔음 (2026-11-01 사용자
            # "살짝만 더 앞으로 간 뒤에 집으면"). 보드 정면 PSD 정지를 먼저 끔(PF0), 닫기는 do_grasp가
            self.board('PF0')
            return self.board(f'C{round(1000 * self.p["grasp_push_m"])}')
        self.grip()

    def grip(self):
        self.board('PF0')  # 운반 모드 먼저 (보드 정면 PSD 정지 끔). 쥔 큐브가 정면 PSD 앞이라 안 끄면 못 움직임
        self.board('8')
        self.grip_tries, self.since = self.grip_tries + 1, self.now()

    def do_grasp(self):
        p = self.p
        if self.grip_tries == 0:  # 더 들어가기(C)가 끝남
            return self.grip()
        if self.gripper_at(p['grip_angles']):
            self.holding = self.ever_held = True
            return self.enter('RETURN')
        if self.now() - self.since > p['grip_timeout_s']:
            if self.grip_tries < 2:
                return self.grip()
            self.get_logger().warn('집게 각도를 확인하지 못했지만 복귀합니다.')
            self.holding = self.ever_held = True
            self.enter('RETURN')

    def start_return(self):
        self.return_retries, self.goal_steps, self.plan = 0, 0, []
        # 왔던 길(파지 전 자취)을 거꾸로, 지도에서 비어 있으면 사선 지름길로 (grid.trail_route)
        pose, p = self.pose(), self.p
        self.trail, self.trail_sent, self.trail_turned, self.trail_prev = [], None, None, None
        self.trail_centred = None  # 구간 중심선 점을 이미 넣은 경유점 (follow_trail, 한 번만)
        if pose is None:
            return
        driven = []
        for _, x, y, _, _ in self.track:
            if not driven or math.dist(driven[-1], (x, y)) >= 0.1:
                driven.append((x, y))
        grid = occupancy(self.hits, self.misses, p['map_min_misses'])
        clear = clear_pixels(grid, p['map_resolution'], p['trail_clearance_m'])
        wide = clear_pixels(grid, p['map_resolution'], p['trail_diag_clearance_m'])
        self.trail = trail_route(pose[:2], driven, clear, self.map_origin, p['map_resolution'],
                                 p['cell_size'], wide)
        self.get_logger().info('복귀: 왔던 길 경유점 ' + ' '.join(f'({x:.2f},{y:.2f})' for x, y in self.trail))

    def follow_trail(self):
        """Next turn or straight along self.trail; False once it is done or abandoned (then grid moves take over).

        직진했는데 경유점에 못 닿았으면(라이다가 막아 섬) 지름길을 버리고 지도 최단 경로(step_toward)로 바꿈
        """
        pose, p = self.pose(), self.p
        if pose is None:
            return True
        # 직진을 보낸 경유점은 trail_short_m 안이면 닿은 것으로: 꺾이는 칸 앞 벽 모서리에 라이다가 0.2~0.3 m 앞에서 세워
        # "못 닿음"으로 지름길을 버렸음 (2026-10-31 Gazebo 6맵 모두 한 번씩)
        # 보내기 전 가까운 경유점은 건너뛰지 않음: 꺾이는 경유점을 건너뛰면 다음 경유점으로 모서리를 깎고 가 옆벽에 붙어
        # 앞 14 cm에서 섰음 (Gazebo l5 7차). 0.2 m 앞 경유점을 두고 TL30·TR27…를 되풀이한 것(m2 4차)은 한 번만 돎이 막음
        while self.trail and math.dist(pose[:2], self.trail[0]) < (
                p['trail_short_m'] if self.trail_sent == self.trail[0] else p['trail_arrive_m']):
            self.trail_prev = self.trail.pop(0)
            self.trail_sent = self.trail_turned = None
        if not self.trail:
            return False
        point = self.trail[0]
        prev = self.trail_prev
        if prev is not None and self.trail_sent != point and math.dist(prev, point) > 0.05:
            # 구간 중심선에서 center_tol_m 넘게 비켜 있으면 먼저 30 cm 앞 중심선 위로: 쥐면 앞끝이 길어 꺾이는 칸 중심 10 cm
            # 앞에서 서고, 돈 뒤 비스듬히 가다 옆 모서리(옆 11 cm)에 라이다가 세워 "못 닿음" → 먼 길로 100 s 넘게 돌았음
            # (2026-11-01 Gazebo m4·real 복귀). 중심선 점은 경유점 0.3 m 앞까지만 (그보다 가까우면 그냥 경유점으로)
            ux, uy = (point[0] - prev[0]) / math.dist(prev, point), (point[1] - prev[1]) / math.dist(prev, point)
            along = (pose[0] - prev[0]) * ux + (pose[1] - prev[1]) * uy
            off = -(pose[0] - prev[0]) * uy + (pose[1] - prev[1]) * ux
            # 경유점마다 한 번만, 20 cm 안으로 비켰을 때만: 28 cm 비킴(칸 반 넘게 = 다른 줄)에서 중심선 점으로 가다 옆벽에 서고
            # "닿음"(45 cm 안)으로 쳐 다시 중심선 점을 넣기를 295번 되풀이했음 (Gazebo 20i real2 2판 142~267 s)
            if (p['center_tol_m'] < abs(off) <= 0.2 and along + 0.6 < math.dist(prev, point)
                    and self.trail_centred != point):
                self.trail_centred = point
                self.trail_prev = None  # 중심선 점에 닿으면 그 점이 다음 구간의 시작
                self.trail.insert(0, (prev[0] + (along + 0.3) * ux, prev[1] + (along + 0.3) * uy))
                self.get_logger().info(f'복귀: 구간 중심선에서 {100 * off:+.0f} cm 비킴 → 30 cm 앞 중심선으로 먼저')
                return self.follow_trail()
        if self.trail_sent == point:
            self.get_logger().info(f'복귀: 경유점 {point[0]:.2f},{point[1]:.2f}에 못 닿음 → 지도 최단 경로로 바꿈')
            self.trail = []
            return False
        err = math.degrees(math.remainder(math.atan2(point[1] - pose[1], point[0] - pose[0]) - pose[2], 2 * math.pi))
        # 칸 경로(turn_min_deg 30°)보다 작은 각도에서도 먼저 돎: 30° 안은 돌지 않고 조향하며 가 통로를 비스듬히 지나
        # 옆벽 모서리(앞 25 cm·옆 11 cm)에 라이다가 섰음 (Gazebo l5: 복귀 중 정지 6번)
        # 한 경유점에는 한 번만 돎: 돈 뒤 남은 틀어짐은 직진 조향이 고침 (돌 때마다 자세가 조금씩 바뀌어 방위가 흔들렸음)
        if abs(err) >= p['trail_turn_deg'] and self.trail_turned != point:
            self.board(f'T{"L" if err > 0 else "R"}{abs(round(err))}')
            self.trail_turned = point
        else:
            # 경유점은 꺾이는 곳이라 중심에 서도록 끝에서 늦춤(stop): 빠르게 들어가 칸 중심 앞에서 라이다로 서면 돈 뒤 새 통로
            # 가운데서 그만큼 비켜 있어 사선으로 가다 모서리에 걸렸음 (Gazebo m1 TL77 → 앞 16 cm·옆 6 cm 정지)
            self.board(f'F{round(1000 * math.dist(pose[:2], point))}', None, point, stop=True)
            self.trail_sent = point
        return True

    def do_return(self):
        """Back along the trail (follow_trail), then grid moves home.

        경로가 없으면 막힘 표시를 비우고 옆 칸으로 한 칸 옮겨 다시 계획합니다 (16회차: 같은 자리에서 같은
        요청만 157번 반복). max_return_retries 넘으면 기록을 모두 비우고 처음부터 다시 (멈추지 않음).
        """
        if not self.ready() or self.fix_due():
            return
        if self.trail and self.follow_trail():
            return
        result = self.step_toward({(0, 0)}, 'return')
        if result == 'arrived':
            if self.home_walls() or self.return_retries >= self.p['max_return_retries']:
                return self.enter('DONE')
            # 집 칸이라는데 뒤·오른쪽 바깥벽이 안 보임: 위치를 잃은 것. 큐브 자리에서 "도착"하고 끝났음 (실물 3차).
            # 옆 칸으로 옮겨 다시 (아래 no_path와 같은 처리). max_return_retries번 넘으면 그냥 끝냄
            self.get_logger().warn('집 칸이라는데 출발 구석 벽(뒤·오른쪽)이 안 보입니다: 다시 찾습니다.')
            result = 'no_path'
        if result != 'no_path':
            return
        self.return_retries += 1
        if self.return_retries > self.p['max_return_retries']:
            # 끝내지 않음: 큐브를 쥔 채 집 앞에서 끝내면 그 판은 무조건 실패 (2026-10-30 Gazebo 시드 1·5: 집에서 0.4~0.6 m).
            # 기록을 다 비우고 처음부터 다시 계획. 끝은 경기 시간이 정함
            self.get_logger().error('복귀 경로를 찾지 못했습니다: 기록을 모두 비우고 다시 계획합니다.')
            self.return_retries = 0
            self.passed.clear()
            self.failed.clear()
            # 확정 막힘(block_confirm번)도 비움: 남겨 두면 사방이 막힌 채 움직이지 않고 매 틱 이 줄만 반복했음 (14:00 뒤 팀원
            # 수정을 되돌렸다가 Gazebo m4에서 400 s 제자리 재계획). 진짜 장애물이면 라이다 검사가 다시 막음
            self.blocked = {c for c in self.blocked if self.now() - self.block_at.get(c, -math.inf) < 10.0}  # 방금 막힌 칸만 남김
            self.block_hits.clear()
        here, states = self.here(), self.states()
        around = ' '.join(f'{c}:{states.get(c, "?")[0]}{"x" if c in self.blocked else ""}'
                          for c in [here] + [(here[0] + di, here[1] + dj) for di, dj in NEIGHBOURS])
        self.get_logger().warn(f'복귀 경로 없음 ({self.return_retries}회): 막힘 표시를 비우고 옆 칸으로 옮겨 다시 계획. '
                               f'칸 상태 {around}, 집 {states.get((0, 0))}, 막힘 표시 {sorted(self.blocked)}')
        self.unblock()
        sideways = {(here[0] + di, here[1] + dj) for di, dj in NEIGHBOURS}
        self.step_toward({c for c in sideways if self.passable(states)(c)} or sideways)

    def home_walls(self, near=0.04, count=8):
        """Both outer walls of the start corner (behind and right of the start pose) show in the scan."""
        pose, points = self.pose(), self.points()
        if pose is None or not points:
            return False
        c, s, h = math.cos(pose[2]), math.sin(pose[2]), self.p['cell_size'] / 2
        world = [(pose[0] + x * c - y * s, pose[1] + x * s + y * c) for x, y in points]
        behind = sum(1 for x, y in world if abs(x + h) < near and abs(y) < h)
        right = sum(1 for x, y in world if abs(y + h) < near and abs(x) < h)
        return behind >= count and right >= count

    def start_done(self):
        self.get_logger().info(
            f'미션 종료 ({self.now() - self.start:.0f} s, 목표물 {"보유" if self.holding else "없음"})')
        self.save_record()

    def save_record(self):
        """Own map, believed path and camera-seen cells → ~/mission1_record_<시각>.npz, once (분석용, 미션 판단에 안 씀).

        실물 판의 경로를 지도 위에 그려 헛움직임을 판단하려면 지도가 있어야 함: 끝난 뒤에는 /mission_map이 없어 경기장 밖
        헛빈칸이 왜 생겼는지 확정 못 했음 (2026-10-31 실물 10차)
        """
        if self.saved:
            return
        self.saved = True
        path = Path.home() / time.strftime('mission1_record_%m%d_%H%M%S.npz')
        try:
            np.savez_compressed(path, hits=self.hits, misses=self.misses, origin=self.map_origin,
                                resolution=self.p['map_resolution'], fix=self.fix,
                                track=np.array([t[:4] for t in self.track]),
                                track_state=np.array([t[4] for t in self.track]),
                                camera_seen=np.array(sorted(self.camera_seen)))
            self.get_logger().info(f'기록 저장: {path}')
        except OSError as e:
            self.get_logger().warn(f'기록 저장 실패: {e}')

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
    except Exception:
        if rclpy.ok():
            raise
        # 종료 신호 뒤 spin이 한 번 더 돌며 나는 RCLError(context invalid): 노드가 exit 1로 끝났음 (Gazebo m2 7차)
    finally:
        if node is not None:
            node.save_record()  # 중간에 멈춰도 그때까지의 지도·경로를 남김
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
