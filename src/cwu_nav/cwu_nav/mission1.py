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
                          scan_points, to_line, wall_push, blocking_points, ray_cells,
                          pocket_entries, viewpoints, best_heading, seen_faces, lattice_fix, compose, invert,
                          return_route, wall_heading, mark_scan, occupancy,
                          wall_faces, walled_off, sweep_hits, open_frontier)
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
            'drive_fix_period_s': 0.5, 'drive_fix_gain': 0.5, 'drive_fix_max_deg': 5.0,
            'look_min_free_m': 0.45, 'look_half_width': 0.12,
            'map_still_turn_s': 0.4,
            'map_resolution': 0.05, 'map_min_misses': 2, 'map_size_m': 16.0, 'map_range_m': 4.0, 'map_period_s': 0.25,
            'mission_map_hz': 3.0,
            'cell_size': 0.4, 'turn_min_deg': 30.0, 'max_cells': 2,
            'body_radius': 0.165, 'front_extent': 0.255, 'front_extent_holding': 0.28, 'front_half_width': 0.12,
            'path_half_width': 0.03,
            'wall_keep': 0.175, 'k_wall': 5.0, 'max_wall_push': 0.25, 'near_wall_m': 0.25,
            'settle_s': 0.0, 'move_timeout_s': 20.0, 'max_goal_steps': 30, 'max_return_retries': 3,
            'block_confirm': 3,
            'drive_speed': 0.12, 'approach_speed': 0.08, 'turn_speed': 0.6,
            'min_turn_speed': 0.4, 'k_heading': 1.5, 'max_steer': 1.0,
            'turn_tol_deg': 4.0, 'arrive_m': 0.04, 'approach_turn_deg': 25.0, 'arm_retry_s': 1.0,
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
        self.walls, self.entries = frozenset(), set()  # 빈 칸 사이 바깥벽·새 길 입구 (states가 갱신)
        self.plan = []  # 지금 따라가는 칸 경로 (step_toward)
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
        if self.target_confirmed():
            return 'yes'
        p = self.p
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
        self.fix = (a + da, c * (tx - px) - s * (ty - py) + px, s * (tx - px) + c * (ty - py) + py)
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
        return [(x, y) for x, y in scan_points(self.scan.ranges, self.scan.angle_min, self.scan.angle_increment, yaw)
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

        경기장 크기 제한 없음 (2026-10-30 사용자 요청): 지도가 커지면 칸 범위도 커집니다. 출발 칸·지금 칸
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
        self.walls = wall_faces(grid, res, (x0, y0), states, size)  # 빈 칸 사이 바깥벽: 카메라 시선이 못 지나감
        self.entries = open_frontier(grid, res, (x0, y0), states, size)  # 빔이 지나간 새 길 입구 (next_goal)
        return walled_off(states, [(0, 0), self.here()], self.walls)  # 벽 너머(경기장 밖) 칸은 막힘

    def here(self):
        pose = self.pose()
        return cell_of(pose[0], pose[1], self.p['cell_size']) if pose is not None else (0, 0)

    def passable(self, states):
        return lambda c: c in states and c not in self.blocked and states[c] != BLOCKED

    # ── 출력 ──
    def block(self, cell):
        self.blocked.add(cell)
        self.plan = []  # 막혔으니 경로를 새로
        self.block_hits[cell] += 1

    def unblock(self):
        """Forget blocked cells except those blocked block_confirm times or more; True if any were forgotten.

        라이다가 바로 옆에서 거듭 막혔다고 본 칸은 진짜 장애물: 지도에서 면을 못 잡아 지나갈 수 있다고 나오는 장애물 칸을
        비울 때마다 다시 경로에 넣어 같은 칸 앞에서 220 s를 멈춰 있었음 (2026-10-31 Gazebo 사용자 맵 2, 복귀 중 (2,4)).
        """
        keep = {c for c in self.blocked if self.block_hits[c] >= self.p['block_confirm']}
        forgotten = self.blocked - keep
        self.blocked = keep
        return bool(forgotten)

    def drive(self, linear=0.0, angular=0.0):
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
        if text[0] not in 'FTB':
            return self.command.publish(String(data=text))
        pose = self.pose() or (0.0, 0.0, 0.0)
        if text[0] == 'F':
            if point is None:
                d = int(text[1:]) / 1000
                point = (pose[0] + d * math.cos(pose[2]), pose[1] + d * math.sin(pose[2]))
            # 시간 제한은 거리에 맞춤: 복귀는 한 번에 여러 칸 직진 (drive_path)
            self.motion = {'kind': 'F', 'point': point, 'stop': stop,
                           'h': math.atan2(point[1] - pose[1], point[0] - pose[0]), 'sight': self.now(),
                           'timeout': self.p['move_timeout_s'] + math.dist(pose[:2], point) / self.p['drive_speed']}
        elif text[0] == 'B':  # 미션 안에서만 쓰는 짧은 후진 (보드 명령 아님, cmd_vel 음수)
            self.motion = {'kind': 'B', 'start': pose[:2], 'dist': int(text[1:]) / 1000}
        else:
            deg = int(text[2:]) * (1 if text[1] == 'L' else -1)
            # 크게 돌 때 집게가 쓸고 지나갈 쪽에 벽이 있고 반대로 돌면 비어 있으면 반대로 (집게가 통로 옆벽보다 김, 13차)
            points = self.points()
            if points and abs(deg) >= 60:
                reach = self.front_extent() + 0.02
                other = deg - math.copysign(360, deg)
                if (sweep_hits(points, math.radians(deg), reach)
                        and not sweep_hits(points, math.radians(other), reach)):
                    self.get_logger().info(f'{text}: 집게가 그쪽 벽에 닿아 반대로 {other:+.0f}° 돎')
                    deg = other
            self.motion = {'kind': 'T', 'dir': math.copysign(1.0, deg), 'total': math.radians(abs(deg)),
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
        if self.move is not None:
            self.watch_move()
            if self.move is not None:
                return
        self.update_fix()  # 멈춰 있을 때만 (돌거나 가는 중에는 스캔과 자세 시각이 어긋남)
        if self.state in ('LOOK', 'GOTO', 'CRUISE'):
            self.passed.add(self.here())  # 큐브를 찾기 전 탐색 중에 들어간 칸
        getattr(self, 'do_' + self.state.lower())()
        if self.move is None:
            self.drive()  # 할 일이 없을 때만 섬 (encoder_odom이 [V0,0] → 보드 정지·무장 유지)

    def watch_move(self):
        text, sent, _ = self.move
        now = self.now()
        if self.result is None:
            if self.state in ('LOOK', 'GOTO', 'CRUISE') and self.target_confirmed(after=sent):
                # 움직이는 중 큐브 확인: 바로 멈추고 접근으로 (멈춘 뒤 검출을 다시 기다리지 않게 기준 시각을 당김)
                self.drive()
                self.result, self.idle_since = ('done', now), sent
                self.move = None
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
        if m['kind'] == 'B':
            if math.dist(pose[:2], m['start']) >= m['dist']:
                self.result = ('done', now)
                return
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
        if self.state == 'APPROACH':
            self.follow_cube(m, pose)
        dx, dy = m['point'][0] - pose[0], m['point'][1] - pose[1]
        left = dx * math.cos(m['h']) + dy * math.sin(m['h'])  # 진행 방향으로 남은 거리
        if left <= p['arrive_m']:
            self.result = ('done', now)  # 멈추지 않음: 다음 이동이 바로 이어 감
            if abs(m.get('yaw_fix', 0.0)) > math.radians(0.5):
                self.get_logger().info(f'직진 중 방향 보정 합계 {math.degrees(m["yaw_fix"]):+.1f}°')
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
        blockers = (blocking_points(front, p['body_radius'], p['body_radius'], 0.02, inside=0.01)
                    + blocking_points(front, p['front_half_width'], self.front_extent(), 0.02, inside=0.01))
        if len(blockers) >= 3:  # 잡음 두 점까지는 무시 (free_distance와 같음)
            # 목표 중심 10 cm(앞끝이 길면 앞끝 - 15 cm) 앞: 더 못 갈 만큼 왔음 → 도착. 막다른 칸·집 칸은 중심 20 cm 뒤가 벽이라
            # 앞끝이 길수록 일찍 섬 (2026-10-31 사용자: 앞 +4 cm → 쥐면 중심 10 cm 앞에서 섬)
            if left < max(0.10, self.front_extent() - 0.15):
                self.drive()
                self.result = ('done', now)
                return
            side = sum(y for _, y in blockers) / len(blockers)
            if min(abs(y) for _, y in blockers) >= 0.08 and now - m.setdefault('pivot', now) < 3.0:
                # 옆쪽 모서리만 걸림: 서지 않고 반대쪽으로 제자리 회전 → 모서리가 띠에서 빠지면 그대로 출발. 서면 같은 자리에서
                # 같은 직진을 1초마다 60번 넘게 되풀이했음 (2026-10-30 Gazebo 사용자 미로: 장애물 모서리 앞 12 cm).
                # 이동 한 번에 처음 비켜 돈 때부터 3 s까지만 (비키기 ↔ 목표 쪽 되돌기 반복 방지), 넘으면 정지
                return self.drive(0.0, -math.copysign(p['min_turn_speed'], side))
            self.drive()
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
        err = 0.0 if math.hypot(dx, dy) < 0.08 else math.remainder(math.atan2(dy, dx) - pose[2], 2 * math.pi)
        speed = p['approach_speed'] if self.state == 'APPROACH' else p['drive_speed']
        if m.get('stop'):
            speed = min(speed, max(0.03, 0.8 * left))  # 꺾이는 칸: 끝에서 늦춰 중심에 섬
        push = 0.0 if points is None or near_cube else wall_push(points, p['wall_keep'], p['k_wall'])
        # 상한: 크면 목표에서 0.5 rad 넘게 돌아가 전진 0으로 버팀 → 20 s 시간 초과 (2026-10-30 Gazebo v10 시드 6)
        push = max(-p['max_wall_push'], min(p['max_wall_push'], push))
        steer = max(-p['max_steer'], min(p['max_steer'], p['k_heading'] * err + push))
        self.drive(speed if abs(err) < 0.5 else 0.0, steer)

    def cube_at_gripper(self):
        """Front PSD is close and it is the tracked cube, not a wall.

        정면 PSD 값만으로는 큐브와 벽을 못 가름: 주머니 벽을 큐브로 보고 10 cm 직진만 38번 되풀이했음 (2026-10-31 Gazebo
        에이전트 분석: 시드 6). 카메라가 최근 중앙에 봤거나(큐브가 벽에 붙은 실물 10-30 경우), 라이다가 정면 near_wall_m 안에서
        아무것도 못 보면(큐브는 라이다보다 낮음, 벽은 보임) 큐브로 봄.
        """
        p, t = self.p, self.telemetry
        if not (self.tracked and t is not None and t[FRONT] >= p['near_front_min']):
            return False
        if self.target_centred():
            return True
        points = self.points()
        return points is None or free_distance(points, 0.08, self.front_extent(), p['near_wall_m']) >= p['near_wall_m']

    def follow_cube(self, m, pose):
        """Approach run: every new detection re-aims the remaining distance at the cube (연속 시각 서보)."""
        p = self.p
        if self.cube_at_gripper():
            return  # 집게 바로 앞: 방향을 더 바꾸지 않고 직진 (큐브가 화면 아래로 잘려 검출이 흔들림)
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
        centred = self.target_centred() or (self.state == 'APPROACH' and self.tracked)
        psd = self.psd_peak()
        # 집게가 다 열려 있을 때만: 닫히는 중·반쯤 닫힌 날도 정면 PSD를 가립니다
        approaching = self.state == 'APPROACH' and text[0] == 'F' and self.gripper_at(p['open_angles'])
        if is_grasp(event, psd, p['grasp_front_min'], approaching, centred):
            self.get_logger().info(f'정면 PSD 최대 {psd[FRONT]}: 목표물 파지')
            return self.enter('GRASP')
        t = self.now()
        # 라이다 예방 정지(Pi)는 부딪힘이 아니므로 세지 않음: 좁은 통로에서 몇 번 나면 미션 전체가 끝났음
        # (2026-10-30 Gazebo 시드 6·7: "60 s 안에 정지 9회"로 큐브를 못 찾은 채 종료)
        if (event.startswith('# ABORT') and 'LIDAR' not in event
                and not (self.aborts and t - self.aborts[-1] < 1.0)):
            self.aborts.append(t)
        self.aborts = [a for a in self.aborts if t - a < p['abort_window_s']]
        self.get_logger().warn(f'{text} → {event} ({self.state}, PSD 앞·뒤·좌·우 최대 {psd[FRONT:FRONT + 4]})')
        if len(self.aborts) > p['max_aborts']:
            self.get_logger().error(f'{p["abort_window_s"]:.0f} s 안에 정지 {len(self.aborts)}회: 미션 중단')
            return self.enter('DONE')
        if cell is not None and any(k in event for k in ('PSD', 'LIDAR')):
            # 장애물 때문에 선 것만 그 칸을 막힘으로 둠. 엔코더·하트비트·시간 초과 정지까지 막힘으로 두면 열린 칸이
            # 쌓여 "갈 곳 없음"이 반복됨 (2026-10-30 실물: ENCODER_DIRECTION마다 칸이 막힘으로 남음)
            self.block(cell)
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
        unseen = {c for c, state in states.items() if state == FREE} - self.camera_seen - self.passed
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
        # 둘러보기가 끝날 때마다 지금 칸이 visited에 들어가므로 "비울 게 남았나"로 고르면 첫 단계만 반복함 (실물)
        if not self.cleared:
            self.cleared = True
            self.get_logger().warn('갈 곳이 없습니다. 막힘·실패·둘러본 칸을 비우고 다시 찾습니다 (지나간 칸 기록은 유지).')
            self.unblock()
            self.failed.clear()
            self.visited.clear()
        else:
            self.get_logger().warn('지나가지 않은 칸이 남지 않았습니다. 지나간 칸 기록을 비우고 다시 찾습니다.')
            self.passed.clear()
            # 본 기록은 두고 막힘 표시를 모두(거듭 막힌 칸까지) 비움: 갈 곳이 없어지는 건 대개 가짜 막힘 때문인데, 본 기록을 지우면
            # 출발 줄·(1,1)만 끝없이 다시 돌아봤음 (2026-10-31 실물 11차: 앞끝 판정이 옆벽을 막힘으로 봄)
            self.blocked.clear()
            self.block_hits.clear()
            self.cleared = False
        self.enter('LOOK')

    def look_around(self, states, pose, dry=False, min_cells=1):
        """Turn toward the unseen cells visible from here; True when a turn was sent (or would be, with dry).

        정면 쪽(±화각/4)이면 직진하며 보이므로 기록만 하고 돌지 않음. 못 본 칸 하나 때문에 90° 넘게 돌지 않음: 둘러보기가
        한 판에 10~17번, 그중 다수가 1~2칸에 60~144°라 시간을 잃었음 (2026-10-30 Gazebo 에이전트 분석: 시드 2·8, 5·6).
        """
        p, here = self.p, self.here()
        unseen = {c for c, state in states.items() if state == FREE} - self.camera_seen - self.passed
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
            need = max(p['look_min_free_m'], nearest - p['cell_size'] / 2)
            free = free_distance(view, p['look_half_width'], 0.0, need)
            if free < need:
                if not dry:
                    self.camera_seen |= looked
                    self.get_logger().info(f'둘러보기 안 함: {turn:+.0f}° 쪽 {free:.2f} m 앞이 벽')
                return False
        if not dry:
            self.camera_seen |= looked  # 돌고 나면 화각 안 (큐브가 있으면 도는 중에 watch_move가 잡음)
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
        # 직진 끝쪽의 카메라로 이미 본 칸은 갈 이유가 없음(다른 곳으로 가는 길이면 GOTO가 최단으로 감). 다 본 막다른
        # 모서리까지 들어갔다 꺾어 나오느라 10 s를 썼음 (2026-10-31 Gazebo 사용자 미로 (1,1))
        while cells and cells[-1] in self.camera_seen:
            cells.pop()
        cells = cells[:self.p['max_cells']]
        # 두 칸을 한 번에 가면 가운데 칸에서 옆을 못 돌아봐, 그 칸 옆을 보러 180° 돌아 되돌아왔음 (2026-10-31 실물 조건
        # Gazebo 맵 1·2·3). 옆을 돌아봐야 할 칸이 중간에 있으면 거기까지만 감 (look_around와 같은 기준)
        p, yaw = self.p, math.atan2(dj, di)
        unseen = {c for c, state in states.items() if state == FREE} - self.camera_seen - self.passed
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

    def drive_path(self, pose, path):
        """Send the next turn or straight run (up to max_cells, shorter if the LiDAR sees it blocked) along path."""
        p = self.p
        # 경로가 한 칸(이미 목표 칸 안, 중심에서 arrive 넘게 떨어짐)이면 next_move가 그 칸 중심으로 직진을 냄 → path[1]이 없어
        # IndexError로 노드가 죽었음 (2026-10-30 Gazebo 시드 3·5·6)
        cell = path[1] if len(path) > 1 else path[0]
        top = p['max_cells']
        if self.state == 'RETURN':
            # 복귀는 경로가 꺾이는 칸까지 한 번에: 2칸마다 끊고 끊을 때마다 격자 보정으로 1초씩 서서 한 칸씩 멈추며
            # 왔음 (2026-10-31 실물 6차 사용자: "직진 하는 부분이면 쭉 직진"). 보정은 꺾일 때 회전 앞뒤에서
            top = max(1, path.index(aim_cell(path, len(path))))
        for cells in range(top, 0, -1):
            # 도착 15 cm: 목표 15 cm 앞 근접 정지를 도착으로 보는 기준과 같게. 10 cm면 벽에 막혀 12 cm 앞에서 "도착"한 뒤
            # 같은 F122를 26번 되풀이했음 (2026-10-30 Gazebo 에이전트 분석: 시드 6)
            move = next_move(pose, path, p['cell_size'], p['turn_min_deg'], arrive=0.15, max_cells=cells)
            if move is None:
                return 'arrived'
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
            self.board(command_text(move), cell, center(aim, p['cell_size']), stop=turn)
        else:
            self.board(command_text(move))
        return 'moving'

    def do_cruise(self):
        """Keep driving straight while the cells ahead are open and not yet passed; look around once blocked."""
        if not self.ready():
            return
        seen = self.target_seen()
        if seen == 'yes':
            return self.enter('APPROACH')
        if seen == 'wait':
            return
        states, pose, here = self.states(), self.pose(), self.here()
        # 바로 옆 칸이 아직 지도에 없고(모르는 칸) 경계에 벽도 안 보이면 지도가 채워지게 잠깐(1 s) 기다렸다 판단: 안 기다리고
        # 지나쳤다가 그 칸이 빈 칸이 되자 180° 돌아 보러 왔음 (2026-10-31 Gazebo 맵 1, 출발 한 칸 앞 (9,2))
        if pose is not None and self.now() - self.idle_since < 1.0:
            di, dj = axis(pose[2])
            if any(states.get(c) == UNKNOWN for c in ((here[0] - dj, here[1] + di), (here[0] + dj, here[1] - di))):
                return
        if pose is not None and here != (0, 0) and self.look_around(states, pose, dry=True):
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
        self.detour = None  # (우회를 시작한 칸, 큐브 방향 칸들): 한 칸 옮길 때까지 큐브 쪽으로 다시 돌지 않음
        self.pocket = None  # 주머니 속 큐브라고 본 칸: 입구로 도는 동안 화면에서 빠져도 이 칸 쪽으로 계속

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
            cells = ray_cells(pose, self.cube_heading, states, size)
            if self.pocket in cells:
                cells = [self.pocket]  # 정한 큐브 칸을 새 검출 방향도 지나면 유지: 검출마다 바뀌어 두 입구 칸 사이를 230 s
                #                        오갔음 (2026-10-30 Gazebo 에이전트 분석: 시드 12)
            entries = pocket_entries(cells[0], states) if cells else []
            if entries and here not in entries:
                self.get_logger().info(f'목표물이 {cells[0]} 칸(옆이 막힘) 안: 입구 칸 {sorted(entries)} 중 가까운 곳으로.')
                self.detour, self.pocket = (here, set(entries)), cells[0]
                return self.go_detour()
            if here in entries and math.dist(pose[:2], center(here, size)) > 0.06:
                c = center(here, size)  # 입구 칸 한가운데로 (칸 축 위에서 큐브를 정면에 둠)
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
            return self.board(f'F{round(d * 1000)}', point=goal)
        # 그 방향 바로 앞이 막힘(라이다): 격자로 장애물을 돌아 그 방향 칸 쪽으로
        size = p['cell_size']
        # 방향선이 중심 근처를 지나는 빈 칸들로 감. 선 위 칸을 아무거나(막힌 칸·경기장 밖 포함) 잡으면 길이 없어 보이는
        # 큐브를 버리고 탐색으로 돌아갔음 (2026-10-30 Gazebo: 사용자 미로 86 s에 보고도 238 s에야 접근)
        ray = set(ray_cells(pose, self.cube_heading, self.states(), size)) - {self.here()} or {
            cell_of(pose[0] + k * size * math.cos(self.cube_heading),
                    pose[1] + k * size * math.sin(self.cube_heading), size) for k in range(1, 6)} - {self.here()}
        self.get_logger().info('목표물 방향 앞이 막힘: 장애물을 돌아 그 방향 칸으로 갑니다.')
        self.detour = (self.here(), ray)
        self.go_detour()

    def go_detour(self):
        if self.step_toward(self.detour[1]) == 'no_path':
            self.detour = None
            # LOOK은 큐브가 보이면 바로 APPROACH로 돌아와 한 번도 안 움직이고 0.3 s마다 반복함 (2026-10-30 Gazebo 시드 7:
            # 400 s). 큐브 칸은 라이다 지도에 안 잡히므로 길이 없으면 쌓인 막힘 표시 탓 → 비우고 다시. 비울 것도 없으면 탐색으로
            if self.unblock():
                self.get_logger().warn('목표물 쪽 길 없음: 막힘 표시를 비우고 다시 찾습니다.')
            else:
                self.next_goal()

    def start_grasp(self):
        self.grip_tries = 0
        self.grip()

    def grip(self):
        self.board('PF0')  # 운반 모드 먼저 (보드 정면 PSD 정지 끔). 쥔 큐브가 정면 PSD 앞이라 안 끄면 못 움직임
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
        self.return_retries, self.goal_steps, self.plan = 0, 0, []

    def do_return(self):
        """Grid moves home.

        경로가 없으면 막힘 표시를 비우고 옆 칸으로 한 칸 옮겨 다시 계획합니다 (16회차: 같은 자리에서 같은
        요청만 157번 반복). max_return_retries 넘으면 기록을 모두 비우고 처음부터 다시 (멈추지 않음).
        """
        if not self.ready() or self.fix_due():
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
    finally:
        if node is not None:
            node.save_record()  # 중간에 멈춰도 그때까지의 지도·경로를 남김
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
