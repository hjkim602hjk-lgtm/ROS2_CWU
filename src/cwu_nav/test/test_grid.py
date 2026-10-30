# 격자 플래너(grid.py)를 로봇 없이 검사하는 pytest 파일입니다.
# 칸 판정, BFS 우회, 다음 이동 명령(회전/직진·펌웨어 최소값), 직진 통로 검사를 확인합니다.

"""Grid planning for the 2026-10-29 firmware: cell states, BFS, next discrete move."""
import math

import numpy as np

from cwu_nav.grid import (BLOCKED, FREE, UNKNOWN, axis, bfs, bfs_avoiding, camera_view, cell_of, cell_state,
                          command_text, corridor_clear, free_distance, frontier, next_move, pocket_entries, ray_cells,
                          scan_points, to_line, wall_push, blocking_points, viewpoints, best_heading, seen_faces,
                          lattice_fix, compose, invert, mark_scan, occupancy, wall_heading, return_route,
                          wall_faces, walled_off, sweep_hits, open_frontier)

RES, SIZE = 0.05, 0.4


def arena(width_cells=5, height_cells=5, blocked=(), unknown=()):
    """SLAM-like grid with the start cell (0, 0) centred on the map origin; walls on the outer boundary."""
    cols, rows = int(width_cells * SIZE / RES), int(height_cells * SIZE / RES)
    grid = np.zeros((rows, cols), np.int8)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 100
    origin = (-SIZE / 2, -SIZE / 2)  # 칸 (0,0) 중심이 원점
    k = int(SIZE / RES)
    for i, j in blocked:
        grid[j * k:(j + 1) * k, i * k:(i + 1) * k] = 100
    for i, j in unknown:
        grid[j * k:(j + 1) * k, i * k:(i + 1) * k] = -1
    return grid, origin


def test_cell_of_rounds_to_the_nearest_centre():
    assert cell_of(0.0, 0.0, SIZE) == (0, 0)
    assert cell_of(0.19, -0.19, SIZE) == (0, 0)
    assert cell_of(0.81, 1.21, SIZE) == (2, 3)


def test_cell_state_ignores_boundary_walls_and_sees_blocks():
    grid, origin = arena(blocked=[(2, 1)], unknown=[(3, 3)])
    assert cell_state(grid, RES, origin, (0, 0), SIZE) == FREE      # 바깥 벽은 칸 가장자리라 무시
    assert cell_state(grid, RES, origin, (2, 1), SIZE) == BLOCKED
    assert cell_state(grid, RES, origin, (3, 3), SIZE) == UNKNOWN
    assert cell_state(grid, RES, origin, (9, 9), SIZE) == UNKNOWN   # 지도 밖
    # 출발 자세가 틀어져 벽 한 줄이 칸 가운데를 지나가도 막힘이 아님 (2026-10-29 실물: 출발 구역이 갇힘)
    grid[int((0.4 + 0.2) / RES), :] = 100   # 칸 (_,1) 가운데 가로줄 (y = 0.4 m)
    assert cell_state(grid, RES, origin, (0, 1), SIZE) == FREE


def test_bfs_goes_around_blocks_and_through_unknown():
    blocked = {(1, 0), (1, 1)}
    path = bfs((0, 0), lambda c: c not in blocked and 0 <= c[0] < 4 and 0 <= c[1] < 4, lambda c: c == (2, 0))
    assert path[0] == (0, 0) and path[-1] == (2, 0)
    assert not blocked & set(path)
    assert len(path) == 7  # (0,0)→(0,1)→(0,2)→(1,2)→(2,2)→(2,1)→(2,0)
    assert bfs((0, 0), lambda c: False, lambda c: c == (2, 0)) is None


def test_bfs_prefers_going_straight_among_equal_paths():
    free = lambda c: abs(c[0]) < 4 and abs(c[1]) < 4  # noqa: E731
    assert bfs((0, 0), free, lambda c: c == (2, 1), (1, 0)) == [(0, 0), (1, 0), (2, 0), (2, 1)]  # 직진 먼저
    assert bfs((0, 0), free, lambda c: c == (2, 1), (0, 1)) == [(0, 0), (0, 1), (1, 1), (2, 1)]  # 위를 보면 위로
    assert axis(math.radians(85)) == (0, 1) and axis(math.radians(-170)) == (-1, 0)


def test_bfs_avoiding_stays_off_passed_cells_unless_it_must():
    inside = lambda c: 0 <= c[0] < 3 and 0 <= c[1] < 3  # noqa: E731
    passed = {(1, 0)}
    path = bfs_avoiding((0, 0), inside, lambda c: c == (2, 0), passed)
    assert (1, 0) not in path and len(path) == 5       # 지나간 (1,0)을 피해 위로 돌아감
    corridor = lambda c: 0 <= c[0] < 3 and c[1] == 0  # noqa: E731  외길
    assert bfs_avoiding((0, 0), corridor, lambda c: c == (2, 0), passed) == [(0, 0), (1, 0), (2, 0)]


def test_frontier_is_seen_floor_next_to_unknown():
    states = {(0, 0): FREE, (1, 0): FREE, (2, 0): UNKNOWN, (0, 1): BLOCKED}
    assert frontier(states) == {(1, 0), (2, 0)}


def test_next_move_turns_then_drives_one_cell():
    path = [(0, 0), (0, 1), (0, 2)]
    assert next_move((0.0, 0.0, 0.0), path, SIZE) == ('T', 90)           # +y는 반시계 90°
    assert next_move((0.0, 0.0, math.pi / 2), path, SIZE) == ('F', 400)  # 한 칸씩 (max_cells=1)
    assert next_move((0.0, 0.0, math.pi / 2), path, SIZE, max_cells=2) == ('F', 800)
    assert next_move((0.0, 0.0, math.pi), [(0, 0), (0, -1)], SIZE) == ('T', 90)  # -y: 180°에서 반시계 90°


def test_next_move_respects_firmware_minimums():
    path = [(0, 0), (1, 0)]
    assert next_move((0.0, 0.0, math.radians(-4)), path, SIZE) == ('F', 400)   # 5° 미만은 그대로 직진
    assert next_move((0.0, 0.0, math.radians(-6)), path, SIZE) == ('T', 10)    # 5~10°는 펌웨어 최소 10°
    assert next_move((0.35, 0.0, 0.0), path, SIZE) == ('F', 100)               # 50 mm 남아도 최소 100 mm
    assert next_move((0.37, 0.02, 0.0), [(1, 0)], SIZE) is None                # 칸 중심 0.1 m 안 = 도착
    assert next_move((0.0, 0.0, 0.0), [(0, 0), (-1, 0)], SIZE) == ('T', 180)


def test_next_move_aims_back_at_the_centre_after_a_side_slip():
    # 옆으로 8 cm 밀린 채 다음 칸으로: 칸 중심을 다시 조준 (atan(0.08/0.4) ≈ 11°)
    assert next_move((0.0, 0.08, 0.0), [(0, 0), (1, 0)], SIZE) == ('T', -11)


def test_command_text():
    assert command_text(('T', 90)) == 'TL90'
    assert command_text(('T', -11)) == 'TR11'
    assert command_text(('F', 400)) == 'F400'


def test_corridor_clear_sees_only_the_swept_strip():
    wall_ahead = [(0.5, y / 100) for y in range(-10, 11)]
    beside = [(0.2, 0.25), (0.3, -0.25), (0.25, 0.3)]
    assert not corridor_clear(wall_ahead, 0.4, 0.165, 0.165)
    assert corridor_clear(wall_ahead, 0.3, 0.165, 0.165)       # 0.3 + 0.165 < 0.5
    assert corridor_clear(beside, 0.4, 0.165, 0.165)
    assert corridor_clear([(0.3, 0.0), (0.3, 0.01)], 0.4, 0.165, 0.165)  # 잡음 2점


def test_scan_points_drop_invalid_and_rotate_by_mount():
    # 라이다가 거꾸로(π) 장착: 라이다 0° 광선은 로봇 뒤
    points = scan_points([1.0, 0.0, math.inf, math.nan], 0.0, math.pi / 2, math.pi)
    assert len(points) == 1
    assert math.isclose(points[0][0], -1.0) and abs(points[0][1]) < 1e-9


def test_free_distance_stops_before_the_third_point_in_the_strip():
    wall = [(0.8, y / 100) for y in range(-10, 11)]
    assert abs(free_distance(wall, 0.165, 0.165, 1.0) - 0.635) < 1e-3   # 0.8 - 몸체 앞끝 0.165
    assert free_distance(wall, 0.165, 0.165, 0.5) == 0.5               # 상한
    assert free_distance([(0.3, 0.0), (0.4, 0.0)], 0.165, 0.165, 0.6) == 0.6  # 잡음 2점은 무시
    assert free_distance([(0.1, 0.1)] * 3, 0.165, 0.165, 0.6) == 0.0          # 몸 옆에 붙음


def test_wall_beside_the_body_corner_is_not_in_the_way():
    # 몸 옆 0.15 m, 앞 0.10 m부터 시작하는 벽: 사각형으로 보면 "막힘"(0.10 < 앞끝 0.165)이었지만 원형 몸체는 0.03 m 더 감
    wall = [(x / 100, 0.15) for x in range(10, 40)]
    assert free_distance(wall, 0.165, 0.165, 0.6) > 0.02
    ahead = [(0.3, 0.0)] * 3                                          # 정면 0.3 m: 몸 앞끝에서 0.135 m
    assert not corridor_clear(ahead, 0.2, 0.165, 0.165) and corridor_clear(ahead, 0.1, 0.165, 0.165)


def test_corridor_is_checked_on_the_cell_line_not_where_the_robot_drifted():
    # 400 mm 통로 (벽 y = ±0.2), 로봇이 칸 중심에서 왼쪽으로 4 cm 밀리고 10° 틀어짐. 로봇 기준 띠는 벽에 걸리지만
    # 칸 중심선 기준으로는 열림
    pose = (0.0, 0.04, math.radians(10))
    walls = [(x / 100, w) for x in range(-20, 120, 2) for w in (0.2, -0.2)]
    c, s = math.cos(pose[2]), math.sin(pose[2])  # map -> base_link
    seen = [((x - pose[0]) * c + (y - pose[1]) * s, -(x - pose[0]) * s + (y - pose[1]) * c) for x, y in walls]
    assert not corridor_clear(seen, 0.8, 0.165, 0.165)
    assert corridor_clear(to_line(seen, pose, (0.0, 0.0), (0.8, 0.0)), 0.8, 0.165, 0.165)
    back = to_line(seen, pose, (0.0, 0.0), (0.8, 0.0))
    assert all(abs(abs(y) - 0.2) < 1e-9 for _, y in back)  # map 벽으로 정확히 되돌아옴


def test_camera_sees_cells_in_view_but_not_behind_what_the_lidar_hits():
    states = {(i, j): FREE for i in range(-1, 6) for j in range(-3, 4)}
    states[(2, 0)] = BLOCKED
    face = [(0.6, v / 100) for v in range(-20, 21, 2)]   # (2,0) 장애물 앞면을 라이다가 맞힘
    seen = camera_view((0.0, 0.0, 0.0), face, states, SIZE, math.radians(30), 1.6)
    assert (1, 0) in seen and (2, 1) in seen          # 정면 한 칸, 막힌 칸 옆
    assert (3, 0) not in seen and (2, 0) not in seen  # 막힌 칸 뒤, 막힌 칸 자체
    assert (0, 0) not in seen                          # 화면 아래로 빠지는 발밑
    assert (0, 2) not in seen and (-1, 0) not in seen  # 화각 밖
    assert (5, 0) not in seen                          # 2.0 m > camera_range
    states[(2, 1)] = UNKNOWN                           # 지도에 아직 없어도 라이다로 뚫려 있으면 봄 (돈 직후)
    assert (2, 1) in camera_view((0.0, 0.0, 0.0), face, states, SIZE, math.radians(30), 1.6)
    seen = camera_view((0.0, 0.0, 0.0), face + [(0.35, 0.37), (0.36, 0.38)], states, SIZE, math.radians(60), 1.6)
    assert (1, 1) in seen and (2, 2) not in seen       # (1,1) 너머 (2,2)는 그 앞 점에 가림


def test_points_already_beside_the_body_do_not_stop_a_run():
    beside = [(0.02, 0.16), (0.03, 0.158), (0.01, -0.162)]  # 400 mm 출구 모서리가 몸 옆에 붙음
    assert free_distance(beside, 0.165, 0.165, 0.4) < 0.02
    assert free_distance(beside, 0.165, 0.165, 0.4, inside=0.01) == 0.4
    ahead = [(0.18, 0.0), (0.18, 0.01), (0.18, -0.01)]  # 정면 벽은 그대로 잡음
    assert free_distance(ahead, 0.165, 0.165, 0.4, inside=0.01) < 0.02


def test_wall_push_steers_off_a_near_wall_and_centres_between_two():
    left_wall = [(x / 100, 0.17) for x in range(-5, 20)]
    right_far = [(x / 100, -0.23) for x in range(-5, 20)]
    assert wall_push(left_wall, 0.19, 3.0) < 0                       # 왼쪽 벽에 붙음 → 오른쪽으로
    assert wall_push(left_wall + right_far, 0.19, 3.0) < 0
    assert wall_push([(x, -y) for x, y in left_wall], 0.19, 3.0) > 0  # 오른쪽 벽 → 왼쪽으로
    assert wall_push([(x / 100, 0.18) for x in range(20)] + [(x / 100, -0.18) for x in range(20)], 0.19, 3.0) == 0
    assert wall_push([(0.5, 0.0)], 0.19, 3.0) == 0                   # 정면 점은 관계없음


def test_obstacle_seen_only_on_its_face_is_blocked_but_a_walled_corridor_is_not():
    # 라이다는 상자의 겉면만 맞힘: 칸 (2,2) 속은 미탐색, 빈 칸 (2,1)과 맞닿은 아래쪽 면 한 줄만 점유 → 막힘.
    # (3,3)은 속이 미탐색이고 양옆 (2,3)·(4,3) 쪽에 벽 점이 있지만 그 이웃은 빈 칸이 아님 → 미탐색 그대로
    grid, origin = arena(unknown=[(2, 2), (3, 3), (2, 3), (4, 3)])
    k = int(SIZE / RES)
    grid[2 * k, 2 * k:3 * k] = 100
    grid[3 * k:4 * k, 3 * k] = 100
    grid[3 * k:4 * k, 4 * k] = 100
    states = {c: cell_state(grid, RES, origin, c, SIZE) for c in [(i, j) for i in range(5) for j in range(5)]}
    states = seen_faces(grid, RES, origin, states, SIZE)
    assert states[(2, 2)] == BLOCKED
    assert states[(2, 1)] == FREE
    assert states[(3, 3)] == UNKNOWN


def test_corridor_entry_between_two_obstacle_corners_stays_open():
    # 시드 1 모양: (2,2)는 위 (2,3)·아래 (2,1)가 장애물인 통로 칸, 속은 미탐색. 왼쪽 (1,2)는 빈 칸.
    # (1,2)와의 경계 양 끝에 두 장애물의 모서리 점만 있음 → 막힘이 아님
    grid, origin = arena(blocked=[(2, 1), (2, 3)], unknown=[(2, 2)])
    k = int(SIZE / RES)
    states = {c: cell_state(grid, RES, origin, c, SIZE) for c in [(i, j) for i in range(5) for j in range(5)]}
    assert states[(1, 2)] == FREE and states[(2, 2)] == UNKNOWN
    assert seen_faces(grid, RES, origin, states, SIZE)[(2, 2)] == UNKNOWN
    grid[2 * k:3 * k, 2 * k] = 100                     # 경계 전체에 면이 보이면 막힘
    assert seen_faces(grid, RES, origin, states, SIZE)[(2, 2)] == BLOCKED


def test_blocking_points_are_what_free_distance_stops_for():
    corner = [(0.12, 0.12), (0.12, 0.13), (0.13, 0.12)]  # 앞 왼쪽 장애물 모서리
    wall = [(0.18, 0.0), (0.18, 0.02), (0.18, -0.02)]
    far = [(0.6, 0.0)]
    assert sorted(blocking_points(corner + wall + far, 0.165, 0.165, 0.02, inside=0.01)) == sorted(corner + wall)
    assert free_distance(corner, 0.165, 0.165, 0.4, inside=0.01) < 0.02


def test_cube_in_a_pocket_is_entered_from_a_neighbour_cell():
    # 시드 6 모양: 큐브 칸 (2,2)의 좌우 (1,2)·(3,2)가 막힘, 위·아래로만 들어감. 로봇은 (1,0)에서 비스듬히 봄
    states = {(i, j): FREE for i in range(0, 5) for j in range(0, 5)}
    states[(1, 2)] = states[(3, 2)] = BLOCKED
    heading = math.atan2(2 * SIZE, 1 * SIZE)          # (1,0) → (2,2) 중심
    cells = ray_cells((SIZE, 0.0, 0.0), heading, states, SIZE)
    assert cells[0] == (2, 2)                          # 방향선이 중심을 지나는 첫 빈 칸
    assert sorted(pocket_entries((2, 2), states)) == [(2, 1), (2, 3)]
    assert pocket_entries((2, 1), states) == []        # 입구 칸 자체는 사방이 비어 주머니가 아님
    assert pocket_entries((4, 4), states) == []        # 사방이 트인 칸은 바로 감


def test_ray_cells_keep_an_unknown_cube_cell_and_stop_at_a_wall():
    # 시드 6 모양: (4,4)에서 남서쪽 대각선. 큐브 칸 (3,3)은 라이다 미탐색, 그 너머 (2,2)는 막힘
    states = {(i, j): FREE for i in range(0, 6) for j in range(0, 6)}
    states[(3, 3)] = UNKNOWN
    states[(2, 2)] = BLOCKED
    pose = (4 * SIZE, 4 * SIZE, 0.0)
    cells = ray_cells(pose, math.atan2(-1, -1), states, SIZE)
    assert cells == [(3, 3)]                           # 막힌 (2,2) 너머 빈 칸 (1,1)·(0,0)은 후보가 아님


def test_viewpoints_are_cells_that_see_unseen_cells_without_entering_them():
    # 0..4 x 0..2 통로, (2,1)이 막힘. 못 본 칸 (4,0)은 1.3 m 안에서 빈 칸만 지나 보이는 자리에서만
    states = {(i, j): FREE for i in range(5) for j in range(3)}
    states[(2, 1)] = BLOCKED
    views = viewpoints(states, {(4, 0)}, SIZE, 1.3)
    assert (1, 0) in views and (4, 0) not in views     # 3칸(1.2 m) 앞에서 봄, 칸 자체는 목적지 아님 (0.3 m 안쪽은 안 보임)
    assert (0, 0) not in views                          # 1.6 m > 1.3 m
    assert (1, 2) not in views                          # 시선이 막힌 칸 (2,1)을 지남
    assert views[(3, 0)] == {(4, 0)}


def test_best_heading_faces_the_most_unseen_cells():
    pose = (0.0, 0.0, 0.0)
    left = {(0, 2), (1, 2), (-1, 2)}                    # 왼쪽(+y)에 셋
    heading, looked = best_heading(pose, left | {(2, 0)}, SIZE, math.radians(35))
    assert abs(heading - math.pi / 2) < math.radians(30) and looked >= {(0, 2)} and (2, 0) not in looked


def test_lattice_fix_recovers_a_pose_error_from_walls_on_cell_boundaries():
    # 벽 점(map): x = 0.2·0.6 (세로 벽), y = -0.2·0.6 (가로 벽). 로봇 실제 (0.05, 0.4, 0), SLAM은 (0.13, 0.33, 3°)로 앎
    walls = [(xw, v / 100) for xw in (-0.2, 0.6) for v in range(-20, 100, 2)]
    walls += [(v / 100, yw) for yw in (-0.2, 0.6) for v in range(-20, 60, 2)]
    true = (0.05, 0.4, 0.0)
    body = [(wx - true[0], wy - true[1]) for wx, wy in walls]          # base_link (실제 yaw 0)
    dx, dy, dyaw = lattice_fix(body, (0.13, 0.33, math.radians(3)), SIZE)
    assert abs(math.degrees(dyaw) + 3) < 0.3
    assert abs(0.13 + dx - true[0]) < 0.01 and abs(0.33 + dy - true[1]) < 0.01
    # 세로 벽만 보이는 통로: y는 알 수 없음 → 0
    only_x = [(wx - true[0], wy - true[1]) for wx, wy in walls if wx in (-0.2, 0.6)]
    dx, dy, dyaw = lattice_fix(only_x, (0.13, 0.4, 0.0), SIZE)
    assert abs(dx + 0.08) < 0.01 and dy == 0.0
    assert lattice_fix([(1.0, 0.0)], (0.0, 0.0, 0.0), SIZE) is None


def test_compose_and_invert_are_rigid_2d_transforms():
    a, b = (1.0, 2.0, math.radians(30)), (0.5, -0.2, math.radians(-50))
    back = compose(invert(a), compose(a, b))
    assert all(abs(u - v) < 1e-9 for u, v in zip(back, b))
    x, y, yaw = compose((0.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0))
    assert abs(x) < 1e-9 and abs(y - 1.0) < 1e-9 and abs(yaw - math.pi / 2) < 1e-9


def test_own_map_marks_the_wall_hit_and_the_free_floor_before_it():
    # 원점 칸 (0,0) 중심에 로봇, +x 쪽 0.6 m에 세로 벽 (y -0.3~0.3). 칸 (1,0)은 빈 바닥, (2,0) 앞면이 벽
    res, origin = 0.05, (-1.0, -1.0)
    hits = np.zeros((40, 40), np.uint16)
    misses = np.zeros((40, 40), np.uint16)
    wall = [(0.6, v / 100) for v in range(-30, 31, 2)]
    for _ in range(3):
        mark_scan(hits, misses, origin, res, (0.0, 0.0, 0.0), wall)
    grid = occupancy(hits, misses)
    assert grid[20, 32] == 100                                         # (0.6, 0) 화소 = 벽
    assert grid[20, 28] == 0                                           # (0.4, 0) = 빈 바닥
    assert grid[20, 36] == -1                                          # 벽 너머는 못 봄
    assert cell_state(grid, res, origin, (1, 0), SIZE) == FREE
    # 로봇이 (0.1, 0)에서 같은 벽을 보아도(자세 반영) 같은 화소
    hits2, misses2 = np.zeros_like(hits), np.zeros_like(misses)
    mark_scan(hits2, misses2, origin, res, (0.1, 0.0, 0.0), [(x - 0.1, y) for x, y in wall])
    assert occupancy(hits2, misses2)[20, 32] == 100


def test_wall_heading_reads_the_robot_yaw_from_walls_modulo_90_degrees():
    # 통로: 세로 벽 x = ±0.2, 가로 벽 y = 0.6. 로봇이 실제로 +20° 돌아 있으면 base_link에서 벽은 -20° (= 90° 주기로 -20°)
    walls = [(xw, v / 100) for xw in (-0.2, 0.2) for v in range(-30, 60, 1)]
    walls += [(v / 100, 0.6) for v in range(-20, 21, 1)]
    walls = sorted(walls, key=lambda q: math.atan2(q[1], q[0]))      # 스캔처럼 각도 순서
    noise = np.random.default_rng(1).normal(0, 0.01, (len(walls), 2))  # 점 잡음 1 cm (시뮬 라이다와 같음)
    for yaw_deg in (0, 20, -37, 44):
        a = math.radians(yaw_deg)
        c, s = math.cos(-a), math.sin(-a)
        body = [(x * c - y * s + nx, x * s + y * c + ny) for (x, y), (nx, ny) in zip(walls, noise)]
        got = math.degrees(wall_heading(body))
        assert abs(((got + yaw_deg) + 45) % 90 - 45) < 2.5, (yaw_deg, got)   # 대강 (정밀은 lattice_fix)
    assert wall_heading([(1.0, 0.0), (2.0, 0.0)]) is None
    # 경기장 밖 비스듬한 물건(1.8 m, 30° 기운 긴 면)이 섞여도 경기장 벽 방향을 고름
    c30, s30 = math.cos(math.radians(30)), math.sin(math.radians(30))
    clutter = [(1.8 + 0.01 * k * c30, -0.4 + 0.01 * k * s30) for k in range(60)]
    got = math.degrees(wall_heading(sorted(walls + clutter, key=lambda q: math.atan2(q[1], q[0]))))
    assert abs((got + 45) % 90 - 45) < 2.5, got


def test_return_route_is_the_shortest_on_the_map_without_crossing_walls_and_avoids_unknown():
    # 0..4 x 0..2 빈 칸, (2,0)|(2,1) 사이 벽 면, (1,1) 장애물
    states = {(i, j): FREE for i in range(5) for j in range(3)}
    states[(1, 1)] = BLOCKED
    ok = lambda c: states.get(c, BLOCKED) != BLOCKED  # noqa: E731
    walls = {frozenset(((2, 0), (2, 1)))}
    path = return_route((2, 1), states, ok, lambda c: c == (0, 0), walls)
    assert (2, 0) != path[1] and (1, 1) not in path and path[-1] == (0, 0)   # 벽·장애물 안 지남
    assert path == [(2, 1), (3, 1), (3, 0), (2, 0), (1, 0), (0, 0)]
    states[(3, 0)] = UNKNOWN                                                  # 모르는 칸은 비쌈 → 위로 돌아감
    path = return_route((2, 1), states, ok, lambda c: c == (0, 0), walls)
    assert (3, 0) not in path and path[-1] == (0, 0)


def test_cells_behind_a_wall_face_are_cut_off_but_a_gap_or_a_corner_is_not():
    # 빈 칸 5x5, 칸 열 2|3 경계에 벽(5 cm 비켜 찍힘): 벽 너머(경기장 밖으로 번진 빈 칸)는 막힘, 카메라도 못 봄.
    # 한 줄이 뚫리면 길, 모서리 점만이면 벽 아님
    grid, origin = arena()
    k = int(SIZE / RES)
    cells = [(i, j) for i in range(5) for j in range(5)]
    states = {c: cell_state(grid, RES, origin, c, SIZE) for c in cells}
    grid[:, 3 * k + 1] = 100
    walls = wall_faces(grid, RES, origin, states, SIZE)
    assert frozenset(((2, 1), (3, 1))) in walls and frozenset(((1, 1), (2, 1))) not in walls
    out = walled_off(states, [(0, 0)], walls)
    assert out[(2, 4)] == FREE and out[(3, 0)] == BLOCKED and out[(4, 4)] == BLOCKED
    assert (3, 0) not in viewpoints(states, {(3, 0)}, SIZE, 1.3, walls=walls).get((2, 0), set())
    assert (3, 0) in viewpoints(states, {(3, 0)}, SIZE, 1.3).get((2, 0), set())
    grid[2 * k:3 * k, 3 * k + 1] = 0                   # (2,2)|(3,2) 경계가 뚫림
    walls = wall_faces(grid, RES, origin, states, SIZE)
    assert walled_off(states, [(0, 0)], walls)[(4, 4)] == FREE
    grid[2 * k:2 * k + 2, 3 * k + 1] = 100             # 그 경계 한쪽 끝에 모서리 점만
    assert frozenset(((2, 2), (3, 2))) not in wall_faces(grid, RES, origin, states, SIZE)


def test_beams_past_a_known_wall_do_not_clear_the_far_side():
    # 낮은 벽 위로 넘어간 빔(벽 너머 1.0 m에서 맞음): 벽이 굳은 뒤에는 벽 너머를 빈 바닥으로 안 칠함
    res, origin = 0.05, (-1.0, -1.0)
    hits = np.zeros((40, 40), np.uint16)
    misses = np.zeros((40, 40), np.uint16)
    wall = [(0.6, v / 100) for v in range(-30, 31, 2)]
    over = [(1.0, v / 100) for v in range(-30, 31, 2)]
    mark_scan(hits, misses, origin, res, (0.0, 0.0, 0.0), wall)
    for _ in range(10):
        mark_scan(hits, misses, origin, res, (0.0, 0.0, 0.0), wall + over)
    grid = occupancy(hits, misses, min_misses=2)
    assert grid[20, 32] == 100 and grid[20, 28] == 0 and grid[20, 36] == -1 and grid[20, 40 - 1] == -1
    # 벽이 굳기 전 첫 장이 벽 너머를 칠한 것도 min_misses로 미탐색
    hits[:], misses[:] = 0, 0
    for _ in range(10):
        mark_scan(hits, misses, origin, res, (0.0, 0.0, 0.0), wall + over)
    assert occupancy(hits, misses, min_misses=2)[20, 36] == -1


def test_turning_toward_a_side_wall_sweeps_the_gripper_into_it_but_the_other_way_is_clear():
    wall = [(x / 100, 0.20) for x in range(-20, 21, 2)]       # 왼쪽 20 cm 벽, 오른쪽은 빔
    gripper = [(0.19, 0.11), (0.19, -0.11)]                    # 자기 집게 (처음 자리라 안 셈)
    assert sweep_hits(wall + gripper, math.radians(90), 0.26)  # 왼쪽으로 돌면 벽을 쓸고 감
    assert not sweep_hits(wall + gripper, math.radians(-90), 0.26)
    assert not sweep_hits(wall + gripper, math.radians(-270), 0.26) == []  # 반대로 270°면 결국 왼쪽 벽도 지남


def test_an_entry_needs_the_beam_to_have_crossed_its_boundary():
    # (2,2) 미탐색. 왼쪽 (1,2) 경계는 빔이 지나감(빈 화소) → 입구, 아래 (2,1) 경계는 한 번도 안 보임(-1) → 입구 아님
    grid, origin = arena(unknown=[(2, 2)])
    k = int(SIZE / RES)
    grid[2 * k + 1:3 * k - 1, 2 * k] = 0      # (1,2)|(2,2) 경계 너머 첫 화소 줄은 빈 바닥으로 보였음 (빔이 지나감)
    states = {c: cell_state(grid, RES, origin, c, SIZE) for c in [(i, j) for i in range(5) for j in range(5)]}
    assert states[(2, 2)] == UNKNOWN
    entries = open_frontier(grid, RES, origin, states, SIZE)
    assert (1, 2) in entries and (2, 2) in entries and (2, 1) not in entries
