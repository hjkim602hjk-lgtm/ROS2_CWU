# 격자 플래너(grid.py)를 로봇 없이 검사하는 pytest 파일입니다.
# 칸 판정, BFS 우회, 다음 이동 명령(회전/직진·펌웨어 최소값), 직진 통로 검사를 확인합니다.

"""Grid planning for the 2026-10-29 firmware: cell states, BFS, next discrete move."""
import math

import numpy as np

from cwu_nav.grid import (BLOCKED, FREE, UNKNOWN, bfs, cell_of, cell_state, command_text, corridor_clear,
                          frontier, next_move, scan_points)

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
    grid, origin = arena(blocked=[(2, 1)], unknown=[(4, 4)])
    assert cell_state(grid, RES, origin, (0, 0), SIZE) == FREE      # 바깥 벽은 칸 가장자리라 무시
    assert cell_state(grid, RES, origin, (2, 1), SIZE) == BLOCKED
    assert cell_state(grid, RES, origin, (4, 4), SIZE) == UNKNOWN
    assert cell_state(grid, RES, origin, (9, 9), SIZE) == UNKNOWN   # 지도 밖


def test_bfs_goes_around_blocks_and_through_unknown():
    blocked = {(1, 0), (1, 1)}
    path = bfs((0, 0), lambda c: c not in blocked and 0 <= c[0] < 4 and 0 <= c[1] < 4, lambda c: c == (2, 0))
    assert path[0] == (0, 0) and path[-1] == (2, 0)
    assert not blocked & set(path)
    assert len(path) == 7  # (0,0)→(0,1)→(0,2)→(1,2)→(2,2)→(2,1)→(2,0)
    assert bfs((0, 0), lambda c: False, lambda c: c == (2, 0)) is None


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
