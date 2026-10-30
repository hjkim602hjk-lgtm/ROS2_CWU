# 랜덤 미션 1 경기장(3.6 m, 9x9 칸, 칸 400 mm)을 Gazebo Classic world 파일로 만듭니다.
#   장애물 = 칸 하나를 채우는 400 x 400 x 200 mm 상자, 외곽 벽 200 mm (규정 5.1·7절. 라이다 170 mm가 30 mm 위로 봄).
#   배치는 (5,5) 기준 점대칭(규정 10절), 빈 칸은 전부 이어져 있음.
#   빨간 큐브 50 mm는 (5,5) 칸 중심, 로봇은 네 모서리 칸 중 하나에서 "경기장이 왼쪽"인 방향으로 출발.
#   ros2 run cwu_sim make_maze --seed 3 --out /tmp/maze_3.world
#   ros2 run cwu_sim make_maze --seed 0 --example --out /tmp/example.world   # 규정 그림 1 예시 배치, (1,9) 출발
#   ros2 run cwu_sim make_maze --seed 0 --cells '3,8 3,9 7,1' --out /tmp/my.world  # 직접 정한 장애물 칸, (1,9) 출발
"""Random 9x9 mission-1 arena as a Gazebo world."""
import argparse
import math
import os
import random

N, CELL, WALL_H = 9, 0.4, 0.2
# 출발 칸과 진행 방향(rad): 경기장이 로봇 왼쪽 (실물 출발 자세와 같게)
STARTS = [((1, 1), 0.0), ((9, 1), math.pi / 2), ((9, 9), math.pi), ((1, 9), -math.pi / 2)]
TARGET = (5, 5)
# 규정 "그림 1 첫 번째 경기장 예시" (미리보기 그림에서 읽은 배치, 점대칭)
EXAMPLE = sorted({(5, 8), (5, 9), (2, 4), (2, 5), (2, 6), (2, 7), (4, 7), (5, 7), (6, 7)}
                 | {(10 - i, 10 - j) for i, j in {(5, 8), (5, 9), (2, 4), (2, 5), (2, 6), (2, 7), (4, 7), (5, 7), (6, 7)}})


def xy(cell):
    return (cell[0] - 5) * CELL, (cell[1] - 5) * CELL


def connected(free):
    """True when every free cell is 4-connected to every other."""
    free = set(free)
    stack, seen = [next(iter(free))], set()
    while stack:
        c = stack.pop()
        if c in seen:
            continue
        seen.add(c)
        stack += [n for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)) if n in free]
    return seen == free


def mirror(cell):
    return 10 - cell[0], 10 - cell[1]


def parse_cells(text):
    """'3,8 3,9' -> [(3, 8), (3, 9)]"""
    return sorted({tuple(int(v) for v in c.split(',')) for c in text.split()})


def layout(seed, example=False, cells=None):
    """(start cell, start yaw, obstacle cells) for a seed; point-symmetric about (5,5), the start corners stay free.

    example: 규정 그림 1 배치, cells: 직접 정한 장애물 칸 목록. 둘 다 (1,9)에서 아래(-y)를 보고 출발.
    """
    if example:
        return (1, 9), -math.pi / 2, EXAMPLE
    if cells is not None:
        return (1, 9), -math.pi / 2, sorted(cells)
    rng = random.Random(seed)
    start, yaw = rng.choice(STARTS)
    count = rng.randint(10, 20)
    keep = {TARGET} | {(c[0] + di, c[1] + dj) for c, _ in STARTS for di in (-1, 0, 1) for dj in (-1, 0, 1)}
    cells = [(i, j) for i in range(1, N + 1) for j in range(1, N + 1)]
    candidates = [c for c in cells if c not in keep]
    obstacles = set()
    for c in rng.sample(candidates, len(candidates)):
        if len(obstacles) >= count:
            break
        pair = {c, mirror(c)}
        if not pair & obstacles and connected(set(cells) - obstacles - pair):
            obstacles |= pair
    return start, yaw, sorted(obstacles)


def box(name, x, y, sx, sy, sz, rgb, static=True, mass=0.1, yaw=0.0):
    colour = f'{rgb[0]} {rgb[1]} {rgb[2]} 1'
    inertia = '' if static else (f'<inertial><mass>{mass}</mass><inertia><ixx>4e-5</ixx><iyy>4e-5</iyy>'
                                 f'<izz>4e-5</izz></inertia></inertial>')
    return f"""
    <model name="{name}"><static>{str(static).lower()}</static><pose>{x:.4f} {y:.4f} {sz / 2} 0 0 {yaw:.4f}</pose>
      <link name="link">{inertia}
        <collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material></visual>
      </link></model>"""


def ray(name, x, y, z, yaw, samples, half, lo, hi, rate, output, topic, frame, noise):
    return f"""
        <sensor name="{name}" type="ray"><pose>{x} {y} {z} 0 0 {yaw}</pose><update_rate>{rate}</update_rate>
          <ray><scan><horizontal><samples>{samples}</samples><resolution>1</resolution>
            <min_angle>{-half}</min_angle><max_angle>{half}</max_angle></horizontal></scan>
            <range><min>{lo}</min><max>{hi}</max><resolution>0.01</resolution></range>
            <noise><type>gaussian</type><mean>0</mean><stddev>{noise}</stddev></noise></ray>
          <plugin name="{name}_plugin" filename="libgazebo_ros_ray_sensor.so">
            <ros><remapping>~/out:={topic}</remapping></ros>
            <output_type>{output}</output_type><radiation_type>infrared</radiation_type>
            <frame_name>{frame}</frame_name></plugin>
        </sensor>"""


def robot(x, y, yaw):
    """Diff-drive body (330 mm), G4-like LiDAR at 170 mm, 640x480 webcam (fx 450), four PSDs.

    base_link 원점 = 바닥 위 구동축 중심 (실물과 같음). 센서 높이는 그 기준.
    odom은 Gazebo 실제 자세(odometry_source 1): 바퀴 기반은 제자리 회전에서 10~20% 틀려(바퀴 모양을 바꿔도) 실물
    (약 3%)보다 훨씬 나쁜 방향 오차가 미션 판단을 오염시켰음. 대신 실물 odom 오차는 재현하지 않음.
    """
    psd = ''.join(ray(f'psd_{n}', px, py, pz, pyaw, 5, 0.05, 0.03, 0.8, 20, 'sensor_msgs/Range',
                      f'/sim/psd_{n}', 'base_link', 0.003)
                  for n, px, py, pz, pyaw in (('front', 0.17, 0, 0.03, 0), ('rear', -0.17, 0, 0.06, math.pi),
                                              ('left', 0, 0.17, 0.06, math.pi / 2),
                                              ('right', 0, -0.17, 0.06, -math.pi / 2)))
    wheels = ''.join(f"""
      <link name="{n}_wheel"><pose>0 {s * 0.1008} 0.042 {-s * math.pi / 2} 0 0</pose>
        <inertial><mass>0.2</mass><inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>2e-4</izz></inertia></inertial>
        <collision name="c"><geometry><cylinder><radius>0.042</radius><length>0.005</length></cylinder></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><cylinder><radius>0.042</radius><length>0.02</length></cylinder></geometry></visual>
      </link>
      <joint name="{n}_joint" type="revolute"><parent>base_link</parent><child>{n}_wheel</child>
        <axis><xyz>0 0 {s}</xyz></axis></joint>""" for n, s in (('left', 1), ('right', -1)))
    casters = ''.join(f"""
        <collision name="caster_{i}"><pose>{cx} 0 0.015 0 0 0</pose><geometry><sphere><radius>0.015</radius></sphere></geometry>
          <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>"""
                      for i, cx in enumerate((0.12, -0.12)))
    return f"""
    <model name="robot"><pose>{x} {y} 0.002 0 0 {yaw}</pose>
      <link name="base_link">
        <inertial><pose>0 0 0.08 0 0 0</pose><mass>2.0</mass><inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.03</izz></inertia></inertial>
        <collision name="body"><pose>0 0 0.10 0 0 0</pose><geometry><cylinder><radius>0.165</radius><length>0.10</length></cylinder></geometry></collision>{casters}
        <visual name="body"><pose>0 0 0.10 0 0 0</pose><geometry><cylinder><radius>0.165</radius><length>0.10</length></cylinder></geometry>
          <material><ambient>0.1 0.3 0.8 1</ambient><diffuse>0.1 0.3 0.8 1</diffuse></material></visual>
        {ray('lidar', 0, 0, 0.17, 0, 720, math.pi, 0.12, 8.0, 9, 'sensor_msgs/LaserScan', '/sim/scan', 'laser_frame', 0.01)}
        {psd}
        <sensor name="webcam" type="camera"><pose>0.12 0 0.20 0 0.35 0</pose><update_rate>15</update_rate>
          <camera><horizontal_fov>{2 * math.atan(320 / 450):.4f}</horizontal_fov>
            <image><width>640</width><height>480</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>8</far></clip></camera>
          <plugin name="webcam_plugin" filename="libgazebo_ros_camera.so">
            <ros><remapping>~/image_raw:=/webcam/image_raw</remapping>
              <remapping>~/camera_info:=/webcam/camera_info</remapping></ros>
            <camera_name>webcam</camera_name><frame_name>camera_optical</frame_name></plugin>
        </sensor>
      </link>{wheels}
      <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
        <ros><remapping>cmd_vel:=/sim/cmd_vel</remapping><remapping>odom:=/odom</remapping></ros>
        <left_joint>left_joint</left_joint><right_joint>right_joint</right_joint>
        <wheel_separation>0.2016</wheel_separation><wheel_diameter>0.084</wheel_diameter>
        <max_wheel_torque>5</max_wheel_torque><max_wheel_acceleration>2.0</max_wheel_acceleration>
        <publish_odom>true</publish_odom><publish_odom_tf>false</publish_odom_tf><publish_wheel_tf>false</publish_wheel_tf>
        <odometry_frame>odom</odometry_frame><robot_base_frame>base_link</robot_base_frame><odometry_source>1</odometry_source>
      </plugin>
    </model>"""


def world(seed, example=False, cells=None):
    """World text with the robot's start pose at the world origin (odom = 실제 자세 = 출발 기준, map 원점 = 출발 칸)."""
    start, yaw, obstacles = layout(seed, example, cells)
    half, t = N * CELL / 2, 0.05
    sx0, sy0 = xy(start)

    # SIM_START_ERR="dx dy 도": 로봇을 출발 칸 중심·축에서 그만큼 어긋나게 놓은 것처럼 경기장을 거꾸로 옮김 (odom은 0에서 시작).
    # 실물에서 출발 자세가 칸 축과 몇 도 틀어진 것을 격자 보정이 잡는지 보려고 (2026-10-31)
    ex, ey, ed = (float(v) for v in os.environ.get('SIM_START_ERR', '0 0 0').split())
    # SIM_RTF=2: 실제 시간의 2배로 (노드는 모두 시뮬레이션 시계를 따름). 시험 한 바퀴가 15분이라 느렸음 (2026-10-31)
    rate = round(500 * float(os.environ.get('SIM_RTF', '1')))
    ce, se = math.cos(math.radians(-ed)), math.sin(math.radians(-ed))

    def place(name, x, y, *rest, **kw):  # 경기장 좌표 → 출발 자세 기준
        dx, dy = x - sx0, y - sy0
        px, py = dx * math.cos(yaw) + dy * math.sin(yaw) - ex, -dx * math.sin(yaw) + dy * math.cos(yaw) - ey
        return box(name, px * ce - py * se, px * se + py * ce, *rest, yaw=-yaw - math.radians(ed), **kw)
    walls = [place(f'wall_{k}', x, y, sx, sy, WALL_H, (0.5, 0.5, 0.5)) for k, (x, y, sx, sy) in enumerate((
        (0, half + t / 2, 2 * half + 2 * t, t), (0, -half - t / 2, 2 * half + 2 * t, t),
        (half + t / 2, 0, t, 2 * half), (-half - t / 2, 0, t, 2 * half)))]
    blocks = [place(f'obstacle_{i}_{j}', *xy((i, j)), CELL, CELL, WALL_H, (0.5, 0.5, 0.5)) for i, j in obstacles]
    target = place('target', *xy(TARGET), 0.05, 0.05, 0.05, (1, 0, 0), static=False)
    grid = '\n'.join(' '.join('S' if (i, j) == start else 'T' if (i, j) == TARGET else
                              '#' if (i, j) in obstacles else '.' for i in range(1, N + 1))
                     for j in range(N, 0, -1))
    return f"""<?xml version="1.0"?>
<!-- seed {seed}, start {start}, obstacles {len(obstacles)} (j up, i right):
{grid}
-->
<sdf version="1.6"><world name="maze">
  <physics type="ode"><max_step_size>0.002</max_step_size><real_time_update_rate>{rate}</real_time_update_rate></physics>
  <light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse>
    <direction>-0.3 0.2 -1</direction><cast_shadows>false</cast_shadows></light>
  <scene><ambient>0.6 0.6 0.6 1</ambient><shadows>false</shadows></scene>
  <model name="floor"><static>true</static><link name="link">
    <collision name="c"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry></collision>
    <visual name="v"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
      <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual></link></model>
  {''.join(walls + blocks)}{target}{robot(0, 0, 0)}
</world></sdf>
""", grid


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--example', action='store_true', help='규정 그림 1 예시 배치')
    parser.add_argument('--cells', help="직접 정한 장애물 칸, 예: '3,8 3,9 7,1'")
    args = parser.parse_args()
    text, grid = world(args.seed, args.example, parse_cells(args.cells) if args.cells else None)
    with open(args.out, 'w') as f:
        f.write(text)
    print(grid)


if __name__ == '__main__':
    main()
