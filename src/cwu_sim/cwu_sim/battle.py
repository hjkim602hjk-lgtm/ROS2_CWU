# 미션 2 배틀럼블 경기장(3.6 m, 장애물 없음)을 Gazebo Classic world 파일로 만듭니다. 미션 1 maze.py의 로봇·상자 모델을 재사용.
#   우리 로봇: 네 출발 칸 (2,2)/(8,2)/(8,8)/(2,8) 중 시드로 하나, "경기장 중앙이 왼쪽"인 방향(설계 4.2 SPIN) ± 작은 오차
#   상대 3대: 나머지 출발 칸, 중앙을 향함. 바퀴로 움직이는 원통(높이 0.25 m, LiDAR 170 mm에 걸림) → 밀고 밀림
#   목표물 'victim' 50 mm 빨간 상자 (5,5), 100 g 동적 (밀림). 이름이 'target'이 아니라 fake_board가 쥘 때 지우지 않음 → 심판(referee)이 옮김
#   ros2 run cwu_sim make_battle --seed 3 --out /tmp/b.world     # 출력 = 우리 출발 자세 (심판 인자)
"""Mission-2 battle arena with three pushable opponents as a Gazebo world."""
import argparse
import math
import random

from cwu_sim.maze import CELL, WALL_H, box, robot, xy

# 출발 칸과 "중앙이 왼쪽"인 방향 (경기장 좌표, x = 칸 i 방향)
STARTS = [((2, 2), 0.0), ((8, 2), math.pi / 2), ((8, 8), math.pi), ((2, 8), -math.pi / 2)]


def opponent(name, x, y, yaw):
    """Diff-drive cylinder r 0.15 m, h 0.25 m; /NAME/cmd_vel in, /NAME/odom (world truth) out."""
    wheels = ''.join(f"""
      <link name="{n}_wheel"><pose>0 {s * 0.11} 0.042 {-s * math.pi / 2} 0 0</pose>
        <inertial><mass>0.2</mass><inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>2e-4</izz></inertia></inertial>
        <collision name="c"><geometry><cylinder><radius>0.042</radius><length>0.005</length></cylinder></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
      </link>
      <joint name="{n}_joint" type="revolute"><parent>base</parent><child>{n}_wheel</child>
        <axis><xyz>0 0 {s}</xyz></axis></joint>""" for n, s in (('left', 1), ('right', -1)))
    casters = ''.join(f"""
        <collision name="caster_{i}"><pose>{cx} 0 0.015 0 0 0</pose><geometry><sphere><radius>0.015</radius></sphere></geometry>
          <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>"""
                      for i, cx in enumerate((0.11, -0.11)))
    return f"""
    <model name="{name}"><pose>{x:.4f} {y:.4f} 0.002 0 0 {yaw:.4f}</pose>
      <link name="base">
        <inertial><pose>0 0 0.08 0 0 0</pose><mass>2.0</mass><inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.03</izz></inertia></inertial>
        <collision name="body"><pose>0 0 0.15 0 0 0</pose><geometry><cylinder><radius>0.15</radius><length>0.2</length></cylinder></geometry></collision>{casters}
        <visual name="body"><pose>0 0 0.15 0 0 0</pose><geometry><cylinder><radius>0.15</radius><length>0.2</length></cylinder></geometry>
          <material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>
      </link>{wheels}
      <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
        <ros><namespace>/{name}</namespace></ros>
        <left_joint>left_joint</left_joint><right_joint>right_joint</right_joint>
        <wheel_separation>0.22</wheel_separation><wheel_diameter>0.084</wheel_diameter>
        <max_wheel_torque>5</max_wheel_torque><max_wheel_acceleration>2.0</max_wheel_acceleration>
        <publish_odom>true</publish_odom><publish_odom_tf>false</publish_odom_tf><publish_wheel_tf>false</publish_wheel_tf>
        <odometry_frame>{name}/odom</odometry_frame><robot_base_frame>{name}/base</robot_base_frame>
        <odometry_source>1</odometry_source>
      </plugin>
    </model>"""


def layout(seed):
    """(our start (x, y, yaw) in arena coordinates, [(opponent start cell, yaw facing the centre)])."""
    rng = random.Random(seed)
    k = rng.randrange(4)
    cell, yaw = STARTS[k]
    x, y = xy(cell)
    ours = (x + rng.uniform(-0.03, 0.03), y + rng.uniform(-0.03, 0.03), yaw + math.radians(rng.uniform(-3, 3)))
    others = [STARTS[(k + i) % 4][0] for i in (1, 2, 3)]
    return ours, [(c, math.atan2(-xy(c)[1], -xy(c)[0])) for c in others]


def world(seed):
    """World text with our robot at the world origin (odom = 출발 자세), and our arena start pose."""
    (sx, sy, syaw), others = layout(seed)
    c, s = math.cos(syaw), math.sin(syaw)

    def local(x, y, yaw=0.0):  # 경기장 좌표 → 출발 자세 기준 (world)
        dx, dy = x - sx, y - sy
        return dx * c + dy * s, -dx * s + dy * c, yaw - syaw
    half, t = 4.5 * CELL, 0.05
    walls = []
    for k, (x, y, lx, ly) in enumerate(((0, half + t / 2, 2 * half + 2 * t, t), (0, -half - t / 2, 2 * half + 2 * t, t),
                                        (half + t / 2, 0, t, 2 * half), (-half - t / 2, 0, t, 2 * half))):
        wx, wy, wyaw = local(x, y)
        walls.append(box(f'wall_{k}', wx, wy, lx, ly, WALL_H, (0.5, 0.5, 0.5), yaw=wyaw))
    tx, ty, tyaw = local(0.0, 0.0)
    # 동적(100 g): 정적 모델은 set_entity_state로 옮겨도 카메라 화면에 반영되지 않았음 (2026-10-30 배틀 1: 쥔 뒤 검출 0)
    target = box('victim', tx, ty, 0.05, 0.05, 0.05, (1, 0, 0), static=False, yaw=tyaw)
    opps = ''.join(opponent(f'opp_{i + 1}', *local(*xy(cell), yaw)) for i, (cell, yaw) in enumerate(others))
    return f"""<?xml version="1.0"?>
<!-- battle seed {seed}: our start ({sx:.3f}, {sy:.3f}, {math.degrees(syaw):.1f} deg) in arena coordinates -->
<sdf version="1.6"><world name="battle">
  <physics type="ode"><max_step_size>0.002</max_step_size><real_time_update_rate>500</real_time_update_rate></physics>
  <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
    <ros><namespace>/gazebo</namespace></ros><update_rate>20</update_rate></plugin>
  <light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse>
    <direction>-0.3 0.2 -1</direction><cast_shadows>false</cast_shadows></light>
  <scene><ambient>0.6 0.6 0.6 1</ambient><shadows>false</shadows></scene>
  <model name="floor"><static>true</static><link name="link">
    <collision name="c"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry></collision>
    <visual name="v"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
      <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual></link></model>
  {''.join(walls)}{target}{opps}{robot(0, 0, 0)}
</world></sdf>
""", (sx, sy, syaw)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    text, start = world(args.seed)
    with open(args.out, 'w') as f:
        f.write(text)
    print(f'{start[0]:.4f} {start[1]:.4f} {start[2]:.5f}')


if __name__ == '__main__':
    main()
