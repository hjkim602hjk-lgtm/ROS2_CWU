# Gazebo 배틀 경기장에서 미션 2를 돌립니다 (PC 전용).
#   실물 mission2.launch.py와 같은 미션·검출기 설정 파일을 쓰고, G4·encoder_odom·카메라·보드 자리만
#   Gazebo 센서 + fake_board로 바꿉니다. 심판(referee)이 상대 3대를 움직이고 목표물·탈락을 판정합니다.
#   ros2 run cwu_sim make_battle --seed 3 --out /tmp/b.world   # 출력 "x y yaw" = start 인자
#   ros2 launch cwu_sim battle.launch.py world:=/tmp/b.world start:="x y yaw" seed:=3
"""Mission 2 in a generated Gazebo battle arena."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def share(package, *parts):
    return str(Path(get_package_share_directory(package), *parts))


def referee(context):
    sx, sy, syaw = (float(v) for v in LaunchConfiguration('start').perform(context).split())
    return [Node(package='cwu_sim', executable='referee', name='referee', output='screen',
                 parameters=[{'use_sim_time': True, 'start_x': sx, 'start_y': sy, 'start_yaw': syaw,
                              'seed': int(LaunchConfiguration('seed').perform(context))}])]


def generate_launch_description():
    sim = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('world'),
        DeclareLaunchArgument('start', description='우리 출발 자세 "x y yaw" (make_battle 출력)'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('gui', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(share('gazebo_ros', 'launch', 'gzserver.launch.py')),
                                 launch_arguments={'world': LaunchConfiguration('world')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(share('gazebo_ros', 'launch', 'gzclient.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='tf2_ros', executable='static_transform_publisher', name='laser_mount',
             arguments=['--z', '0.17', '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'],
             parameters=[sim]),
        Node(package='cwu_perception', executable='target_detector', name='target_detector',
             parameters=[share('cwu_perception', 'config', 'target.yaml'),
                         dict(sim, color_topic='/webcam/image_raw', camera_info_topic='/webcam/camera_info')]),
        Node(package='cwu_sim', executable='fake_board', name='fake_board', parameters=[sim]),
        OpaqueFunction(function=referee),
        Node(package='cwu_nav', executable='mission2', name='mission2',
             parameters=[share('cwu_nav', 'config', 'mission2.yaml'), sim], output='screen'),
    ])
