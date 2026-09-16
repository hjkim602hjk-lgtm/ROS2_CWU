# 미션 상태머신을 뺀 자율주행 스택 전체를 한 번에 띄우는 진입점입니다.
# SLAM(cwu_slam) → Nav2(cwu_nav) → D415 인식(cwu_perception) 순서로 포함하고,
# 목표물 방향을 바퀴 속도로 바꾸는 시각 서보(target_follower)를 함께 띄웁니다.
# demo:=true면 하드웨어 없이, demo_drive:=cmd_vel이면 Nav2 속도로 가상 로봇이 움직입니다.
# 실물에서는 /cmd_vel을 받는 모터 노드가 아직 없어 로봇이 움직이지 않습니다(계획 Phase 1b).

"""SLAM + Nav2 + target detection in one launch; the mission state machine is not here."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package, launch_file, arguments, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / 'launch' / launch_file)),
        launch_arguments=arguments.items(), condition=condition)


def generate_launch_description():
    common = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('demo', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('demo_drive', default_value='cmd_vel',
                              choices=['circle', 'cmd_vel']),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('encoder_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        # 카메라는 인식 없이 주행만 시험할 때 빼고 띄웁니다. Pi 4 CPU 측정에도 씁니다.
        DeclareLaunchArgument('camera', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('follow', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'servo_params_file',
            default_value=str(Path(get_package_share_directory('cwu_nav'))
                              / 'config' / 'servo.yaml')),

        # SLAM Toolbox 고정: Nav2 코스트맵은 /map과 map → odom TF에 의존합니다.
        # 두 백엔드를 동시에 띄우면 TF가 중복되므로 여기서 하나로 고정합니다.
        _include('cwu_slam', 'slam_toolbox.launch.py', dict(
            common,
            demo=LaunchConfiguration('demo'),
            demo_drive=LaunchConfiguration('demo_drive'),
            port=LaunchConfiguration('port'),
            encoder_port=LaunchConfiguration('encoder_port'),
            rviz=LaunchConfiguration('rviz'))),
        _include('cwu_nav', 'nav2.launch.py', dict(common)),
        _include('cwu_perception', 'camera.launch.py', dict(common),
                 condition=IfCondition(LaunchConfiguration('camera'))),
        # /target/bearing → /cmd_vel_servo. Nav2의 /cmd_vel과 이 토픽 중 무엇을
        # 모터로 보낼지는 안전 감시자(cwu_safety, 미구현)가 미션 상태를 보고 고릅니다.
        # 지금은 두 토픽 다 소비자가 없으므로 동시에 발행해도 로봇은 움직이지 않습니다.
        Node(condition=IfCondition(LaunchConfiguration('follow')),
             package='cwu_nav', executable='target_follower', name='target_follower',
             parameters=[LaunchConfiguration('servo_params_file'), common],
             output='screen'),
    ])
