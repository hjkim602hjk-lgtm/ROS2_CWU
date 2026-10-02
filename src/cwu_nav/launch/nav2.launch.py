# Nav2 기성 스택(navigation_launch.py)을 이 경기장 설정으로 실행하는 진입점입니다.
# use_composition일 때는 컨테이너를 여기서 직접 띄웁니다. navigation_launch.py는
# 실을 컨테이너를 만들지 않고, 없으면 오류 없이 아무것도 띄우지 않습니다.
# 사전 지도를 쓰지 않으므로 map_server와 AMCL은 띄우지 않습니다.
# 위치 추정은 이미 실행 중인 SLAM Toolbox의 map → odom TF를 그대로 사용합니다.
# 모터 연결은 autonomy.launch.py의 drive:=true에서만 별도로 실행합니다.
#
# [공부 노트] Nav2 가 뭔가?
#   "목표 지점(x, y)을 주면 지도를 보고 경로를 짜서 /cmd_vel 속도 명령을 내는" ROS 2 표준 내비게이션 묶음.
#   이 파일이 띄우는 것
#     - planner_server     : 지도 위 경로 계획 (전역)
#     - controller_server  : 경로를 따라가는 속도 계산 (지역)
#     - bt_navigator       : 위 둘을 행동 트리(XML) 순서대로 지휘
#     - velocity_smoother  : 속도 급변 방지 → /cmd_vel
#     - collision_monitor  : /cmd_vel 을 받아 장애물이 가까우면 0 으로 → /cmd_vel_safe
#     - lifecycle_manager  : 위 노드들을 순서대로 켜 줌(configure → activate)
#   흐름: 목표 → planner → controller → smoother(/cmd_vel)
#         → collision_monitor(/cmd_vel_safe) → motor_bridge
#
# [공부 노트] 실행 예
#   ros2 launch cwu_nav nav2.launch.py                 # SLAM 이 이미 떠 있어야 함
#   보통은 autonomy.launch.py 가 SLAM 과 함께 포함해서 띄웁니다.

"""Stock Nav2 bringup parameterised for the 3.6 m arena, on top of online SLAM."""
from pathlib import Path
import tempfile  # 실행 중에 만든 YAML 을 임시 파일로 저장

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnShutdown  # launch 종료 시 실행할 동작 등록
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# motor.yaml 실측값을 nav2 설정에 반영하는 함수 (cwu_nav/drive_config.py)
from cwu_nav.drive_config import configure_navigation


def generate_launch_description():
    # 원본 Nav2 설정: install/cwu_nav/share/cwu_nav/config/nav2.yaml
    params = Path(get_package_share_directory('cwu_nav')) / 'config' / 'nav2.yaml'

    # 인자 값이 정해진 뒤(OpaqueFunction) 실행 — bringup.py 와 같은 패턴
    def nodes(context):
        # 드라이버와 같은 이유로 요청받을 때만 해석합니다.
        # Nav2는 선택 설치이므로 없는 환경에서 이 파일을 import하는 것만으로 실패하지 않게 합니다.
        bringup = Path(get_package_share_directory('nav2_bringup'))
        # 1) nav2.yaml 읽기
        source = Path(LaunchConfiguration('nav2_params_file').perform(context))
        config = yaml.safe_load(source.read_text())
        # 2) drive:=true 면 motor.yaml 실측값도 읽고, 확인 표시가 없으면 거부
        measured = None
        if IfCondition(LaunchConfiguration('drive')).evaluate(context):
            motor_file = Path(LaunchConfiguration('motor_params_file').perform(context))
            measured = yaml.safe_load(motor_file.read_text())['motor_bridge']['ros__parameters']
            if not measured['calibration_confirmed'] or not measured['mount_calibration_confirmed']:
                raise ValueError('Real drive requires measured motor and LiDAR calibration')
        # 3) 실측값을 nav2 설정에 반영 (drive_config.py)
        configure_navigation(config, params.parent, measured)
        sim_time = IfCondition(LaunchConfiguration('use_sim_time')).evaluate(context)
        config['collision_monitor']['ros__parameters']['use_sim_time'] = sim_time
        # Derived onboard from fixed calibration; no arena-dependent manual edits.
        # 4) 고친 설정을 임시 파일로 저장 → 아래 노드들이 이 파일을 읽음
        #    delete=False : with 블록이 끝나도 파일을 지우지 않음 (노드가 나중에 읽어야 하므로)
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as output:
            yaml.safe_dump(config, output)
            generated = output.name

        # launch 종료 시 임시 파일 삭제
        def cleanup(context):
            Path(generated).unlink(missing_ok=True)
            return []

        return [
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
            # Collision Monitor: /cmd_vel → (정지 영역 검사) → /cmd_vel_safe
            #   Nav2 기본 launch 에는 없어서 여기서 따로 띄움
            Node(package='nav2_collision_monitor', executable='collision_monitor',
                 name='collision_monitor', parameters=[generated], output='screen'),
            # Collision Monitor 는 lifecycle 노드라 누가 "켜라(activate)"고 해야 동작함
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_collision', parameters=[{
                     'use_sim_time': sim_time,
                     'autostart': IfCondition(LaunchConfiguration('autostart')).evaluate(context),
                     'node_names': ['collision_monitor']}], output='screen'),
            # Pi 4에서 프로세스 수와 메모리를 줄이려고 한 컨테이너에 싣습니다.
            # 이름은 nav2_container 고정입니다. navigation_launch.py의 기본 target과 같아야 합니다.
            #   컴포지션 = 여러 노드를 프로세스 하나 안에서 돌리기 (메모리 절약, 통신 빠름)
            Node(condition=IfCondition(LaunchConfiguration('use_composition')),
                 name='nav2_container', package='rclcpp_components',
                 executable='component_container_isolated',
                 parameters=[generated,
                             {'autostart': LaunchConfiguration('autostart')}],
                 output='screen'),
            # Nav2 공식 launch 를 포함하되 설정 파일만 우리가 만든 임시 파일로
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(bringup / 'launch' / 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': generated,
                    'autostart': LaunchConfiguration('autostart'),
                    'use_composition': LaunchConfiguration('use_composition'),
                }.items()),
        ]

    return LaunchDescription([
        # drive:=true 일 때만 motor.yaml 실측값을 반영 (기본 false = 구동 안 함)
        DeclareLaunchArgument('drive', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('motor_params_file', default_value=str(
            Path(get_package_share_directory('cwu_base')) / 'config' / 'motor.yaml')),
        DeclareLaunchArgument('nav2_params_file', default_value=str(params)),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # true 면 켜자마자 모든 Nav2 노드를 활성화
        DeclareLaunchArgument('autostart', default_value='true'),
        # Pi 4에서는 컴포지션이 프로세스 수와 메모리를 줄여 줍니다.
        DeclareLaunchArgument('use_composition', default_value='True'),
        OpaqueFunction(function=nodes),
    ])
