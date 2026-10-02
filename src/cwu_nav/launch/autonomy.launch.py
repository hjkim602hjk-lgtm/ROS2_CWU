# 미션 상태머신을 뺀 자율주행 스택 전체를 한 번에 띄우는 진입점입니다.
# SLAM(cwu_slam) → Nav2(cwu_nav) → D415 인식(cwu_perception) 순서로 포함하고,
# 목표물 방향을 바퀴 속도로 바꾸는 시각 서보(target_follower)를 함께 띄웁니다.
# demo:=true면 하드웨어 없이, demo_drive:=cmd_vel이면 Nav2 속도로 가상 로봇이 움직입니다.
# drive:=true에서만 실물 모터 연결을 실행합니다. 기본은 구동 비활성입니다.
#
# [공부 노트] 전체 그림 — 이 파일 하나로 뜨는 것
#   slam_toolbox.launch.py  → LiDAR, (엔코더 or 없음), 장착 TF, SLAM
#   nav2.launch.py          → Nav2 + Collision Monitor
#   motor_bridge            → drive:=true 일 때만. 이때는 encoder_odom 대신 이 노드가 /odom 을 냄
#   camera.launch.py        → camera:=true 일 때 RealSense + 빨간 목표 검출
#   target_follower         → follow:=true 일 때 목표 방향 → /cmd_vel_servo (아직 모터에 연결 안 됨)
#
# [공부 노트] 실행 예
#   ros2 launch cwu_nav autonomy.launch.py demo:=true rviz:=true           # PC 가상 시험
#   ros2 launch cwu_nav autonomy.launch.py camera:=false follow:=false     # 실물, 구동 없이 매핑
#   ros2 launch cwu_nav autonomy.launch.py drive:=true camera:=false follow:=false  # 실물 구동
#     (motor.yaml 실측 전에는 check_drive 가 거부)

"""SLAM + Nav2 + target detection in one launch; the mission state machine is not here."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
# PythonExpression: 인자 값을 넣어 파이썬 식을 계산 (예: "drive 가 true 가 아니면")
from launch.substitutions import LaunchConfiguration, PythonExpression
import yaml
from launch_ros.actions import Node


# _include: "다른 패키지의 launch 파일을 인자와 함께 포함"을 한 줄로 쓰기 위한 도우미
#   필요한 입력: 패키지 이름, launch 파일 이름, 넘길 인자 사전, (선택) 조건
def _include(package, launch_file, arguments, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / 'launch' / launch_file)),
        launch_arguments=arguments.items(), condition=condition)


def generate_launch_description():
    # 모든 하위 launch 에 공통으로 넘기는 인자
    common = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    # check_drive: drive:=true 일 때 위험한 조합을 시작 전에 막음
    #   - 데모/카메라/서보/가짜시계와 함께 실물 구동 금지 (첫 실물 시험은 단순하게)
    #   - motor.yaml 이 실측·확인 안 됐으면 거부 (motor_safety.validate_config)
    def check_drive(context):
        # lambda: 이름 없는 짧은 함수. enabled('drive') → True/False
        enabled = lambda key: IfCondition(LaunchConfiguration(key)).evaluate(context)
        if enabled('drive'):
            if any(enabled(k) for k in ('demo', 'camera', 'follow', 'use_sim_time')):
                raise ValueError('drive:=true requires demo/camera/follow/use_sim_time:=false')
            from cwu_base.motor_safety import validate_config
            config = yaml.safe_load(Path(LaunchConfiguration(
                'motor_params_file').perform(context)).read_text())['motor_bridge']['ros__parameters']
            validate_config(config)
        return []  # 띄울 노드는 없음 — 검사만

    return LaunchDescription([
        # ── 인자 선언 ──
        DeclareLaunchArgument('drive', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('motor_params_file', default_value=str(
            Path(get_package_share_directory('cwu_base')) / 'config' / 'motor.yaml')),
        DeclareLaunchArgument('demo', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('demo_drive', default_value='cmd_vel',
                              choices=['circle', 'cmd_vel']),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),          # LiDAR
        DeclareLaunchArgument('encoder_port', default_value='/dev/ttyAMA0'),  # Nucleo UART
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        # 카메라는 인식 없이 주행만 시험할 때 빼고 띄웁니다. Pi 4 CPU 측정에도 씁니다.
        DeclareLaunchArgument('camera', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('follow', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'servo_params_file',
            default_value=str(Path(get_package_share_directory('cwu_nav'))
                              / 'config' / 'servo.yaml')),

        # ── 시작 전 안전 검사 ──
        OpaqueFunction(function=check_drive),

        # SLAM Toolbox 고정: Nav2 코스트맵은 /map과 map → odom TF에 의존합니다.
        # 두 백엔드를 동시에 띄우면 TF가 중복되므로 여기서 하나로 고정합니다.
        # dict(common, a=1, b=2) : common 사전을 복사하고 항목 추가
        _include('cwu_slam', 'slam_toolbox.launch.py', dict(
            common,
            demo=LaunchConfiguration('demo'),
            demo_drive=LaunchConfiguration('demo_drive'),
            port=LaunchConfiguration('port'),
            encoder_port=LaunchConfiguration('encoder_port'),
            # drive 가 true 면 encoder_odom 을 끔 (motor_bridge 가 같은 포트로 /odom 을 내므로)
            #   결과 문자열 예: "'false' != 'true'" → 파이썬으로 계산하면 True
            start_encoder=PythonExpression(["'", LaunchConfiguration('drive'), "' != 'true'"]),
            demo_cmd_vel_topic='/cmd_vel_safe',
            rviz=LaunchConfiguration('rviz'))),
        _include('cwu_nav', 'nav2.launch.py', dict(common,
                 drive=LaunchConfiguration('drive'),
                 motor_params_file=LaunchConfiguration('motor_params_file'))),
        # 실물 모터 브리지 (drive:=true 일 때만)
        Node(condition=IfCondition(LaunchConfiguration('drive')),
             package='cwu_base', executable='motor_bridge', name='motor_bridge',
             parameters=[LaunchConfiguration('motor_params_file'),
                         {'port': LaunchConfiguration('encoder_port')}], output='screen'),
        _include('cwu_perception', 'camera.launch.py', dict(common),
                 condition=IfCondition(LaunchConfiguration('camera'))),
        # /target/bearing → /cmd_vel_servo. Nav2의 /cmd_vel과 이 토픽 중 무엇을
        # 모터로 보낼지는 안전 감시자(cwu_safety, 미구현)가 미션 상태를 보고 고릅니다.
        # 실물 drive 모드에서는 follow를 거부합니다. 서보 출력은 아직 구동에 연결하지 않습니다.
        Node(condition=IfCondition(LaunchConfiguration('follow')),
             package='cwu_nav', executable='target_follower', name='target_follower',
             parameters=[LaunchConfiguration('servo_params_file'), common],
             output='screen'),
    ])
