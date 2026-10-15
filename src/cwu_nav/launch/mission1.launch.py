# 미션 1 전체를 한 번에 띄웁니다. 경기에서 허용된 "프로그램 실행 명령" 하나가 이것입니다.
#   SLAM(라이다·encoder_odom 주행) → 15 s 뒤 Nav2 → RealSense·목표물 검출기 → 미션 상태머신
# 출발 자세가 map 원점(집)이 되므로, 로봇을 출발 칸에 놓은 뒤 실행합니다.
#   ros2 launch cwu_nav mission1.launch.py        # 경기에서는 tools/field/start_mission1.sh

"""Mission 1: SLAM + Nav2 + camera + state machine, from one command."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _include(package, launch_file, arguments):
    path = Path(get_package_share_directory(package)) / 'launch' / launch_file
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments=arguments.items())


def generate_launch_description():
    config = Path(get_package_share_directory('cwu_nav')) / 'config' / 'mission1.yaml'
    return LaunchDescription([
        # 모든 주행 명령이 collision monitor(/cmd_vel → /cmd_vel_safe)를 거칩니다.
        _include('cwu_slam', 'slam.launch.py', {'drive': 'true', 'cmd_vel_topic': 'cmd_vel_safe'}),
        _include('cwu_perception', 'camera.launch.py', {}),
        # SLAM이 map → odom 을 내기 시작한 뒤 Nav2를 올립니다 (tools/field/restart_stack.sh와 같은 순서).
        TimerAction(period=15.0, actions=[_include('cwu_nav', 'nav2.launch.py', {'drive': 'true'})]),
        Node(package='cwu_nav', executable='mission1', name='mission1',
             parameters=[str(config)], output='screen'),
    ])
