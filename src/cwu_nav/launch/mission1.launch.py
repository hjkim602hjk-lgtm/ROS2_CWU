# 미션 1 전체를 한 번에 띄웁니다. 경기에서 허용된 "프로그램 실행 명령" 하나가 이것입니다.
#   SLAM(라이다·encoder_odom) + 카메라·목표물 검출기 + 미션 상태머신 (2026-10-29: Nav2 없음,
#   미션이 /base/command로 보드에 [F]·[TL/TR] 이산 이동을 보냄)
# 출발 자세가 map 원점(집)이 되므로, 로봇을 출발 칸에 놓은 뒤 실행합니다.
#   ros2 launch cwu_nav mission1.launch.py        # 경기에서는 tools/field/start_mission1.sh

"""Mission 1: SLAM + camera + state machine, from one command."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _include(package, launch_file, arguments):
    path = Path(get_package_share_directory(package)) / 'launch' / launch_file
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments=arguments.items())


def generate_launch_description():
    config = Path(get_package_share_directory('cwu_nav')) / 'config' / 'mission1.yaml'
    return LaunchDescription([
        _include('cwu_slam', 'slam.launch.py', {}),
        _include('cwu_perception', 'camera.launch.py', {}),
        Node(package='cwu_nav', executable='mission1', name='mission1',
             parameters=[str(config)], output='screen'),
    ])
