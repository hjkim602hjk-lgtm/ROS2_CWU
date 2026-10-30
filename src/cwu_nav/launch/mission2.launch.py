# 미션 2 전체를 한 번에 띄웁니다. 경기에서 허용된 "프로그램 실행 명령" 하나가 이것입니다.
#   G4 LiDAR + encoder_odom + LiDAR 장착 TF + 카메라·목표물 검출기 + mission2 상태머신.
#   SLAM은 띄우지 않음: 미션 2는 장애물 없는 정사각형이라 odom + 벽 맞춤으로 위치를 잡습니다 (mission2.py).
# 로봇을 출발 칸에 경기장이 왼쪽에 오게 놓은 뒤 실행합니다 (출발 자세 = (2,2) 칸 중심, +x).
#   ros2 launch cwu_nav mission2.launch.py        # 경기에서는 tools/field/m2_start.sh

"""Mission 2: LiDAR + encoder odometry + camera + state machine, from one command."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from cwu_slam.mounts import load_mount, static_tf_node


def _share(package, *parts):
    return str(Path(get_package_share_directory(package), *parts))


def _include(package, launch_file):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(_share(package, 'launch', launch_file)))


def generate_launch_description():
    mount = load_mount(_share('cwu_slam', 'config', 'mount.yaml'), 'laser_mount')
    return LaunchDescription([
        static_tf_node('laser_mount', 'laser_frame', mount, False),
        Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node', name='ydlidar_ros2_driver_node',
             parameters=[_share('cwu_slam', 'config', 'ydlidar_g4.yaml'), {'port': '/dev/ttyUSB0'}], output='screen'),
        _include('cwu_base', 'encoder.launch.py'),
        _include('cwu_perception', 'camera.launch.py'),
        Node(package='cwu_nav', executable='mission2', name='mission2',
             parameters=[_share('cwu_nav', 'config', 'mission2.yaml')], output='screen'),
    ])
