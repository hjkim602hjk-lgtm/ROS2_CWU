# Gazebo 랜덤 경기장에서 미션 1을 그대로 돌립니다 (PC 전용).
#   실물 mission1.launch.py와 같은 미션·SLAM·검출기 설정 파일을 쓰고, G4·encoder_odom·usb_cam·보드 자리만
#   Gazebo 센서 + fake_board로 바꿉니다.
#   ros2 run cwu_sim make_maze --seed 3 --out /tmp/m.world && ros2 launch cwu_sim maze.launch.py world:=/tmp/m.world
"""Mission 1 in a generated Gazebo arena."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def share(package, *parts):
    return str(Path(get_package_share_directory(package), *parts))


def generate_launch_description():
    sim = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('world'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('track', default_value='', description='경로 기록 파일 (비우면 기록 안 함)'),
        DeclareLaunchArgument('mission', default_value='true', description='false면 mission1 없이 (도구 시험용)'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(share('gazebo_ros', 'launch', 'gzserver.launch.py')),
                                 launch_arguments={'world': LaunchConfiguration('world')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(share('gazebo_ros', 'launch', 'gzclient.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='tf2_ros', executable='static_transform_publisher', name='laser_mount',
             arguments=['--z', '0.17', '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'],
             parameters=[sim]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share('slam_toolbox', 'launch', 'online_async_launch.py')),
            launch_arguments={'use_sim_time': 'true',
                              # SIM_SLAM: SLAM 설정 비교 실험용 (기본 = 실물과 같은 cwu_slam slam.yaml)
                              'slam_params_file': EnvironmentVariable(
                                  'SIM_SLAM', default_value=share('cwu_slam', 'config', 'slam.yaml'))}.items()),
        Node(package='cwu_perception', executable='target_detector', name='target_detector',
             parameters=[share('cwu_perception', 'config', 'target.yaml'),
                         dict(sim, color_topic='/webcam/image_raw', camera_info_topic='/webcam/camera_info')]),
        Node(package='cwu_sim', executable='fake_board', name='fake_board', parameters=[sim]),
        Node(package='cwu_sim', executable='track_recorder', name='track_recorder',
             parameters=[dict(sim, out_file=LaunchConfiguration('track'))],
             condition=UnlessCondition(PythonExpression(["'", LaunchConfiguration('track'), "' == ''"]))),
        Node(package='cwu_nav', executable='mission1', name='mission1',
             parameters=[share('cwu_nav', 'config', 'mission1.yaml'), sim], output='screen',
             condition=IfCondition(LaunchConfiguration('mission'))),
    ])
