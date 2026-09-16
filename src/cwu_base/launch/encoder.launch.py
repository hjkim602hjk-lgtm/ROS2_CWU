# 엔코더 오도메트리 노드만 단독으로 실행하는 launch 진입점입니다.
# cwu_slam의 slam.launch.py가 이 파일을 포함해 SLAM과 함께 띄웁니다.
# port와 encoder_params_file 인자로 포트와 설정 파일을 바꿀 수 있습니다.

"""Standalone entrypoint for the encoder odometry node."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory('cwu_base')) / 'config' / 'encoder.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('encoder_params_file', default_value=str(config)),
        DeclareLaunchArgument('encoder_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='cwu_base', executable='encoder_odom', name='encoder_odom',
             parameters=[LaunchConfiguration('encoder_params_file'), {
                 'port': ParameterValue(LaunchConfiguration('encoder_port'),
                                        value_type=str),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                value_type=bool),
             }], output='screen'),
    ])
