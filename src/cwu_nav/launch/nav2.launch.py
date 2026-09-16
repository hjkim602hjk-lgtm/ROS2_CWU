# Nav2 기성 스택(navigation_launch.py)을 이 경기장 설정으로 실행하는 진입점입니다.
# use_composition일 때는 컨테이너를 여기서 직접 띄웁니다. navigation_launch.py는
# 실을 컨테이너를 만들지 않고, 없으면 오류 없이 아무것도 띄우지 않습니다.
# 사전 지도를 쓰지 않으므로 map_server와 AMCL은 띄우지 않습니다.
# 위치 추정은 이미 실행 중인 SLAM Toolbox의 map → odom TF를 그대로 사용합니다.
# 이 launch 단독으로는 로봇이 움직이지 않습니다. /cmd_vel을 모터로 보내는 노드가 아직 없습니다.

"""Stock Nav2 bringup parameterised for the 3.6 m arena, on top of online SLAM."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = Path(get_package_share_directory('cwu_nav')) / 'config' / 'nav2.yaml'

    def nodes(context):
        # 드라이버와 같은 이유로 요청받을 때만 해석합니다.
        # Nav2는 선택 설치이므로 없는 환경에서 이 파일을 import하는 것만으로 실패하지 않게 합니다.
        bringup = Path(get_package_share_directory('nav2_bringup'))
        print('[cwu_nav] /cmd_vel을 모터로 보내는 노드는 아직 없습니다. '
              'Nav2가 속도를 내보내도 실물 로봇은 움직이지 않습니다 (계획 Phase 1b).')
        return [
            # Pi 4에서 프로세스 수와 메모리를 줄이려고 한 컨테이너에 싫습니다.
            # 이름은 nav2_container 고정입니다. navigation_launch.py의 기본 target과 같아야 합니다.
            Node(condition=IfCondition(LaunchConfiguration('use_composition')),
                 name='nav2_container', package='rclcpp_components',
                 executable='component_container_isolated',
                 parameters=[LaunchConfiguration('nav2_params_file'),
                             {'autostart': LaunchConfiguration('autostart')}],
                 output='screen'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(bringup / 'launch' / 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': LaunchConfiguration('nav2_params_file'),
                    'autostart': LaunchConfiguration('autostart'),
                    'use_composition': LaunchConfiguration('use_composition'),
                }.items()),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('nav2_params_file', default_value=str(params)),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        # Pi 4에서는 컴포지션이 프로세스 수와 메모리를 줄여 줍니다.
        DeclareLaunchArgument('use_composition', default_value='True'),
        OpaqueFunction(function=nodes),
    ])
