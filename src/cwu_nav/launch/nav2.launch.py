# Nav2 기성 스택(navigation_launch.py)을 이 경기장 설정으로 실행하는 진입점입니다.
# use_composition일 때는 컨테이너를 여기서 직접 띄웁니다. navigation_launch.py는
# 실을 컨테이너를 만들지 않고, 없으면 오류 없이 아무것도 띄우지 않습니다.
# 사전 지도를 쓰지 않으므로 map_server와 AMCL은 띄우지 않습니다.
# 위치 추정은 이미 실행 중인 SLAM Toolbox의 map → odom TF를 그대로 사용합니다.
# 모터 연결은 autonomy.launch.py의 drive:=true에서만 별도로 실행합니다.

"""Stock Nav2 bringup parameterised for the 3.6 m arena, on top of online SLAM."""
from pathlib import Path
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from cwu_nav.drive_config import configure_navigation


def generate_launch_description():
    params = Path(get_package_share_directory('cwu_nav')) / 'config' / 'nav2.yaml'

    def nodes(context):
        # 드라이버와 같은 이유로 요청받을 때만 해석합니다.
        # Nav2는 선택 설치이므로 없는 환경에서 이 파일을 import하는 것만으로 실패하지 않게 합니다.
        bringup = Path(get_package_share_directory('nav2_bringup'))
        source = Path(LaunchConfiguration('nav2_params_file').perform(context))
        config = yaml.safe_load(source.read_text())
        measured = None
        if IfCondition(LaunchConfiguration('drive')).evaluate(context):
            motor_file = Path(LaunchConfiguration('motor_params_file').perform(context))
            measured = yaml.safe_load(motor_file.read_text())['motor_bridge']['ros__parameters']
            if not measured['calibration_confirmed'] or not measured['mount_calibration_confirmed']:
                raise ValueError('Real drive requires measured motor and LiDAR calibration')
        configure_navigation(config, params.parent, measured)
        sim_time = IfCondition(LaunchConfiguration('use_sim_time')).evaluate(context)
        config['collision_monitor']['ros__parameters']['use_sim_time'] = sim_time
        # Derived onboard from fixed calibration; no arena-dependent manual edits.
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as output:
            yaml.safe_dump(config, output)
            generated = output.name
        def cleanup(context):
            Path(generated).unlink(missing_ok=True)
            return []

        return [
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
            Node(package='nav2_collision_monitor', executable='collision_monitor',
                 name='collision_monitor', parameters=[generated], output='screen'),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_collision', parameters=[{
                     'use_sim_time': sim_time,
                     'autostart': IfCondition(LaunchConfiguration('autostart')).evaluate(context),
                     'node_names': ['collision_monitor']}], output='screen'),
            # Pi 4에서 프로세스 수와 메모리를 줄이려고 한 컨테이너에 싫습니다.
            # 이름은 nav2_container 고정입니다. navigation_launch.py의 기본 target과 같아야 합니다.
            Node(condition=IfCondition(LaunchConfiguration('use_composition')),
                 name='nav2_container', package='rclcpp_components',
                 executable='component_container_isolated',
                 parameters=[generated,
                             {'autostart': LaunchConfiguration('autostart')}],
                 output='screen'),
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
        DeclareLaunchArgument('drive', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('motor_params_file', default_value=str(
            Path(get_package_share_directory('cwu_base')) / 'config' / 'motor.yaml')),
        DeclareLaunchArgument('nav2_params_file', default_value=str(params)),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        # Pi 4에서는 컴포지션이 프로세스 수와 메모리를 줄여 줍니다.
        DeclareLaunchArgument('use_composition', default_value='True'),
        OpaqueFunction(function=nodes),
    ])
