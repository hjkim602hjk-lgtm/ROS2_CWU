# 실물·데모 실행에서 공통으로 사용하는 ROS 2 노드 구성 파일입니다.
# 설정 YAML을 읽어 센서, 장착 TF, SLAM Toolbox와 선택적 RViz를 연결합니다.
# 실물에서는 G4 드라이버를, 데모에서는 가상 센서를 실행합니다.

"""Launch composition shared by the real and synthetic input entrypoints."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def generate_bringup(demo=False):
    share = Path(get_package_share_directory('cwu_slam'))
    config = share / 'config'
    arguments = [
        DeclareLaunchArgument('slam_params_file', default_value=str(config / 'slam.yaml')),
        DeclareLaunchArgument('mount_file', default_value=str(config / 'mount.yaml')),
        DeclareLaunchArgument('publish_mount_tf', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]
    if demo:
        arguments.append(DeclareLaunchArgument(
            'demo_params_file', default_value=str(config / 'demo.yaml')))
    else:
        arguments.extend([
            DeclareLaunchArgument('start_lidar', default_value='true'),
            DeclareLaunchArgument('lidar_params_file', default_value=str(config / 'ydlidar_g4.yaml')),
            DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        ])

    def nodes(context):
        use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
        mount = None
        if demo or IfCondition(LaunchConfiguration('publish_mount_tf')).evaluate(context):
            with open(LaunchConfiguration('mount_file').perform(context), encoding='utf-8') as f:
                mount = yaml.safe_load(f)['laser_mount']
            for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
                if isinstance(mount[key], bool) or not math.isfinite(float(mount[key])):
                    raise ValueError('Mount values must be finite numbers')
                mount[key] = float(mount[key])
            if demo and (mount['roll'] != 0 or mount['pitch'] != 0):
                raise ValueError('The 2D demo requires zero mount roll and pitch')

        result = []
        if IfCondition(LaunchConfiguration('publish_mount_tf')).evaluate(context):
            args = []
            for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
                args.extend(['--' + key, str(mount[key])])
            result.append(Node(
                package='tf2_ros', executable='static_transform_publisher',
                name='laser_mount', arguments=args + [
                    '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'],
                parameters=[{'use_sim_time': use_sim_time}], output='screen'))

        if demo:
            result.append(Node(
                package='cwu_slam', executable='demo_sensors', name='demo_sensors',
                parameters=[LaunchConfiguration('demo_params_file'), {
                    'use_sim_time': use_sim_time, 'laser_x': mount['x'],
                    'laser_y': mount['y'], 'laser_yaw': mount['yaw'],
                }], output='screen'))
        elif IfCondition(LaunchConfiguration('start_lidar')).evaluate(context):
            # Resolve only when requested: demo and recorded-data modes need no driver.
            get_package_share_directory('ydlidar_ros2_driver')
            result.append(Node(
                package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
                name='ydlidar_ros2_driver_node', parameters=[
                    LaunchConfiguration('lidar_params_file'), {
                        'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                        'use_sim_time': use_sim_time,
                    }], output='screen'))

        result.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory('slam_toolbox')) /
                                               'launch' / 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': LaunchConfiguration('slam_params_file'),
            }.items()))
        result.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', str(share / 'rviz' / 'slam.rviz')],
            parameters=[{'use_sim_time': use_sim_time}], output='screen'))
        return result

    return LaunchDescription(arguments + [OpaqueFunction(function=nodes)])
