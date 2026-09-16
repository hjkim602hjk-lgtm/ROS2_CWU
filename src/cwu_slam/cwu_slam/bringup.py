# 실물·데모 실행에서 공통으로 사용하는 ROS 2 노드 구성 파일입니다.
# 설정 YAML을 읽어 센서, 장착 TF, SLAM 백엔드와 선택적 RViz를 연결합니다.
# 기존 launch는 backend 인자로 선택하며, 백엔드별 전용 launch는 선택을 고정합니다.
# 전용 launch의 demo:=true는 실제 장치 대신 가상 센서를 사용합니다.
# 실물에서는 G4 드라이버를, 데모에서는 가상 센서를 실행합니다.

"""Launch composition shared by the real and synthetic input entrypoints."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from cwu_slam.mounts import load_mount, static_tf_node, warn_if_uncalibrated


def generate_bringup(demo=False, backend=None):
    share = Path(get_package_share_directory('cwu_slam'))
    config = share / 'config'
    arguments = [
        DeclareLaunchArgument('slam_params_file', default_value=str(config / 'slam.yaml')),
        DeclareLaunchArgument('cartographer_config',
                              default_value=str(config / 'cartographer_2d.lua')),
        DeclareLaunchArgument('mount_file', default_value=str(config / 'mount.yaml')),
        DeclareLaunchArgument('publish_mount_tf', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]
    if backend is None:
        arguments.append(DeclareLaunchArgument(
            'backend', default_value='slam_toolbox',
            choices=['slam_toolbox', 'cartographer']))
    if demo is None:
        arguments.append(DeclareLaunchArgument('demo', default_value='false',
                                               choices=['true', 'false']))
    if demo is not False:
        arguments.extend([
            DeclareLaunchArgument('demo_params_file', default_value=str(config / 'demo.yaml')),
            # circle: 기존 고정 원운동(SLAM 단독 검증용).
            # cmd_vel: Nav2가 보낸 속도로 움직임(주행 경로 전체 검증용).
            DeclareLaunchArgument('demo_drive', default_value='circle',
                                  choices=['circle', 'cmd_vel']),
        ])
    if demo is not True:
        arguments.extend([
            DeclareLaunchArgument('start_lidar', default_value='true'),
            DeclareLaunchArgument('lidar_params_file', default_value=str(config / 'ydlidar_g4.yaml')),
            DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
            DeclareLaunchArgument('start_encoder', default_value='true'),
            DeclareLaunchArgument('encoder_port', default_value='/dev/ttyACM0'),
        ])

    def nodes(context):
        is_demo = demo if demo is not None else IfCondition(
            LaunchConfiguration('demo')).evaluate(context)
        use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
        mount = None
        publish_mount = IfCondition(
            LaunchConfiguration('publish_mount_tf')).evaluate(context)
        if is_demo or publish_mount:
            mount = load_mount(
                LaunchConfiguration('mount_file').perform(context), 'laser_mount')
            if is_demo and (mount['roll'] != 0 or mount['pitch'] != 0):
                raise ValueError('The 2D demo requires zero mount roll and pitch')
            if not is_demo:
                warn_if_uncalibrated(mount, 'laser_mount')

        result = []
        if publish_mount:
            result.append(static_tf_node(
                'laser_mount', 'laser_frame', mount, use_sim_time))

        if is_demo:
            result.append(Node(
                package='cwu_slam', executable='demo_sensors', name='demo_sensors',
                parameters=[LaunchConfiguration('demo_params_file'), {
                    'use_sim_time': use_sim_time, 'laser_x': mount['x'],
                    'laser_y': mount['y'], 'laser_yaw': mount['yaw'],
                    'drive': ParameterValue(LaunchConfiguration('demo_drive'),
                                            value_type=str),
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

        if not is_demo and IfCondition(LaunchConfiguration('start_encoder')).evaluate(context):
            # 엔코더가 odom -> base_link TF를 주지 않으면 SLAM은 지도를 만들지 못합니다.
            # 드라이버와 같은 이유로 요청받을 때만 해석합니다.
            encoder = Path(get_package_share_directory('cwu_base'))
            result.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(encoder / 'launch' / 'encoder.launch.py')),
                launch_arguments={
                    'encoder_port': LaunchConfiguration('encoder_port'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items()))

        selected_backend = backend or LaunchConfiguration('backend').perform(context)
        # SLAM Toolbox 전용 구성: YAML 설정으로 비동기 온라인 매핑을 실행합니다.
        if selected_backend == 'slam_toolbox':
            result.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(Path(get_package_share_directory('slam_toolbox')) /
                                                   'launch' / 'online_async_launch.py')),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'slam_params_file': LaunchConfiguration('slam_params_file'),
                }.items()))
        elif selected_backend == 'cartographer':
            # Cartographer는 YAML이 아니라 Lua 파일을 디렉터리와 파일명으로 나눠 받습니다.
            lua = Path(LaunchConfiguration('cartographer_config').perform(context))
            if not lua.is_file():
                raise ValueError('cartographer_config가 가리키는 Lua 파일이 없습니다: ' + str(lua))
            result.append(Node(
                package='cartographer_ros', executable='cartographer_node',
                name='cartographer_node', arguments=[
                    '-configuration_directory', str(lua.parent),
                    '-configuration_basename', lua.name],
                parameters=[{'use_sim_time': use_sim_time}], output='screen'))
            # cartographer_node는 submap만 내보내므로 /map은 이 노드가 따로 만듭니다.
            # resolution은 slam.yaml의 0.05와 맞춰 두 백엔드의 지도 축척을 같게 유지합니다.
            result.append(Node(
                package='cartographer_ros', executable='cartographer_occupancy_grid_node',
                name='cartographer_occupancy_grid_node', arguments=[
                    '-resolution', '0.05', '-publish_period_sec', '1.0'],
                parameters=[{'use_sim_time': use_sim_time}], output='screen'))
        else:
            raise ValueError("backend는 'slam_toolbox' 또는 'cartographer'여야 합니다: " + selected_backend)
        result.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', str(share / 'rviz' / 'slam.rviz')],
            parameters=[{'use_sim_time': use_sim_time}], output='screen'))
        return result

    return LaunchDescription(arguments + [OpaqueFunction(function=nodes)])
