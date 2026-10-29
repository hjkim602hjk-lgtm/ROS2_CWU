# 실물·데모 실행에서 공통으로 사용하는 ROS 2 노드 구성 파일입니다.
# 설정 YAML을 읽어 센서, 장착 TF, SLAM 백엔드와 선택적 RViz를 연결합니다.
# 기존 launch는 backend 인자로 선택하며, 백엔드별 전용 launch는 선택을 고정합니다.
# 전용 launch의 demo:=true는 실제 장치 대신 가상 센서를 사용합니다.
# 실물에서는 G4 드라이버를, 데모에서는 가상 센서를 실행합니다.
#
# [공부 노트] 이 파일이 띄우는 것 (실물 기준, slam_toolbox.launch.py 실행 시)
#   ┌ static_transform_publisher : base_link → laser_frame   (mount.yaml, mounts.py)
#   ├ ydlidar_ros2_driver_node   : /scan                      (ydlidar_g4.yaml)
#   ├ encoder_odom               : /odom, odom → base_link    (cwu_base/encoder.launch.py 포함)
#   ├ slam_toolbox               : /map,  map → odom          (slam.yaml)
#   └ rviz2 (rviz:=true 일 때만)  : 화면 보기 — 경기 중에는 쓰지 않음 (외부 시각화 금지 규정)
#   데모(demo:=true)면 LiDAR·엔코더 대신 demo_sensors 가 가짜 /scan, /odom 을 냅니다.
#
# [공부 노트] 왜 launch 파일이 아니라 일반 .py 에 있나?
#   launch 파일 4개(slam, slam_toolbox, cartographer, demo)가 거의 같은 구성을 써서,
#   공통 부분을 함수 generate_bringup() 하나로 빼고 각 launch 는 옵션만 다르게 불러 씁니다.
#
# [공부 노트] OpaqueFunction 이 뭔가?
#   launch 인자(port:=...)는 "실행하는 순간"에야 값이 정해집니다. 그런데 이 파일은
#   "demo 면 A 노드, 아니면 B 노드"처럼 값에 따라 분기해야 합니다.
#   OpaqueFunction(function=nodes) 로 감싸면, 인자 값이 정해진 뒤 nodes(context) 를 불러 줍니다.
#   그 안에서 .perform(context) / .evaluate(context) 로 실제 값을 꺼낼 수 있습니다.

"""Launch composition shared by the real and synthetic input entrypoints."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory  # 설치된 패키지 폴더 찾기
from launch import LaunchDescription
# DeclareLaunchArgument : 인자 선언 / IncludeLaunchDescription : 다른 launch 파일 포함
# OpaqueFunction        : 인자 값이 정해진 뒤 파이썬 함수 실행
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition  # 'true'/'false' 문자열 인자를 조건으로 사용
from launch.launch_description_sources import PythonLaunchDescriptionSource  # .py launch 불러오기
from launch.substitutions import LaunchConfiguration  # 인자 값 자리표시
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # 문자열 인자 → bool/str 변환

# 같은 패키지의 장착값 도우미 (mounts.py)
from cwu_slam.mounts import load_mount, static_tf_node, warn_if_uncalibrated


# ─────────────────────────────────────────────────────────────────────────────
# generate_bringup: SLAM 실행 구성 전체를 만들어 LaunchDescription 으로 돌려줍니다.
#   필요한 입력 (launch 파일이 고정해서 넘김)
#     demo    : True = 가상 센서만 / False = 실물만 / None = demo:=true|false 인자로 고름
#     backend : 'slam_toolbox' / 'cartographer' / None = backend:= 인자로 고름
# ─────────────────────────────────────────────────────────────────────────────
def generate_bringup(demo=False, backend=None):
    # 설치 폴더: install/cwu_slam/share/cwu_slam (config/, rviz/ 가 여기 복사돼 있음)
    share = Path(get_package_share_directory('cwu_slam'))
    config = share / 'config'
    # ── 항상 선언하는 인자 (명령줄에서 이름:=값 으로 바꿀 수 있음) ──
    arguments = [
        DeclareLaunchArgument('slam_params_file', default_value=str(config / 'slam.yaml')),
        DeclareLaunchArgument('cartographer_config',
                              default_value=str(config / 'cartographer_2d.lua')),
        DeclareLaunchArgument('mount_file', default_value=str(config / 'mount.yaml')),
        # 다른 곳에서 장착 TF 를 이미 낼 때 false 로 중복 방지
        DeclareLaunchArgument('publish_mount_tf', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]
    # backend 를 호출자가 안 정했을 때만 인자로 열어 줌. choices 밖의 값은 launch 가 거부
    if backend is None:
        arguments.append(DeclareLaunchArgument(
            'backend', default_value='slam_toolbox',
            choices=['slam_toolbox', 'cartographer']))
    # demo 도 마찬가지 (None 일 때만 선택 가능)
    if demo is None:
        arguments.append(DeclareLaunchArgument('demo', default_value='false',
                                               choices=['true', 'false']))
    # 데모가 "될 수도 있으면"(True 또는 None) 데모용 인자 선언
    if demo is not False:
        arguments.extend([
            # 가짜 로봇이 따라 움직일 속도 명령 토픽
            DeclareLaunchArgument('demo_cmd_vel_topic', default_value='/cmd_vel_safe'),
            DeclareLaunchArgument('demo_params_file', default_value=str(config / 'demo.yaml')),
            # circle: 기존 고정 원운동(SLAM 단독 검증용).
            # cmd_vel: Nav2가 보낸 속도로 움직임(주행 경로 전체 검증용).
            DeclareLaunchArgument('demo_drive', default_value='circle',
                                  choices=['circle', 'cmd_vel']),
        ])
    # 실물이 "될 수도 있으면"(False 또는 None) 실물 장치 인자 선언
    if demo is not True:
        arguments.extend([
            DeclareLaunchArgument('start_lidar', default_value='true'),
            DeclareLaunchArgument('lidar_params_file', default_value=str(config / 'ydlidar_g4.yaml')),
            # LiDAR USB 포트. G4 는 USB 변환보드라 ttyUSB0 로 잡힘
            DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
            DeclareLaunchArgument('start_encoder', default_value='true'),
            # Nucleo UART (Pi GPIO 14/15)
            DeclareLaunchArgument('encoder_port', default_value='/dev/ttyAMA0'),
        ])

    # ── nodes: 인자 값이 정해진 뒤 실행되어 "실제로 띄울 것" 목록을 돌려줌 ──
    #   context : launch 가 넘겨주는 실행 상황. 인자 값을 꺼낼 때 필요
    def nodes(context):
        # demo 가 None 이면 demo:= 인자를 읽어 True/False 결정
        is_demo = demo if demo is not None else IfCondition(
            LaunchConfiguration('demo')).evaluate(context)
        # 모든 노드에 넘길 use_sim_time (문자열 'false' → bool False)
        use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
        mount = None
        publish_mount = IfCondition(
            LaunchConfiguration('publish_mount_tf')).evaluate(context)
        # 장착값이 필요한 경우(데모의 가짜 LiDAR 위치, 또는 장착 TF 발행)만 YAML 읽기
        if is_demo or publish_mount:
            # .perform(context) : 자리표시 → 실제 문자열 경로
            mount = load_mount(
                LaunchConfiguration('mount_file').perform(context), 'laser_mount')
            # 가짜 센서는 평면(2D)만 흉내내므로 기울기(roll/pitch)가 있으면 거부
            if is_demo and (mount['roll'] != 0 or mount['pitch'] != 0):
                raise ValueError('The 2D demo requires zero mount roll and pitch')
            # 실물인데 6개 다 0 이면 "안 쟀다" 경고
            if not is_demo:
                warn_if_uncalibrated(mount, 'laser_mount')

        # 여기에 띄울 것들을 차례로 담아서 마지막에 return
        result = []
        # ① 장착 TF: base_link → laser_frame
        #    'laser_frame' 은 ydlidar_g4.yaml 의 frame_id 와 같아야 TF 사슬이 이어짐
        if publish_mount:
            result.append(static_tf_node(
                'laser_mount', 'laser_frame', mount, use_sim_time))

        # ② 센서: 데모면 가짜 센서, 아니면 진짜 LiDAR
        if is_demo:
            result.append(Node(
                package='cwu_slam', executable='demo_sensors', name='demo_sensors',
                # YAML 설정 + 장착 위치를 넘겨서 가짜 LiDAR 가 같은 위치에서 쏘도록 함
                parameters=[LaunchConfiguration('demo_params_file'), {
                    'use_sim_time': use_sim_time, 'laser_x': mount['x'],
                    'laser_y': mount['y'], 'laser_yaw': mount['yaw'],
                    'cmd_vel_topic': LaunchConfiguration('demo_cmd_vel_topic'),
                    'drive': ParameterValue(LaunchConfiguration('demo_drive'),
                                            value_type=str),
                }], output='screen'))
        elif IfCondition(LaunchConfiguration('start_lidar')).evaluate(context):
            # Resolve only when requested: demo and recorded-data modes need no driver.
            # ↑ 드라이버 패키지가 설치 안 돼 있으면 여기서 바로 에러 → 원인을 빨리 알 수 있음
            get_package_share_directory('ydlidar_ros2_driver')
            result.append(Node(
                package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
                name='ydlidar_ros2_driver_node', parameters=[
                    LaunchConfiguration('lidar_params_file'), {
                        'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                        'use_sim_time': use_sim_time,
                    }], output='screen'))

        # ③ 엔코더 오도메트리 (실물이고 start_encoder:=true 일 때)
        if not is_demo and IfCondition(LaunchConfiguration('start_encoder')).evaluate(context):
            # 엔코더가 odom -> base_link TF를 주지 않으면 SLAM은 지도를 만들지 못합니다.
            # 드라이버와 같은 이유로 요청받을 때만 해석합니다.
            encoder = Path(get_package_share_directory('cwu_base'))
            # 다른 패키지의 launch 파일을 통째로 포함 + 인자 전달
            result.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(encoder / 'launch' / 'encoder.launch.py')),
                launch_arguments={
                    'encoder_port': LaunchConfiguration('encoder_port'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items()))

        # ④ SLAM 백엔드 선택 (호출자가 정했으면 그것, 아니면 backend:= 인자)
        selected_backend = backend or LaunchConfiguration('backend').perform(context)
        # SLAM Toolbox 전용 구성: YAML 설정으로 비동기 온라인 매핑을 실행합니다.
        #   online : 돌면서 실시간으로 지도 생성 / async : 처리 못 따라가면 스캔을 건너뜀 (Pi 에 유리)
        #   slam_toolbox 패키지가 제공하는 launch 를 그대로 포함하고 설정 파일만 우리 것으로 바꿈
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
        # ⑤ RViz: condition=IfCondition(...) 이면 rviz:=true 일 때만 실제로 뜸
        result.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', str(share / 'rviz' / 'slam.rviz')],  # -d : 저장된 화면 배치 불러오기
            parameters=[{'use_sim_time': use_sim_time}], output='screen'))
        return result

    # 인자 선언들 + (인자 값이 정해진 뒤 nodes 를 실행하라는 지시)
    return LaunchDescription(arguments + [OpaqueFunction(function=nodes)])
