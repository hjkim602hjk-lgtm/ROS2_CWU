# D415 드라이버, 카메라 장착 TF, 빨간 목표물 검출기를 함께 띄우는 진입점입니다.
# 장착값은 cwu_slam의 mount.yaml을 그대로 공유해 TF 트리 정의를 한 곳에 둡니다.
# start_camera:=false로 드라이버를 빼면 bag 재생이나 정지 이미지 시험에 쓸 수 있습니다.
#
# [공부 노트] 이 파일이 띄우는 것
#   ① static TF base_link → camera_link   (mount.yaml 의 camera_mount)
#   ② realsense2_camera_node              → /camera/color/image_raw, camera_info (realsense.yaml)
#   ③ target_detector                     → /target/bearing (target.yaml)
#   실행 예: ros2 launch cwu_perception camera.launch.py
#           ros2 launch cwu_perception camera.launch.py start_camera:=false   # 녹화 재생 시험

"""RealSense D415 bringup plus the red target detector."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 다른 패키지(cwu_slam)의 장착값 도우미를 재사용 — package.xml 에 cwu_slam 의존성이 있어야 함
from cwu_slam.mounts import load_mount, static_tf_node, warn_if_uncalibrated


def generate_launch_description():
    share = Path(get_package_share_directory('cwu_perception')) / 'config'
    mount_file = Path(get_package_share_directory('cwu_slam')) / 'config' / 'mount.yaml'

    # 인자 값이 정해진 뒤 실행 (bringup.py 와 같은 OpaqueFunction 패턴)
    def nodes(context):
        use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
        result = []
        # ① 카메라 장착 TF
        if IfCondition(LaunchConfiguration('publish_mount_tf')).evaluate(context):
            mount = load_mount(
                LaunchConfiguration('mount_file').perform(context), 'camera_mount')
            warn_if_uncalibrated(mount, 'camera_mount')
            # realsense2_camera가 camera_link 아래 광학 프레임을 스스로 발행하므로
            # 여기서 만드는 것은 base_link → camera_link 하나뿐입니다.
            result.append(static_tf_node(
                'camera_mount', 'camera_link', mount, use_sim_time))

        # ② RealSense 드라이버
        if IfCondition(LaunchConfiguration('start_camera')).evaluate(context):
            # 드라이버와 같은 이유로 요청받을 때만 해석합니다. RealSense는 선택 설치입니다.
            get_package_share_directory('realsense2_camera')
            result.append(Node(
                package='realsense2_camera', executable='realsense2_camera_node',
                # namespace + name → 토픽이 /camera/color/... 로 시작하게 됨
                namespace='camera', name='camera',
                parameters=[LaunchConfiguration('camera_params_file'),
                            {'use_sim_time': use_sim_time}], output='screen'))

        # ③ 빨간 목표 검출기
        if IfCondition(LaunchConfiguration('start_detector')).evaluate(context):
            result.append(Node(
                package='cwu_perception', executable='target_detector',
                name='target_detector',
                parameters=[LaunchConfiguration('target_params_file'),
                            {'use_sim_time': use_sim_time}], output='screen'))
        return result

    return LaunchDescription([
        DeclareLaunchArgument('camera_params_file',
                              default_value=str(share / 'realsense.yaml')),
        DeclareLaunchArgument('target_params_file',
                              default_value=str(share / 'target.yaml')),
        DeclareLaunchArgument('mount_file', default_value=str(mount_file)),
        DeclareLaunchArgument('publish_mount_tf', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_detector', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=nodes),
    ])
