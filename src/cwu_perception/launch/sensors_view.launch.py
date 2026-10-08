# 개발용: G4 LiDAR + RealSense 컬러 영상을 한 RViz 화면에서 확인합니다. 경기 중에는 쓰지 않습니다.
# SLAM·엔코더 없이 base_link 기준으로 /scan 과 /camera/camera/color/image_raw 만 봅니다.
# 깊이 스트림은 프로젝트 결정(CLAUDE.md "Confirmed Camera Role")에 따라 꺼져 있어 컬러만 나옵니다.
#
# 파이는 headless라 센서만 띄우고, 화면은 같은 네트워크의 PC에서 rviz2 로 봅니다.
#   파이: ros2 launch cwu_perception sensors_view.launch.py
#   PC  : rviz2 -d $(ros2 pkg prefix cwu_perception)/share/cwu_perception/rviz/sensors.rviz
#   PC 한 대에 센서를 꽂았을 때: ros2 launch cwu_perception sensors_view.launch.py rviz:=true

"""Bring up G4 LiDAR and the RealSense colour stream, and show both in RViz."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from cwu_slam.mounts import load_mount, static_tf_node


def generate_launch_description():
    share = Path(get_package_share_directory('cwu_perception'))
    slam_config = Path(get_package_share_directory('cwu_slam')) / 'config'
    use_sim_time = ParameterValue(False, value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        # base_link → laser_frame (base_link → camera_link 는 camera.launch.py 가 발행)
        static_tf_node('laser_mount', 'laser_frame',
                       load_mount(str(slam_config / 'mount.yaml'), 'laser_mount'), use_sim_time),
        Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
             name='ydlidar_ros2_driver_node', output='screen',
             condition=IfCondition(LaunchConfiguration('start_lidar')),
             parameters=[str(slam_config / 'ydlidar_g4.yaml'),
                         {'port': ParameterValue(LaunchConfiguration('port'), value_type=str)}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'camera.launch.py')),
            launch_arguments={'start_camera': LaunchConfiguration('start_camera')}.items()),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(share / 'rviz' / 'sensors.rviz')]),
    ])
