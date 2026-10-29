# 엔코더 오도메트리 노드만 단독으로 실행하는 launch 진입점입니다.
# cwu_slam의 slam.launch.py가 이 파일을 포함해 SLAM과 함께 띄웁니다.
# port와 encoder_params_file 인자로 포트와 설정 파일을 바꿀 수 있습니다.
#
# [공부 노트] launch 파일이란?
#   노드를 하나씩 `ros2 run` 으로 켜는 대신, "어떤 노드를 어떤 설정으로 켤지"를 적어둔 대본입니다.
#   ROS 2 는 이 파일의 generate_launch_description() 함수를 불러 그 결과대로 노드를 띄웁니다.
#
#   실행 예
#     ros2 launch cwu_base encoder.launch.py
#     ros2 launch cwu_base encoder.launch.py encoder_port:=/dev/ttyACM0   # 인자 바꾸기
#   이 파일이 설치 폴더에 복사되려면 setup.py 의 data_files 에 launch/ 가 등록돼 있어야 합니다.

"""Standalone entrypoint for the encoder odometry node."""
from pathlib import Path  # 경로를 / 로 이어 붙이는 표준 라이브러리

# 설치된 패키지의 share 폴더 위치를 찾아줌 (install/cwu_base/share/cwu_base)
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription                   # "띄울 것 목록"을 담는 상자
from launch.actions import DeclareLaunchArgument       # 명령줄에서 바꿀 수 있는 인자 선언
from launch.substitutions import LaunchConfiguration   # 인자의 값을 "나중에" 꺼내 쓰는 자리표시
from launch_ros.actions import Node                    # ROS 노드 하나를 띄우는 동작
from launch_ros.parameter_descriptions import ParameterValue  # 인자 문자열을 원하는 자료형으로 변환


def generate_launch_description():
    # 기본 설정 파일 경로: <설치폴더>/share/cwu_base/config/encoder.yaml
    #   소스 폴더(src/...)가 아니라 설치 폴더를 보므로 YAML 을 고친 뒤엔 colcon build 가 필요합니다
    #   (--symlink-install 로 빌드했으면 바로 반영).
    config = Path(get_package_share_directory('cwu_base')) / 'config' / 'encoder.yaml'
    return LaunchDescription([
        # 인자 3개 선언. 형식: 이름, 기본값.  바꿀 때는  이름:=값
        DeclareLaunchArgument('encoder_params_file', default_value=str(config)),
        DeclareLaunchArgument('encoder_port', default_value='/dev/ttyAMA0'),
        # 시뮬레이션(가짜 시계)으로 돌릴 때만 true
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # 노드 하나 띄우기
        #   package    : 어느 패키지의
        #   executable : setup.py 에 등록한 실행 이름 (encoder_odom → encoder_odom.py 의 main)
        #   name       : 노드 이름. YAML 맨 위 키(encoder_odom:)와 같아야 파라미터가 들어감
        Node(package='cwu_base', executable='encoder_odom', name='encoder_odom',
             # 파라미터는 목록 순서대로 적용되고 뒤가 앞을 덮어씁니다.
             #   1) YAML 파일 전체  →  2) 아래 사전(port, use_sim_time)이 YAML 값을 덮음
             parameters=[LaunchConfiguration('encoder_params_file'), {
                 # 인자는 전부 문자열이라, 노드가 기대하는 자료형(str/bool)으로 바꿔 넘김
                 'port': ParameterValue(LaunchConfiguration('encoder_port'),
                                        value_type=str),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                value_type=bool),
             }], output='screen'),  # 노드 로그를 터미널에 바로 출력
    ])
