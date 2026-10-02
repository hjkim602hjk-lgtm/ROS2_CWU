# SLAM Toolbox 전용 실행 파일입니다. 설정은 config/slam.yaml에서 읽습니다.
# 기본은 실제 G4·엔코더이며, demo:=true를 주면 하드웨어 없이 가상 센서를 실행합니다.
# 공통 센서·TF 구성은 bringup.py를 공유하고 SLAM 백엔드는 SLAM Toolbox로 고정합니다.

#
# [공부 노트] 실행 예
#   ros2 launch cwu_slam slam_toolbox.launch.py                 # 실물 (LiDAR + 엔코더)
#   ros2 launch cwu_slam slam_toolbox.launch.py demo:=true      # 가상 센서
#   ros2 launch cwu_slam slam_toolbox.launch.py start_encoder:=false  # LiDAR 만
#   확인: ros2 topic hz /map  ,  ros2 run tf2_ros tf2_echo map base_link
#   지도 저장: ros2 run nav2_map_server map_saver_cli -f ~/map
#   실제 구성 내용은 전부 cwu_slam/bringup.py 에 있습니다. 이 파일은 옵션만 고정합니다.

from cwu_slam.bringup import generate_bringup


# ros2 launch 가 부르는 약속된 함수 이름. 반드시 LaunchDescription 을 돌려줘야 함
def generate_launch_description():
    # demo=None → demo:= 인자로 선택 가능 / backend 는 SLAM Toolbox 로 고정
    return generate_bringup(demo=None, backend='slam_toolbox')
