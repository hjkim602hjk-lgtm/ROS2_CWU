# 실제 G4 LiDAR와 SLAM Toolbox를 실행하는 launch 진입점입니다.
# 엔코더 오도메트리(cwu_base)도 함께 띄워 odom → base_link TF를 제공합니다.
# port는 LiDAR, encoder_port는 엔코더 시리얼 포트이며 start_lidar/start_encoder로 개별 제외합니다.

#
# [공부 노트] 실행 예
#   ros2 launch cwu_slam slam.launch.py                            # 기본 SLAM Toolbox
#   ros2 launch cwu_slam slam.launch.py backend:=cartographer      # 백엔드 바꾸기
#   실물 전용(demo=False)이라 demo:= 인자는 없습니다. 구성은 bringup.py 참고.

from cwu_slam.bringup import generate_bringup


# ros2 launch 가 부르는 약속된 함수 이름
def generate_launch_description():
    # demo=False 고정, backend 는 backend:= 인자로 선택
    return generate_bringup(demo=False)
