# 실제 G4 LiDAR와 SLAM Toolbox를 실행하는 launch 진입점입니다.
# 엔코더 오도메트리(cwu_base)도 함께 띄워 odom → base_link TF를 제공합니다.
# port는 LiDAR, encoder_port는 엔코더 시리얼 포트이며 start_lidar/start_encoder로 개별 제외합니다.

from cwu_slam.bringup import generate_bringup


def generate_launch_description():
    return generate_bringup(demo=False)
