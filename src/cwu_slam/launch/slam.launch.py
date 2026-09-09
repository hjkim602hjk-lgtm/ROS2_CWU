# 실제 G4 LiDAR와 SLAM Toolbox를 실행하는 launch 진입점입니다.
# 엔코더 드라이버는 별도로 실행하여 odom → base_link TF를 제공해야 합니다.
# port 인자로 센서 포트를 지정하고, start_lidar:=false로 기존 센서 입력을 사용할 수 있습니다.

from cwu_slam.bringup import generate_bringup


def generate_launch_description():
    return generate_bringup(demo=False)
