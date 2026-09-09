# 하드웨어 없이 가상 센서와 SLAM Toolbox를 함께 실행하는 launch 진입점입니다.
# 공통 실행 구성인 bringup.py에 데모 모드를 전달합니다. rviz:=true로 화면을 켤 수 있습니다.

from cwu_slam.bringup import generate_bringup


def generate_launch_description():
    return generate_bringup(demo=True)
