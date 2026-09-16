# SLAM Toolbox 전용 실행 파일입니다. 설정은 config/slam.yaml에서 읽습니다.
# 기본은 실제 G4·엔코더이며, demo:=true를 주면 하드웨어 없이 가상 센서를 실행합니다.
# 공통 센서·TF 구성은 bringup.py를 공유하고 SLAM 백엔드는 SLAM Toolbox로 고정합니다.

from cwu_slam.bringup import generate_bringup


def generate_launch_description():
    return generate_bringup(demo=None, backend='slam_toolbox')
