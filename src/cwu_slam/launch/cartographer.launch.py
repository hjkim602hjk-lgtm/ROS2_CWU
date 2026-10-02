# Cartographer 전용 실행 파일입니다. 설정은 config/cartographer_2d.lua에서 읽습니다.
# 기본은 실제 G4·엔코더이며, demo:=true를 주면 하드웨어 없이 가상 센서를 실행합니다.
# 공통 센서·TF 구성은 bringup.py를 공유하고 SLAM 백엔드는 Cartographer로 고정합니다.

#
# [공부 노트] SLAM Toolbox 대신 Cartographer(구글 SLAM)로 같은 일을 합니다. 비교 시험용.
#   ros2 launch cwu_slam cartographer.launch.py demo:=true

from cwu_slam.bringup import generate_bringup


# ros2 launch 가 부르는 약속된 함수 이름. demo 는 인자로 선택, backend 는 고정
def generate_launch_description():
    return generate_bringup(demo=None, backend='cartographer')
