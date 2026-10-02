# 하드웨어 없이 가상 센서와 SLAM Toolbox를 함께 실행하는 launch 진입점입니다.
# 공통 실행 구성인 bringup.py에 데모 모드를 전달합니다. rviz:=true로 화면을 켤 수 있습니다.

#
# [공부 노트] 로봇 없이 PC 에서 SLAM 흐름을 공부할 때 가장 먼저 돌려볼 파일입니다.
#   ros2 launch cwu_slam demo.launch.py rviz:=true
#   → demo_sensors 가 가짜 방(demo.yaml)을 원운동하며 /scan·/odom 을 내고, 지도가 그려집니다.

from cwu_slam.bringup import generate_bringup


# ros2 launch 가 부르는 약속된 함수 이름. demo=True 고정 → 실물 장치 인자는 없음
def generate_launch_description():
    return generate_bringup(demo=True)
