#!/bin/bash
# 속도 추종·회전 각도 시험 (PC에서): Pi에 라이다·encoder_odom(slam.launch.py)을 띄우고 motion_test.py를 돌린 뒤
# 결과 CSV를 PC로 가져와 그림을 그립니다. 로봇 앞뒤 0.6 m·둘레 0.35 m를 비우고 벽이 2 m 안에 보이는 곳에 놓습니다.
#   tools/field/motion_test.sh [all|speed|turn]     # 결과: tools/field/motion_results/ (CSV, PNG)
#   tools/field/motion_test.sh run1m [최고속도=0.25]  # 1 m 가기 → 180° → 1 m 오기 (앞 1.2 m 비울 것)
cd "$(dirname "$0")/../.." || exit 1
which=${1:-all}
out=tools/field/motion_results
mkdir -p "$out"
tools/field/stop_mission1.sh > /dev/null  # 미션·이전 노드 정리 (보드 q)
scp -q tools/field/motion_test.py tools/field/turn_calib.py rpi:~/ros_CWU/tools/field/
timeout 30 ssh rpi 'setsid nohup bash -c "export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; source ~/robot_ws/install/setup.bash; source ~/ros_CWU/install/setup.bash; exec ros2 launch cwu_slam slam.launch.py" > ~/motion_test_launch.log 2>&1 < /dev/null &'
sleep 12  # 라이다·TF·텔레메트리가 나올 때까지
timeout 600 ssh rpi "export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; source ~/ros_CWU/install/setup.bash; python3 ~/ros_CWU/tools/field/motion_test.py $which ${2:-}"
tools/field/stop_mission1.sh > /dev/null
scp -q "rpi:~/motion_speed_*.csv" "rpi:~/motion_turn_*.csv" "$out/" 2>/dev/null
speed=$(ls -t "$out"/motion_speed_*.csv | head -1)
turn=$(ls -t "$out"/motion_turn_*.csv | head -1)
/usr/bin/python3 tools/field/plot_motion_test.py "$speed" "$turn"
