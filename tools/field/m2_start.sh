#!/bin/bash
# 미션 2 경기 시작 명령 (규정 8): 로봇을 출발 칸에 경기장이 왼쪽에 오게 놓은 뒤 원격 PC에서 한 번 실행합니다.
# Pi에서 미션 2 전체(LiDAR·엔코더·카메라·mission2)를 띄우고 바로 빠져나오므로, 이후 통신 차단과 무관하게 계속 돕니다.
#   tools/field/m2_start.sh        # 로그: Pi ~/mission2.log (보기: tools/field/m2_logs.sh)
#   보드 텔레메트리는 Pi ~/telemetry_날짜_시각.csv 에 같이 기록 (tools/telemetry_csv.py)
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
# Pi 코드가 PC와 다르면 경고만 (고친 코드를 안 올린 채 시험하는 실수 방지). 올리기: tools/field/deploy.sh
for f in src/cwu_nav/cwu_nav/mission2.py src/cwu_nav/cwu_nav/mission2_logic.py src/cwu_nav/config/mission2.yaml; do
  [ "$(md5sum < "$ROOT/$f")" = "$(timeout 10 ssh rpi "md5sum < ~/ros_CWU/$f")" ] || echo "경고: Pi의 $f 가 PC와 다름 (deploy.sh 안 함?)"
done
"$(dirname "$0")/stop_mission1.sh" > /dev/null  # 미션 1이 떠 있으면 정리 (보드 포트를 같이 쓰면 안 됨)
"$(dirname "$0")/m2_stop.sh" > /dev/null        # 이전 미션 2 실행 정리 (encoder_odom은 보드에 q)
scp -q "$(dirname "$0")/../telemetry_csv.py" rpi:~/telemetry_csv.py
timeout 30 ssh rpi 'setsid nohup bash -c "export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; source ~/robot_ws/install/setup.bash; source ~/ros_CWU/install/setup.bash; exec ros2 launch cwu_nav mission2.launch.py" > ~/mission2.log 2>&1 < /dev/null &
  setsid nohup bash -c "export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; exec python3 ~/telemetry_csv.py" > ~/telemetry.log 2>&1 < /dev/null &
  for i in $(seq 1 10); do pgrep -f "[b]in/ros2 launch cwu_nav mission2" > /dev/null && break; sleep 1; done
  pgrep -f "[b]in/ros2 launch cwu_nav mission2" > /dev/null && echo "미션 2 시작 (Pi ~/mission2.log)" || echo "시작 실패: ~/mission2.log 확인"'
