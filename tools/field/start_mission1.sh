#!/bin/bash
# 경기 시작 명령 (규정 8): 로봇을 출발 칸에 놓은 뒤 원격 PC에서 한 번 실행합니다.
# Pi에서 미션 1 전체를 띄우고 바로 빠져나오므로, 이후 로그아웃·통신 차단과 무관하게 계속 돕니다.
#   tools/field/start_mission1.sh        # 로그: Pi ~/mission1.log
#   보드 텔레메트리(엔코더·PWM·PSD·서보)는 Pi ~/telemetry_날짜_시각.csv 에 같이 기록 (tools/telemetry_csv.py)
"$(dirname "$0")/stop_mission1.sh" > /dev/null  # 이전 실행 정리 (encoder_odom은 보드에 q)
scp -q "$(dirname "$0")/../telemetry_csv.py" rpi:~/telemetry_csv.py
timeout 30 ssh rpi 'setsid nohup bash -c "export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; source ~/robot_ws/install/setup.bash; source ~/ros_CWU/install/setup.bash; exec ros2 launch cwu_nav mission1.launch.py" > ~/mission1.log 2>&1 < /dev/null &
  setsid nohup bash -c "export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; exec python3 ~/telemetry_csv.py" > ~/telemetry.log 2>&1 < /dev/null &
  # 설정 파일을 읽는 동안은 bash로 보이므로 launch 프로세스가 뜰 때까지 최대 10 s 기다림
  for i in $(seq 1 10); do pgrep -f "[b]in/ros2 launch cwu_nav mission1" > /dev/null && break; sleep 1; done
  pgrep -f "[b]in/ros2 launch cwu_nav mission1" > /dev/null && echo "미션 1 시작 (Pi ~/mission1.log)" || echo "시작 실패: ~/mission1.log 확인"'
