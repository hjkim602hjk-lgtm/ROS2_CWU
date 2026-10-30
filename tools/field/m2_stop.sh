#!/bin/bash
# 시험용 미션 2 정지 (경기에서는 쓰지 않음): 노드마다 SIGINT → 남은 것 강제 종료 → 집게 열기. 방식은 stop_mission1.sh와 같음
# (비대화형 셸에서 &로 띄운 ros2 launch는 SIGINT를 무시하므로 노드를 직접 끔. encoder_odom은 SIGINT를 받아야 보드에 q).
timeout 40 ssh rpi 'N="[c]wu_base/encoder_odom|[c]wu_nav/mission2|[u]sb_cam_node_exe|[r]ealsense2_camera_node|[t]arget_detector|[y]dlidar_ros2_driver_node|[s]tatic_transform_publisher|[t]elemetry_csv"
  pkill -INT -f "$N"
  for i in 1 2 3 4 5 6 7 8; do pgrep -f "$N" > /dev/null || break; sleep 1; done
  pkill -9 -f "$N"; pkill -9 -f "[b]in/ros2 launch cwu_nav mission2"; sleep 1
  pgrep -af "$N|[b]in/ros2 launch cwu_nav mission2" | cut -c1-80 || echo "미션 2 정지"
  stty -F /dev/ttyAMA0 115200 raw -echo && printf q > /dev/ttyAMA0 && sleep 0.5 && printf 7 > /dev/ttyAMA0 &&
    echo "집게 열기 보냄" || echo "집게 열기 실패 (/dev/ttyAMA0)"'
