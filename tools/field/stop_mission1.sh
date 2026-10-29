#!/bin/bash
# 시험용 정지 (경기에서는 쓰지 않음): 미션 1 노드마다 SIGINT → 남은 것 강제 종료.
# start_mission1.sh가 비대화형 셸에서 &로 띄운 ros2 launch는 SIGINT를 무시한 채 시작하므로(2026-10-14 확인),
# launch가 아니라 노드를 직접 끕니다. encoder_odom은 SIGINT를 받아야 보드에 q(정지)를 보냅니다.
# 패턴의 [x] 괄호는 이 명령줄 자체가 걸리지 않게 하기 위한 것입니다.
timeout 40 ssh rpi 'N="[c]wu_base/encoder_odom|[c]wu_nav/mission1|[u]sb_cam_node_exe|[t]arget_detector|[y]dlidar_ros2_driver_node|[a]sync_slam_toolbox_node|[s]tatic_transform_publisher|[t]elemetry_csv"
  pkill -INT -f "$N"
  for i in 1 2 3 4 5 6 7 8; do pgrep -f "$N" > /dev/null || break; sleep 1; done
  pkill -9 -f "$N"; pkill -9 -f "[b]in/ros2 launch cwu_nav mission1"; sleep 1
  pgrep -af "$N|[b]in/ros2 launch cwu_nav mission1" | cut -c1-80 || echo "미션 1 정지"'
