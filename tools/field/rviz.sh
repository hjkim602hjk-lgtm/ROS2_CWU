#!/bin/bash
# 개발 PC에서 RViz 보기 (시험용 — 경기 중에는 원격 시각화 금지). 지도·스캔·웹캠.
# 웹캠은 PC 디코더(tools/webcam_view.py → /webcam/view)가 있어야 보이므로 같이 띄우고, RViz를 닫으면 같이 끕니다.
# PlatformIO 가상환경의 python3에는 cv2가 없어 시스템 파이썬을 씁니다.
#   tools/field/rviz.sh
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
export ROS_DOMAIN_ID=30
/usr/bin/python3 "$ROOT/tools/webcam_view.py" &
DECODER=$!
trap 'kill $DECODER 2>/dev/null' EXIT
rviz2 -d "$ROOT/src/cwu_slam/rviz/slam.rviz"
