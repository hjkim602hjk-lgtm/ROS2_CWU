#!/bin/bash
# PC의 지금 코드(커밋 안 한 변경 포함)를 Pi ~/ros_CWU로 보내고 Pi에서 빌드합니다 (시험 전 한 번, 경기 중에는 쓰지 않음).
# 덮어쓰기 전에 Pi의 기존 src·tools를 ~/ros_CWU_backup_날짜_시각.tgz로 묶어 둡니다 (Pi에서만 고친 보정값 보호).
# ydlidar 드라이버는 Pi ~/robot_ws에 따로 빌드되어 있어 보내지 않고, cwu_sim은 PC 전용이라 뺍니다.
#   tools/field/deploy.sh
cd "$(dirname "$0")/../.." || exit 1
PKGS="cwu_base cwu_nav cwu_perception cwu_slam"
tar czf - --exclude=__pycache__ $(for p in $PKGS; do echo "src/$p"; done) tools |
  timeout 180 ssh rpi 'cd ~/ros_CWU && tar czf ~/ros_CWU_backup_$(date +%m%d_%H%M).tgz src tools && tar xzmf -' ||
  { echo "보내기 실패 (Pi 연결 확인)"; exit 1; }
# Pi가 전에 --symlink-install로 빌드했으면 같은 방식으로 (방식을 바꾸면 colcon이 기존 install을 못 덮어씀)
timeout 600 ssh rpi "bash -c 'set -o pipefail; source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash &&
  cd ~/ros_CWU && SYM= && { [ ! -L install/cwu_nav/share/cwu_nav/package.xml ] || SYM=--symlink-install; } &&
  colcon build \$SYM --packages-select $PKGS 2>&1 | tail -3'" ||
  { echo "Pi 빌드 실패"; exit 1; }
echo "배포 완료: $PKGS (백업 Pi ~/ros_CWU_backup_*.tgz)"
