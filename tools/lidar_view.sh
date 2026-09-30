#!/bin/bash
# 파이에서 G4 LiDAR와 브라우저용 웹 화면을 한 번에 켜고 끕니다. 개발·점검 전용입니다.
# 경기 중에는 켜지 마십시오(규정: 미션 중 외부 통신 금지).
#
# PC에서:  ssh rpi ~/ros_CWU/tools/lidar_view.sh start   → 브라우저로 http://<파이 IP>:8080
#          ssh rpi ~/ros_CWU/tools/lidar_view.sh stop
#          ssh rpi ~/ros_CWU/tools/lidar_view.sh status
#
# start는 먼저 stop을 합니다. 두 번 실행해도 드라이버가 LiDAR 포트를 두고 다투지 않습니다.
# 엔코더가 없으므로 start_encoder:=false입니다. 이때 SLAM은 지도를 만들지 못하고
# 웹 화면은 로봇 기준 스캔만 보여줍니다. 엔코더를 연결하면 이 인자를 빼면 됩니다.
# set -u는 쓰지 않습니다. ROS의 setup.bash가 정의되지 않은 변수를 참조합니다.
LIDAR_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
# [s]처럼 한 글자를 괄호로 감싸면 이 스크립트 자신의 명령줄과는 맞지 않아 자기를 죽이지 않습니다.
PATTERN='ros2 launch cwu_[s]lam|ydlidar_ros2_driver_nod[e]|rosbridge_websocke[t]|rosapi_nod[e]|http.server 808[0]|async_slam_toolbox_nod[e]|static_transform_publisher.*laser_moun[t]'

stop() {
  pkill -INT -f "$PATTERN"
  for _ in 1 2 3 4 5 6 7 8 9 10; do pgrep -f "$PATTERN" >/dev/null || return 0; sleep 1; done
  pkill -KILL -f "$PATTERN"
}

status() {
  pgrep -af "$PATTERN" | cut -c1-100 || echo '실행 중인 것이 없습니다.'
}

case "${1:-}" in
  start)
    stop
    source /opt/ros/humble/setup.bash
    source ~/ros_CWU/install/setup.bash
    setsid nohup ros2 launch cwu_slam slam_toolbox.launch.py start_encoder:=false \
      port:="$LIDAR_PORT" > /tmp/cwu-lidar.log 2>&1 < /dev/null &
    setsid nohup ros2 launch cwu_slam web.launch.py > /tmp/cwu-web.log 2>&1 < /dev/null &
    sleep 12
    status
    echo "브라우저: http://$(hostname -I | awk '{print $1}'):8080"
    ;;
  stop) stop; echo '정지했습니다.' ;;
  status) status ;;
  *) echo "사용법: $0 start|stop|status" >&2; exit 1 ;;
esac
