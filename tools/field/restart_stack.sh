#!/bin/bash
# Restart SLAM (drive, cmd_vel_safe) + Nav2 on the Pi. The robot's current pose becomes the map origin (home).
T=$HOME/ros_CWU/tools; scp -q $T/field/save_scan.py $T/field/find_obstacle.py $T/loop_obstacle.py $T/wander.py rpi: || exit 1
timeout 200 ssh rpi 'source /opt/ros/humble/setup.bash
  pkill -INT -f "[r]os2 launch cwu_nav"; pkill -INT -f "[r]os2 launch cwu_slam"; sleep 5
  pkill -9 -f "[c]omponent_container_isolated|[c]ollision_monitor|[l]ifecycle_manager|[a]sync_slam_toolbox_node|[y]dlidar_ros2_driver_node|[c]wu_base/encoder_odom|[s]tatic_transform_publisher"; sleep 1
  nohup bash -c "export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; source ~/robot_ws/install/setup.bash; source ~/ros_CWU/install/setup.bash; exec ros2 launch cwu_slam slam.launch.py drive:=true cmd_vel_topic:=cmd_vel_safe" > ~/slam_v3.log 2>&1 < /dev/null &
  sleep 15
  nohup bash -c "export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; source ~/ros_CWU/install/setup.bash; exec ros2 launch cwu_nav nav2.launch.py drive:=true" > ~/nav2.log 2>&1 < /dev/null &
  sleep 25
  echo "slam sensor: $(grep -c "Registering sensor" ~/slam_v3.log), nav2 active: $(grep -c "Managed nodes are active" ~/nav2.log)"'
