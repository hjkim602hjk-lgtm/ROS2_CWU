#!/bin/bash
# loop_trial.sh N CX CY R DIR : start scan -> record -> loop around obstacle + home -> end scan -> evaluate
N=$1; CX=$2; CY=$3; R=$4; D=$5; S=${FIELD:-$HOME/ros_CWU/log/field}; T=$HOME/ros_CWU/tools/field; mkdir -p $S/bags
timeout 500 ssh rpi "export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; source ~/ros_CWU/install/setup.bash;
  python3 ~/save_scan.py /tmp/t${N}_start.json 2>&1 | grep -v WARN
  rm -rf ~/bags/trial${N}; nohup ros2 bag record -o ~/bags/trial${N} /scan /odom /tf /tf_static /cmd_vel /cmd_vel_safe /plan > /dev/null 2>&1 < /dev/null &
  sleep 3; timeout 400 python3 ~/loop_obstacle.py $CX $CY $R $D 8 2>&1 | grep -vE 'WARN|^\s*$'
  sleep 2; python3 ~/save_scan.py /tmp/t${N}_home.json 2>&1 | grep -v WARN
  pkill -INT -f '[r]os2 bag record'; sleep 3; echo ABORTS \$(grep -c ABORT ~/slam_v3.log)"
scp -q rpi:/tmp/t${N}_start.json rpi:/tmp/t${N}_home.json $S/
source /opt/ros/humble/setup.bash
echo "-- home:"; /usr/bin/python3 $T/home_err.py $S/t${N}_start.json $S/t${N}_home.json
rsync -a rpi:bags/trial${N} $S/bags/ && cd ~/ros_CWU && /usr/bin/python3 tools/odom_calib.py $S/bags/trial${N} --sep 0.1975 --sep-cw 0.2040 --radius 0.04158 --dt 0.10 --pairs $S/bags/tp${N}.json 2>&1 | grep -E "쌍|배율|간격|방향|회전비"
echo "-- timeline:"; /usr/bin/python3 $T/timeline.py $S/bags/trial${N} $S/bags/tp${N}.json 2>&1 | grep -v INFO | head -12
echo TRIAL_DONE
