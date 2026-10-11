#!/usr/bin/env python3
"""Replay a robot bag through slam_toolbox offline with a chosen calibration (PC only).

    python3 tools/slam_replay.py BAG OUT_BAG [--yaw 3.14159] [--a_l 1 --a_r 1 --sep 0.1977]
        [--sep0 0.20686] [--params src/cwu_slam/config/slam.yaml] [--set name=value ...] [--rate 2]

Runs on ROS_DOMAIN_ID 77 so it cannot touch the robot (domain 30). Plays only /scan
and /odom from BAG; odom->base_link comes from tools/odom_replay.py (corrected
scales) and base_link->laser_frame from a static publisher (--yaw), so both can be
varied per run. slam_toolbox runs in sync mode (every scan, no drops). OUT_BAG gets
/tf and /map for tools/slam_eval.py.
"""
import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--yaw', type=float, default=3.14159)
    ap.add_argument('--a_l', type=float, default=1.0)
    ap.add_argument('--a_r', type=float, default=1.0)
    ap.add_argument('--sep0', type=float, default=0.20686)
    ap.add_argument('--sep', type=float, default=0.20686)
    ap.add_argument('--params', default=str(ROOT / 'src/cwu_slam/config/slam.yaml'))
    ap.add_argument('--set', nargs='*', default=[], help='slam_toolbox overrides name=value')
    ap.add_argument('--rate', type=float, default=2.0)
    ap.add_argument('--scan-shift', type=float, default=0.0,
                    help='add to scan angles and subtract from the mount yaw (same geometry)')
    ap.add_argument('--stamp-offset', type=float, default=0.0, help='seconds added to scan stamps')
    ap.add_argument('--backend', choices=['slam_toolbox', 'cartographer'], default='slam_toolbox')
    a = ap.parse_args()
    env = dict(os.environ, ROS_DOMAIN_ID='77')
    sim = ['-p', 'use_sim_time:=true']
    procs = []

    log = open(a.out.rstrip('/') + '.log', 'w')

    def start(cmd):
        procs.append(subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                                      start_new_session=True))

    try:
        start(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--z', '0.17',
               '--yaw', str(a.yaw - a.scan_shift), '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'])
        # sys.executable: 'python3' on PATH may be a non-ROS interpreter (PlatformIO).
        start([sys.executable, str(ROOT / 'tools/odom_replay.py'), '--ros-args', *sim,
               '-p', f'a_l:={a.a_l}', '-p', f'a_r:={a.a_r}', '-p', f'sep0:={a.sep0}', '-p', f'sep:={a.sep}'])
        overrides = [x for kv in a.set for x in ('-p', kv.replace('=', ':=', 1))]
        if a.backend == 'slam_toolbox':
            start(['ros2', 'run', 'slam_toolbox', 'sync_slam_toolbox_node', '--ros-args',
                   '-r', '__node:=slam_toolbox', '--params-file', a.params, *sim, *overrides])
        else:
            start(['ros2', 'run', 'cartographer_ros', 'cartographer_node',
                   '-configuration_directory', str(ROOT / 'src/cwu_slam/config'),
                   '-configuration_basename', 'cartographer_2d.lua',
                   '--ros-args', *sim, '-r', 'odom:=odom_corrected'])
            start(['ros2', 'run', 'cartographer_ros', 'cartographer_occupancy_grid_node',
                   '-resolution', '0.05', '--ros-args', *sim])
        subprocess.run(['rm', '-rf', a.out])
        start(['ros2', 'bag', 'record', '-o', a.out, '/tf', '/map'])
        time.sleep(4)
        play = ['ros2', 'bag', 'play', a.bag, '--clock', '--topics', '/scan', '/odom', '-r', str(a.rate)]
        if a.scan_shift or a.stamp_offset:
            start([sys.executable, str(ROOT / 'tools/scan_shift.py'), '--ros-args',
                   '-p', f'shift:={a.scan_shift}', '-p', f'stamp_offset:={a.stamp_offset}'])
            play += ['--remap', '/scan:=/scan_raw']
            time.sleep(2)
        subprocess.run(play, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(5)  # final map publication (map_update_interval)
    finally:
        for p in reversed(procs):
            os.killpg(p.pid, signal.SIGINT)
        for p in reversed(procs):
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)


if __name__ == '__main__':
    main()
