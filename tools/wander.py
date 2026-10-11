#!/usr/bin/env python3
"""Reactive LiDAR wander for data collection (development tool, not the mission explorer).

Run on the Pi with the SLAM launch (drive:=true) up:
    python3 wander.py [seconds]          # default 120 s
Drives forward slowly, turns in place toward open space when the front is
blocked, and spins 360 deg every SPIN_EVERY s when clear (rotation data for the
odometry calibration). Stops on timeout, stale scans, Ctrl+C. The robot is a
330 mm disc, so turning in place never sweeps new area.
Blind spot: the G4 sits 170 mm up. Lower objects are not seen; when the robot
pushes one, the wheels spin while the scan stays still (2026-10-11), so a
forward run whose scan does not change for STUCK_AFTER s backs off and turns.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

LASER_YAW = math.pi   # mount.yaml laser_mount.yaw
SPEED = 0.08          # m/s
TURN = 0.4            # rad/s
FRONT_STOP = 0.45     # m from robot centre, +-30 deg
DIAG_STOP = 0.30      # m from robot centre, 30-75 deg either side
CLEAR = 0.65          # front distance that ends a turn
SPIN_EVERY = 40.0     # s
SPIN_CLEAR = 0.25     # every return must be farther than this to spin in place
STUCK_AFTER = 3.0     # s of forward command with an unchanged scan
STUCK_DIFF = 0.015    # m, mean |range change| below which the scan "did not change"
BACK_CLEAR = 0.35     # m behind (|angle| > 150 deg) needed to back off


def scan_change(a, b):
    """Mean |range difference| over returns valid in both scans (same bins)."""
    d = [abs(x - y) for x, y in zip(a, b) if 0.15 < x < 8 and 0.15 < y < 8]
    return sum(d) / len(d) if d else 0.0


def sectors(scan):
    """Min range per base-frame sector, ignoring invalid (0) returns."""
    out = {'front': 9.9, 'left': 9.9, 'right': 9.9, 'back': 9.9, 'all': 9.9,
           'left_mean': 0.0, 'right_mean': 0.0}
    left, right = [], []
    for i, r in enumerate(scan.ranges):
        if not (scan.range_min < r < scan.range_max):
            continue
        a = math.degrees(math.remainder(scan.angle_min + i * scan.angle_increment + LASER_YAW,
                                        2 * math.pi))
        out['all'] = min(out['all'], r)
        if abs(a) >= 150:
            out['back'] = min(out['back'], r)
        if abs(a) <= 30:
            out['front'] = min(out['front'], r)
        elif 30 < a <= 75:
            out['left'] = min(out['left'], r)
        elif -75 <= a < -30:
            out['right'] = min(out['right'], r)
        if 20 < a <= 120:
            left.append(min(r, 3.0))
        elif -120 <= a < -20:
            right.append(min(r, 3.0))
    out['left_mean'] = sum(left) / len(left) if left else 0.0
    out['right_mean'] = sum(right) / len(right) if right else 0.0
    return out


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    rclpy.init()
    node = rclpy.create_node('wander')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)
    state = {'scan': None, 'stamp': 0.0}

    def on_scan(msg):
        state['scan'], state['stamp'] = msg, time.monotonic()

    node.create_subscription(LaserScan, 'scan', on_scan, qos_profile_sensor_data)
    start = last_spin = time.monotonic()
    mode, turn_dir, spin_until, log_at = 'forward', 1.0, 0.0, 0.0
    ref, ref_at, back_until = None, 0.0, 0.0  # scan snapshot for stuck detection
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            cmd = Twist()
            if now - start > duration:
                print('시간 종료')
                break
            if state['scan'] is None or now - state['stamp'] > 0.5:
                pub.publish(cmd)  # stale scan: stand still
                continue
            s = sectors(state['scan'])
            if mode != 'forward' or ref is None:
                ref, ref_at = list(state['scan'].ranges), now
            elif now - ref_at > STUCK_AFTER:
                if scan_change(ref, state['scan'].ranges) < STUCK_DIFF:
                    print('끼임 감지: 후진 후 회전', flush=True)
                    turn_dir = 1.0 if s['left_mean'] >= s['right_mean'] else -1.0
                    mode = 'back' if s['back'] > BACK_CLEAR else 'turn'
                    back_until = now + 1.5
                ref, ref_at = list(state['scan'].ranges), now
            if mode == 'back':
                if now >= back_until or s['back'] < BACK_CLEAR:
                    mode = 'turn'
                else:
                    cmd.linear.x = -SPEED
            if mode == 'forward':
                if s['front'] < FRONT_STOP or s['left'] < DIAG_STOP or s['right'] < DIAG_STOP:
                    mode = 'turn'
                    turn_dir = 1.0 if s['left_mean'] >= s['right_mean'] else -1.0
                elif now - last_spin > SPIN_EVERY and s['all'] > SPIN_CLEAR:
                    mode, spin_until = 'spin', now + 2 * math.pi / TURN  # one turn
                    turn_dir = -turn_dir
                else:
                    cmd.linear.x = SPEED
                    # Lean gently toward the more open side.
                    cmd.angular.z = max(-0.25, min(0.25, 0.3 * (s['left_mean'] - s['right_mean'])))
            if mode == 'turn':
                if s['front'] > CLEAR and s['left'] > DIAG_STOP and s['right'] > DIAG_STOP:
                    mode = 'forward'
                else:
                    cmd.angular.z = TURN * turn_dir
            if mode == 'spin':
                if now >= spin_until:
                    mode, last_spin = 'forward', now
                else:
                    cmd.angular.z = TURN * turn_dir
            pub.publish(cmd)
            if now - log_at > 2.0:
                log_at = now
                print(f'{now - start:5.0f}s {mode:7s} 앞 {s["front"]:.2f} 좌 {s["left"]:.2f} '
                      f'우 {s["right"]:.2f} v {cmd.linear.x:.2f} w {cmd.angular.z:+.2f}', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            pub.publish(Twist())
            time.sleep(0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
