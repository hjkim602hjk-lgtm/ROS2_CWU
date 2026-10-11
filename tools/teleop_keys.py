#!/usr/bin/env python3
"""Minimal keyboard teleop for development mapping (teleop_twist_keyboard is not on the Pi).

Run on the Pi in a real terminal (ssh -t), with ROS sourced:
    python3 teleop_keys.py
w/s forward/back, a/d turn left/right, space stop, q quit.
Publishes /cmd_vel at 10 Hz; encoder_odom (drive:=true) zeroes motors if this stops.
Development only: during a mission the robot must not take remote commands.
"""
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist

LINEAR = 0.08   # m/s
ANGULAR = 0.40  # rad/s (slower stalls the wheels on the floor)
KEYS = {'w': (LINEAR, 0.0), 's': (-LINEAR, 0.0), 'a': (0.0, ANGULAR),
        'd': (0.0, -ANGULAR), ' ': (0.0, 0.0)}


def main():
    rclpy.init()
    node = rclpy.create_node('teleop_keys')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)
    old = termios.tcgetattr(sys.stdin)
    command = (0.0, 0.0)
    print('w 전진 / s 후진 / a 좌회전 / d 우회전 / space 정지 / q 종료')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1).lower()
                if key == 'q':
                    break
                if key in KEYS:
                    command = KEYS[key]
                    print(f'\r명령: 직진 {command[0]:+.2f} m/s, 회전 {command[1]:+.2f} rad/s   ',
                          end='', flush=True)
            msg = Twist()
            msg.linear.x, msg.angular.z = command
            pub.publish(msg)
    finally:
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.try_shutdown()
        print()


if __name__ == '__main__':
    main()
