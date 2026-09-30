"""ROS bridge + virtual serial safety check, no powered hardware required."""
import math
import os
import pty
import select
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from cwu_base.motor_bridge import MotorBridge


def main():
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    params = dict(calibration_confirmed=True, mount_calibration_confirmed=True,
                  wheel_radius=.04265, wheel_separation=.20686, ticks_per_rev=3009.5,
                  max_wheel_speed=.1, kp_left=1., ki_left=1., kp_right=1., ki_right=1.,
                  max_accel_ticks=100., pwm_limit=50, robot_radius=.15,
                  braking_distance=.05, safety_margin=.02, safety_latency=1.5,
                  max_linear_speed=.1, max_angular_speed=.5,
                  max_linear_accel=.1, max_angular_accel=.5, port=os.ttyname(slave))
    args = ['--ros-args']
    for key, value in params.items():
        args += ['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}']
    rclpy.init(args=args)
    node = MotorBridge()
    node.tf_ok = lambda: True

    def output():
        data = b''
        while select.select([master], [], [], .02)[0]:
            data += os.read(master, 65536)
        return data.decode()

    def frame(line):
        os.write(master, (line+'\n').encode())
        time.sleep(.01)
        node.poll()

    def healthy():
        scan = LaserScan()
        scan.header.stamp = node.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.range_min, scan.range_max = .1, 10.
        scan.angle_increment = .01
        scan.ranges = [1.]
        node.scan(scan)
        return scan

    try:
        node.poll()
        assert 'CFG,' in output()
        frame(f'ACK,{node.session}')
        frame(f'ENC,100,0,0,{node.session}')
        healthy()
        cmd = Twist()
        cmd.linear.x = .05
        node.command(cmd)
        node.send()
        assert 'VEL,' in output()
        frame(f'ENC,110,1,1,{node.session}')
        assert node.ticks == (1, 1)
        scan = healthy()
        scan.ranges = [math.inf]
        node.scan(scan)
        node.send()
        assert 'STOP' in output()
        healthy()
        node.send()
        assert 'STOP' in output()  # scan recovery must not restore old velocity
        node.command(cmd)
        node.send()
        assert 'VEL,' in output()
        node.tf_ok = lambda: False
        node.send()
        assert 'STOP' in output()
        node.tf_ok = lambda: True
        node.send()
        assert 'STOP' in output()
        old_session = node.session
        frame('READY,1')
        assert node.session != old_session and not node.safety.acknowledged
        node.command(cmd)
        node.send()
        assert output().splitlines()[-1] == 'STOP'
        node.disconnect()
        assert 'STOP' in output()
        print('motor bridge virtual serial checks passed')
    finally:
        node.disconnect()
        node.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)


if __name__ == '__main__':
    main()
