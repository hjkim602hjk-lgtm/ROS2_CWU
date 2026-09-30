"""Actual DDS + Collision Monitor + PTY bridge fault checks. No hardware."""
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml

from cwu_base.motor_bridge import MotorBridge


def main():
    os.environ.setdefault('ROS_DOMAIN_ID', '77')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    directory = Path(tempfile.mkdtemp(prefix='cwu-drive-safety-'))
    os.environ['ROS_LOG_DIR'] = str(directory / 'ros-log')
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    params = yaml.safe_load((Path(__file__).parents[1] / 'config/motor.yaml').read_text())['motor_bridge']['ros__parameters']
    # Synthetic fixture values, never copied to the hardware configuration.
    params.update(calibration_confirmed=True, mount_calibration_confirmed=True,
                  max_wheel_speed=.15, kp_left=1., ki_left=1., kp_right=1., ki_right=1.,
                  max_accel_ticks=100., pwm_limit=50, robot_radius=.15,
                  braking_distance=.05, safety_margin=.02, safety_latency=1.5,
                  max_linear_speed=.1, max_angular_speed=.5,
                  max_linear_accel=.1, max_angular_accel=.5, port=os.ttyname(slave))
    config = directory / 'motor.yaml'
    config.write_text(yaml.safe_dump({'motor_bridge': {'ros__parameters': params}}))
    rclpy.init(args=['--ros-args', '--params-file', str(config)])
    bridge = MotorBridge()
    test = Node('drive_safety_check')
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(test)
    raw = test.create_publisher(Twist, '/cmd_vel', 1)
    scans = test.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    dynamic = TransformBroadcaster(test)
    static = StaticTransformBroadcaster(test)
    mount = TransformStamped()
    mount.header.frame_id, mount.child_frame_id = 'base_link', 'laser'
    mount.transform.rotation.w = 1.
    static.sendTransform(mount)
    odom = []
    test.create_subscription(Odometry, '/odom', lambda msg: odom.append(msg), 1)
    state = dict(scan=True, encoder=True, tf=True, cmd=True, invalid=False, obstacle=False)
    session, received, wire = 0, [], b''
    start = time.monotonic()
    last_publish = last_encoder = -math.inf
    logfile = (directory / 'monitor.log').open('w')
    monitor = subprocess.Popen([
        str(Path(get_package_prefix('nav2_collision_monitor')) / 'lib/nav2_collision_monitor/collision_monitor'), '--ros-args',
        '--params-file', str(Path(__file__).parents[2] / 'cwu_nav/config/nav2.yaml')],
        stdout=logfile, stderr=subprocess.STDOUT)

    def pump(duration):
        nonlocal session, wire, last_publish, last_encoder
        end = time.monotonic()+duration
        while time.monotonic() < end:
            now = time.monotonic()
            executor.spin_once(timeout_sec=.005)
            try:
                wire += os.read(master, 65536)
            except BlockingIOError:
                pass
            while b'\n' in wire:
                line, wire = wire.split(b'\n', 1)
                line = line.decode().strip()
                received.append((time.monotonic(), line))
                if line.startswith('CFG,'):
                    session = int(line.split(',')[1])
                    os.write(master, f'ACK,{session}\n'.encode())
            if state['encoder'] and session and now-last_encoder >= .1:
                os.write(master, f'ENC,{int((now-start)*1000)},0,0,{session}\n'.encode())
                last_encoder = now
            if now-last_publish >= .05:
                last_publish = now
                stamp = test.get_clock().now().to_msg()
                if state['tf']:
                    tf = TransformStamped()
                    tf.header.stamp = stamp
                    tf.header.frame_id, tf.child_frame_id = 'map', 'odom'
                    tf.transform.rotation.w = 1.
                    dynamic.sendTransform(tf)
                if state['scan']:
                    scan = LaserScan()
                    scan.header.stamp, scan.header.frame_id = stamp, 'laser'
                    scan.range_min, scan.range_max = .1, 10.
                    scan.angle_min, scan.angle_max, scan.angle_increment = -.1, .1, .1
                    distance = math.inf if state['invalid'] else (.2 if state['obstacle'] else 2.)
                    scan.ranges = [distance]*3
                    scans.publish(scan)
                if state['cmd']:
                    cmd = Twist()
                    cmd.linear.x = .05
                    raw.publish(cmd)

    def moving():
        received.clear()
        pump(.8)
        assert any(line.startswith('VEL,') for _, line in received), received[-10:]

    def stops(fault, limit):
        began = time.monotonic()
        state[fault] = False
        received.clear()
        pump(limit+.15)
        assert any(t-began <= limit and line == 'STOP' for t, line in received), (fault, received)
        assert received[-1][1] == 'STOP', (fault, received[-5:])
        state[fault] = True
        moving()

    try:
        client = test.create_client(ChangeState, '/collision_monitor/change_state')
        deadline = time.monotonic()+10
        while not client.service_is_ready() and time.monotonic() < deadline:
            pump(.1)
        assert client.service_is_ready(), 'monitor did not start'
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            deadline = time.monotonic()+5
            while not future.done() and time.monotonic() < deadline:
                pump(.05)
            assert future.done() and future.result().success
        moving()
        assert odom and odom[-1].child_frame_id == 'base_link'
        assert test.count_publishers('/odom') == 1
        assert bridge.tf_buffer.can_transform('odom', 'base_link', rclpy.time.Time())
        for fault, limit in (('scan', .4), ('encoder', .4), ('tf', .6), ('cmd', .4)):
            stops(fault, limit)
        for fault in ('invalid', 'obstacle'):
            state[fault] = True
            received.clear()
            pump(.3)
            assert received[-1][1] == 'STOP', (fault, received[-5:])
            state[fault] = False
            moving()
        monitor.send_signal(signal.SIGINT)
        monitor.wait(timeout=5)
        received.clear()
        pump(.4)
        assert received[-1][1] == 'STOP'
        bridge.disconnect()
        print('PASS: DDS + Collision Monitor + single-port odom/TF; scan/encoder/TF/command loss, invalid scan, obstacle, monitor exit stop')
        print('Artifacts:', directory)
    finally:
        if monitor.poll() is None:
            monitor.terminate()
            monitor.wait(timeout=5)
        bridge.disconnect()
        executor.shutdown()
        bridge.destroy_node()
        test.destroy_node()
        rclpy.shutdown()
        logfile.close()
        os.close(master)
        os.close(slave)


if __name__ == '__main__':
    main()
