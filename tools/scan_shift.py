#!/usr/bin/env python3
"""Republish /scan_raw as /scan with its angles shifted (angle_min/max += shift)
and optionally its stamp moved (stamp_offset s; tests the scan timing offset).

Used by tools/slam_replay.py --scan-shift to test whether slam_toolbox mishandles
a LiDAR mounted with a yaw offset: shifting the scan angles by the mount yaw and
publishing the mount TF with yaw - shift gives the same geometry without the offset.
    python3 tools/scan_shift.py --ros-args -p shift:=3.14159
"""
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    node = rclpy.create_node('scan_shift')
    shift = node.declare_parameter('shift', 0.0).value
    offset = int(node.declare_parameter('stamp_offset', 0.0).value * 1e9)
    pub = node.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)

    def on_scan(msg):
        msg.angle_min += shift
        msg.angle_max += shift
        if offset:
            ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec + offset
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        pub.publish(msg)

    node.create_subscription(LaserScan, 'scan_raw', on_scan, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
