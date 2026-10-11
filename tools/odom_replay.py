#!/usr/bin/env python3
"""Re-integrate recorded /odom with corrected wheel scales and publish odom -> base_link TF.

Offline SLAM comparison helper (PC, use_sim_time), started by tools/slam_replay.py:
    python3 tools/odom_replay.py --ros-args -p use_sim_time:=true -p a_l:=1.0 -p a_r:=1.0 \
        -p sep0:=0.20686 -p sep:=0.1977
Each /odom increment is split back into wheel travel with the recorded track width
(sep0), scaled by a_l/a_r, and re-integrated with the corrected width (sep), the
same model encoder_odom uses. Lets one bag be replayed under several calibrations.
"""
import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from cwu_base.kinematics import integrate


class OdomReplay(Node):
    def __init__(self):
        super().__init__('odom_replay')
        self.declare_parameters('', [('a_l', 1.0), ('a_r', 1.0), ('sep0', 0.20686), ('sep', 0.20686)])
        self.k = {n: self.get_parameter(n).value for n in ('a_l', 'a_r', 'sep0', 'sep')}
        self.prev, self.pose = None, (0.0, 0.0, 0.0)
        self.tf = TransformBroadcaster(self)
        # Same corrected pose as a topic, for backends that read odometry messages (Cartographer).
        self.out = self.create_publisher(Odometry, 'odom_corrected', 50)
        self.create_subscription(Odometry, 'odom', self.on_odom, 50)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        cur = (m.pose.pose.position.x, m.pose.pose.position.y, 2 * math.atan2(q.z, q.w))
        if self.prev is not None:
            x0, y0, t0 = self.prev
            ds = (cur[0] - x0) * math.cos(t0) + (cur[1] - y0) * math.sin(t0)
            dth = math.remainder(cur[2] - t0, 2 * math.pi)
            s_l = (ds - dth * self.k['sep0'] / 2) * self.k['a_l']
            s_r = (ds + dth * self.k['sep0'] / 2) * self.k['a_r']
            self.pose = integrate(self.pose, s_l, s_r, self.k['sep'])
        self.prev = cur
        x, y, th = self.pose
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = m.header.stamp, 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation = Quaternion(z=math.sin(th / 2), w=math.cos(th / 2))
        self.tf.sendTransform(t)
        o = Odometry()
        o.header, o.child_frame_id = t.header, 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation = t.transform.rotation
        self.out.publish(o)


def main():
    rclpy.init()
    node = OdomReplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
