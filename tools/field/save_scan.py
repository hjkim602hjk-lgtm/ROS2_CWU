"""Save one averaged /scan (median of 5) and the SLAM pose to a JSON file: save_scan.py OUT."""
import json, math, sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = rclpy.create_node('save_scan'); scans = []
n.create_subscription(LaserScan, 'scan', lambda m: scans.append(m), qos_profile_sensor_data)
buf = Buffer(); TransformListener(buf, n)
e = time.time() + 10
while len(scans) < 5 and time.time() < e: rclpy.spin_once(n, timeout_sec=0.1)
pose = None
for _ in range(100):
    rclpy.spin_once(n, timeout_sec=0.1)
    if buf.can_transform('map', 'base_link', rclpy.time.Time()):
        t = buf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
        pose = [t.translation.x, t.translation.y, 2 * math.atan2(t.rotation.z, t.rotation.w)]
        break
s = scans[-1]
ranges = [sorted(m.ranges[i] for m in scans[-5:])[2] for i in range(len(s.ranges))]
json.dump(dict(ranges=ranges, angle_min=s.angle_min, inc=s.angle_increment, range_min=s.range_min, slam=pose),
          open(sys.argv[1], 'w'))
print('saved', sys.argv[1], 'slam pose', pose)
