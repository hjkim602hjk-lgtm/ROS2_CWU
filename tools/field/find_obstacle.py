"""Find the nearest object cluster in front of the robot (base frame +-45 deg, < 2 m).

Prints its centre in the robot frame and the map frame, and its size, so a loop
around it can be planned.
"""
import math
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

YAW = 3.1381  # mount.yaml laser yaw
import sys
CENTER = math.radians(float(sys.argv[1])) if len(sys.argv) > 1 else 0.0  # search direction (base frame)
rclpy.init()
n = rclpy.create_node('find_obstacle')
st = {}
n.create_subscription(LaserScan, 'scan', lambda m: st.__setitem__('s', m), qos_profile_sensor_data)
buf = Buffer()
TransformListener(buf, n)
e = time.time() + 15
while time.time() < e and not ('s' in st and buf.can_transform('map', 'base_link', rclpy.time.Time())):
    rclpy.spin_once(n, timeout_sec=0.1)
s = st['s']
pts = []
for i, r in enumerate(s.ranges):
    if 0.15 < r < 2.0:
        a = math.remainder(s.angle_min + i * s.angle_increment + YAW, 2 * math.pi)
        if abs(math.remainder(a - CENTER, 2 * math.pi)) < math.radians(45):
            pts.append((a, r, r * math.cos(a), r * math.sin(a)))
pts.sort(key=lambda p: math.remainder(p[0] - CENTER, 2 * math.pi))
# Clusters: consecutive returns closer than 8 cm to each other.
clusters, cur = [], []
for p in pts:
    if cur and math.hypot(p[2] - cur[-1][2], p[3] - cur[-1][3]) > 0.08:
        clusters.append(cur)
        cur = []
    cur.append(p)
if cur:
    clusters.append(cur)
clusters = [c for c in clusters if len(c) >= 3]
t = buf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
yaw = 2 * math.atan2(t.rotation.z, t.rotation.w)
print(f'로봇 map 자세 ({t.translation.x:.2f}, {t.translation.y:.2f}, {math.degrees(yaw):.0f}°)')
for c in clusters:
    cx = sum(p[2] for p in c) / len(c); cy = sum(p[3] for p in c) / len(c)
    width = math.hypot(c[0][2] - c[-1][2], c[0][3] - c[-1][3])
    mx = t.translation.x + cx * math.cos(yaw) - cy * math.sin(yaw)
    my = t.translation.y + cx * math.sin(yaw) + cy * math.cos(yaw)
    print(f'  방향 {math.degrees(math.atan2(cy, cx)):+4.0f}°, 가장 가까운 면 {min(p[1] for p in c):.2f} m, '
          f'보이는 폭 {width:.2f} m (점 {len(c)}), 중심 로봇기준 ({cx:.2f}, {cy:+.2f}), map ({mx:.2f}, {my:.2f})')
