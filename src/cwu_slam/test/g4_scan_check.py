#!/usr/bin/env python3
"""ROS 레벨 G4 검증. 드라이버가 발행하는 /scan과 장착 TF를 확인합니다.
엔코더가 없어 odom → base_link TF가 없으므로 /map은 검사하지 않습니다."""
import math, statistics, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

SECS = 5.0
rclpy.init()
node = Node('scan_check')
buf = Buffer()
TransformListener(buf, node)
msgs = []
node.create_subscription(LaserScan, '/scan', msgs.append, qos_profile_sensor_data)

t0 = time.time()
while time.time() - t0 < SECS:
    rclpy.spin_once(node, timeout_sec=0.1)
elapsed = time.time() - t0

assert msgs, '/scan 메시지를 하나도 못 받았습니다 (드라이버 로그 확인)'
hz = (len(msgs) - 1) / (
    (msgs[-1].header.stamp.sec - msgs[0].header.stamp.sec) +
    (msgs[-1].header.stamp.nanosec - msgs[0].header.stamp.nanosec) / 1e9) if len(msgs) > 1 else 0.0
s = msgs[-1]
print(f'[/scan] {len(msgs)}개 / {elapsed:.1f}s → 수신 {len(msgs)/elapsed:.1f} Hz, '
      f'헤더시각 기준 {hz:.1f} Hz')
print(f'[frame] frame_id={s.header.frame_id!r}')
print(f'[angle] {math.degrees(s.angle_min):.1f}° ~ {math.degrees(s.angle_max):.1f}°, '
      f'{len(s.ranges)}점, 증분 {math.degrees(s.angle_increment):.3f}°')

# 이 드라이버는 무효 측정을 inf가 아니라 0.0으로 채웁니다(ranges.resize 기본값).
# LaserScan 규격상 range_min~range_max 밖의 값은 소비자가 버리므로 0.0을 무효로 봅니다.
good = [r for r in s.ranges if not math.isinf(r) and not math.isnan(r) and r > 0]
print(f'[range] 유효 {len(good)}/{len(s.ranges)} '
      f'min={min(good):.3f}m median={statistics.median(good):.3f}m max={max(good):.3f}m')

age = (node.get_clock().now() - rclpy.time.Time.from_msg(s.header.stamp)).nanoseconds / 1e9
print(f'[stamp] 마지막 스캔 나이 {age:.3f}s')

assert s.header.frame_id == 'laser_frame', \
    f'frame_id가 {s.header.frame_id!r} — SLAM은 laser_frame을 기대합니다'
assert len(s.ranges) >= 4, '스캔 점 개수가 비정상입니다'
assert abs(s.angle_max - s.angle_min - (len(s.ranges) - 1) * s.angle_increment) < 1e-4, \
    '각도 범위와 점 개수·증분이 서로 맞지 않습니다'
assert not any(math.isnan(r) for r in s.ranges), 'NaN이 섞여 있습니다'
assert all(s.range_min <= r <= s.range_max for r in good), \
    '0이 아닌데 range_min/max를 벗어난 값이 있습니다'
assert len(good) / len(s.ranges) > 0.2, \
    f'유효 점 비율 {len(good)/len(s.ranges):.0%} — 센서 시야가 막혔거나 드라이버 이상'
assert 0 <= age < 2.0, f'스캔 타임스탬프가 {age:.1f}s 낡았습니다 (시계/드라이버 확인)'
assert good, '유효 거리값이 0개입니다'

try:
    tf = buf.lookup_transform('base_link', 'laser_frame', rclpy.time.Time())
    t, q = tf.transform.translation, tf.transform.rotation
    print(f'[TF] base_link → laser_frame  xyz=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) '
          f'quat=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f})')
except Exception as e:
    raise SystemExit(f'[TF] base_link → laser_frame 조회 실패: {e}')

try:
    buf.lookup_transform('odom', 'base_link', rclpy.time.Time())
    print('[TF] odom → base_link 존재 — 엔코더 드라이버가 이미 돌고 있습니다')
except Exception:
    print('[TF] odom → base_link 없음 — 엔코더 미연결이므로 정상. '
          'SLAM은 이 TF가 있어야 /map을 만듭니다')

print(f'\nOK: 드라이버가 유효한 /scan을 {hz:.1f} Hz로 발행하고 장착 TF도 연결됨')
node.destroy_node()
rclpy.shutdown()
