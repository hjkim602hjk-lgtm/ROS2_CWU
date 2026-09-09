# 실행 중인 데모의 ROS 2 데이터 연결을 검사하는 통합 검증 파일입니다.
# /scan·/odom·/map을 수신해 이동, 스캔 유효성, 빈 공간·장애물 셀과 TF 연결을 확인합니다.
# 데모를 직접 실행하지는 않으며, 제한 시간 안에 조건이 충족되지 않으면 실패합니다.

"""Run against demo.launch.py: python3 src/cwu_slam/test/slam_smoke.py.

Integration check, deliberately separate from hardware-independent colcon tests.
"""
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    rclpy.init()
    node = rclpy.create_node('cwu_slam_smoke')
    data = {'map': None, 'scan': None, 'first': None, 'last': None}
    buffer = Buffer()
    listener = TransformListener(buffer, node)

    def odometry(msg):
        assert msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_link'
        position = msg.pose.pose.position
        xy = (position.x, position.y)
        if data['first'] is None:
            data['first'] = xy
        data['last'] = xy

    subscriptions = [
        node.create_subscription(OccupancyGrid, '/map', lambda m: data.update(map=m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(LaserScan, '/scan', lambda m: data.update(scan=m),
                                 qos_profile_sensor_data),
        node.create_subscription(Odometry, '/odom', odometry, 10),
    ]
    deadline = time.monotonic() + 45
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            grid, scan = data['map'], data['scan']
            if grid is None or scan is None or data['first'] is None:
                continue
            if math.dist(data['first'], data['last']) < 0.15:
                continue
            free = sum(v == 0 for v in grid.data)
            occupied = sum(v >= 65 for v in grid.data)
            if free < 100 or occupied < 20:
                continue
            try:
                buffer.lookup_transform('map', 'laser_frame', rclpy.time.Time())
                buffer.lookup_transform('odom', 'base_link', rclpy.time.Time.from_msg(scan.header.stamp))
            except TransformException:
                continue
            assert grid.header.frame_id == 'map'
            assert grid.info.resolution > 0
            assert len(grid.data) == grid.info.width * grid.info.height
            assert scan.header.frame_id == 'laser_frame'
            assert len(scan.ranges) >= 4
            assert abs(scan.angle_max - scan.angle_min -
                       (len(scan.ranges) - 1) * scan.angle_increment) < 1e-5
            assert all(math.isinf(r) or scan.range_min <= r <= scan.range_max
                       for r in scan.ranges)
            age = (node.get_clock().now() - rclpy.time.Time.from_msg(scan.header.stamp)).nanoseconds
            assert 0 <= age < 2_000_000_000, 'Scan is stale'
            print(f'PASS: map={grid.info.width}x{grid.info.height}, '
                  f'free={free}, occupied={occupied}; moving odometry, scan and TF verified')
            return
        raise AssertionError('No healthy moving SLAM pipeline within 45 seconds')
    finally:
        del subscriptions, listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
