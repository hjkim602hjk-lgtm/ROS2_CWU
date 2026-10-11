#!/usr/bin/env python3
"""Drive once around an obstacle with Nav2, then return to the start pose (map origin).

    python3 loop_obstacle.py CX CY R [ccw|cw] [N]     # map-frame centre, loop radius (m)
Waypoints: N points on the circle, starting at the one nearest the robot and going
a full turn in the given direction, each facing along the circle; the last pose is
home (0, 0, 0). Sent as one NavigateThroughPoses goal. Development test for the
return-to-start error (rotation-direction dependent odometry); not mission code.
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener


def loop_poses(cx, cy, r, ccw, n, rx, ry):
    """(x, y, yaw) waypoints: full circle from the point nearest (rx, ry), tangent heading."""
    start = math.atan2(ry - cy, rx - cx)
    step = 2 * math.pi / n * (1 if ccw else -1)
    out = []
    for k in range(1, n + 1):
        a = start + k * step
        yaw = a + (math.pi / 2 if ccw else -math.pi / 2)
        out.append((cx + r * math.cos(a), cy + r * math.sin(a), yaw))
    return out


def to_msg(x, y, yaw, stamp):
    p = PoseStamped()
    p.header.frame_id, p.header.stamp = 'map', stamp
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    cx, cy, r = map(float, sys.argv[1:4])
    ccw = (sys.argv[4] if len(sys.argv) > 4 else 'ccw') == 'ccw'
    n = int(sys.argv[5]) if len(sys.argv) > 5 else 8
    rclpy.init()
    node = rclpy.create_node('loop_obstacle')
    buf = Buffer()
    TransformListener(buf, node)
    while not buf.can_transform('map', 'base_link', rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.1)
    t = buf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform.translation
    poses = loop_poses(cx, cy, r, ccw, n, t.x, t.y) + [(0.0, 0.0, 0.0)]
    client = ActionClient(node, NavigateThroughPoses, 'navigate_through_poses')
    if not client.wait_for_server(timeout_sec=10.0):
        print('navigate_through_poses 서버 없음')
        return
    goal = NavigateThroughPoses.Goal()
    stamp = node.get_clock().now().to_msg()
    goal.poses = [to_msg(*p, stamp) for p in poses]
    print(f'경유점 {n}개 ({"반시계" if ccw else "시계"}) + 출발점', flush=True)
    handle_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, handle_future)
    handle = handle_future.result()
    if not handle.accepted:
        print('목표 거부')
        return
    result = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result)
        res = result.result()
        print('한 바퀴 + 복귀 완료' if res and res.status == 4 else
              f'실패 (status {res.status if res else None})', flush=True)
    except KeyboardInterrupt:
        rclpy.spin_until_future_complete(node, handle.cancel_goal_async(), timeout_sec=3.0)
        print('취소')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    pts = loop_poses(0.0, 0.0, 1.0, True, 4, 2.0, 0.0)
    assert all(abs(math.hypot(x, y) - 1.0) < 1e-9 for x, y, _ in pts)
    assert abs(pts[-1][0] - 1.0) < 1e-9 and abs(pts[0][1] - 1.0) < 1e-9  # CCW from (1, 0)
    main()
