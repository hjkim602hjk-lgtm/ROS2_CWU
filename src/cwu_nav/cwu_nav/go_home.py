"""Send Nav2 back to the start pose: the map origin, i.e. where SLAM was started.

    ros2 run cwu_nav go_home            # Ctrl+C cancels the goal
Development command. In a mission the state machine must trigger the return itself
after acquisition; no remote command is allowed once the mission starts.
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def home_goal(stamp):
    """Map-frame pose (0, 0, yaw 0): slam_toolbox puts the map origin at the start pose."""
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = stamp
    goal.pose.pose.orientation.w = 1.0
    return goal


def main():
    rclpy.init()
    node = rclpy.create_node('go_home')
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error('Nav2 navigate_to_pose 서버가 없습니다 (nav2.launch.py 실행 확인).')
        return
    last = {'t': 0.0}

    def feedback(msg):
        now = node.get_clock().now().nanoseconds / 1e9
        if now - last['t'] > 2.0:
            last['t'] = now
            print(f"남은 거리 {msg.feedback.distance_remaining:.2f} m", flush=True)

    future = client.send_goal_async(home_goal(node.get_clock().now().to_msg()), feedback)
    rclpy.spin_until_future_complete(node, future)
    handle = future.result()
    if not handle.accepted:
        node.get_logger().error('복귀 목표가 거부되었습니다.')
        return
    print('복귀 시작', flush=True)
    result = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result)
        res = result.result()
        status = res.status if res is not None else None
        print('복귀 완료' if status == 4 else f'복귀 실패 (status {status})', flush=True)
    except KeyboardInterrupt:
        rclpy.spin_until_future_complete(node, handle.cancel_goal_async(), timeout_sec=3.0)
        print('복귀 취소', flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
