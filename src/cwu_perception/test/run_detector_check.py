# D415 없이 target_detector 노드의 ROS 연결을 검증하는 스크립트입니다.
# 합성 CameraInfo·컬러 영상을 발행하고 /target/bearing이 기대 방향으로 나오는지 봅니다.
# 순수 계산은 test_detect.py가 보고, 여기서는 토픽·QoS·cv_bridge 연결을 봅니다.

"""Feed the detector node synthetic frames and verify the published bearing."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

WIDTH, HEIGHT, FX, CX, CY = 160, 120, 300.0, 80.0, 60.0
CENTRED = (70, 50, 20, 20)      # 광학 중심 바로 앞의 빨간 사각형
TO_THE_RIGHT = (120, 50, 20, 20)


class Feeder(Node):
    def __init__(self):
        super().__init__('detector_check')
        self.bridge = CvBridge()
        self.got = None
        self.info = self.create_publisher(
            CameraInfo, '/camera/camera/color/camera_info', qos_profile_sensor_data)
        self.color = self.create_publisher(
            Image, '/camera/camera/color/image_raw', qos_profile_sensor_data)
        self.create_subscription(PointStamped, '/target/bearing', self.on_bearing, 10)

    def on_bearing(self, msg):
        self.got = msg

    def frame(self, box):
        now = self.get_clock().now().to_msg()
        info = CameraInfo()
        info.header.stamp = now
        info.header.frame_id = 'camera_color_optical_frame'
        info.width, info.height = WIDTH, HEIGHT
        info.k = [FX, 0.0, CX, 0.0, FX, CY, 0.0, 0.0, 1.0]
        self.info.publish(info)

        image = np.full((HEIGHT, WIDTH, 3), 127, np.uint8)   # 경기장 회색
        x, y, w, h = box
        image[y:y + h, x:x + w] = (0, 0, 255)                # BGR 빨강
        color = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        color.header = info.header
        self.color.publish(color)


def wait_for_bearing(node, box, timeout=30.0):
    node.got = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and node.got is None:
        node.frame(box)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.1)
    assert node.got is not None, '/target/bearing이 나오지 않았습니다'
    return node.got


def verify():
    rclpy.init()
    node = Feeder()
    try:
        got = wait_for_bearing(node, CENTRED)
        p = got.point
        assert got.header.frame_id == 'camera_color_optical_frame', \
            '프레임이 컬러 광학 프레임이 아닙니다: ' + got.header.frame_id
        length = (p.x ** 2 + p.y ** 2 + p.z ** 2) ** 0.5
        assert abs(length - 1.0) < 1e-6, '단위 벡터가 아닙니다: %.6f' % length
        assert abs(p.x) < 0.02 and abs(p.y) < 0.02, \
            '광축 위 목표물이 중심에서 벗어났습니다 (%.3f, %.3f)' % (p.x, p.y)
        print('PASS: 중앙 /target/bearing = (%.3f, %.3f, %.3f) in %s'
              % (p.x, p.y, p.z, got.header.frame_id), flush=True)

        # 부호 확인: 오른쪽의 목표물은 +x여야 합니다. 뒤집히면 서보가 반대로 돕니다.
        right = wait_for_bearing(node, TO_THE_RIGHT).point
        assert right.x > 0.05, '오른쪽 목표물의 x가 양수가 아닙니다: %.3f' % right.x
        print('PASS: 오른쪽 /target/bearing = (%.3f, %.3f, %.3f)'
              % (right.x, right.y, right.z), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1')
    env.setdefault('ROS_DOMAIN_ID', '70')
    # 검사 프로세스도 같은 도메인·전송 설정으로 붙어야 합니다.
    # 노드 쪽에만 걸면 서로 안 보여서 '발행이 없다'로 잘못 나옵니다.
    os.environ.update(env)
    directory = Path(tempfile.mkdtemp(prefix='cwu-detector-check-'))
    env['ROS_LOG_DIR'] = str(directory / 'ros-log')
    logfile = directory / 'detector.log'
    print('Check artifacts: %s' % directory, flush=True)
    with logfile.open('w') as output:
        node = subprocess.Popen(
            ['ros2', 'run', 'cwu_perception', 'target_detector'],
            env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            verify()
            assert node.poll() is None, 'target_detector exited unexpectedly'
        finally:
            if node.poll() is None:
                node.send_signal(signal.SIGINT)
                try:
                    node.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(node.pid, signal.SIGKILL)
                    node.wait()
            print(logfile.read_text()[-4000:], flush=True)


if __name__ == '__main__':
    main()
