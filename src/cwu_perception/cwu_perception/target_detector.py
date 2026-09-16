# D415의 컬러 영상에서 빨간 목표물을 찾아 /target/bearing으로 발행하는 노드입니다.
# 계산은 detect.py의 순수 함수가 하고, 이 파일은 구독·발행만 담당합니다.
# 깊이는 구독하지 않습니다. 이 노드가 답하는 것은 "목표물이 어느 쪽인가" 하나이며,
# 방향은 카메라 광학 프레임의 단위 벡터로 그대로 내보냅니다.

"""Red cube detector: colour image in, unit bearing out. No depth."""
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from cwu_perception.detect import bearing, largest_blob, red_mask


class TargetDetector(Node):
    def __init__(self):
        super().__init__('target_detector')
        defaults = {
            'color_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'bearing_topic': '/target/bearing',
            'hue_low_1': [0, 120, 60], 'hue_high_1': [10, 255, 255],
            'hue_low_2': [170, 120, 60], 'hue_high_2': [179, 255, 255],
            'min_area_px': 60, 'max_aspect': 2.0, 'min_fill': 0.5,
        }
        self.declare_parameters('', list(defaults.items()))
        self.p = {key: self.get_parameter(key).value for key in defaults}
        for key in ('hue_low_1', 'hue_high_1', 'hue_low_2', 'hue_high_2'):
            if len(self.p[key]) != 3 or not all(0 <= v <= 255 for v in self.p[key]):
                raise ValueError(key + ' must be three HSV values in 0..255')
        if self.p['min_area_px'] < 1 or not 0 < self.p['min_fill'] <= 1:
            raise ValueError('min_area_px >= 1 and 0 < min_fill <= 1')

        self.bridge = CvBridge()
        self.info = None
        self.publisher = self.create_publisher(
            PointStamped, self.p['bearing_topic'], 10)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'],
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.p['color_topic'],
                                 self.on_color, qos_profile_sensor_data)
        self.get_logger().info('빨간 목표물 검출 시작. 결과는 방향뿐이며 거리는 없습니다.')

    def on_info(self, msg):
        self.info = msg

    def on_color(self, msg):
        if self.info is None:
            return
        color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if (self.info.width, self.info.height) != (color.shape[1], color.shape[0]):
            # 내부 파라미터가 다른 해상도의 것이면 방향이 통째로 틀어집니다.
            self.get_logger().warn(
                'camera_info와 컬러 영상의 해상도가 다릅니다.',
                throttle_duration_sec=5.0)
            return

        p = self.p
        blob = largest_blob(red_mask(color, p['hue_low_1'], p['hue_high_1'],
                                     p['hue_low_2'], p['hue_high_2']),
                            p['min_area_px'], p['max_aspect'], p['min_fill'])
        if blob is None:
            # 안 보이면 발행하지 않습니다. 소비자가 header.stamp로 상실을 판정합니다.
            return
        k = self.info.k
        x, y, z = bearing(blob[0], blob[1], k[0], k[4], k[2], k[5])
        point = PointStamped()
        point.header = msg.header
        point.point.x = x
        point.point.y = y
        point.point.z = z
        self.publisher.publish(point)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TargetDetector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
