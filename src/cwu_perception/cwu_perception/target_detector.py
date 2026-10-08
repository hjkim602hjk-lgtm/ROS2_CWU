# D415의 컬러 영상에서 빨간 목표물을 찾아 /target/bearing으로 발행하는 노드입니다.
# 계산은 detect.py의 순수 함수가 하고, 이 파일은 구독·발행만 담당합니다.
# 깊이는 구독하지 않습니다. 이 노드가 답하는 것은 "목표물이 어느 쪽인가" 하나이며,
# 방향은 카메라 광학 프레임의 단위 벡터로 그대로 내보냅니다.
#
# [공부 노트] 입력·출력
#   입력  : /camera/camera/color/image_raw   (sensor_msgs/Image)      — 컬러 사진
#           /camera/camera/color/camera_info (sensor_msgs/CameraInfo) — 초점거리·중심 (bearing 계산에 필요)
#   설정  : config/target.yaml (HSV 범위, 덩어리 크기 조건)
#   출력  : /target/bearing (geometry_msgs/PointStamped) — point 에 길이 1 방향 (x, y, z)
#           ※ Point 이지만 "위치"가 아니라 "방향"입니다. 안 보이면 아예 발행하지 않습니다.
#   실행 : ros2 launch cwu_perception camera.launch.py  →  ros2 topic echo /target/bearing

"""Red cube detector: colour image in, unit bearing out. No depth."""
import rclpy
from cv_bridge import CvBridge  # ROS Image 메시지 ↔ OpenCV(numpy) 이미지 변환기
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException  # launch 가 종료시킬 때 나는 예외
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data  # 카메라 드라이버의 QoS(best effort)와 맞춰야 수신됨
from sensor_msgs.msg import CameraInfo, Image

from cwu_perception.detect import bearing, largest_blob, red_mask


class TargetDetector(Node):
    def __init__(self):
        super().__init__('target_detector')
        # 파라미터 이름: 기본값. target.yaml 로 덮어씀
        defaults = {
            'color_topic': '/camera/camera/color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'bearing_topic': '/target/bearing',
            'hue_low_1': [0, 120, 60], 'hue_high_1': [10, 255, 255],     # 빨강 범위 1 (H 0~10)
            'hue_low_2': [170, 120, 60], 'hue_high_2': [179, 255, 255],  # 빨강 범위 2 (H 170~179)
            'min_area_px': 60, 'max_aspect': 2.0, 'min_fill': 0.5,
        }
        # 사전의 items() → [(이름, 기본값), ...] 로 한꺼번에 선언
        self.declare_parameters('', list(defaults.items()))
        self.p = {key: self.get_parameter(key).value for key in defaults}
        # 설정값 검사: 틀린 값으로 조용히 돌지 않게 시작 단계에서 멈춤
        for key in ('hue_low_1', 'hue_high_1', 'hue_low_2', 'hue_high_2'):
            if len(self.p[key]) != 3 or not all(0 <= v <= 255 for v in self.p[key]):
                raise ValueError(key + ' must be three HSV values in 0..255')
        if self.p['min_area_px'] < 1 or not 0 < self.p['min_fill'] <= 1:
            raise ValueError('min_area_px >= 1 and 0 < min_fill <= 1')

        self.bridge = CvBridge()
        self.info = None  # 가장 최근 camera_info. 오기 전까지는 방향 계산 불가
        self.publisher = self.create_publisher(
            PointStamped, self.p['bearing_topic'], 10)
        # 구독 2개: 카메라 정보, 컬러 영상. 메시지가 오면 on_info / on_color 가 호출됨
        self.create_subscription(CameraInfo, self.p['camera_info_topic'],
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.p['color_topic'],
                                 self.on_color, qos_profile_sensor_data)
        self.get_logger().info('빨간 목표물 검출 시작. 결과는 방향뿐이며 거리는 없습니다.')

    # camera_info 는 저장만 해둠
    def on_info(self, msg):
        self.info = msg

    # 사진 한 장마다 호출: 검출 → 방향 계산 → 발행
    def on_color(self, msg):
        if self.info is None:
            return
        # ROS Image → OpenCV BGR 배열
        color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # color.shape = (높이, 너비, 3)
        if (self.info.width, self.info.height) != (color.shape[1], color.shape[0]):
            # 내부 파라미터가 다른 해상도의 것이면 방향이 통째로 틀어집니다.
            #   throttle_duration_sec : 같은 경고는 5초에 한 번만
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
        # k = 3x3 카메라 행렬을 1줄로 편 것 [fx 0 cx / 0 fy cy / 0 0 1]
        #   k[0]=fx, k[4]=fy, k[2]=cx, k[5]=cy
        k = self.info.k
        x, y, z = bearing(blob[0], blob[1], k[0], k[4], k[2], k[5])
        point = PointStamped()
        # 사진의 header 를 그대로 씀 → 촬영 시각 + 광학 프레임 이름이 따라감
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
        if rclpy.ok():  # 이미 종료됐으면 두 번 끄지 않음
            rclpy.shutdown()
