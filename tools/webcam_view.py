#!/usr/bin/env python3
"""Decode the robot webcam's JPEG stream on the PC for RViz (development viewing only).

The PC has no compressed_image_transport plugin, so RViz cannot subscribe to
/webcam/image_raw/compressed directly. This node decodes it and republishes a raw
image on /webcam/view, which stays on the PC (only JPEG crosses Wi-Fi).
    python3 tools/webcam_view.py       # with ROS_DOMAIN_ID=30
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


def main():
    rclpy.init()
    node = rclpy.create_node('webcam_view')
    bridge = CvBridge()
    pub = node.create_publisher(Image, 'webcam/view', 1)

    def on_jpeg(msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            out = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out.header = msg.header
            pub.publish(out)

    node.create_subscription(CompressedImage, 'webcam/image_raw/compressed', on_jpeg,
                             qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
