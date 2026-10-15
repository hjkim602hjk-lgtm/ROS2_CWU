#!/usr/bin/env python3
"""Print where the red cube sits in the webcam image, to measure the focal length fx.

Run on the Pi with nothing else using /dev/video0 (usb_cam stopped):
    python3 webcam_fx.py
1) Cube straight ahead of the camera, Z m away   -> read u0 (also the principal point cx)
2) Same Z, moved X m sideways                     -> read u1
   fx = |u1 - u0| * Z / X   (fy = fx). Put both into cwu_perception/config/webcam_info.yaml.
Uses the mission detector's own maths (cwu_perception.detect), 640x480 MJPEG like webcam.yaml.
"""
import time

import cv2

from cwu_perception.detect import largest_blob, red_mask

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            print('프레임 없음')
            time.sleep(0.5)
            continue
        mask = red_mask(frame, [0, 120, 60], [10, 255, 255], [170, 120, 60], [179, 255, 255])
        blob = largest_blob(mask, 60, 2.0, 0.5)
        size = (frame.shape[1], frame.shape[0], int((mask > 0).sum()))
        print('안 보임' if blob is None else
              'u %.1f  v %.1f  (화면 %dx%d, 빨간 화소 %d)' % (*blob, *size), flush=True)
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
finally:
    cap.release()
