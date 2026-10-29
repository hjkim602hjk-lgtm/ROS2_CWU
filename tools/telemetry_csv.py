#!/usr/bin/env python3
# 보드 텔레메트리(/base/telemetry, encoder_odom이 보드 CSV 12필드를 그대로 발행)를 받는 대로 CSV에 씁니다.
# 보드 포트는 encoder_odom만 열 수 있으므로 미션 실행 중에도 이 토픽으로 기록합니다. 줄마다 flush → 중간에 꺼도 남음.
#   python3 telemetry_csv.py [출력.csv]     # 기본 ~/telemetry_YYYYmmdd_HHMMSS.csv, Ctrl+C로 종료
# 열 순서는 README_STM_PI_INTERFACE.md 6절. 보드 20 Hz, stm_ms 간격이 50보다 크면 그만큼 줄이 빠진 것.
import csv
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

COLUMNS = ['stm_ms', 'mode', 'enc_l', 'enc_r', 'pwm_l', 'pwm_r',
           'psd_front', 'psd_rear', 'psd_left', 'psd_right', 'servo_l', 'servo_r']


class TelemetryCsv(Node):
    def __init__(self, path):
        super().__init__('telemetry_csv')
        self.file = open(path, 'w', newline='')
        self.out = csv.writer(self.file)
        self.out.writerow(['pi_time'] + COLUMNS)
        self.rows = 0
        self.create_subscription(Int32MultiArray, '/base/telemetry', self.on_msg, 50)
        self.get_logger().info(f'기록 시작: {path}')

    def on_msg(self, msg):
        self.out.writerow([f'{time.time():.3f}'] + list(msg.data[:len(COLUMNS)]))
        self.file.flush()
        self.rows += 1


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        time.strftime('~/telemetry_%Y%m%d_%H%M%S.csv'))
    rclpy.init()
    node = TelemetryCsv(path)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C / stop_mission1.sh의 SIGINT
        pass
    finally:
        node.file.close()
        print(f'{node.rows}줄 저장: {path}')


if __name__ == '__main__':
    main()
