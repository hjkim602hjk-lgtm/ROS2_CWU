"""회전 배율 측정 (Pi): odom으로 두 바퀴(720°)씩 제자리 회전하고, 회전 전후 라이다 벽 방향 차이로 실제 회전을 잽니다.

경기장 벽·장애물은 서로 평행·직각이라 실제로 정확히 바퀴 수만큼 돌았으면 벽이 같은 방향으로 보여야 합니다. 남는 각도 = 실제 - odom.
결과 = 회전 방향별 배율(실제 / odom) → src/cwu_base/config/encoder.yaml의 turn_scale_ccw·turn_scale_cw에 넣을 값.
실물 시험(2026-10-31)에서 시계 방향 회전을 약 3% 적게 세어 방향 오차가 쌓이고 복귀를 못 했습니다.

PC에서 tools/field/turn_calib.sh [방향마다 횟수=2] [PSD 정지값=550] 로 실행 (라이다·encoder_odom을 띄우고 이 파일을 Pi에서 돌림).
로봇 둘레 30 cm 안에 아무것도 없고 벽이 라이다에 보이는 곳에 놓습니다. PSD가 550 이상이면 바로 섭니다.
"""
import math
import re
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

from cwu_nav.grid import lattice_fix, scan_points, wall_heading

SPEED, MIN_SPEED, PSD_STOP = 0.5, 0.35, 550   # rad/s, rad/s (모터 최소), ADC
ENCODER_YAML = Path.home() / 'ros_CWU/src/cwu_base/config/encoder.yaml'


class TurnCalib:
    def __init__(self):
        self.node = rclpy.create_node('turn_calib')
        self.tf = Buffer()
        TransformListener(self.tf, self.node)
        self.cmd = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.command = self.node.create_publisher(String, '/base/command', 10)
        self.scan, self.psd, self.armed = None, None, False
        self.psd_stop = int(sys.argv[2]) if len(sys.argv) > 2 else PSD_STOP
        self.node.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.node.create_subscription(Int32MultiArray, '/base/telemetry',
                                      lambda m: setattr(self, 'psd', list(m.data[6:10])), 10)
        self.node.create_subscription(String, '/base/event', self.on_event, 10)

    def on_scan(self, msg):
        self.scan = (time.time(), msg)

    def on_event(self, msg):
        if msg.data.startswith('# ARMED'):
            self.armed = True
        elif msg.data.startswith(('# ABORT', '# STOP')):
            self.armed = False

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def tf2d(self, parent, child):
        try:
            t = self.tf.lookup_transform(parent, child, Time()).transform
        except TransformException:
            return None
        return t.translation.x, t.translation.y, 2 * math.atan2(t.rotation.z, t.rotation.w)

    def wall_yaw(self, samples=6):
        """Robot yaw relative to the walls modulo 90° (rad), median of a few scans taken while standing still."""
        values = []
        for _ in range(samples * 3):
            self.spin(0.2)
            if self.scan is None or len(values) >= samples:
                continue
            msg = self.scan[1]
            laser = self.tf2d('base_link', msg.header.frame_id)
            if laser is None:
                continue
            points = [(x, y) for x, y in scan_points(msg.ranges, msg.angle_min, msg.angle_increment, laser[2])
                      if msg.range_min <= math.hypot(x, y) <= msg.range_max]
            heading = wall_heading(points)
            if heading is None:
                continue
            coarse = math.remainder(-heading, math.pi / 2)
            fine = lattice_fix(points, (0.0, 0.0, coarse), 0.4, max_yaw=math.radians(3))
            values.append(coarse + (fine[2] if fine else 0.0))
        if len(values) < samples // 2:
            raise RuntimeError('벽이 안 보입니다: 벽이 라이다에 보이는 곳에 놓아 주세요.')
        # 90° 주기라 ±45° 경계 근처 값이 섞일 수 있음: 첫 값 기준으로 펴서 중앙값
        values = [values[0] + math.remainder(v - values[0], math.pi / 2) for v in values]
        return sorted(values)[len(values) // 2]

    def stop(self):
        for _ in range(5):
            self.cmd.publish(Twist())
            self.spin(0.05)

    def arm(self):
        self.armed = False
        for _ in range(3):
            self.command.publish(String(data='ARM'))
            self.spin(1.0)
            if self.armed:
                return
        raise RuntimeError('보드가 ARM 되지 않습니다 (/base/event 확인).')

    def turn(self, degrees):
        """Turn in place until the odometry says `degrees` (signed); returns the odometry's measured turn (rad)."""
        self.arm()
        goal, turned = math.radians(degrees), 0.0
        last, start = self.tf2d('odom', 'base_link'), time.time()
        while abs(turned) < abs(goal) - math.radians(0.5):
            if time.time() - start > 60:
                self.stop()
                raise RuntimeError('60 s 안에 다 못 돌았습니다.')
            if self.psd and max(self.psd) >= self.psd_stop:
                self.stop()
                raise RuntimeError(f'PSD {self.psd}: 가까이 뭔가 있어 섰습니다. 둘레를 비우고 다시.')
            left = abs(goal) - abs(turned)
            speed = max(MIN_SPEED, min(SPEED, 1.5 * left))
            msg = Twist()
            msg.angular.z = math.copysign(speed, goal)
            self.cmd.publish(msg)
            self.spin(0.05)
            now = self.tf2d('odom', 'base_link')
            if now is not None and last is not None:
                turned += math.remainder(now[2] - last[2], 2 * math.pi)
            last = now or last
        self.stop()
        self.spin(1.5)  # 멈춘 뒤 마지막 각도 + 스캔 지연
        now = self.tf2d('odom', 'base_link')
        if now is not None and last is not None:
            turned += math.remainder(now[2] - last[2], 2 * math.pi)
        return turned


def current_scales():
    text = ENCODER_YAML.read_text() if ENCODER_YAML.exists() else ''
    found = {k: float(v) for k, v in re.findall(r'(turn_scale_c?cw):\s*([0-9.]+)', text)}
    return found.get('turn_scale_ccw', 1.0), found.get('turn_scale_cw', 1.0)


def main():
    trials = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    rclpy.init()
    calib = TurnCalib()
    ratios = {'ccw': [], 'cw': []}
    try:
        calib.spin(2.0)
        for k in range(trials):
            for name, degrees in (('ccw', 720), ('cw', -720)):  # 두 바퀴: 벽 방향 측정 잡음의 몫을 반으로
                before = calib.wall_yaw()
                odom = calib.turn(degrees)
                after = calib.wall_yaw()
                true = odom + math.remainder((after - before) - odom, math.pi / 2)
                ratios[name].append(true / odom)
                print(f'{name} {k + 1}: odom {math.degrees(odom):+.1f}°, 실제 {math.degrees(true):+.1f}° '
                      f'→ 배율 {true / odom:.4f}', flush=True)
    finally:
        calib.stop()
        calib.command.publish(String(data='q'))
        calib.spin(0.3)
    old = current_scales()
    new = [old[0] * sum(ratios['ccw']) / len(ratios['ccw']), old[1] * sum(ratios['cw']) / len(ratios['cw'])]
    print(f'\n지금 encoder.yaml: turn_scale_ccw {old[0]:.4f}, turn_scale_cw {old[1]:.4f}')
    print('넣을 값 (src/cwu_base/config/encoder.yaml):')
    print(f'    turn_scale_ccw: {new[0]:.4f}')
    print(f'    turn_scale_cw: {new[1]:.4f}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
