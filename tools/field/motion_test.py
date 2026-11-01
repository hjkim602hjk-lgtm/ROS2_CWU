"""속도 추종·회전 각도 시험 (Pi): 목표 속도 계단 응답과 미션식 제자리 회전의 실제 각도를 CSV로 남깁니다.

PC에서 tools/field/motion_test.sh [all|speed|turn] 로 실행 (라이다·encoder_odom을 띄우고 이 파일을 Pi에서 돌림), 그림은
tools/field/plot_motion_test.py. 로봇 앞뒤 0.6 m·둘레 0.35 m를 비우고, 벽이 2 m 안에 보이는 곳에 놓습니다.
  속도: 0.08~0.25 m/s로 2 s 전진 → 1 s 정지 → 2 s 후진 → 1 s 정지 (제자리 근처로 돌아옴)
  회전: TL90·TR90·TL180·TR180을 두 번씩, 미션과 같은 속도(0.6 rad/s, 끝 무렵 0.4)·허용 4°로.
        실제 각도 = 회전 전후 라이다 벽 방향 차이 (turn_calib.py와 같은 방법)
결과: ~/motion_speed_<시각>.csv (시각, 단계, 목표 v·w, 보드 텔레메트리), ~/motion_turn_<시각>.csv
PSD가 550 이상이면 바로 섭니다 (전진 중 정면, 후진 중 뒤, 회전 중 아무 쪽).
"""
import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from turn_calib import TurnCalib  # noqa: E402

SPEEDS = (0.08, 0.12, 0.16, 0.20, 0.25)          # m/s
TURNS = (90, -90, 180, -180) * 2                  # deg, + = 반시계(TL)
TURN_SPEED, MIN_TURN_SPEED, TURN_TOL = 0.6, 0.4, math.radians(4)  # mission1.yaml과 같게
TURN_EXTRA = math.radians(1.0)    # mission1.yaml turn_extra_deg와 같게
FRONT, REAR = 0, 1                                # telemetry[6:10] 안 PSD 순서 (앞·뒤·좌·우)


class MotionTest(TurnCalib):
    def __init__(self, stamp):
        super().__init__()
        self.cmd_now, self.hot = (0.0, 0.0, 'idle'), 0
        self.rows = csv.writer(open(os.path.expanduser(f'~/motion_speed_{stamp}.csv'), 'w', newline=''))
        self.rows.writerow(['pi_time', 'phase', 'cmd_v', 'cmd_w', 'stm_ms', 'mode', 'enc_l', 'enc_r', 'pwm_l', 'pwm_r',
                            'psd_front', 'psd_rear', 'psd_left', 'psd_right'])
        self.node.create_subscription(Int32MultiArray, '/base/telemetry', self.on_telemetry, 50)

    def near(self, index=None):
        """PSD가 두 표본 연속 psd_stop 이상 (한 표본은 70~150씩 튀어 회전 중에 잘못 섰음, 미션도 두 표본)."""
        hot = bool(self.psd) and (max(self.psd) if index is None else self.psd[index]) >= self.psd_stop
        self.hot = (self.hot + 1) if hot else 0
        return self.hot >= 2

    def on_telemetry(self, msg):
        v, w, phase = self.cmd_now
        self.rows.writerow([f'{time.time():.3f}', phase, v, w] + list(msg.data[:10]))

    def drive(self, v, w, seconds, phase, psd_index=None):
        self.cmd_now = (v, w, phase)
        end = time.time() + seconds
        while time.time() < end:
            if self.near(psd_index):
                self.stop()
                raise RuntimeError(f'{phase}: PSD {self.psd} — 가까이 뭔가 있어 섰습니다.')
            msg = Twist()
            msg.linear.x, msg.angular.z = v, w
            self.cmd.publish(msg)
            self.spin(0.05)

    def speed_steps(self):
        self.arm()
        for v in SPEEDS:
            self.drive(v, 0.0, 2.0, f'+{v:.2f}', FRONT)
            self.drive(0.0, 0.0, 1.0, f'+{v:.2f} 정지')
            self.drive(-v, 0.0, 2.0, f'-{v:.2f}', REAR)
            self.drive(0.0, 0.0, 1.0, f'-{v:.2f} 정지')
        self.cmd_now = (0.0, 0.0, 'idle')

    def straight(self, distance, vmax, phase, accel=0.3, k_yaw=2.0, settle=True):
        """Drive `distance` m (odometry) holding the odometry heading, up to vmax, slowing near the end.
        Returns the odometry heading change (rad) — 좌우 바퀴가 같은 거리를 갔다고 엔코더가 본 만큼의 휨."""
        self.arm()
        start = self.tf2d('odom', 'base_link')
        yaw0, gone = start[2], 0.0
        while gone < distance:
            if self.near(FRONT):
                self.stop()
                raise RuntimeError(f'{phase}: 정면 PSD {self.psd[FRONT]} — 앞이 막혀 섰습니다.')
            now = self.tf2d('odom', 'base_link')
            gone = math.dist(now[:2], start[:2])
            v = min(vmax, max(0.04, math.sqrt(2 * accel * max(0.0, distance - gone))))
            w = -k_yaw * math.remainder(now[2] - yaw0, 2 * math.pi)  # 엔코더 방향을 붙잡음 (좌우 바퀴 속도 차로)
            self.cmd_now = (v, w, phase)
            msg = Twist()
            msg.linear.x, msg.angular.z = v, w
            self.cmd.publish(msg)
            self.spin(0.05)
        if settle:
            self.stop()
            self.cmd_now = (0.0, 0.0, phase + ' 정지')
            self.spin(1.0)
        end = self.tf2d('odom', 'base_link')
        side = -(end[0] - start[0]) * math.sin(yaw0) + (end[1] - start[1]) * math.cos(yaw0)
        print(f'{phase}: 엔코더 기준 옆으로 {100 * side:+.1f} cm (+ = 왼쪽)', flush=True)
        return math.remainder(end[2] - yaw0, 2 * math.pi)

    def run_1m(self, vmax, turns):
        """1 m 가고 180° 돌아 1 m 돌아옴. 구간 사이 정지는 약 3 s (사용자): 멈춤 1~1.5 s + 라이다 벽 방향 한 번
        (앞 구간의 끝 = 다음 구간의 시작으로 같이 씀). 구간마다 실제 휨 대 엔코더 휨을 turns에 남김."""
        yaw = self.wall_yaw()
        for name, deg in (('가기', None), ('회전', 180), ('오기', None)):
            odom = self.mission_turn(deg) if deg else self.straight(1.0, vmax, f'{name} {vmax:.2f}')
            after = self.wall_yaw()
            actual = odom + math.remainder((after - yaw) - odom, math.pi / 2)
            yaw = after
            turns.writerow([f'{name}', f'{math.degrees(odom):.2f}', f'{math.degrees(actual):.2f}',
                            f'{math.degrees(actual - odom):+.2f}'])
            print(f'{name}: 엔코더 {math.degrees(odom):+.1f}°, 실제 {math.degrees(actual):+.1f}° '
                  f'(실제-엔코더 {math.degrees(actual - odom):+.1f}°)', flush=True)

    def mission_turn(self, degrees, settle=True):
        """Turn like mission1 (0.6 rad/s, slowing to 0.4 near the end, done within 4°); odometry turn (rad)."""
        self.arm()
        goal = math.copysign(math.radians(abs(degrees)) + TURN_EXTRA, degrees)
        turned, last = 0.0, self.tf2d('odom', 'base_link')
        start = time.time()
        while abs(goal) - abs(turned) > TURN_TOL:
            if time.time() - start > 20:
                self.stop()
                raise RuntimeError('20 s 안에 다 못 돌았습니다.')
            if self.near():
                self.stop()
                raise RuntimeError(f'PSD {self.psd}: 가까이 뭔가 있어 섰습니다.')
            left = abs(goal) - abs(turned)
            msg = Twist()
            msg.angular.z = math.copysign(max(MIN_TURN_SPEED, min(TURN_SPEED, 2.0 * left)), goal)
            self.cmd_now = (0.0, msg.angular.z, f'회전 {degrees:+d}')
            self.cmd.publish(msg)
            self.spin(0.05)
            now = self.tf2d('odom', 'base_link')
            if now is not None and last is not None:
                turned += math.remainder(now[2] - last[2], 2 * math.pi)
            last = now or last
        if not settle:
            return turned
        self.stop()
        self.cmd_now = (0.0, 0.0, f'회전 {degrees:+d} 정지')
        self.spin(1.5)  # 멈춘 뒤 마지막 각도 + 스캔 지연
        now = self.tf2d('odom', 'base_link')
        if now is not None and last is not None:
            turned += math.remainder(now[2] - last[2], 2 * math.pi)
        self.cmd_now = (0.0, 0.0, 'idle')
        return turned


def main():
    stamp = time.strftime('%Y%m%d_%H%M%S')
    which = sys.argv[1] if len(sys.argv) > 1 else 'all'
    vmax = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
    del sys.argv[2:]  # turn_calib.TurnCalib는 두 번째 인자를 PSD 정지값으로 읽음
    rclpy.init()
    test = MotionTest(stamp)
    turns = csv.writer(open(os.path.expanduser(f'~/motion_turn_{stamp}.csv'), 'w', newline=''))
    turns.writerow(['cmd_deg', 'odom_deg', 'actual_deg', 'error_deg'])
    try:
        test.spin(2.0)
        if which == 'run1m':
            test.run_1m(vmax, turns)
        if which in ('all', 'speed'):
            print('속도 시험 시작', flush=True)
            test.speed_steps()
            print('속도 시험 끝', flush=True)
        if which in ('all', 'turn'):
            for deg in TURNS:
                before = test.wall_yaw()
                odom = test.mission_turn(deg)
                after = test.wall_yaw()
                actual = odom + math.remainder((after - before) - odom, math.pi / 2)
                turns.writerow([deg, f'{math.degrees(odom):.2f}', f'{math.degrees(actual):.2f}',
                                f'{math.degrees(actual) - deg:+.2f}'])
                print(f'회전 {deg:+d}°: odom {math.degrees(odom):+.1f}°, 실제 {math.degrees(actual):+.1f}°', flush=True)
    finally:
        test.stop()
        test.command.publish(String(data='q'))
        test.spin(0.3)
        rclpy.shutdown()
    print(f'결과: ~/motion_speed_{stamp}.csv, ~/motion_turn_{stamp}.csv', flush=True)


if __name__ == '__main__':
    main()
