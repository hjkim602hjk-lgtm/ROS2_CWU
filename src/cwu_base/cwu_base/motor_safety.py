"""Strict MCU frames and monotonic safety gates; independent of ROS/hardware."""
import math
import re


def parse_frame(line):
    fields = line.decode('ascii').split(',')
    if fields == ['READY', '1']:
        return ('READY', 1)
    if fields[0] not in ('ACK', 'ENC') or len(fields) != (2 if fields[0] == 'ACK' else 5):
        raise ValueError('unknown frame')
    if not all(re.fullmatch(r'-?\d+', x) for x in fields[1:]):
        raise ValueError('invalid integer')
    values = tuple(map(int, fields[1:]))
    if fields[0] == 'ACK':
        valid = 0 < values[0] <= 2147483647
    else:
        ms, left, right, session = values
        valid = (0 <= ms <= 0xffffffff and 0 <= session <= 2147483647
                 and all(-2147483648 <= x <= 2147483647 for x in (left, right)))
    if not valid:
        raise ValueError('out of range')
    return (fields[0], *values)


def wheel_ticks(linear, angular, separation, radius, ticks_per_rev, limit):
    if not all(math.isfinite(x) for x in (linear, angular)):
        raise ValueError('nonfinite command')
    scale = ticks_per_rev / (2 * math.pi * radius)
    left, right = ((linear - angular * separation / 2) * scale,
                   (linear + angular * separation / 2) * scale)
    ratio = max(1., abs(left) / limit, abs(right) / limit)
    return left / ratio, right / ratio


class Safety:
    def __init__(self, command_timeout, scan_timeout, encoder_timeout):
        self.command_timeout = command_timeout
        self.scan_timeout = scan_timeout
        self.encoder_timeout = encoder_timeout
        self.reset()

    def reset(self):
        self.acknowledged = False
        self.scan = self.enc = -math.inf
        self.ms = None
        self.discard()

    def discard(self):
        self.velocity = None
        self.cmd = -math.inf

    def encoder(self, ms, now):
        if self.ms is not None and not 0 < (ms-self.ms) % (1 << 32) < (1 << 31):
            return False
        self.ms, self.enc = ms, now
        return True

    def healthy(self, now, tf_ok):
        return (self.acknowledged and tf_ok and 0 <= now-self.scan <= self.scan_timeout
                and 0 <= now-self.enc <= self.encoder_timeout)

    def command(self, left, right, now, tf_ok):
        if not self.healthy(now, tf_ok) or not all(math.isfinite(x) for x in (left, right)):
            self.discard()
            return False
        self.velocity, self.cmd = (left, right), now
        return True

    def output(self, now, tf_ok):
        if not self.healthy(now, tf_ok) or not 0 <= now-self.cmd <= self.command_timeout:
            self.discard()
        return self.velocity


def validate_config(c):
    measured = ('wheel_radius', 'wheel_separation', 'ticks_per_rev', 'max_wheel_speed',
                'kp_left', 'ki_left', 'kp_right', 'ki_right', 'max_accel_ticks',
                'robot_radius', 'braking_distance', 'safety_margin', 'max_linear_speed',
                'max_angular_speed', 'max_linear_accel', 'max_angular_accel', 'safety_latency')
    if (not c['calibration_confirmed'] or not c['mount_calibration_confirmed']
            or any(not math.isfinite(c[k]) or c[k] <= 0 for k in measured)
            or not 0 < c['pwm_limit'] <= 255 or not 50 <= c['mcu_timeout_ms'] <= 300
            or not c['publish_tf'] or c['robot_radius'] > .20 or c['left_sign'] not in (-1., 1.)
            or c['right_sign'] not in (-1., 1.)
            or any(not math.isfinite(c[k]) or c[k] <= 0 for k in
                   ('command_timeout', 'scan_timeout', 'encoder_timeout', 'tf_timeout', 'tf_future_tolerance'))
            or c['safety_latency'] < c['command_timeout'] + .05 + c['mcu_timeout_ms']/1000 + max(c['scan_timeout'], c['encoder_timeout'], c['tf_timeout']) + .2):
        raise ValueError('motor.yaml requires confirmed measured geometry, gains and safety limits')

    max_ticks = c['max_wheel_speed'] * c['ticks_per_rev'] / (2*math.pi*c['wheel_radius'])
    if (not 1 <= max_ticks <= 1000000 or c['max_accel_ticks'] > 10000000
            or any(c[k] > 1000000 for k in ('kp_left', 'ki_left', 'kp_right', 'ki_right'))):
        raise ValueError('motor.yaml exceeds MCU protocol limits')
