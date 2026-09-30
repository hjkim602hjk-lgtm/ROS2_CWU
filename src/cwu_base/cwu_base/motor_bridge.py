"""One serial owner for closed-loop wheel commands and encoder odometry."""
import math
import secrets
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener, TransformException

from cwu_base.encoder_odom import EncoderOdometry
from cwu_base.kinematics import integrate, ticks_to_metres, wrap_delta
from cwu_base.motor_safety import Safety, parse_frame, wheel_ticks, validate_config


class MotorBridge(EncoderOdometry):
    def __init__(self):
        Node.__init__(self, 'motor_bridge')
        measured = ('wheel_radius', 'wheel_separation', 'ticks_per_rev', 'max_wheel_speed',
                    'kp_left', 'ki_left', 'kp_right', 'ki_right', 'max_accel_ticks',
                    'robot_radius', 'braking_distance', 'safety_margin', 'max_linear_speed',
                    'max_angular_speed', 'max_linear_accel', 'max_angular_accel', 'safety_latency')
        defaults = [(k, -1.0) for k in measured] + [
            ('calibration_confirmed', False), ('mount_calibration_confirmed', False),
            ('port', '/dev/ttyAMA0'), ('baud', 115200), ('pwm_limit', -1),
            ('mcu_timeout_ms', 300), ('command_timeout', .2), ('scan_timeout', .3),
            ('encoder_timeout', .3), ('tf_timeout', .5), ('tf_future_tolerance', .2),
            ('left_sign', 1.0), ('right_sign', 1.0), ('odom_frame', 'odom'),
            ('base_frame', 'base_link'), ('map_frame', 'map'), ('publish_tf', True),
            ('pose_covariance', [.01, .01, .05]), ('twist_covariance', [.01, .01, .05])]
        self.cfg = {p.name: p.value for p in self.declare_parameters('', defaults)}
        c = self.cfg
        validate_config(c)
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Physical motor bridge cannot use simulated time')
        # Integer cap is exactly representable by the MCU float parser.
        self.max_ticks = math.floor(c['max_wheel_speed'] * c['ticks_per_rev'] / (2*math.pi*c['wheel_radius']))
        self.safety = Safety(c['command_timeout'], c['scan_timeout'], c['encoder_timeout'])
        self.pose, self.ticks, self.encoder_ms = (0., 0., 0.), None, None
        self.odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_frame = ''
        self.last_scan_stamp = -math.inf
        self.serial, self.buffer = None, bytearray()
        self.next_connect = 0.
        self.create_subscription(Twist, '/cmd_vel_safe', self.command, 1)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_timer(.005, self.poll)
        self.create_timer(.05, self.send)

    def write(self, line):
        data = (line+'\n').encode('ascii')
        if self.serial.write(data) != len(data):
            raise serial.SerialException('short serial write')

    def configure(self):
        self.safety.reset()
        self.ticks = self.encoder_ms = None
        self.mcu_time = self.mcu_stamp = None
        self.session = secrets.randbelow(2147483646)+1
        c = self.cfg
        self.write('STOP')
        fields = (self.session, self.max_ticks, c['kp_left'], c['ki_left'], c['kp_right'],
                  c['ki_right'], c['pwm_limit'], c['max_accel_ticks'], c['mcu_timeout_ms'],
                  int(c['left_sign']), int(c['right_sign']))
        self.write('CFG,'+','.join(str(v) for v in fields))
        self.configured_at = time.monotonic()

    def disconnect(self):
        self.safety.reset()
        if self.serial is not None:
            try:
                self.write('STOP')
            except (serial.SerialException, OSError):
                pass
            self.serial.close()
        self.serial = None
        self.next_connect = time.monotonic()+1.

    def tf_ok(self):
        if not self.scan_frame:
            return False
        try:
            laser = self.tf_buffer.lookup_transform(self.cfg['base_frame'], self.scan_frame, Time())
            stamp = Time.from_msg(laser.header.stamp).nanoseconds
            age = (self.get_clock().now().nanoseconds-stamp)/1e9
            if stamp and not -self.cfg['tf_future_tolerance'] <= age <= self.cfg['tf_timeout']:
                return False
            tf = self.tf_buffer.lookup_transform(self.cfg['map_frame'], self.cfg['odom_frame'], Time())
            age = (self.get_clock().now().nanoseconds - Time.from_msg(tf.header.stamp).nanoseconds)/1e9
            return -self.cfg['tf_future_tolerance'] <= age <= self.cfg['tf_timeout']
        except TransformException:
            return False

    def scan(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        age = self.get_clock().now().nanoseconds/1e9-stamp
        valid = (bool(msg.header.frame_id) and 0 <= age <= self.cfg['scan_timeout']
                 and stamp > self.last_scan_stamp and math.isfinite(msg.range_min)
                 and math.isfinite(msg.range_max) and 0 < msg.range_min < msg.range_max
                 and math.isfinite(msg.angle_min) and math.isfinite(msg.angle_max)
                 and math.isfinite(msg.angle_increment) and msg.angle_increment != 0
                 and any(math.isfinite(r) and msg.range_min <= r <= msg.range_max for r in msg.ranges))
        if valid:
            self.scan_frame, self.last_scan_stamp = msg.header.frame_id, stamp
            self.safety.scan = time.monotonic()-age
        else:
            self.safety.scan = -math.inf
            self.safety.discard()

    def command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
        if (not all(math.isfinite(v) for v in values)
                or msg.linear.x < 0 or any(v != 0 for v in values[1:5])):
            self.safety.discard()
            return
        c = self.cfg
        scale = max(1., abs(msg.linear.x)/c['max_linear_speed'], abs(msg.angular.z)/c['max_angular_speed'])
        left, right = wheel_ticks(msg.linear.x/scale, msg.angular.z/scale, c['wheel_separation'],
                                  c['wheel_radius'], c['ticks_per_rev'], self.max_ticks)
        self.safety.command(left, right, time.monotonic(), self.tf_ok())

    def encoder(self, ms, left, right):
        now = time.monotonic()
        if self.mcu_stamp is None:
            self.mcu_time, self.mcu_stamp = now, ms
        else:
            elapsed = (ms-self.mcu_stamp) % (1 << 32)
            expected = self.mcu_time+elapsed/1000.
            if elapsed >= (1 << 31) or not -.1 <= now-expected <= self.cfg['encoder_timeout']:
                self.safety.discard()
                return
            self.mcu_time, self.mcu_stamp = expected, ms
        if not self.safety.encoder(ms, now):
            self.safety.discard()
            return
        if self.ticks is not None:
            dt = ((ms-self.encoder_ms) % (1 << 32))/1000.
            distances = [sign*ticks_to_metres(wrap_delta(old, new, 32), self.cfg['ticks_per_rev'], self.cfg['wheel_radius'])
                         for old, new, sign in zip(self.ticks, (left, right), (self.cfg['left_sign'], self.cfg['right_sign']))]
            if max(map(abs, distances)) > self.cfg['max_wheel_speed']*dt*1.5:
                self.safety.enc = -math.inf
                self.safety.discard()
                return
            self.pose = integrate(self.pose, *distances, self.cfg['wheel_separation'])
            self.publish(self.get_clock().now(), sum(distances)/2/dt,
                         (distances[1]-distances[0])/self.cfg['wheel_separation']/dt)
        self.ticks, self.encoder_ms = (left, right), ms

    def poll(self):
        try:
            if self.serial is None:
                if time.monotonic() < self.next_connect:
                    return
                self.serial = serial.Serial(self.cfg['port'], self.cfg['baud'], timeout=0,
                                            write_timeout=.02, exclusive=True)
                self.serial.reset_input_buffer()
                self.buffer.clear()
                self.configure()
            self.buffer.extend(self.serial.read(4096))
            if len(self.buffer) > 8192:
                raise serial.SerialException('serial frame overflow')
            while b'\n' in self.buffer:
                line, _, self.buffer = self.buffer.partition(b'\n')
                try:
                    frame = parse_frame(line.rstrip(b'\r'))
                except (ValueError, UnicodeError):
                    self.safety.discard()
                    continue
                if frame[0] == 'READY':
                    self.configure()
                elif frame[0] == 'ACK' and frame[1] == self.session:
                    self.safety.acknowledged = True
                elif frame[0] == 'ENC':
                    if frame[4] != self.session:
                        self.configure()
                    elif self.safety.acknowledged:
                        self.encoder(*frame[1:4])
            if not self.safety.acknowledged and time.monotonic()-self.configured_at > 1.:
                self.configure()
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(str(e))
            self.disconnect()

    def send(self):
        if self.serial is None:
            return
        velocity = self.safety.output(time.monotonic(), self.tf_ok())
        try:
            self.write('STOP' if velocity is None or velocity == (0., 0.) else
                       f'VEL,{self.session},{int(velocity[0])},{int(velocity[1])}')
        except (serial.SerialException, OSError):
            self.disconnect()


def main():
    rclpy.init()
    node = None
    try:
        node = MotorBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.disconnect()
            node.destroy_node()
        rclpy.try_shutdown()
