#!/usr/bin/env python3
"""Open Challenge main node (WRO Future Engineers).

Cascaded lidar + IMU control. State machine:

  IDLE --(button)--> CALIBRANDO --(4 s)--> RECTO <--> GIRANDO --(3 laps)--> FIN

The lidar (scan_callback, ~10 Hz) sets the target heading that centres the
robot. The IMU (imu_callback, ~50 Hz) is the only loop that writes the servo:
it integrates the rotation, computes the steering angle and runs the corner
turns.
"""
import math
import statistics
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Float32
from ros_robot_controller_msgs.msg import (SetAckerServoState, ButtonState,
                                           BuzzerState)

from renkay_open.config import Config, clamp, clamp_duty, wrap180
from renkay_open.control import (InnerLoop, OuterLoop, Corner, LapCounter,
                                 scan_range, delta_to_us, detect_corner,
                                 start_turn, turn_complete, turn_timed_out)

BUTTON_MIN_INTERVAL_S = 1.5


class State(Enum):
    IDLE = auto()
    CALIBRANDO = auto()
    RECTO = auto()
    GIRANDO = auto()
    FIN = auto()


class OpenChallenge(Node):
    def __init__(self):
        super().__init__('open_challenge')
        self.cfg = Config()

        self.servo_pub = self.create_publisher(
            SetAckerServoState, '/ros_robot_controller/acker_servo/set_state', 10)
        self.motor_pub = self.create_publisher(Float32, '/motor_vel', 10)
        self.buzzer_pub = self.create_publisher(
            BuzzerState, '/ros_robot_controller/set_buzzer', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.create_subscription(
            Imu, '/ros_robot_controller/imu_raw', self.imu_callback, 1)
        self.create_subscription(
            ButtonState, '/ros_robot_controller/button', self.button_callback, 10)

        self.state = State.IDLE
        self.inner = InnerLoop()
        self.outer = OuterLoop()
        self.corner = Corner()
        self.laps = LapCounter()

        self.dL = self.dR = self.front = None
        self.theta_ref = 0.0
        self.cal_samples = []
        self.cal_start_t = None
        self.last_imu_t = None
        self.last_scan_t = None
        self.last_button_t = None

        # watchdog latch: once a sensor is lost the robot stays stopped until
        # both sensors have been valid for FAULT_RECOVERY_S
        self.fault_latched = False
        self.fault_t = None
        self.recovery_since = None

        self.create_timer(self.cfg.WATCHDOG_S, self.watchdog_callback)
        self.get_logger().info('Ready. Press the button to start.')

    # ------------------------------------------------------------------ #
    # utilities
    # ------------------------------------------------------------------ #
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def driving(self):
        return self.state in (State.RECTO, State.GIRANDO)

    def beep(self, freq, dur=0.2, repeat=1):
        msg = BuzzerState()
        msg.freq = int(freq)
        msg.on_time = float(dur)
        msg.off_time = 0.1
        msg.repeat = int(repeat)
        self.buzzer_pub.publish(msg)

    def send_servo(self, us):
        msg = SetAckerServoState()
        msg.position = int(round(clamp(us, self.cfg.SERVO_MIN_US, self.cfg.SERVO_MAX_US)))
        msg.duration = self.cfg.SERVO_CMD_S
        self.servo_pub.publish(msg)

    def send_motor(self, pct):
        """Signed duty: negative reverses, magnitude is preserved."""
        self.motor_pub.publish(Float32(data=float(clamp_duty(pct))))

    # ------------------------------------------------------------------ #
    # state transitions
    # ------------------------------------------------------------------ #
    def button_callback(self, msg):
        if msg.state not in (1, 5):        # only press edges
            return
        t = self.now()
        if self.last_button_t is not None and \
                t - self.last_button_t < BUTTON_MIN_INTERVAL_S:
            return
        self.last_button_t = t
        if self.state in (State.IDLE, State.FIN):
            self.start_calibration()
        else:
            self.stop('button')

    def start_calibration(self):
        self.reset_fault()
        self.state = State.CALIBRANDO
        self.cal_samples = []
        self.cal_start_t = self.now()
        self.send_motor(0.0)
        self.send_servo(self.cfg.SERVO_CENTER_US)
        self.get_logger().info('Calibrating gyro, keep the robot still...')

    def finish_calibration(self):
        self.inner.bias_deg_s = statistics.fmean(self.cal_samples)
        self.inner.reset(0.0)
        self.outer.reset()
        self.corner.reset()
        self.laps.reset()
        self.theta_ref = 0.0
        self.last_imu_t = self.now()
        self.state = State.RECTO
        self.beep(880, 0.1)
        self.get_logger().info(
            f'gz bias = {self.inner.bias_deg_s:.3f} deg/s. Starting.')

    def finish(self):
        self.send_motor(0.0)
        self.send_servo(self.cfg.SERVO_CENTER_US)
        self.state = State.FIN
        self.get_logger().info(f'{self.cfg.MAX_LAPS} laps completed.')

    def stop(self, reason):
        self.send_motor(0.0)
        self.send_servo(self.cfg.SERVO_CENTER_US)
        self.corner.turning = False
        self.state = State.IDLE
        self.get_logger().info(f'STOP ({reason}).')

    # ------------------------------------------------------------------ #
    # inner loop (IMU): heading -> servo
    # ------------------------------------------------------------------ #
    def imu_callback(self, msg):
        gz = getattr(getattr(msg, 'angular_velocity', None), 'z', None)
        if gz is None or not math.isfinite(gz):
            return
        gz_deg = math.degrees(float(gz))
        t = self.now()

        if self.state == State.CALIBRANDO:
            self.cal_samples.append(gz_deg)
            if t - self.cal_start_t >= self.cfg.GYRO_CAL_S:
                self.finish_calibration()
            return

        if not self.driving():
            return

        dt = t - self.last_imu_t if self.last_imu_t is not None else 0.0
        self.last_imu_t = t
        if self.fault_latched:
            return                  # keep 0 duty until reset_fault() clears it
        if dt <= 0.0 or dt > self.cfg.IMU_TIMEOUT_S:
            return

        gz_deb = self.inner.integrate(gz_deg, dt, self.cfg)

        # end (or timeout) of the corner turn
        if self.corner.turning:
            if turn_timed_out(self.corner, t, self.cfg):
                self.stop('turn timeout')
                return
            if turn_complete(self.corner, self.inner.theta_deg, gz_deb, self.cfg):
                self.corner.turning = False
                self.outer.reset()
                self.get_logger().info('Turn complete.')
                # a lap is one corner exit after another, not integrated yaw
                if self.laps.on_corner_exit(self.cfg):
                    self.get_logger().info(f'Lap {self.laps.laps}.')
                    if self.laps.laps >= self.cfg.MAX_LAPS:
                        self.beep(self.cfg.FINISH_BEEP_FREQ, 0.3, repeat=2)
                        self.finish()
                        return
                    self.beep(self.cfg.LAP_BEEP_FREQ, 0.2)

        # target heading: during a turn only the base heading counts
        if self.corner.turning:
            ref = self.corner.heading_base_deg
        else:
            ref = self.corner.heading_base_deg + self.theta_ref

        delta = self.inner.delta(ref, gz_deb, self.cfg)
        self.send_servo(delta_to_us(delta, self.cfg))

        pct = self.cfg.MOTOR_TURN_PCT if self.corner.turning else self.cfg.MOTOR_CRUISE_PCT
        self.send_motor(pct)

    # ------------------------------------------------------------------ #
    # outer loop (lidar): centring and corner detection
    # ------------------------------------------------------------------ #
    def scan_callback(self, msg):
        self.last_scan_t = self.now()
        if not self.driving():
            return

        self.dL = scan_range(msg, self.cfg.LIDAR_LEFT_DEG, self.cfg)
        self.dR = scan_range(msg, self.cfg.LIDAR_RIGHT_DEG, self.cfg)
        self.front = scan_range(msg, self.cfg.LIDAR_FRONT_DEG, self.cfg)

        if self.fault_latched:
            return

        # while turning the outer loop is frozen
        if self.corner.turning:
            return

        theta_rel = wrap180(self.inner.theta_deg - self.corner.heading_base_deg)

        if detect_corner(self.front, self.dL, self.dR, theta_rel,
                         self.corner, self.now(), self.cfg):
            start_turn(self.corner, self.dL, self.dR, self.now(), self.cfg)
            self.outer.reset()
            self.theta_ref = 0.0
            side = 'right' if self.corner.direction > 0 else 'left'
            self.get_logger().info(f'Corner: turning {side}.')
            return

        self.theta_ref, _ = self.outer.update(
            self.dL, self.dR, theta_rel, self.cfg.SCAN_DT, self.cfg)

    # ------------------------------------------------------------------ #
    # safety: stop if a sensor drops while driving
    # ------------------------------------------------------------------ #
    def latch_fault(self):
        """Latch the stop: nothing but reset_fault() clears it."""
        if not self.fault_latched:
            self.fault_latched = True
            self.fault_t = self.now()
            self.recovery_since = None
            self.get_logger().error('Sensor lost: motor latched off.')
        self.send_motor(0.0)
        self.send_servo(self.cfg.SERVO_CENTER_US)

    def reset_fault(self):
        """Clear a latched fault and re-arm the recovery timer."""
        self.fault_latched = False
        self.fault_t = None
        self.recovery_since = None

    def watchdog_callback(self):
        if not self.driving():
            self.recovery_since = None
            return
        t = self.now()
        imu_lost = self.last_imu_t and t - self.last_imu_t > self.cfg.IMU_TIMEOUT_S
        scan_lost = self.last_scan_t and t - self.last_scan_t > self.cfg.LIDAR_TIMEOUT_S
        if imu_lost or scan_lost:
            self.latch_fault()
            return
        if not self.fault_latched:
            return
        # both sensors are valid again: require FAULT_RECOVERY_S of valid data
        if self.recovery_since is None:
            self.recovery_since = t
        if t - self.recovery_since >= self.cfg.FAULT_RECOVERY_S:
            self.get_logger().info('Sensors back: fault cleared.')
            self.reset_fault()
            return
        self.send_motor(0.0)
        self.send_servo(self.cfg.SERVO_CENTER_US)


def main(args=None):
    rclpy.init(args=args)
    node = OpenChallenge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_motor(0.0)
        node.send_servo(node.cfg.SERVO_CENTER_US)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
