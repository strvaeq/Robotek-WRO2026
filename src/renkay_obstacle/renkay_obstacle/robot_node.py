#!/usr/bin/env python3
"""Obstacle Challenge main node (WRO Future Engineers).

Wall following with the IMU loop + colour vision. State machine:

  IDLE --(button)--> CALIBRANDO --(4 s)--> SIGUIENDO <--> GIRANDO
        --(3 laps)--> PARKING --> FIN

The camera detects the nearest block and picks the wall to follow (red ->
right, green -> left). The lidar gives the wall geometry and triggers the
corner turns. The IMU is the only loop that writes the servo.
"""
import math
import statistics
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan, Imu, Image
from std_msgs.msg import Float32
from ros_robot_controller_msgs.msg import (SetAckerServoState, ButtonState,
                                           BuzzerState)

from renkay_obstacle.config import Config, clamp, clamp_duty, wrap180
from renkay_obstacle.color_vision import LIBRE, ROJO, VERDE, detect_block
from renkay_obstacle.control import (InnerLoop, Corner, LapCounter, scan_range,
                                     delta_to_us, estimate_wall,
                                     wall_follow_offset, pillar_nudge_deg,
                                     side_for_color, detect_corner, start_turn,
                                     turn_complete, turn_timed_out)
from renkay_obstacle.parking import ParallelPark, ParkState

BUTTON_MIN_INTERVAL_S = 1.5


class State(Enum):
    IDLE = auto()
    CALIBRANDO = auto()
    SIGUIENDO = auto()
    GIRANDO = auto()
    PARKING = auto()
    FIN = auto()


class ObstacleChallenge(Node):
    def __init__(self):
        super().__init__('obstacle_challenge')
        self.cfg = Config()
        self.bridge = CvBridge()

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
        self.create_subscription(
            Image, self.cfg.CAMERA_TOPIC, self.camera_callback, 1)

        self.state = State.IDLE
        self.inner = InnerLoop()
        self.corner = Corner()
        self.laps = LapCounter()
        self.parker = ParallelPark(self.cfg)

        self.wall_side = 'right'
        self.active_color = LIBRE
        self.color_candidate = LIBRE
        self.color_count = 0
        self.block = None
        self.block_time = None

        self.front = self.dL = self.dR = self.rear = None
        self.wall_dist = None
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
        return self.state in (State.SIGUIENDO, State.GIRANDO, State.PARKING)

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

    def current_block(self):
        """Last camera block, or None once it is older than BLOCK_TIMEOUT_S."""
        if self.block is None or self.block_time is None:
            return None
        if self.now() - self.block_time > self.cfg.BLOCK_TIMEOUT_S:
            return None
        return self.block

    # ------------------------------------------------------------------ #
    # state transitions
    # ------------------------------------------------------------------ #
    def button_callback(self, msg):
        if msg.state not in (1, 5):
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
        self.corner.reset()
        self.laps.reset()
        self.theta_ref = 0.0
        self.wall_dist = None
        self.last_imu_t = self.now()
        self.state = State.SIGUIENDO
        self.beep(880, 0.1)
        self.get_logger().info(
            f'gz bias = {self.inner.bias_deg_s:.3f} deg/s. Starting '
            f'(wall {self.wall_side}).')

    def start_parking(self):
        self.parker.reset()
        self.state = State.PARKING
        self.send_motor(0.0)
        self.send_servo(self.cfg.SERVO_CENTER_US)
        self.get_logger().info('Parking manoeuvre started.')

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

    def reanchor_heading(self):
        """Re-zero the heading reference after the followed wall flips side."""
        self.corner.heading_base_deg = self.inner.theta_deg
        self.theta_ref = 0.0
        self.wall_dist = None    # forget the distance history of the old wall

    # ------------------------------------------------------------------ #
    # camera: colour -> wall to follow
    # ------------------------------------------------------------------ #
    def camera_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'invalid frame: {exc}')
            return
        found = detect_block(frame, self.cfg)
        if found is not None:
            self.block = found
            self.block_time = self.now()
        block = self.current_block()
        detected = LIBRE if block is None else block[0]

        # confirm the colour over a few frames before trusting it
        if detected == self.color_candidate:
            self.color_count += 1
        else:
            self.color_candidate = detected
            self.color_count = 1
        if self.color_count < self.cfg.CONFIRM_FRAMES:
            return

        if detected != self.active_color and detected in (ROJO, VERDE):
            self.active_color = detected
            # only switch walls while following or cornering, never while parking
            if not self.corner.turning and \
                    self.state in (State.SIGUIENDO, State.GIRANDO):
                side = side_for_color(detected)
                if side != self.wall_side:
                    self.wall_side = side
                    self.reanchor_heading()
            name = 'GREEN -> left' if detected == VERDE else 'RED -> right'
            self.beep(self.cfg.LAP_BEEP_FREQ, 0.1)
            self.get_logger().info(f'Colour {name}.')
        elif detected == LIBRE:
            self.active_color = LIBRE

    # ------------------------------------------------------------------ #
    # lidar: wall geometry and corner detection
    # ------------------------------------------------------------------ #
    def scan_callback(self, msg):
        self.last_scan_t = self.now()
        if not self.driving():
            return

        self.front = scan_range(msg, self.cfg.LIDAR_FRONT_DEG, self.cfg)
        self.dL = scan_range(msg, self.cfg.LIDAR_LEFT_DEG, self.cfg)
        self.dR = scan_range(msg, self.cfg.LIDAR_RIGHT_DEG, self.cfg)

        if self.state == State.PARKING:
            self.rear = scan_range(msg, self.cfg.LIDAR_REAR_DEG, self.cfg)
            return

        if self.fault_latched:
            return

        if self.corner.turning:
            return

        theta_rel = wrap180(self.inner.theta_deg - self.corner.heading_base_deg)

        if detect_corner(self.front, self.dL, self.dR, theta_rel,
                         self.corner, self.now(), self.cfg):
            start_turn(self.corner, self.dL, self.dR, self.now(), self.cfg)
            self.theta_ref = 0.0
            side = 'right' if self.corner.direction > 0 else 'left'
            self.get_logger().info(f'Corner: turning {side}.')
            return

        # rays of the followed wall: side at 60, forward at 30
        if self.wall_side == 'right':
            side_ray = scan_range(msg,
                                  self.cfg.LIDAR_FRONT_DEG - self.cfg.SIDE_RAY_DEG, self.cfg)
            fwd_ray = scan_range(msg,
                                 self.cfg.LIDAR_FRONT_DEG - self.cfg.FORWARD_RAY_DEG, self.cfg)
        else:
            side_ray = scan_range(msg,
                                  self.cfg.LIDAR_FRONT_DEG + self.cfg.SIDE_RAY_DEG, self.cfg)
            fwd_ray = scan_range(msg,
                                 self.cfg.LIDAR_FRONT_DEG + self.cfg.FORWARD_RAY_DEG, self.cfg)

        wall = estimate_wall(side_ray, fwd_ray, self.wall_side, self.cfg,
                             self.wall_dist)
        if wall.valid:
            self.wall_dist = wall.distance_m
        offset = wall_follow_offset(wall, self.wall_side, self.cfg)
        offset += pillar_nudge_deg(self.current_block(), self.cfg)
        self.theta_ref = clamp(offset, -self.cfg.FOLLOW_MAX_DEG - self.cfg.PILLAR_MAX_DEG,
                               self.cfg.FOLLOW_MAX_DEG + self.cfg.PILLAR_MAX_DEG)

    # ------------------------------------------------------------------ #
    # imu: heading -> servo (the only loop that commands the servo)
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

        if self.state == State.PARKING:
            side = self.dR if self.wall_side == 'right' else self.dL
            yaw_rel = wrap180(self.inner.theta_deg - self.corner.heading_base_deg)
            steer, duty, park_state = self.parker.update(
                side, self.front, self.rear, yaw_rel, dt)
            self.send_servo(delta_to_us(steer, self.cfg))
            self.send_motor(duty)
            if park_state in (ParkState.DONE, ParkState.ABORT):
                self.get_logger().info(f'Parking {park_state.name.lower()}.')
                self.finish()
            return

        if self.corner.turning:
            if turn_timed_out(self.corner, t, self.cfg):
                self.stop('turn timeout')
                return
            if turn_complete(self.corner, self.inner.theta_deg, gz_deb, self.cfg):
                self.corner.turning = False
                self.get_logger().info('Turn complete.')
                # a lap is one corner exit after another, not integrated yaw
                if self.laps.on_corner_exit(self.cfg):
                    self.get_logger().info(f'Lap {self.laps.laps}.')
                    if self.laps.laps >= self.cfg.MAX_LAPS:
                        self.beep(self.cfg.FINISH_BEEP_FREQ, 0.3, repeat=2)
                        if self.cfg.PARKING_ENABLED:
                            self.start_parking()
                        else:
                            self.finish()
                        return
                    self.beep(self.cfg.LAP_BEEP_FREQ, 0.2)

        if self.corner.turning:
            ref = self.corner.heading_base_deg
        else:
            ref = self.corner.heading_base_deg + self.theta_ref

        delta = self.inner.delta(ref, gz_deb, self.cfg)
        self.send_servo(delta_to_us(delta, self.cfg))

        pct = self.cfg.MOTOR_TURN_PCT if self.corner.turning else self.cfg.MOTOR_CRUISE_PCT
        self.send_motor(pct)

    # ------------------------------------------------------------------ #
    # safety
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
    node = ObstacleChallenge()
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
