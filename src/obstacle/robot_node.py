#!/usr/bin/env python3
"""Nodo principal del Obstacle Challenge (WRO Future Engineers).

Seguimiento de pared con lazo IMU + vision de color. Maquina de estados:

  IDLE --(boton)--> CALIBRANDO --(4 s)--> SIGUIENDO <--> GIRANDO --(3 vueltas)--> FIN

La camara detecta el bloque mas cercano y elige la pared a seguir (rojo ->
derecha, verde -> izquierda). El lidar da la geometria de la pared y dispara
los giros de esquina. El IMU es el unico lazo que escribe al servo.
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

from config import Config, clamp, wrap180
from color_vision import LIBRE, ROJO, VERDE, detect_block
from control import (InnerLoop, Corner, LapCounter, window_median, delta_to_us,
                     estimate_wall, wall_follow_offset, pillar_nudge_deg,
                     side_for_color, detect_corner, start_turn, turn_complete,
                     turn_timed_out)

BUTTON_MIN_INTERVAL_S = 1.5


class State(Enum):
    IDLE = auto()
    CALIBRANDO = auto()
    SIGUIENDO = auto()
    GIRANDO = auto()
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

        self.wall_side = 'right'
        self.active_color = LIBRE
        self.color_candidate = LIBRE
        self.color_count = 0
        self.block = None

        self.front = self.dL = self.dR = None
        self.theta_ref = 0.0
        self.cal_samples = []
        self.cal_start_t = None
        self.last_imu_t = None
        self.last_scan_t = None
        self.last_button_t = None

        self.create_timer(self.cfg.WATCHDOG_S, self.watchdog_callback)
        self.get_logger().info('Listo. Pulsa el boton para arrancar.')

    # ------------------------------------------------------------------ #
    # utilidades
    # ------------------------------------------------------------------ #
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def driving(self):
        return self.state in (State.SIGUIENDO, State.GIRANDO)

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
        self.motor_pub.publish(Float32(data=float(max(0.0, min(100.0, pct)))))

    # ------------------------------------------------------------------ #
    # transiciones de estado
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
            self.stop('boton')

    def start_calibration(self):
        self.state = State.CALIBRANDO
        self.cal_samples = []
        self.cal_start_t = self.now()
        self.send_motor(0.0)
        self.send_servo(self.cfg.SERVO_CENTER_US)
        self.get_logger().info('Calibrando giroscopio, mantener el robot quieto...')

    def finish_calibration(self):
        self.inner.bias_deg_s = statistics.fmean(self.cal_samples)
        self.inner.reset(0.0)
        self.corner.reset()
        self.laps.reset()
        self.theta_ref = 0.0
        self.last_imu_t = self.now()
        self.state = State.SIGUIENDO
        self.beep(880, 0.1)
        self.get_logger().info(
            f'Bias gz = {self.inner.bias_deg_s:.3f} deg/s. Arrancando '
            f'(pared {self.wall_side}).')

    def finish(self):
        self.send_motor(0.0)
        self.send_servo(self.cfg.SERVO_CENTER_US)
        self.state = State.FIN
        self.get_logger().info(f'{self.cfg.MAX_LAPS} vueltas completadas.')

    def stop(self, reason):
        self.send_motor(0.0)
        self.send_servo(self.cfg.SERVO_CENTER_US)
        self.corner.turning = False
        self.state = State.IDLE
        self.get_logger().info(f'PARADA ({reason}).')

    # ------------------------------------------------------------------ #
    # camara: color -> pared a seguir
    # ------------------------------------------------------------------ #
    def camera_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'frame invalido: {exc}')
            return
        self.block = detect_block(frame, self.cfg)
        detected = LIBRE if self.block is None else self.block[0]

        # confirmar el color unos frames antes de fiarse (evita falsos beeps)
        if detected == self.color_candidate:
            self.color_count += 1
        else:
            self.color_candidate = detected
            self.color_count = 1
        if self.color_count < self.cfg.CONFIRM_FRAMES:
            return

        if detected != self.active_color and detected in (ROJO, VERDE):
            self.active_color = detected
            # solo se cambia de pared si aun no se esta girando
            if not self.corner.turning:
                self.wall_side = side_for_color(detected)
            name = 'VERDE -> izquierda' if detected == VERDE else 'ROJO -> derecha'
            self.beep(self.cfg.LAP_BEEP_FREQ, 0.1)
            self.get_logger().info(f'Color {name}.')
        elif detected == LIBRE:
            self.active_color = LIBRE

    # ------------------------------------------------------------------ #
    # lidar: geometria de pared y deteccion de esquina
    # ------------------------------------------------------------------ #
    def scan_callback(self, msg):
        self.last_scan_t = self.now()
        if not self.driving():
            return

        self.front = window_median(msg.ranges, self.cfg.LIDAR_FRONT_DEG, self.cfg)
        self.dL = window_median(msg.ranges, self.cfg.LIDAR_LEFT_DEG, self.cfg)
        self.dR = window_median(msg.ranges, self.cfg.LIDAR_RIGHT_DEG, self.cfg)

        if self.corner.turning:
            return

        theta_rel = wrap180(self.inner.theta_deg - self.corner.heading_base_deg)

        if detect_corner(self.front, self.dL, self.dR, theta_rel,
                         self.corner, self.now(), self.cfg):
            start_turn(self.corner, self.dL, self.dR, self.now(), self.cfg)
            self.theta_ref = 0.0
            lado = 'derecha' if self.corner.direction > 0 else 'izquierda'
            self.get_logger().info(f'Esquina: giro a la {lado}.')
            return

        # rayos de la pared seguida: lateral a 60, adelantado a 30
        if self.wall_side == 'right':
            side_ray = window_median(msg.ranges,
                                     self.cfg.LIDAR_FRONT_DEG - self.cfg.SIDE_RAY_DEG, self.cfg)
            fwd_ray = window_median(msg.ranges,
                                    self.cfg.LIDAR_FRONT_DEG - self.cfg.FORWARD_RAY_DEG, self.cfg)
        else:
            side_ray = window_median(msg.ranges,
                                     self.cfg.LIDAR_FRONT_DEG + self.cfg.SIDE_RAY_DEG, self.cfg)
            fwd_ray = window_median(msg.ranges,
                                    self.cfg.LIDAR_FRONT_DEG + self.cfg.FORWARD_RAY_DEG, self.cfg)

        wall = estimate_wall(side_ray, fwd_ray, self.wall_side, self.cfg)
        offset = wall_follow_offset(wall, self.wall_side, self.cfg)
        offset += pillar_nudge_deg(self.block, self.cfg)
        self.theta_ref = clamp(offset, -self.cfg.FOLLOW_MAX_DEG - self.cfg.PILLAR_MAX_DEG,
                               self.cfg.FOLLOW_MAX_DEG + self.cfg.PILLAR_MAX_DEG)

    # ------------------------------------------------------------------ #
    # imu: rumbo -> servo (unico lazo que manda al servo)
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
        if dt <= 0.0 or dt > self.cfg.IMU_TIMEOUT_S:
            return

        gz_deb = self.inner.integrate(gz_deg, dt, self.cfg)

        if self.laps.update(self.inner.theta_deg, self.cfg):
            self.get_logger().info(f'Vuelta {self.laps.laps}.')
            if self.laps.laps >= self.cfg.MAX_LAPS:
                self.beep(self.cfg.FINISH_BEEP_FREQ, 0.3, repeat=2)
                self.finish()
                return
            self.beep(self.cfg.LAP_BEEP_FREQ, 0.2)

        if self.corner.turning:
            if turn_timed_out(self.corner, t, self.cfg):
                self.stop('timeout de giro')
                return
            if turn_complete(self.corner, self.inner.theta_deg, gz_deb, self.cfg):
                self.corner.turning = False
                self.get_logger().info('Giro completo.')

        if self.corner.turning:
            ref = self.corner.heading_base_deg
        else:
            ref = self.corner.heading_base_deg + self.theta_ref

        delta = self.inner.delta(ref, gz_deb, self.cfg)
        self.send_servo(delta_to_us(delta, self.cfg))

        pct = self.cfg.MOTOR_TURN_PCT if self.corner.turning else self.cfg.MOTOR_CRUISE_PCT
        self.send_motor(pct)

    # ------------------------------------------------------------------ #
    # seguridad
    # ------------------------------------------------------------------ #
    def watchdog_callback(self):
        if not self.driving():
            return
        t = self.now()
        imu_lost = self.last_imu_t and t - self.last_imu_t > self.cfg.IMU_TIMEOUT_S
        scan_lost = self.last_scan_t and t - self.last_scan_t > self.cfg.LIDAR_TIMEOUT_S
        if imu_lost or scan_lost:
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
