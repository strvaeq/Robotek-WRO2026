#!/usr/bin/env python3
"""Nodo principal del Open Challenge (WRO Future Engineers).

Control en cascada lidar + IMU. Maquina de estados:

  IDLE --(boton)--> CALIBRANDO --(4 s)--> RECTO <--> GIRANDO --(3 vueltas)--> FIN

El lidar (scan_callback, ~10 Hz) fija el rumbo objetivo para centrar el robot.
El IMU (imu_callback, ~50 Hz) es el unico lazo que escribe al servo: integra
el giro, calcula el angulo de direccion y ejecuta los giros de esquina.
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

from config import Config, clamp, wrap180
from control import (InnerLoop, OuterLoop, Corner, LapCounter, window_median,
                     delta_to_us, detect_corner, start_turn, turn_complete,
                     turn_timed_out)

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

        self.create_timer(self.cfg.WATCHDOG_S, self.watchdog_callback)
        self.get_logger().info('Listo. Pulsa el boton para arrancar.')

    # ------------------------------------------------------------------ #
    # utilidades
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
        self.motor_pub.publish(Float32(data=float(max(0.0, min(100.0, pct)))))

    # ------------------------------------------------------------------ #
    # transiciones de estado
    # ------------------------------------------------------------------ #
    def button_callback(self, msg):
        if msg.state not in (1, 5):        # solo flancos de pulsacion
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
        self.outer.reset()
        self.corner.reset()
        self.laps.reset()
        self.theta_ref = 0.0
        self.last_imu_t = self.now()
        self.state = State.RECTO
        self.beep(880, 0.1)
        self.get_logger().info(
            f'Bias gz = {self.inner.bias_deg_s:.3f} deg/s. Arrancando.')

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
    # lazo interno (IMU): rumbo -> servo
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

        # conteo de vueltas por yaw integrado
        if self.laps.update(self.inner.theta_deg, self.cfg):
            self.get_logger().info(f'Vuelta {self.laps.laps}.')
            if self.laps.laps >= self.cfg.MAX_LAPS:
                self.beep(self.cfg.FINISH_BEEP_FREQ, 0.3, repeat=2)
                self.finish()
                return
            self.beep(self.cfg.LAP_BEEP_FREQ, 0.2)

        # fin (o timeout) del giro de esquina
        if self.corner.turning:
            if turn_timed_out(self.corner, t, self.cfg):
                self.stop('timeout de giro')
                return
            if turn_complete(self.corner, self.inner.theta_deg, gz_deb, self.cfg):
                self.corner.turning = False
                self.outer.reset()
                self.get_logger().info('Giro completo.')

        # rumbo objetivo: durante el giro solo cuenta el rumbo base
        if self.corner.turning:
            ref = self.corner.heading_base_deg
        else:
            ref = self.corner.heading_base_deg + self.theta_ref

        delta = self.inner.delta(ref, gz_deb, self.cfg)
        self.send_servo(delta_to_us(delta, self.cfg))

        pct = self.cfg.MOTOR_TURN_PCT if self.corner.turning else self.cfg.MOTOR_CRUISE_PCT
        self.send_motor(pct)

    # ------------------------------------------------------------------ #
    # lazo externo (lidar): centrado y deteccion de esquina
    # ------------------------------------------------------------------ #
    def scan_callback(self, msg):
        self.last_scan_t = self.now()
        if not self.driving():
            return

        self.dL = window_median(msg.ranges, self.cfg.LIDAR_LEFT_DEG, self.cfg)
        self.dR = window_median(msg.ranges, self.cfg.LIDAR_RIGHT_DEG, self.cfg)
        self.front = window_median(msg.ranges, self.cfg.LIDAR_FRONT_DEG, self.cfg)

        # mientras gira, el lazo externo esta congelado
        if self.corner.turning:
            return

        theta_rel = wrap180(self.inner.theta_deg - self.corner.heading_base_deg)

        if detect_corner(self.front, self.dL, self.dR, theta_rel,
                         self.corner, self.now(), self.cfg):
            start_turn(self.corner, self.dL, self.dR, self.now(), self.cfg)
            self.outer.reset()
            self.theta_ref = 0.0
            lado = 'derecha' if self.corner.direction > 0 else 'izquierda'
            self.get_logger().info(f'Esquina: giro a la {lado}.')
            return

        self.theta_ref, _ = self.outer.update(
            self.dL, self.dR, theta_rel, self.cfg.SCAN_DT, self.cfg)

    # ------------------------------------------------------------------ #
    # seguridad: frenar si se cae un sensor mientras conduce
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
