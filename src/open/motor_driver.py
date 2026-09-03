#!/usr/bin/env python3
"""Driver ROS 2 del TB6612FNG (canal A) para Raspberry Pi 5.

Recibe std_msgs/Float32 en /motor_vel (0..100 %) y mueve el motor hacia
adelante. Al recibir 0 frena brevemente en reversa para quitar la inercia y
despues deja el puente en estado seguro. Un watchdog apaga el motor si dejan
de llegar comandos.
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import lgpio
except ImportError:
    lgpio = None


class Tb6612Motor(Node):
    def __init__(self):
        super().__init__('tb6612fng_motor')

        self.declare_parameter('gpiochip', 4)
        self.declare_parameter('pwm_a', 12)
        self.declare_parameter('ain1', 5)
        self.declare_parameter('ain2', 6)
        self.declare_parameter('stby', 13)
        self.declare_parameter('pwm_hz', 1000)
        self.declare_parameter('watchdog_s', 0.30)

        self.chip = int(self.get_parameter('gpiochip').value)
        self.pins = {name: int(self.get_parameter(name).value)
                     for name in ('pwm_a', 'ain1', 'ain2', 'stby')}
        self.hz = int(self.get_parameter('pwm_hz').value)
        self.watchdog_s = float(self.get_parameter('watchdog_s').value)

        self.gpio = None
        self.last_command = time.monotonic()
        self.last_pct = 0.0
        self.braking_until = None

        if lgpio is None:
            self.get_logger().error('Falta lgpio; motor deshabilitado.')
        else:
            self.gpio = lgpio.gpiochip_open(self.chip)
            for pin in self.pins.values():
                lgpio.gpio_claim_output(self.gpio, pin, 0)
            self.stop_motor()
            self.get_logger().info('TB6612FNG listo.')

        self.create_subscription(Float32, '/motor_vel', self.on_command, 10)
        self.create_timer(0.05, self.on_watchdog)

    def stop_motor(self):
        if self.gpio is None:
            return
        lgpio.tx_pwm(self.gpio, self.pins['pwm_a'], self.hz, 0)
        for name in ('ain1', 'ain2', 'stby'):
            lgpio.gpio_write(self.gpio, self.pins[name], 0)
        self.last_pct = 0.0
        self.braking_until = None

    def drive(self, pct):
        lgpio.gpio_write(self.gpio, self.pins['ain1'], 1)
        lgpio.gpio_write(self.gpio, self.pins['ain2'], 0)
        lgpio.gpio_write(self.gpio, self.pins['stby'], 1)
        if pct >= 100.0:
            lgpio.gpio_write(self.gpio, self.pins['pwm_a'], 1)
        else:
            lgpio.tx_pwm(self.gpio, self.pins['pwm_a'], self.hz, pct)
        self.last_pct = pct

    def brake(self):
        """Invierte el motor un instante para frenar la inercia."""
        pct = self.last_pct
        lgpio.gpio_write(self.gpio, self.pins['ain1'], 0)
        lgpio.gpio_write(self.gpio, self.pins['ain2'], 1)
        lgpio.gpio_write(self.gpio, self.pins['stby'], 1)
        lgpio.tx_pwm(self.gpio, self.pins['pwm_a'], self.hz, pct)
        self.braking_until = time.monotonic() + 0.12
        self.last_pct = 0.0

    def on_command(self, msg):
        try:
            pct = float(msg.data)
        except (TypeError, ValueError):
            pct = 0.0
        pct = max(0.0, min(100.0, pct))
        self.last_command = time.monotonic()
        if self.gpio is None:
            return
        if pct <= 0.0:
            if self.braking_until is None and self.last_pct > 0.0:
                self.brake()
            elif self.braking_until is None:
                self.stop_motor()
            return
        self.braking_until = None
        self.drive(pct)

    def on_watchdog(self):
        if self.braking_until is not None:
            if time.monotonic() >= self.braking_until:
                self.stop_motor()
            return
        if time.monotonic() - self.last_command > self.watchdog_s:
            self.stop_motor()

    def destroy_node(self):
        self.stop_motor()
        if self.gpio is not None:
            lgpio.gpiochip_close(self.gpio)
            self.gpio = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Tb6612Motor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
