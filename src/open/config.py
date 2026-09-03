"""Parametros del control en cascada lidar + IMU.

Convencion de signos:
  - theta y delta (angulo de servo) positivos = giro a la derecha.
  - e_lat positivo = robot corrido a la izquierda (corregir hacia la derecha).
Si el robot corrige al lado equivocado, invertir GYRO_SIGN o SERVO_SIGN
antes de tocar las ganancias.
"""
from dataclasses import dataclass


@dataclass
class Config:
    # --- lazo interno (IMU -> angulo de servo) ---
    KP_IN: float = 1.6
    KD_IN: float = 0.28            # termino D = gyro directo, no derivada de theta
    DELTA_MAX_DEG: float = 29.0    # tope mecanico de la direccion

    # --- lazo externo 
    KP_OUT: float = 27.0
    KD_OUT: float = 4.5
    DERIV_ALPHA: float = 0.25      # filtro del termino derivativo
    THETA_REF_MAX_DEG: float = 9.0
    THETA_REL_MAX_DEG: float = 40.0  

    # --- ventanas del lidar
    LIDAR_LEFT_DEG: float = 240.0
    LIDAR_RIGHT_DEG: float = 120.0
    LIDAR_FRONT_DEG: float = 180.0
    LIDAR_WINDOW_HALF_DEG: float = 10.0
    LIDAR_SIDE_ANGLE_DEG: float = 60.0   
    LIDAR_MIN_M: float = 0.05
    LIDAR_MAX_M: float = 3.3

    # --- giroscopio ---
    GYRO_SIGN: float = -1.0
    GYRO_CAL_S: float = 4.0       

    # --- servo 
    SERVO_CENTER_US: float = 1500.0
    SERVO_MIN_US: float = 1000.0
    SERVO_MAX_US: float = 2050.0
    SERVO_GAIN_POS: float = 550.0 / 29.0
    SERVO_GAIN_NEG: float = 500.0 / 29.0
    SERVO_SIGN: float = -1.0
    SERVO_CMD_S: float = 0.05

    # --- motor (%) ---
    MOTOR_CRUISE_PCT: float = 75.0
    MOTOR_TURN_PCT: float = 42.0   # mas lento en los giros

    # --- esquinas ---
    CORNER_FRONT_M: float = 0.80         # pared de frente que dispara el giro
    CORNER_COOLDOWN_S: float = 1.5
    CORNER_MAX_THETA_REL_DEG: float = 10.0
    CORNER_EXIT_MARGIN_DEG: float = 4.0
    CORNER_EXIT_MAX_GZ_DEG_S: float = 20.0  # el giro termina cuando ya freno
    CORNER_TIMEOUT_S: float = 3.3
    SIDE_OPEN_M: float = 1.8             # lateral que ve mas que esto = abertura

    # --- vueltas ---
    MAX_LAPS: int = 3
    LAP_MARGIN_DEG: float = 10.0
    LAP_BEEP_FREQ: int = 523
    FINISH_BEEP_FREQ: int = 784

    # --- timing ---
    SCAN_DT: float = 0.1           # periodo nominal del lidar (~10 Hz)
    IMU_TIMEOUT_S: float = 0.15
    LIDAR_TIMEOUT_S: float = 0.3
    WATCHDOG_S: float = 0.05


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def wrap180(deg: float) -> float:
    """Normaliza un angulo a [-180, 180)."""
    return (deg + 180.0) % 360.0 - 180.0
