"""Parametros del Obstacle Challenge: seguimiento de pared + vision de color.

Convencion de signos:
  - theta y delta (angulo de servo) positivos = giro a la derecha.
  - wall_side: "right" sigue la pared derecha, "left" la izquierda.
El color elige que pared seguir: rojo -> derecha, verde -> izquierda.
"""
from dataclasses import dataclass


@dataclass
class Config:
    # --- lazo interno (IMU -> angulo de servo) ---
    KP_IN: float = 1.8
    KD_IN: float = 0.22            
    DELTA_MAX_DEG: float = 26.0

    # --- servo ---
    SERVO_CENTER_US: float = 1500.0
    SERVO_MIN_US: float = 1000.0
    SERVO_MAX_US: float = 2050.0
    SERVO_GAIN_POS: float = 550.0 / 30.0
    SERVO_GAIN_NEG: float = 500.0 / 30.0
    SERVO_SIGN: float = -1.0
    SERVO_CMD_S: float = 0.06

    # --- giroscopio ---
    GYRO_SIGN: float = -1.0
    GYRO_CAL_S: float = 4.0

    # --- lidar  ---
    LIDAR_FRONT_DEG: float = 180.0
    LIDAR_RIGHT_DEG: float = 120.0
    LIDAR_LEFT_DEG: float = 240.0
    LIDAR_WINDOW_HALF_DEG: float = 10.0
    LIDAR_MIN_M: float = 0.05
    LIDAR_MAX_M: float = 3.3

    # --- seguimiento de pared ---
    TARGET_M: float = 0.18         # distancia objetivo a la pared seguida
    SIDE_RAY_DEG: float = 60.0     # rayo lateral (a 60 del frente)
    FORWARD_RAY_DEG: float = 30.0  # rayo adelantado (a 30 del frente)
    WALL_MAX_ANGLE_DEG: float = 55.0
    WALL_MAX_DIST_M: float = 0.90
    LATERAL_GAIN: float = 1.5      # ganancia del termino de distancia
    FOLLOW_MAX_DEG: float = 7.0    # tope del rumbo pedido por el seguidor

    # --- esquiva de bloques (color) ---
    PILLAR_NEAR_BASE: float = 0.5  # base_norm sobre esto = bloque cerca, ya esquivar
    PILLAR_GAIN_DEG: float = 14.0
    PILLAR_MAX_DEG: float = 12.0
    CONFIRM_FRAMES: int = 3        # frames seguidos para confirmar un color

    # --- deteccion de color (HSV) ---
    RED_H1_LOW: int = 0
    RED_H1_HIGH: int = 13
    RED_H2_LOW: int = 169
    RED_H2_HIGH: int = 180
    RED_S_MIN: int = 80
    RED_V_MIN: int = 50
    GREEN_H_LOW: int = 53
    GREEN_H_HIGH: int = 89
    GREEN_S_MIN: int = 25
    GREEN_V_MIN: int = 25
    COLOR_MIN_AREA_PX: int = 650
    COLOR_MIN_FILL: float = 0.42
    COLOR_MAX_WIDTH_HEIGHT: float = 2.6
    COLOR_MORPH_KERNEL: int = 7
    COLOR_MORPH_ITERS: int = 2
    CAMERA_TOPIC: str = '/usb_cam/image_raw'

    # --- esquinas ---
    CORNER_FRONT_M: float = 0.65
    CORNER_COOLDOWN_S: float = 1.5
    CORNER_MAX_THETA_REL_DEG: float = 15.0
    CORNER_EXIT_MARGIN_DEG: float = 4.0
    CORNER_EXIT_MAX_GZ_DEG_S: float = 20.0
    CORNER_TIMEOUT_S: float = 3.3
    SIDE_OPEN_M: float = 1.2

    # --- motor (%) ---
    MOTOR_CRUISE_PCT: float = 35.0   # mas lento que el Open: hay que esquivar
    MOTOR_TURN_PCT: float = 35.0

    # --- vueltas ---
    MAX_LAPS: int = 3
    LAP_MARGIN_DEG: float = 10.0
    LAP_BEEP_FREQ: int = 523
    FINISH_BEEP_FREQ: int = 784

    # --- timing ---
    SCAN_DT: float = 0.1
    IMU_TIMEOUT_S: float = 0.20
    LIDAR_TIMEOUT_S: float = 0.35
    WATCHDOG_S: float = 0.05


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def wrap180(deg: float) -> float:
    """Normaliza un angulo a [-180, 180)."""
    return (deg + 180.0) % 360.0 - 180.0
