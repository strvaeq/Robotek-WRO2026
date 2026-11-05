"""Cascaded lidar + IMU control parameters.

Sign convention:
  - positive theta and delta (servo angle) = turn right.
  - positive e_lat = robot offset to the left (correct to the right).
If the robot corrects to the wrong side, invert GYRO_SIGN or SERVO_SIGN
before touching the gains.
"""
from dataclasses import dataclass


@dataclass
class Config:
    # --- inner loop (IMU -> servo angle) ---
    KP_IN: float = 1.6
    KD_IN: float = 0.28            # D term = raw gyro, not the derivative of theta
    DELTA_MAX_DEG: float = 30.0    # mechanical steering limit

    # --- outer loop
    KP_OUT: float = 27.0
    KD_OUT: float = 4.5
    DERIV_ALPHA: float = 0.25      # derivative term filter
    THETA_REF_MAX_DEG: float = 9.0
    THETA_REL_MAX_DEG: float = 40.0

    # --- lidar windows
    LIDAR_LEFT_DEG: float = 240.0
    LIDAR_RIGHT_DEG: float = 120.0
    LIDAR_FRONT_DEG: float = 180.0
    LIDAR_WINDOW_HALF_DEG: float = 10.0
    LIDAR_SIDE_ANGLE_DEG: float = 60.0
    LIDAR_MIN_M: float = 0.05
    LIDAR_MAX_M: float = 3.3

    # --- gyroscope ---
    GYRO_SIGN: float = -1.0
    GYRO_CAL_S: float = 4.0

    # --- servo
    SERVO_CENTER_US: float = 1500.0
    SERVO_MIN_US: float = 1000.0
    SERVO_MAX_US: float = 2050.0
    SERVO_GAIN_POS: float = 550.0 / 30.0
    SERVO_GAIN_NEG: float = 500.0 / 30.0
    SERVO_SIGN: float = -1.0
    SERVO_CMD_S: float = 0.05

    # --- motor (%) ---
    MOTOR_CRUISE_PCT: float = 75.0
    MOTOR_TURN_PCT: float = 42.0   # slower through the corners

    # --- corners ---
    CORNER_FRONT_M: float = 0.80         # front wall that triggers the turn
    CORNER_COOLDOWN_S: float = 1.5
    CORNER_MAX_THETA_REL_DEG: float = 10.0
    CORNER_EXIT_MARGIN_DEG: float = 4.0
    CORNER_EXIT_MAX_GZ_DEG_S: float = 20.0  # the turn ends once rotation stopped
    CORNER_TIMEOUT_S: float = 3.3
    SIDE_OPEN_M: float = 1.8             # a flank seeing more than this is an opening

    # --- laps ---
    MAX_LAPS: int = 3
    CORNERS_PER_LAP: int = 4
    LAP_MARGIN_DEG: float = 10.0
    LAP_BEEP_FREQ: int = 523
    FINISH_BEEP_FREQ: int = 784

    # --- timing ---
    SCAN_DT: float = 0.1           # nominal lidar period (~10 Hz)
    IMU_TIMEOUT_S: float = 0.15
    LIDAR_TIMEOUT_S: float = 0.3
    WATCHDOG_S: float = 0.05
    FAULT_RECOVERY_S: float = 1.0  # valid IMU + lidar needed before clearing a fault


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def clamp_duty(pct: float) -> float:
    """Signed motor duty in percent, -100 (full reverse) .. +100 (full forward)."""
    return clamp(pct, -100.0, 100.0)


def duty_to_pins(duty: float) -> tuple:
    """TB6612 direction pins (AIN1, AIN2) and PWM magnitude for a signed duty."""
    d = clamp_duty(duty)
    return (1, 0, d) if d >= 0.0 else (0, 1, -d)


def wrap180(deg: float) -> float:
    """Normalise an angle to [-180, 180)."""
    return (deg + 180.0) % 360.0 - 180.0
