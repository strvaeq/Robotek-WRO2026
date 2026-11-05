"""Obstacle Challenge parameters: wall following + colour vision.

Sign convention:
  - positive theta and delta (servo angle) = turn right.
  - wall_side: "right" follows the right wall, "left" the left one.
The colour picks the wall to follow: red -> right, green -> left.
"""
from dataclasses import dataclass


@dataclass
class Config:
    # --- inner loop (IMU -> servo angle) ---
    KP_IN: float = 1.8
    KD_IN: float = 0.22
    DELTA_MAX_DEG: float = 30.0

    # --- servo ---
    SERVO_CENTER_US: float = 1500.0
    SERVO_MIN_US: float = 1000.0
    SERVO_MAX_US: float = 2050.0
    SERVO_GAIN_POS: float = 550.0 / 30.0
    SERVO_GAIN_NEG: float = 500.0 / 30.0
    SERVO_SIGN: float = -1.0
    SERVO_CMD_S: float = 0.06

    # --- gyroscope ---
    GYRO_SIGN: float = -1.0
    GYRO_CAL_S: float = 4.0

    # --- lidar
    LIDAR_FRONT_DEG: float = 180.0
    LIDAR_RIGHT_DEG: float = 120.0
    LIDAR_LEFT_DEG: float = 240.0
    LIDAR_REAR_DEG: float = 0.0        # used only by the parking manoeuvre
    LIDAR_WINDOW_HALF_DEG: float = 10.0
    LIDAR_MIN_M: float = 0.05
    LIDAR_MAX_M: float = 3.3

    # --- wall following ---
    TARGET_M: float = 0.18         # target distance to the followed wall
    SIDE_RAY_DEG: float = 60.0     # side ray (60 deg off the front)
    FORWARD_RAY_DEG: float = 30.0  # forward ray (30 deg off the front)
    WALL_MAX_ANGLE_DEG: float = 55.0
    WALL_MAX_DIST_M: float = 0.90
    WALL_JUMP_MAX_M: float = 0.35  # reject a one-frame jump (pillar return)
    LATERAL_GAIN: float = 1.5      # gain of the distance term
    FOLLOW_MAX_DEG: float = 7.0    # cap on the heading asked by the follower

    # --- block avoidance (colour) ---
    BLOCK_TIMEOUT_S: float = 1.5   # a stale camera block is ignored
    PILLAR_NEAR_BASE: float = 0.5  # base_norm above this = block near, avoid now
    PILLAR_GAIN_DEG: float = 14.0
    PILLAR_MAX_DEG: float = 12.0
    CONFIRM_FRAMES: int = 3        # consecutive frames to confirm a colour

    # --- colour detection (HSV) ---
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

    # --- corners ---
    CORNER_FRONT_M: float = 0.65
    CORNER_COOLDOWN_S: float = 1.5
    CORNER_MAX_THETA_REL_DEG: float = 15.0
    CORNER_EXIT_MARGIN_DEG: float = 4.0
    CORNER_EXIT_MAX_GZ_DEG_S: float = 20.0
    CORNER_TIMEOUT_S: float = 3.3
    SIDE_OPEN_M: float = 1.2

    # --- motor (%) ---
    MOTOR_CRUISE_PCT: float = 35.0   # slower than the Open: blocks to avoid
    MOTOR_TURN_PCT: float = 35.0

    # --- laps ---
    MAX_LAPS: int = 3
    CORNERS_PER_LAP: int = 4
    LAP_MARGIN_DEG: float = 10.0
    LAP_BEEP_FREQ: int = 523
    FINISH_BEEP_FREQ: int = 784

    # --- parking (after the last lap) ---
    PARKING_ENABLED: bool = True
    PARK_SLOT_LEN_M: float = 0.60
    PARK_TARGET_GAP_M: float = 0.03
    PARK_REVERSE_PCT: float = 25.0
    PARK_ABORT_FRONT_M: float = 0.10
    PARK_MAX_TIME_S: float = 8.0
    # Steering sign that swings the rear towards the bay; flip after the first field run.
    PARK_STEER_SIGN: float = -1.0

    # --- timing ---
    SCAN_DT: float = 0.1
    IMU_TIMEOUT_S: float = 0.20
    LIDAR_TIMEOUT_S: float = 0.35
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
