"""Open Challenge control logic.

Two loops:
  - outer (lidar): looks at the side walls and computes a target heading that
    keeps the robot centred in the corridor.
  - inner (IMU): integrates the gyro to know the current heading and produces
    the steering angle that chases that target.

The lidar decides where to go and the IMU decides how to hold the heading.
Every corner adds +-90 to the base heading.
"""
import math
from dataclasses import dataclass
from typing import Optional, Sequence

from renkay_open.config import Config, clamp


def scan_angles(count: int, angle_min_rad: float,
                angle_increment_rad: float) -> tuple:
    """Scan geometry in the config convention (0..360 deg, 180 = front).

    ROS reports angles from the robot x axis; the config files count from the
    rear so the front sits at 180 deg. Returns (angle_min_deg, increment_deg).
    """
    if count <= 0:
        return 0.0, 0.0
    inc = math.degrees(angle_increment_rad)
    amin = math.degrees(angle_min_rad)
    if not (math.isfinite(inc) and math.isfinite(amin)) or inc <= 0.0:
        # fallback: the simulator sends no usable geometry, assume a full turn
        # starting at 0 deg with 180 deg facing forward.
        return 0.0, 360.0 / count
    return amin + 180.0, inc


def window_median(ranges: Sequence[float], center_deg: float, cfg: Config,
                  angle_min_deg: float = 0.0,
                  angle_increment_deg: Optional[float] = None) -> Optional[float]:
    """Median of a fan of rays around center_deg.

    Drops NaN/inf and readings outside range. Without angle metadata the
    indices are spread evenly over 360 deg, which is what the simulator scan
    expects.
    """
    n = len(ranges)
    if n == 0:
        return None
    inc = angle_increment_deg if angle_increment_deg else 360.0 / n
    half = max(1, int(round(cfg.LIDAR_WINDOW_HALF_DEG / inc)))
    center = int(round((center_deg - angle_min_deg) / inc)) % n
    vals = []
    for i in range(-half, half + 1):
        d = ranges[(center + i) % n]
        if math.isfinite(d) and cfg.LIDAR_MIN_M < d < cfg.LIDAR_MAX_M:
            vals.append(d)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def scan_range(msg, center_deg: float, cfg: Config) -> Optional[float]:
    """window_median on a LaserScan message, using its own angle metadata."""
    amin, inc = scan_angles(len(msg.ranges), msg.angle_min, msg.angle_increment)
    return window_median(msg.ranges, center_deg, cfg, amin, inc)


# --------------------------------------------------------------------------- #
# Inner loop: heading (IMU) -> servo angle
# --------------------------------------------------------------------------- #
@dataclass
class InnerLoop:
    theta_deg: float = 0.0
    bias_deg_s: float = 0.0

    def reset(self, theta0_deg: float = 0.0) -> None:
        self.theta_deg = theta0_deg

    def integrate(self, gz_deg_s: float, dt: float, cfg: Config) -> float:
        """Integrate angular rate into theta. Returns the corrected gz for the
        derivative term."""
        gz = cfg.GYRO_SIGN * (gz_deg_s - self.bias_deg_s)
        if dt > 0.0:
            self.theta_deg += gz * dt
        return gz

    def delta(self, theta_ref_deg: float, gz_deg_s: float, cfg: Config) -> float:
        """PD on the heading error, clamped to the mechanical limit."""
        err = theta_ref_deg - self.theta_deg
        d = cfg.KP_IN * err - cfg.KD_IN * gz_deg_s
        return clamp(d, -cfg.DELTA_MAX_DEG, cfg.DELTA_MAX_DEG)


# --------------------------------------------------------------------------- #
# Outer loop: lateral position (lidar) -> target heading
# --------------------------------------------------------------------------- #
@dataclass
class OuterLoop:
    prev_err: float = 0.0
    have_prev: bool = False
    deriv: float = 0.0
    theta_ref_deg: float = 0.0

    def reset(self) -> None:
        self.prev_err = 0.0
        self.have_prev = False
        self.deriv = 0.0
        self.theta_ref_deg = 0.0

    def update(self, dL: Optional[float], dR: Optional[float],
               theta_rel_deg: float, dt: float, cfg: Config) -> tuple:
        """Target heading (deg) from the centring error."""
        if dL is None or dR is None or abs(theta_rel_deg) > cfg.THETA_REL_MAX_DEG:
            return self.theta_ref_deg, False
        # The beams point at 60 deg, not 90: project them on each wall normal
        # so heading error and lateral error are not mixed.
        ang = math.radians(cfg.LIDAR_SIDE_ANGLE_DEG)
        rel = math.radians(theta_rel_deg)
        left = dL * math.sin(ang - rel)
        right = dR * math.sin(ang + rel)
        err = (right - left) / 2.0          # + = offset to the left
        if self.have_prev and dt > 0.0:
            raw = (err - self.prev_err) / dt
            self.deriv = cfg.DERIV_ALPHA * raw + (1.0 - cfg.DERIV_ALPHA) * self.deriv
        self.prev_err = err
        self.have_prev = True
        ref = cfg.KP_OUT * err + cfg.KD_OUT * self.deriv
        self.theta_ref_deg = clamp(ref, -cfg.THETA_REF_MAX_DEG, cfg.THETA_REF_MAX_DEG)
        return self.theta_ref_deg, True


# --------------------------------------------------------------------------- #
# Corners
# --------------------------------------------------------------------------- #
@dataclass
class Corner:
    turning: bool = False
    heading_base_deg: float = 0.0
    direction: int = 0                 # +1 right, -1 left
    last_turn_t: Optional[float] = None
    start_t: Optional[float] = None

    def reset(self) -> None:
        self.turning = False
        self.heading_base_deg = 0.0
        self.direction = 0
        self.last_turn_t = None
        self.start_t = None


def side_open(d: Optional[float], cfg: Config) -> bool:
    """True if that flank sees an opening (corner) instead of the wall.

    No reading is not an opening: a missing beam must not start a turn.
    """
    if d is None:
        return False
    return d > cfg.SIDE_OPEN_M


def detect_corner(front, dL, dR, theta_rel_deg, corner, now, cfg) -> bool:
    """Corner = close front wall AND an open flank."""
    if corner.turning or front is None or front >= cfg.CORNER_FRONT_M:
        return False
    if corner.last_turn_t is not None and \
            now - corner.last_turn_t < cfg.CORNER_COOLDOWN_S:
        return False
    if abs(theta_rel_deg) > cfg.CORNER_MAX_THETA_REL_DEG:
        return False
    return side_open(dL, cfg) or side_open(dR, cfg)


def start_turn(corner, dL, dR, now, cfg) -> None:
    """Add +-90 to the base heading. The direction is fixed on the first
    corner and kept for the rest of the run: the track has one direction.
    """
    if corner.direction == 0:
        left_open = side_open(dL, cfg)
        right_open = side_open(dR, cfg)
        if right_open and not left_open:
            corner.direction = 1
        elif left_open and not right_open:
            corner.direction = -1
        else:
            l = dL if dL is not None else float('inf')
            r = dR if dR is not None else float('inf')
            corner.direction = 1 if r >= l else -1
    corner.heading_base_deg += corner.direction * 90.0
    corner.turning = True
    corner.last_turn_t = now
    corner.start_t = now


def turn_complete(corner, theta_deg, gz_deg_s, cfg) -> bool:
    """True once theta reached the new heading AND rotation has stopped."""
    if abs(theta_deg - corner.heading_base_deg) >= cfg.CORNER_EXIT_MARGIN_DEG:
        return False
    return abs(gz_deg_s) < cfg.CORNER_EXIT_MAX_GZ_DEG_S


def turn_timed_out(corner, now, cfg) -> bool:
    """True if an active turn exceeded its maximum time."""
    return (corner.turning and corner.start_t is not None and
            now - corner.start_t > cfg.CORNER_TIMEOUT_S)


# --------------------------------------------------------------------------- #
# Laps (counted from confirmed corner exits)
# --------------------------------------------------------------------------- #
@dataclass
class LapCounter:
    laps: int = 0
    corners: int = 0

    def reset(self) -> None:
        self.laps = 0
        self.corners = 0

    def on_corner_exit(self, cfg: Config) -> bool:
        """Count a confirmed corner exit; True when the lap total increased.

        Counting corners instead of |integrated yaw| keeps a yaw bias from
        adding or dropping a lap.
        """
        self.corners += 1
        laps = self.corners // cfg.CORNERS_PER_LAP
        if laps > self.laps:
            self.laps = laps
            return True
        return False


# --------------------------------------------------------------------------- #
# Servo
# --------------------------------------------------------------------------- #
def delta_to_us(delta_deg: float, cfg: Config) -> float:
    """Convert the steering angle (deg) to PWM microseconds."""
    delta_deg = clamp(delta_deg, -cfg.DELTA_MAX_DEG, cfg.DELTA_MAX_DEG) * cfg.SERVO_SIGN
    gain = cfg.SERVO_GAIN_POS if delta_deg >= 0.0 else cfg.SERVO_GAIN_NEG
    us = cfg.SERVO_CENTER_US + delta_deg * gain
    return clamp(us, cfg.SERVO_MIN_US, cfg.SERVO_MAX_US)
