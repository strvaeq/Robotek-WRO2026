"""Obstacle Challenge control logic.

The robot follows a wall at a fixed distance with the IMU loop. The camera
picks which wall to follow from the colour of the nearest block (red -> right,
green -> left) and, when the block is close, adds a small heading to drive
around it. Every corner adds +-90 to the base heading.
"""
import math
from dataclasses import dataclass
from typing import Optional, Sequence

from renkay_obstacle.config import Config, clamp
from renkay_obstacle.color_vision import ROJO, VERDE


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
    """Median of a fan of rays around center_deg (removes noise).

    Without angle metadata the indices are spread evenly over 360 deg, which is
    what the simulator scan expects.
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


def side_sign(side: str) -> int:
    return 1 if side == 'right' else -1


def side_for_color(color: int) -> Optional[str]:
    """Red -> follow the right wall; green -> the left one."""
    if color == ROJO:
        return 'right'
    if color == VERDE:
        return 'left'
    return None


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
        gz = cfg.GYRO_SIGN * (gz_deg_s - self.bias_deg_s)
        if dt > 0.0:
            self.theta_deg += gz * dt
        return gz

    def delta(self, theta_ref_deg: float, gz_deg_s: float, cfg: Config) -> float:
        err = theta_ref_deg - self.theta_deg
        d = cfg.KP_IN * err - cfg.KD_IN * gz_deg_s
        return clamp(d, -cfg.DELTA_MAX_DEG, cfg.DELTA_MAX_DEG)


def delta_to_us(delta_deg: float, cfg: Config) -> float:
    delta_deg = clamp(delta_deg, -cfg.DELTA_MAX_DEG, cfg.DELTA_MAX_DEG) * cfg.SERVO_SIGN
    gain = cfg.SERVO_GAIN_POS if delta_deg >= 0.0 else cfg.SERVO_GAIN_NEG
    us = cfg.SERVO_CENTER_US + delta_deg * gain
    return clamp(us, cfg.SERVO_MIN_US, cfg.SERVO_MAX_US)


# --------------------------------------------------------------------------- #
# Wall following
# --------------------------------------------------------------------------- #
@dataclass
class WallEstimate:
    valid: bool = False
    distance_m: float = 0.0
    angle_deg: float = 0.0


def estimate_wall(side_range, forward_range, side, cfg,
                  prev_distance_m: Optional[float] = None) -> WallEstimate:
    """Fit a line to two wall hits (side ray at 60 deg, forward ray at 30) in
    robot coordinates. Works even if the side ray is not exactly at 90 deg.
    Returns the perpendicular distance and the wall angle relative to the
    driving axis.

    A pillar hit only lands on one ray and produces a sharp angle or a distant
    point, so results outside WALL_MAX_ANGLE_DEG or jumping more than
    WALL_JUMP_MAX_M from the previous distance are rejected.
    """
    if side_range is None or forward_range is None:
        return WallEstimate()
    if not (cfg.LIDAR_MIN_M < side_range < cfg.LIDAR_MAX_M and
            cfg.LIDAR_MIN_M < forward_range < cfg.LIDAR_MAX_M):
        return WallEstimate()
    s = side_sign(side)
    a1 = math.radians(s * cfg.SIDE_RAY_DEG)
    a2 = math.radians(s * cfg.FORWARD_RAY_DEG)
    p1 = (side_range * math.cos(a1), side_range * math.sin(a1))
    p2 = (forward_range * math.cos(a2), forward_range * math.sin(a2))
    vx, vy = p2[0] - p1[0], p2[1] - p1[1]
    span = math.hypot(vx, vy)
    if span < 0.03:
        return WallEstimate()
    if vx < 0.0:                       # point forward -> 0 deg = parallel
        vx, vy = -vx, -vy
    angle = math.degrees(math.atan2(vy, vx))
    if abs(angle) > cfg.WALL_MAX_ANGLE_DEG:
        return WallEstimate()
    dist = abs(vx * (0.0 - p1[1]) - vy * (0.0 - p1[0])) / span   # point-line to origin
    if not (cfg.LIDAR_MIN_M < dist < cfg.WALL_MAX_DIST_M):
        return WallEstimate()
    if prev_distance_m is not None and \
            abs(dist - prev_distance_m) > cfg.WALL_JUMP_MAX_M:
        return WallEstimate()
    return WallEstimate(True, dist, angle)


def wall_follow_offset(est: WallEstimate, side: str, cfg: Config) -> float:
    """Target (relative) heading that leaves the wall at TARGET_M.

    Two terms: align the chassis with the wall (angle) and correct the distance
    with an atan, which does not grow without bound for a large error.
    """
    if not est.valid:
        return 0.0
    err = est.distance_m - cfg.TARGET_M
    corr = side_sign(side) * math.degrees(math.atan(cfg.LATERAL_GAIN * err))
    return clamp(est.angle_deg + corr, -cfg.FOLLOW_MAX_DEG, cfg.FOLLOW_MAX_DEG)


def pillar_nudge_deg(block, cfg: Config) -> float:
    """Extra heading to drive around the block while it is close.

    Moves away from the side it appears on: a block on the right of the image
    (cx_norm > 0) turns the robot left, and the other way round.
    """
    if block is None:
        return 0.0
    _color, cx_norm, base_norm, _area = block
    if base_norm < cfg.PILLAR_NEAR_BASE:      # still far: no need to avoid yet
        return 0.0
    return clamp(-cx_norm * cfg.PILLAR_GAIN_DEG, -cfg.PILLAR_MAX_DEG, cfg.PILLAR_MAX_DEG)


# --------------------------------------------------------------------------- #
# Corners
# --------------------------------------------------------------------------- #
@dataclass
class Corner:
    turning: bool = False
    heading_base_deg: float = 0.0
    direction: int = 0
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
    """Add +-90 to the base heading. The direction is fixed on the first corner
    and kept for the rest of the run (the track has one driving direction)."""
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
        """Count a confirmed corner exit; True when the lap total increased."""
        self.corners += 1
        laps = self.corners // cfg.CORNERS_PER_LAP
        if laps > self.laps:
            self.laps = laps
            return True
        return False
