"""Logica de control del Obstacle Challenge. 

El robot sigue una pared a distancia fija con el lazo IMU. La camara decide
que pared seguir segun el color del bloque mas cercano (rojo -> derecha,
verde -> izquierda) y, cuando el bloque esta cerca, se anade un pequeno rumbo
para rodearlo. En las esquinas se suman +-90 al rumbo base.
"""
import math
from dataclasses import dataclass
from typing import Optional, Sequence

from config import Config, clamp
from color_vision import ROJO, VERDE


def window_median(ranges: Sequence[float], center_deg: float,
                  cfg: Config) -> Optional[float]:
    """Mediana de un abanico de rayos alrededor de center_deg (filtra ruido)."""
    n = len(ranges)
    if n == 0:
        return None
    half = max(1, int(round(cfg.LIDAR_WINDOW_HALF_DEG / 360.0 * n)))
    center = int(round(center_deg / 360.0 * n)) % n
    vals = []
    for i in range(-half, half + 1):
        d = ranges[(center + i) % n]
        if math.isfinite(d) and cfg.LIDAR_MIN_M < d < cfg.LIDAR_MAX_M:
            vals.append(d)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def side_sign(side: str) -> int:
    return 1 if side == 'right' else -1


def side_for_color(color: int) -> Optional[str]:
    """Rojo -> seguir la pared derecha; verde -> la izquierda."""
    if color == ROJO:
        return 'right'
    if color == VERDE:
        return 'left'
    return None


# --------------------------------------------------------------------------- #
# Lazo interno: rumbo (IMU) -> angulo de servo
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
# Seguimiento de pared
# --------------------------------------------------------------------------- #
@dataclass
class WallEstimate:
    valid: bool = False
    distance_m: float = 0.0
    angle_deg: float = 0.0


def estimate_wall(side_range, forward_range, side, cfg) -> WallEstimate:
    """Ajusta una recta a dos impactos del muro (rayo lateral a 60 grados y
    rayo adelantado a 30) en coordenadas del robot. Funciona aunque el rayo
    lateral no este exactamente a 90. Devuelve la distancia perpendicular y el
    angulo de la pared respecto al eje de avance.
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
    if vx < 0.0:                       # orientar hacia adelante -> 0 grados = paralelo
        vx, vy = -vx, -vy
    angle = math.degrees(math.atan2(vy, vx))
    if abs(angle) > cfg.WALL_MAX_ANGLE_DEG:
        return WallEstimate()
    dist = abs(vx * (0.0 - p1[1]) - vy * (0.0 - p1[0])) / span   # punto-recta al origen
    if not (cfg.LIDAR_MIN_M < dist < cfg.WALL_MAX_DIST_M):
        return WallEstimate()
    return WallEstimate(True, dist, angle)


def wall_follow_offset(est: WallEstimate, side: str, cfg: Config) -> float:
    """Rumbo objetivo (relativo) para dejar la pared a TARGET_M.

    Suma dos terminos: alinear el chasis con la pared (angulo) y corregir la
    distancia con un atan, que no crece sin limite cuando el error es grande.
    """
    if not est.valid:
        return 0.0
    err = est.distance_m - cfg.TARGET_M
    corr = side_sign(side) * math.degrees(math.atan(cfg.LATERAL_GAIN * err))
    return clamp(est.angle_deg + corr, -cfg.FOLLOW_MAX_DEG, cfg.FOLLOW_MAX_DEG)


def pillar_nudge_deg(block, cfg: Config) -> float:
    """Rumbo extra para rodear el bloque mientras esta cerca.

    Se aleja del lado donde aparece: si el bloque cae a la derecha de la
    imagen (cx_norm > 0) se gira a la izquierda, y viceversa.
    """
    if block is None:
        return 0.0
    _color, cx_norm, base_norm, _area = block
    if base_norm < cfg.PILLAR_NEAR_BASE:      # todavia lejos: no hace falta esquivar
        return 0.0
    return clamp(-cx_norm * cfg.PILLAR_GAIN_DEG, -cfg.PILLAR_MAX_DEG, cfg.PILLAR_MAX_DEG)


# --------------------------------------------------------------------------- #
# Esquinas
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
    if d is None:
        return True
    return d > cfg.SIDE_OPEN_M


def detect_corner(front, dL, dR, theta_rel_deg, corner, now, cfg) -> bool:
    """Esquina = pared de frente cerca Y un lateral abierto."""
    if corner.turning or front is None or front >= cfg.CORNER_FRONT_M:
        return False
    if corner.last_turn_t is not None and \
            now - corner.last_turn_t < cfg.CORNER_COOLDOWN_S:
        return False
    if abs(theta_rel_deg) > cfg.CORNER_MAX_THETA_REL_DEG:
        return False
    return side_open(dL, cfg) or side_open(dR, cfg)


def start_turn(corner, dL, dR, now, cfg) -> None:
    """Suma +-90 al rumbo base. El sentido se fija en la primera esquina y se
    mantiene toda la ronda (la pista tiene un unico sentido de circulacion)."""
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
    """True cuando theta llego al nuevo rumbo Y la rotacion ya freno."""
    if abs(theta_deg - corner.heading_base_deg) >= cfg.CORNER_EXIT_MARGIN_DEG:
        return False
    return abs(gz_deg_s) < cfg.CORNER_EXIT_MAX_GZ_DEG_S


def turn_timed_out(corner, now, cfg) -> bool:
    return (corner.turning and corner.start_t is not None and
            now - corner.start_t > cfg.CORNER_TIMEOUT_S)


# --------------------------------------------------------------------------- #
# Vueltas (por yaw integrado: 4 esquinas de 90 = 360)
# --------------------------------------------------------------------------- #
@dataclass
class LapCounter:
    laps: int = 0

    def reset(self) -> None:
        self.laps = 0

    def update(self, theta_deg: float, cfg: Config) -> bool:
        laps = int((abs(theta_deg) + cfg.LAP_MARGIN_DEG) // 360.0)
        if laps > self.laps:
            self.laps = laps
            return True
        return False
