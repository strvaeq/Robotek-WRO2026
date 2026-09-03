"""Logica de control del Open Challenge. 

Dos lazos:
  - externo (lidar): mira las paredes laterales y calcula un rumbo objetivo
    para mantener el robot centrado en el pasillo.
  - interno (IMU): integra el giroscopio para saber el rumbo actual y produce
    el angulo de direccion que persigue ese objetivo.

El lidar decide a donde ir y el IMU decide como sostener el rumbo. En cada
esquina se suman +-90 al rumbo base.
"""
import math
from dataclasses import dataclass
from typing import Optional, Sequence

from config import Config, clamp


def window_median(ranges: Sequence[float], center_deg: float,
                  cfg: Config) -> Optional[float]:
    """Mediana de un abanico de rayos alrededor de center_deg.

    Descarta NaN/inf y lecturas fuera de rango.
    """
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
        """Integra la velocidad angular sobre
        theta. Devuelve el gz corregido para usarlo como termino derivativo."""
        gz = cfg.GYRO_SIGN * (gz_deg_s - self.bias_deg_s)
        if dt > 0.0:
            self.theta_deg += gz * dt
        return gz

    def delta(self, theta_ref_deg: float, gz_deg_s: float, cfg: Config) -> float:
        """PD sobre el error de rumbo, con clamp al tope mecanico.
        """
        err = theta_ref_deg - self.theta_deg
        d = cfg.KP_IN * err - cfg.KD_IN * gz_deg_s
        return clamp(d, -cfg.DELTA_MAX_DEG, cfg.DELTA_MAX_DEG)


# --------------------------------------------------------------------------- #
# Lazo externo: posicion lateral (lidar) -> rumbo objetivo
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
        """Rumbo objetivo (grados) a partir del error de centrado.
        """
        if dL is None or dR is None or abs(theta_rel_deg) > cfg.THETA_REL_MAX_DEG:
            return self.theta_ref_deg, False
        # Los haces apuntan a 60 grados, no a 90: se proyectan sobre la normal
        # de cada pared para no mezclar error de rumbo con error lateral.
        ang = math.radians(cfg.LIDAR_SIDE_ANGLE_DEG)
        rel = math.radians(theta_rel_deg)
        left = dL * math.sin(ang - rel)
        right = dR * math.sin(ang + rel)
        err = (right - left) / 2.0          # + = corrido a la izquierda
        if self.have_prev and dt > 0.0:
            raw = (err - self.prev_err) / dt
            self.deriv = cfg.DERIV_ALPHA * raw + (1.0 - cfg.DERIV_ALPHA) * self.deriv
        self.prev_err = err
        self.have_prev = True
        ref = cfg.KP_OUT * err + cfg.KD_OUT * self.deriv
        self.theta_ref_deg = clamp(ref, -cfg.THETA_REF_MAX_DEG, cfg.THETA_REF_MAX_DEG)
        return self.theta_ref_deg, True


# --------------------------------------------------------------------------- #
# Esquinas
# --------------------------------------------------------------------------- #
@dataclass
class Corner:
    turning: bool = False
    heading_base_deg: float = 0.0
    direction: int = 0                 # +1 derecha, -1 izquierda
    last_turn_t: Optional[float] = None
    start_t: Optional[float] = None

    def reset(self) -> None:
        self.turning = False
        self.heading_base_deg = 0.0
        self.direction = 0
        self.last_turn_t = None
        self.start_t = None


def side_open(d: Optional[float], cfg: Config) -> bool:
    """True si ese lateral ve una abertura (esquina) en vez de la pared."""
    if d is None:
        return True
    return d > cfg.SIDE_OPEN_M


def detect_corner(front, dL, dR, theta_rel_deg, corner, now, cfg) -> bool:
    """Esquina = pared de frente cerca Y un lateral abierto.
    """
    if corner.turning or front is None or front >= cfg.CORNER_FRONT_M:
        return False
    if corner.last_turn_t is not None and \
            now - corner.last_turn_t < cfg.CORNER_COOLDOWN_S:
        return False
    if abs(theta_rel_deg) > cfg.CORNER_MAX_THETA_REL_DEG:
        return False
    return side_open(dL, cfg) or side_open(dR, cfg)


def start_turn(corner, dL, dR, now, cfg) -> None:
    """Suma +-90 al rumbo base. El sentido se decide en la primera esquina y
    se mantiene toda la ronda. La pista tiene un unico sentido de circulacion.
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
    """True cuando theta llego al nuevo rumbo Y la rotacion ya freno.
    """
    if abs(theta_deg - corner.heading_base_deg) >= cfg.CORNER_EXIT_MARGIN_DEG:
        return False
    return abs(gz_deg_s) < cfg.CORNER_EXIT_MAX_GZ_DEG_S


def turn_timed_out(corner, now, cfg) -> bool:
    """True si un giro activo excedio su tiempo maximo."""
    return (corner.turning and corner.start_t is not None and
            now - corner.start_t > cfg.CORNER_TIMEOUT_S)


# --------------------------------------------------------------------------- #
# Vueltas
# --------------------------------------------------------------------------- #
@dataclass
class LapCounter:
    laps: int = 0

    def reset(self) -> None:
        self.laps = 0

    def update(self, theta_deg: float, cfg: Config) -> bool:
        """Cuenta por yaw integrado: 4 esquinas de 90 = 360 = una vuelta."""
        laps = int((abs(theta_deg) + cfg.LAP_MARGIN_DEG) // 360.0)
        if laps > self.laps:
            self.laps = laps
            return True
        return False


# --------------------------------------------------------------------------- #
# Servo
# --------------------------------------------------------------------------- #
def delta_to_us(delta_deg: float, cfg: Config) -> float:
    """Convierte el angulo de direccion (grados) a microsegundos de PWM.
    """
    delta_deg = clamp(delta_deg, -cfg.DELTA_MAX_DEG, cfg.DELTA_MAX_DEG) * cfg.SERVO_SIGN
    gain = cfg.SERVO_GAIN_POS if delta_deg >= 0.0 else cfg.SERVO_GAIN_NEG
    us = cfg.SERVO_CENTER_US + delta_deg * gain
    return clamp(us, cfg.SERVO_MIN_US, cfg.SERVO_MAX_US)
