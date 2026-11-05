"""Parallel parking state machine (pure Python, no ROS imports).

Geometry: drive forward past the slot until the rear clearance is one slot
length, align with the wall, then reverse on full steering lock until the
heading has swept ARC_YAW_DEG, then straighten while reversing until the
lateral gap is PARK_TARGET_GAP_M.

States: APPROACH -> ALIGN -> REVERSE_ARC -> STRAIGHTEN -> DONE, or ABORT.

update() returns (steer_deg, duty_pct, state); a negative duty means reverse.

shortcut: the arc target angle and the stop distances come from geometry on
paper, not from a run of the car; calibrate them on the field before trusting
the manoeuvre outside the simulator.
"""
from enum import Enum, auto

from renkay_obstacle.config import Config, clamp


class ParkState(Enum):
    APPROACH = auto()
    ALIGN = auto()
    REVERSE_ARC = auto()
    STRAIGHTEN = auto()
    DONE = auto()
    ABORT = auto()


APPROACH_PCT = 12.0          # slow creep while looking for the slot
ALIGN_PCT = 8.0              # creep used to straighten before reversing
ALIGN_YAW_TOL_DEG = 2.0      # aligned enough to start the arc
ARC_YAW_DEG = 45.0           # heading swept by the reverse arc
STOP_REAR_M = 0.05           # never reverse closer than this to the rear wall


class ParallelPark:
    """Bounded parallel parking manoeuvre for the Obstacle Challenge."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.state = ParkState.APPROACH
        self.elapsed_s = 0.0

    def reset(self) -> None:
        self.state = ParkState.APPROACH
        self.elapsed_s = 0.0

    def update(self, side_distance_m, front_distance_m, rear_distance_m,
               yaw_deg, dt) -> tuple:
        """Advance the manoeuvre; returns (steer_deg, duty_pct, state)."""
        if self.state in (ParkState.DONE, ParkState.ABORT):
            return 0.0, 0.0, self.state

        if dt > 0.0:
            self.elapsed_s += dt
        if self.elapsed_s > self.cfg.PARK_MAX_TIME_S or \
                (front_distance_m is not None and
                 front_distance_m < self.cfg.PARK_ABORT_FRONT_M):
            self.state = ParkState.ABORT
            return 0.0, 0.0, self.state

        if self.state == ParkState.APPROACH:
            # the slot is behind once the rear clearance is one slot length
            if rear_distance_m is not None and \
                    rear_distance_m <= self.cfg.PARK_SLOT_LEN_M:
                self.state = ParkState.ALIGN
            return 0.0, APPROACH_PCT, self.state

        if self.state == ParkState.ALIGN:
            if abs(yaw_deg) <= ALIGN_YAW_TOL_DEG:
                self.state = ParkState.REVERSE_ARC
            else:
                steer = clamp(-yaw_deg, -self.cfg.DELTA_MAX_DEG, self.cfg.DELTA_MAX_DEG)
                return steer, ALIGN_PCT, self.state

        if self.state == ParkState.REVERSE_ARC:
            if rear_distance_m is not None and rear_distance_m <= STOP_REAR_M:
                self.state = ParkState.DONE
                return 0.0, 0.0, self.state
            if yaw_deg >= ARC_YAW_DEG:
                self.state = ParkState.STRAIGHTEN
            else:
                # full lock: PARK_STEER_SIGN selects the side the slot is on
                return (self.cfg.PARK_STEER_SIGN * self.cfg.DELTA_MAX_DEG,
                        -self.cfg.PARK_REVERSE_PCT, self.state)

        if self.state == ParkState.STRAIGHTEN:
            gap_reached = (side_distance_m is not None and
                           side_distance_m <= self.cfg.PARK_TARGET_GAP_M)
            rear_reached = (rear_distance_m is not None and
                            rear_distance_m <= STOP_REAR_M)
            if gap_reached or rear_reached:
                self.state = ParkState.DONE
                return 0.0, 0.0, self.state
            return 0.0, -self.cfg.PARK_REVERSE_PCT, self.state

        return 0.0, 0.0, self.state
