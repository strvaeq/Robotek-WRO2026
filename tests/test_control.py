"""Control helper tests. No rclpy and no camera are imported.

The packages live under src/<package>/<package>, so both source roots are put
on sys.path. color_vision imports cv2 at module level; the tests below never
call the camera functions, so a stub module is enough when cv2 is absent.
"""
import math
import os
import sys
import types
from dataclasses import dataclass

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for package in ('renkay_open', 'renkay_obstacle'):
    sys.path.insert(0, os.path.join(ROOT, 'src', package))

if 'cv2' not in sys.modules:
    try:
        import cv2  # noqa: F401
    except ImportError:
        sys.modules['cv2'] = types.ModuleType('cv2')

from renkay_open import config as open_config          # noqa: E402
from renkay_open import control as open_control        # noqa: E402
from renkay_obstacle import config as obstacle_config  # noqa: E402
from renkay_obstacle import control as obstacle_control  # noqa: E402

CASES = [(open_control, open_config.Config()),
         (obstacle_control, obstacle_config.Config())]


@dataclass
class FakeScan:
    ranges: list
    angle_min: float
    angle_increment: float


def wall_hits(distance_m, cfg, side='right'):
    """Both ray lengths a flat wall parallel to the robot gives at distance_m."""
    s = 1 if side == 'right' else -1
    side_range = distance_m / math.sin(math.radians(s * cfg.SIDE_RAY_DEG))
    forward_range = distance_m / math.sin(math.radians(s * cfg.FORWARD_RAY_DEG))
    return side_range, forward_range


# --------------------------------------------------------------------------- #
# Median filter
# --------------------------------------------------------------------------- #
def test_median_rejects_single_spike():
    cfg = open_config.Config()
    ranges = [1.0] * 360
    ranges[182] = 3.2
    assert open_control.window_median(ranges, 180.0, cfg) == pytest.approx(1.0)


def test_median_drops_nan_inf_and_out_of_range():
    cfg = open_config.Config()
    ranges = [1.0] * 360
    ranges[175] = float('nan')
    ranges[176] = float('inf')
    ranges[177] = 0.01          # below LIDAR_MIN_M
    ranges[178] = 9.0           # above LIDAR_MAX_M
    assert open_control.window_median(ranges, 180.0, cfg) == pytest.approx(1.0)


def test_median_returns_none_when_window_is_empty():
    cfg = open_config.Config()
    assert open_control.window_median([], 180.0, cfg) is None
    assert open_control.window_median([0.01] * 360, 180.0, cfg) is None


# --------------------------------------------------------------------------- #
# side_open
# --------------------------------------------------------------------------- #
def test_side_open_none_is_closed():
    for control, cfg in CASES:
        assert control.side_open(None, cfg) is False
        assert control.side_open(cfg.SIDE_OPEN_M - 0.1, cfg) is False
        assert control.side_open(cfg.SIDE_OPEN_M + 0.1, cfg) is True


# --------------------------------------------------------------------------- #
# Pillar returns in the wall estimate
# --------------------------------------------------------------------------- #
def test_wall_estimate_accepts_a_flat_wall():
    cfg = obstacle_config.Config()
    side, forward = wall_hits(0.20, cfg)
    est = obstacle_control.estimate_wall(side, forward, 'right', cfg)
    assert est.valid
    assert est.distance_m == pytest.approx(0.20, abs=1e-6)
    assert est.angle_deg == pytest.approx(0.0, abs=1e-6)


def test_pillar_return_is_rejected_by_angle():
    cfg = obstacle_config.Config()
    # one ray hits the pillar short: the fitted line turns beyond the limit
    assert not obstacle_control.estimate_wall(0.23, 0.15, 'right', cfg).valid


def test_pillar_return_is_rejected_by_distance_jump():
    cfg = obstacle_config.Config()
    side, forward = wall_hits(0.20, cfg)
    assert obstacle_control.estimate_wall(side, forward, 'right', cfg, 0.20).valid
    far_side, far_forward = wall_hits(0.60, cfg)   # +0.40 m in one frame
    assert not obstacle_control.estimate_wall(
        far_side, far_forward, 'right', cfg, 0.20).valid
    near_side, near_forward = wall_hits(0.21, cfg)
    assert obstacle_control.estimate_wall(
        near_side, near_forward, 'right', cfg, 0.20).valid


# --------------------------------------------------------------------------- #
# Lap counting
# --------------------------------------------------------------------------- #
def test_lap_counts_on_confirmed_corner_exits():
    cfg = open_config.Config()
    laps = open_control.LapCounter()
    results = []
    lap_totals = []
    for _ in range(8):
        results.append(laps.on_corner_exit(cfg))
        lap_totals.append(laps.laps)
    assert results == [False] * 3 + [True] + [False] * 3 + [True]
    assert lap_totals == [0, 0, 0, 1, 1, 1, 1, 2]


def test_lap_counter_reset_clears_corners():
    cfg = open_config.Config()
    laps = obstacle_control.LapCounter()
    for _ in range(6):
        laps.on_corner_exit(cfg)
    laps.reset()
    assert laps.laps == 0 and laps.corners == 0
    for _ in range(3):
        assert laps.on_corner_exit(cfg) is False


# --------------------------------------------------------------------------- #
# Signed reverse duty
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize('config_module', [open_config, obstacle_config])
def test_duty_keeps_reverse_magnitude(config_module):
    assert config_module.clamp_duty(-150.0) == -100.0
    assert config_module.clamp_duty(150.0) == 100.0
    assert config_module.clamp_duty(-25.0) == -25.0
    assert config_module.duty_to_pins(40.0) == (1, 0, 40.0)
    assert config_module.duty_to_pins(-25.0) == (0, 1, 25.0)
    assert config_module.duty_to_pins(0.0) == (1, 0, 0.0)


# --------------------------------------------------------------------------- #
# LiDAR index mapping
# --------------------------------------------------------------------------- #
def test_scan_angles_puts_front_at_180():
    amin, inc = open_control.scan_angles(360, -math.pi, math.radians(1.0))
    assert amin == pytest.approx(0.0, abs=1e-9)
    assert inc == pytest.approx(1.0)
    # no usable geometry (simulator): full turn from 0 deg
    assert open_control.scan_angles(360, float('nan'), 0.0) == (0.0, 1.0)
    assert open_control.scan_angles(0, 0.0, 0.0) == (0.0, 0.0)


def test_scan_range_uses_angle_min():
    cfg = obstacle_config.Config()
    ranges = [2.0] * 360
    ranges[85:96] = [0.5] * 11          # wall in front of the robot, index 90
    scan = FakeScan(ranges, math.radians(-90.0), math.radians(1.0))
    assert obstacle_control.scan_range(scan, 180.0, cfg) == pytest.approx(0.5)


def test_scan_range_falls_back_to_even_spacing():
    cfg = obstacle_config.Config()
    ranges = [2.0] * 360
    ranges[175:186] = [0.5] * 11        # simulator scan, 180 = front
    scan = FakeScan(ranges, float('nan'), 0.0)
    assert obstacle_control.scan_range(scan, 180.0, cfg) == pytest.approx(0.5)
