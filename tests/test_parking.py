"""Parallel parking state machine tests. Pure Python, no ROS imports."""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for package in ('renkay_open', 'renkay_obstacle'):
    sys.path.insert(0, os.path.join(ROOT, 'src', package))

from renkay_obstacle.config import Config          # noqa: E402
from renkay_obstacle.parking import ParallelPark, ParkState  # noqa: E402


def test_state_sequence_approach_to_done():
    cfg = Config()
    park = ParallelPark(cfg)

    # 1. approaching: rear clearance still larger than the slot
    steer, duty, state = park.update(0.50, 1.0, 2.0, 0.0, 0.1)
    assert state is ParkState.APPROACH
    assert steer == 0.0 and duty > 0.0

    # 2. rear clearance is one slot length: align
    _steer, _duty, state = park.update(0.50, 1.0, 0.60, 0.0, 0.1)
    assert state is ParkState.ALIGN

    # 3. still rotated: creep forward and steer back toward zero
    steer, duty, state = park.update(0.50, 1.0, 0.60, 10.0, 0.1)
    assert state is ParkState.ALIGN
    assert steer < 0.0 and duty > 0.0

    # 4. aligned: reverse on full lock
    steer, duty, state = park.update(0.50, 1.0, 0.60, 0.0, 0.1)
    assert state is ParkState.REVERSE_ARC
    assert steer == pytest.approx(-cfg.DELTA_MAX_DEG)
    assert duty == pytest.approx(-cfg.PARK_REVERSE_PCT)

    # 5. arc swept: straighten while still reversing
    steer, duty, state = park.update(0.50, 1.0, 0.60, 50.0, 0.1)
    assert state is ParkState.STRAIGHTEN
    assert steer == 0.0 and duty == pytest.approx(-cfg.PARK_REVERSE_PCT)

    # 6. lateral gap reached: done, stopped
    steer, duty, state = park.update(
        cfg.PARK_TARGET_GAP_M - 0.01, 1.0, 0.60, 50.0, 0.1)
    assert state is ParkState.DONE
    assert steer == 0.0 and duty == 0.0

    # 7. done is terminal
    assert park.update(0.50, 1.0, 0.60, 50.0, 0.1)[2] is ParkState.DONE


def test_abort_on_front_clearance():
    cfg = Config()
    park = ParallelPark(cfg)
    park.update(0.50, 1.0, 2.0, 0.0, 0.1)
    steer, duty, state = park.update(
        0.50, cfg.PARK_ABORT_FRONT_M - 0.01, 2.0, 0.0, 0.1)
    assert state is ParkState.ABORT
    assert steer == 0.0 and duty == 0.0
    # latched: later good readings do not restart the manoeuvre
    assert park.update(0.50, 1.0, 2.0, 0.0, 0.1)[2] is ParkState.ABORT


def test_abort_on_time_limit():
    cfg = Config()
    park = ParallelPark(cfg)
    state = ParkState.APPROACH
    for _ in range(int(cfg.PARK_MAX_TIME_S / 0.1) + 2):
        _steer, _duty, state = park.update(0.50, 1.0, 2.0, 0.0, 0.1)
        if state is ParkState.ABORT:
            break
    assert state is ParkState.ABORT


def test_reset_restarts_the_manoeuvre():
    cfg = Config()
    park = ParallelPark(cfg)
    park.update(0.50, 0.05, 2.0, 0.0, 0.1)
    assert park.state is ParkState.ABORT
    park.reset()
    assert park.state is ParkState.APPROACH
    assert park.elapsed_s == 0.0
    assert park.update(0.50, 1.0, 2.0, 0.0, 0.1)[2] is ParkState.APPROACH
