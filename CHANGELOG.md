# Changelog

All notable changes to this project are documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-11-04

First versioned release: the car that ran the 2026 season, packaged so another
team can build, run and audit it.

### Added

- MIT license (`LICENSE`, copyright Renkay Team).
- ROS 2 Jazzy packages `renkay_open` and `renkay_obstacle` (`ament_python`,
  `package.xml` format 3, `setup.py` with the `open_node` and `obstacle_node`
  entry points) and `renkay_bringup` with `open.launch.py`,
  `obstacle.launch.py` and `bringup.launch.py`; every device path is a launch
  argument and `use_sim_time` / `params_file` are exposed.
- `config/open.yaml` and `config/obstacle.yaml`: every constant of both
  `config.py` files with its value and unit, grouped as in the code. These are
  reference documents; the nodes read the dataclass, not ROS parameters.
- `SETUP.md`: hardware list, OS image, ROS 2 Jazzy install, clone, dependencies,
  build, device permissions, how to run each challenge, troubleshooting.
- `tools/setup-pi.sh` (Ubuntu 24.04 arm64, with `--check` and `--dry-run`) and
  `tools/99-renkay.rules` (`/dev/renkay_rrc`, `/dev/renkay_lidar`).
- `requirements.txt` (pinned numpy, opencv-python-headless, pytest, pyyaml) and
  `.github/workflows/tests.yml` (Python 3.12 unit tests plus a README/docs link
  check).
- `data/measurements.csv`: measurement register with date and instrument for
  each number used in the documentation.
- Pure-Python tests under `tests/` for the control helpers and the parking
  state machine (no ROS, no camera).
- `renkay_obstacle/parking.py`: parallel parking state machine (APPROACH,
  ALIGN, REVERSE_ARC, STRAIGHTEN, DONE, ABORT) driven by LiDAR side and rear
  distances plus yaw, with its parameters in `config.py`.
- Written documentation under `docs/` covering the mechanical design, the
  software architecture, the engineering decisions and the reproduction steps.

### Changed

- Source tree moved from `src/open` and `src/obstacle` to
  `src/renkay_open/renkay_open` and `src/renkay_obstacle/renkay_obstacle`;
  imports and the motor driver node now live in their package.
- All code comments, docstrings and log messages translated to English.
- Lap counting changed from integrated yaw to confirmed corner exits
  (`CORNERS_PER_LAP`), so a gyro bias can no longer add or drop a lap.
- LiDAR beam indices are derived from the scan's `angle_min` and
  `angle_increment` instead of assuming an even 360 deg split.
- The heading reference is re-anchored when the followed wall changes side
  (red/green block), and the distance history of the old wall is discarded.
- Watchdog faults latch: once the IMU or the scan times out the car stays
  stopped until both sensors have been valid for `FAULT_RECOVERY_S`.
- The TB6612FNG driver accepts a signed duty (`clamp_duty`, `duty_to_pins`):
  negative duty reverses with the magnitude preserved, and braking uses the
  opposite direction for 0.12 s.

### Fixed

- `side_open(None)` returns `False`: a missing side beam no longer starts a
  corner turn.
- Pillar returns are rejected from the wall estimate by a maximum wall angle
  and a maximum one-frame distance jump (`WALL_JUMP_MAX_M`).
- A camera block expires after `BLOCK_TIMEOUT_S`, so a stale detection no
  longer steers the car.
- The motor no longer clamps every command to 0..100 %, which made reverse
  impossible.

[1.0.0]: https://github.com/strvaeq/Robotek-WRO2026/releases/tag/v1.0.0
