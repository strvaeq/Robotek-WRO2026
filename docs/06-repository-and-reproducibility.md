# Repository and Reproducibility

This page explains how the repository is organised, how to verify it, and the commitments the team makes about its availability.

## 1. Public availability

The repository is public from the moment of submission and stays public for at least twelve months after the international final, as required by the 2026 Future Engineers rules. The MIT licence in [`LICENSE`](../LICENSE) allows reuse with attribution.

## 2. Structure

| Path | Content |
|---|---|
| `README.md` | Project entry point, results and navigation |
| `docs/` | Engineering documentation by rubric criterion |
| `src/renkay_open/` | ROS 2 package for the Open Challenge |
| `src/renkay_obstacle/` | ROS 2 package for the Obstacle Challenge |
| `src/renkay_bringup/` | Launch files and shared parameters |
| `config/` | Reference parameter files with units |
| `data/` | Measurements, power budget, test runs |
| `models/` | Printable parts, STL and source files |
| `schemes/` | Wiring, power and software diagrams |
| `v-photos/`, `t-photos/` | Vehicle and team photographs |
| `tools/` | Setup script and udev rules |
| `tests/` | Unit tests that run without hardware |

## 3. How to verify reproducibility

A reader with no access to the vehicle can verify the following in order:

1. Clone the repository and read [`SETUP.md`](../SETUP.md).
2. Run `tools/setup-pi.sh --check` on a fresh Ubuntu 24.04 install; the script reports missing packages without changing the system.
3. Run `tools/setup-pi.sh` to install ROS 2 Jazzy, the udev rules and the workspace.
4. Build with `colcon build --symlink-install`.
5. Run the unit tests with `python -m pytest tests -q`; they do not require ROS or hardware.
6. Launch `ros2 launch renkay_bringup bringup.launch.py` and confirm the topics listed in `SETUP.md`.

The measurement register in [`data/measurements.csv`](../data/measurements.csv) records how each number in the documentation was obtained, with date and uncertainty, so a reader can repeat the measurement.

## 4. Commit and release policy

Commits are small and describe one change. The prefix is one of `feat`, `fix`, `docs`, `test`, `chore`. Merge commits are avoided in favour of squashed pull requests.

The 2026 rules require at least three commits with staggered dates: the first no later than two months before the competition, the second no later than one month, and the third no later than two weeks. The third commit is the one mainly evaluated. The history of this repository is therefore dated accordingly and is not rewritten.

Releases follow semantic versioning. Each release has a tag, a `CHANGELOG.md` entry and release notes that name the design version of the vehicle.

## 5. Assets

All images are stored in the repository. No diagram or photograph depends on a personal GitHub attachment URL or on another team's repository.

Diagrams are exported as PNG at full resolution. Photographs are resized to 1600 px on the long edge to keep the clone small.

## 6. Third-party material

The following components are third-party and are credited here:

- ROS 2 Jazzy, Apache 2.0.
- OpenCV, Apache 2.0.
- `ros_robot_controller_msgs`, distributed with the HiWonder RRC Lite image.
- LD19 LiDAR driver, provided by the manufacturer.

No code, image or video from another WRO team is included.

## 7. Known limitations

The repository documents the v11 vehicle. Measurements marked `pending` in the register were taken from the CAD model or from a single sample and are flagged as such; they are not presented as test results.

The parking manoeuvre is implemented in software but its success rate is measured in the test log only after the next hardware session.
