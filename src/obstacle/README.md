# Obstacle Challenge — WRO Future Engineers

Autonomous robot that follows a wall, drives around red/green pillars, and
stops after 3 laps, using an IMU heading loop, LiDAR wall geometry and a
camera color detector on ROS 2 (Raspberry Pi 5).

## How it works

- **Camera:** detects the nearest red or green block. Color chooses which wall
  to follow — **red → right wall, green → left wall** — and adds a small
  heading nudge to steer around the block while it is close.
- **LiDAR (~10 Hz):** fits a line to two side beams (60° and 30°) to get the
  distance and angle to the followed wall, and detects corners (front wall
  close + open side).
- **IMU (~50 Hz):** integrates the gyroscope to track heading and outputs the
  steering angle that holds the target. It is the only loop that writes to the
  servo. At each corner ±90° is added to the base heading.

Laps are counted from integrated yaw (4 corners of 90° = 360° = 1 lap).

State machine:

```
IDLE --(button)--> CALIBRATING --(4 s)--> FOLLOWING <--> TURNING --(3 laps)--> DONE
```

## Files

| File | Purpose |
|---|---|
| `config.py` | All tunable parameters (gains, wall target, HSV ranges, corners). |
| `color_vision.py` | HSV red/green block detector (OpenCV). |
| `control.py` | Pure logic: heading loop, wall follow, block avoidance, corners, laps. |
| `robot_node.py` | Main node: state machine tying the three sensors together. |
| `motor_driver.py` | ROS node for the TB6612FNG motor driver (shared with the Open Challenge). |

## Running

```bash
python3 motor_driver.py    # motor driver
python3 robot_node.py      # main controller
```

Press the controller button to calibrate and start.
