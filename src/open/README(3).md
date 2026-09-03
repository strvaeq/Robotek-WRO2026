# Open Challenge — WRO Future Engineers

Autonomous robot that drives the track and stops after 3 laps, using a
cascaded **lidar + IMU** controller on ROS 2 (Raspberry Pi 5).

## How it works

Two loops that complement each other:

- **Outer loop (lidar, ~10 Hz):** reads the side walls and computes a target
  heading to keep the robot centered in the corridor.
- **Inner loop (IMU, ~50 Hz):** integrates the gyroscope to track the current
  heading and outputs the steering angle that chases that target. It is the
  only loop that writes to the servo.

The lidar decides *where* to go; the IMU decides *how* to hold the heading. At
each corner ±90° is added to the base heading and the inner loop performs the
turn on its own. Laps are counted from integrated yaw (4 corners of 90° =
360° = 1 lap).

State machine:

```
IDLE --(button)--> CALIBRATING --(4 s)--> STRAIGHT <--> TURNING --(3 laps)--> DONE
```

## Files

| File | Purpose |
|---|---|
| `config.py` | All tunable parameters (gains, thresholds, servo, motor). |
| `control.py` | Pure logic: inner/outer loops, corners, lap counting, servo mapping. |
| `motor_driver.py` | ROS node for the TB6612FNG motor driver. |
| `robot_node.py` | Main node: state machine tying both loops together. |

## Running

```bash
python3 motor_driver.py    # motor driver
python3 robot_node.py      # main controller
```

Press the controller button to calibrate and start.
