# 02. Power and Sensor Architecture

Power and sensing of Renkay, vehicle version 11: budget, rail topology, protection, GPIO and bus map, sensor trade-offs against the 2026 field, placement, calibration, failure modes and iterations. Every number is measured, derived, from a datasheet, or marked `pending`.

Sources: `[M]` [`data/measurements.csv`](../data/measurements.csv), `[U]` URDF, `[C]` code in `src/`, `[D]` datasheet, `[R:line]` `README.md`, `[P]` pending. Machine-readable budget: [`data/power-budget.csv`](../data/power-budget.csv).

## 1. Power budget

Pack: 2S Li-Po, 7.4 V, 5000 mAh, 20C, 37 Wh [M]. `duty` is the fraction of a round a load is active, not PWM duty.

| Component | Rail | V | Typical A | Peak A | Duty | Basis | Status |
|---|---|---:|---:|---:|---:|---|---|
| Raspberry Pi 5 | 5V | 5.0 | 0.60 | 2.50 | 1.00 | 575 mA idle plus USB allowance [D] | datasheet |
| USB camera | 5V | 5.0 | 0.20 | 0.25 | 1.00 | 640x480 YUYV at 6 fps [D] | datasheet |
| LD19 LiDAR and adapter | 5V | 5.0 | 0.25 | 0.35 | 1.00 | 5 V, 250 mA [D] | datasheet |
| RRC Lite board | 5V | 5.0 | 0.12 | 0.20 | 1.00 | MCU, IMU, LEDs, buzzer | pending |
| Steering servo | 5V | 5.0 | 0.25 | 2.00 | 0.35 | moving no load / stall at 5 V [D] | datasheet |
| LED ring, 16 pixels | 5V | 5.0 | 0.30 | 0.96 | 0.60 | 20 mA average, 60 mA white [D] | datasheet |
| TB6612 logic | 3V3 | 3.3 | 0.002 | 0.005 | 1.00 | logic only [D] | datasheet |
| Drive motor | 7V4 | 7.4 | pending | 3.20 | 0.60 | TB6612 1.2 A continuous, 3.2 A peak [D] | pending |
| **5 V rail total** | 5V | 5.0 | **1.72** | **8.26** | 1.00 | sum of the rows above | derived |
| **Pack total** | 7V4 | 7.4 | **1.72** | **9.54** | 0.60 | 5 V rows reflected at 5/7.4 and 0.88 efficiency, plus motor | derived |

The 8.26 A peak is every 5 V load at maximum at once; the credible simultaneous peak is 4.40 A. Motor current is the gap: the TB6612 caps the branch at 1.2 A continuous and 3.2 A peak [D], but the motor's stall and cruise currents are `pending` with the clamp (7.6), so autonomy uses a 0.40 A allowance.

```
nominal: 1.72 x 5.0 / 0.88 + 0.40 x 7.4 = 12.73 W
worst:   8.26 x 5.0 / 0.88 + 3.20 x 7.4 = 70.6 W
```

Autonomy uses 80 % of 37 Wh = 29.6 Wh: a Li-Po should not reach 3.0 V per cell and the last 20 % sags enough to brown out the Pi. That gives 2.3 h nominal and 25 min at the bound. A 3 min round needs 2.2 % nominal, so energy is not the constraint; wire rating, sag and the 5 V regulator are. The discharge test (constant 35 % to 7.0 V, time logged) is `pending`.

## 2. Rail topology

```
7.4 V pack -> 5 A fuse -> switch ->+---- TB6612 VM -> motor (7.4 V)
                                   |
                                   +---- RRC Lite -> 5 V ->+-- Pi 5 (header 5 V)
                                                           +-- steering servo
                                                           +-- LED ring
                                                           +-- RRC Lite board
                                                           +-- Pi USB -> camera, LiDAR, RRC data
```

The 7.4 V branch is the motor only. The TB6612 chops the pack at 1 kHz, so the motor sees the pack minus the bridge drop. The 5 V branch carries everything that cannot take 8.4 V: Pi 5, servo, LED ring, RRC board, and through the Pi's USB ports the camera and LiDAR. The RRC Lite does not drive the motor; it provides servo, IMU, button and buzzer.

The split keeps the motor's chopped current and back-EMF off the 5 V rail, where a dip would brown out the Pi. Ground is a star at the RRC Lite GND: pack, TB6612, Pi, servo and adapter return there; motor current returns through the TB6612, not the Pi header, and the logic grounds stay common for the 3.3 V PWM.

The servo is not on the Pi 5 V GPIO rail: its 2.00 A stall exceeds the camera, LiDAR and RRC together, the header is not a power bus, and its noise would couple into the camera and USB links. Its 3-pin lead lands on the RRC Lite steering channel, which supplies 5 V, signal and ground.

**Superseded figure.** `schemes/power-distribution-diagram.png` is the v10 drawing: a 6 V UBEC for the servo and sensors taken from the Pi 5 V pins. v11 replaced that with the topology above; it is iteration evidence, not a wiring instruction.

## 3. Protection

**Fuse.** Continuous pack current 1.72 A, credible peak 6.58 A.

```
I_fuse >= 1.25 x 1.72 = 2.15 A
I_fuse >= 6.58 A for under 1 s
I_fuse < 16 A, the 18 AWG chassis rating
```

A 5 A mini blade fuse meets all three: 2.9 times continuous, 132 % of the peak for under a second, below the wire rating and far below the 100 A pack capability. It sits in the pack positive 30 mm from the XT60, before the switch, so a short in the switch or harness is protected.

**Polarity.** The keyed XT60 prevents reverse connection; no reverse-polarity MOSFET is fitted.

**Low-voltage cutoff.** A 2S pack is 8.4 V full and 6.0 V empty; the loaded floor is 3.5 V per cell, 7.0 V. Measure at rest before and after a run and retire the pack below 7.0 V. The RRC Lite reports pack voltage, but automatic cutoff is not implemented in v11; recorded gap F-01.

**Switch.** The RRC Lite switch removes the 5 V rail and the Pi; the motor branch stays at pack voltage because it is upstream, but STBY has an internal pull-down, so the bridge is in standby with the Pi off. The harness is still live; a master switch on the pack positive is `pending fitment`.

**Brownout.** Below 4.63 V the Pi resets, its GPIOs become inputs, STBY is pulled low and the motor coasts; the servo relaxes. That is a safe stop, not a controlled one (section 8).

**Connectors.** XT60 for the pack, 5 A blade holder, 3-pin JR servo lead, ferrules on the TB6612 terminals, USB-C for the RRC link, USB-A to USB-C for the LiDAR, USB-A for the camera, 2.54 mm Dupont for the four GPIO signals.

## 4. GPIO and bus map

TB6612 on GPIO chip 4, channel A, PWM at 1 kHz [C]:

| Signal | BCM | Header pin | TB6612 pin | Direction |
|---|---:|---:|---|---|
| PWMA | 12 | 32 | PWMA | Pi output, 1 kHz |
| AIN1 | 5 | 29 | AIN1 | Pi output |
| AIN2 | 6 | 31 | AIN2 | Pi output |
| STBY | 13 | 33 | STBY | Pi output, high enables |
| GND | GND | 30 | GND | common |
| VCC | 3V3 | 1 | VCC | 3.3 V logic |

The motor driver reads these pins as ROS parameters [C]; the strategy nodes read none and use `config.py` as their source of truth. The RRC Lite only provides the servo, IMU, button and buzzer. STBY is high only while a command is active; the 0.30 s command watchdog drops it, and the 0.05 s node watchdog stops the motor when the IMU or scan stops [C].

| Device | Bus | Stable name | Fallback | Rate | Topic and type |
|---|---|---|---|---:|---|
| LD19 LiDAR | USB, CP210x | `/dev/ldlidar` | `/dev/ttyUSB0`, 230400 8N1 | 10 Hz | `/scan`, `LaserScan` |
| Camera | USB video | `/dev/v4l/by-id/*-video-index0` | `/dev/video0` | 6 fps | `/usb_cam/image_raw`, `Image` |
| RRC Lite | USB CDC ACM | `/dev/rrc_lite` | `/dev/ttyACM0`, 1 Mbaud | 50 Hz | `/ros_robot_controller/imu_raw`, `/button`, `/acker_servo/set_state` |
| Motor command | GPIO, no bus | n/a | n/a | 20 Hz | `/motor_vel`, `Float32` |

There is no I2C device in v11: the IMU is on the RRC Lite MCU and shares the USB link with the button and servo. Two udev rules in `/etc/udev/rules.d/99-renkay.rules` give the stable names:

```
ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", GROUP="dialout", MODE="0660", SYMLINK+="rrc_lite"
ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0660", SYMLINK+="ldlidar"
```

The README abbreviates `/ros_robot_controller/acker_servo/set_state` as `/positionServo`; the code uses the full names.

## 5. Sensor selection

The field fixes the problem: 100 mm walls 40 mm thick, a 1000 mm lane (600 mm minimum), 50 x 50 x 100 mm signs, and a 200 mm deep parking lot 1.5 car lengths long (350 mm). At 180 mm from one wall in the 600 mm case, 272 mm remains on the other side.

| Option | Range | Rate | FOV | Cost | Failure mode | Decision |
|---|---|---|---:|---:|---|---|
| LD19 LiDAR | 0.03 to 12 m | 10 Hz, 5000 Hz ranging | 360 deg, 220 deg usable | 70 USD | Dark or angled surfaces, 30 mm blind zone, body returns | **Selected**: bearing plus a wall line |
| Ultrasonic HC-SR04 | 0.02 to 4 m | 40 Hz | 15 deg cone | 2 USD | Soft walls, cross-talk, no bearing | Rejected: blurred wall angle |
| IR Sharp GP2Y0A21 | 0.10 to 0.80 m | 25 Hz | 10 deg | 10 USD | Ambient light, surface colour | Rejected: shorter than the lane |
| ToF VL53L1X | 0.04 to 4 m | 50 Hz | 27 deg | 15 USD | Sunlight, no bearing | Rejected: point range only |
| Monocular camera | 0.28 m to horizon | 6 fps | 66.7 deg | 15 USD | Glare, occlusion, blur | **Selected** for colour only |
| Depth camera | 0.3 to 3 m | 30 fps | 60 to 90 deg | 200 to 350 USD | IR sunlight, USB load, compute | Rejected: LiDAR gives geometry |
| RRC Lite IMU | n/a | 50 Hz | 1 axis | 0 USD | Drift, vibration, axis remap | **Selected**: shares the servo link |

Costs are budget prices, not supplier quotes.

The LiDAR wins on geometry: wall following needs distance at a known bearing and the wall angle, and the code uses rays at 60 and 30 deg [C]. Three ultrasonic cones at 1 m overlap into a 0.5 m patch that cannot resolve a 40 mm wall, while the LD19 gives 500 samples per revolution at 10 Hz, 0.72 deg apart. The camera wins on colour: at 9.6 px/deg a 50 x 100 mm sign at 1.5 m is 670 px, just above the 650 px threshold, and at 1.6 m it is 590 px and rejected, so colour works to 1.5 m. The IMU is on the RRC Lite because the PD loop runs at 50 Hz and the servo delay is 60 ms [M]; a faster IMU would not outrun the actuator.

## 6. Placement

| Item | x (mm) | y (mm) | z (mm) | Source |
|---|---:|---:|---:|---|
| LiDAR scan axis | 128.0 | 0 | 30.0 | URDF [U] |
| LiDAR nose | 147.7 | 0 | 12 to 51 | URDF [U] |
| Camera body centre | 56.5 | 0 | 144.6 | URDF, STEP [U] |
| IMU | 0 | 0 | 61.0 | URDF [U] |
| Plate front / rear | 94.5 / -94.5 | 0 | 15 to 45 | URDF [U] |

The LiDAR is 128 mm ahead, scan plane 30 mm, 70 mm below the wall top, so a level beam cannot pass over the wall at 0.18 m. Pitch at 1.0 m/s^2 is 5.8 deg, raising the beam 18 mm at 0.18 m (52 mm margin); at 3 m it raises 300 mm, so distant walls can be missed under acceleration, the followed wall cannot.

The plane is above the wheel axle (30.5 mm) and plate bottom (15 mm), with no chassis ahead of 94.5 mm at that height. The housing is 19.7 mm ahead of the axis, inside the 30 mm minimum range, so it cannot occlude. The nose at 147.7 mm is the front contact, 53.2 mm ahead of the plate, so a wall touches the tab before the lens.

The camera is 56.5 mm ahead and 144.6 mm high, pitched 10 deg down and yawed 4.4 deg left. Its axis meets the floor at `144.6 / tan(10) = 820 mm`. With f = 570 px over 480 px, the vertical half-field is `atan(240/570) = 22.8 deg`; the bottom ray is 32.8 deg down and meets the floor 224 mm from the camera, 280 mm ahead of `base_link`, while the top ray is 12.8 deg up. The camera sees the floor from 280 mm and the LiDAR covers 30 to 280 mm.

The camera is mounted upside down and the frame is used as captured: `color_vision.py` does not rotate, undistort or crop. Colour is unaffected, but `cx_norm` is mirrored: a block on the car's left appears at positive `cx_norm`, so the nudge sign is measured (7.4). Cables: LiDAR left under the scan plane, camera right, servo away from the twisted motor pair.

## 7. Calibration

**7.1 Gyro bias and axis.** The node averages `angular_velocity.z` for 4.0 s stationary at the start of each run (`GYRO_CAL_S = 4.0`) [C].

```
ros2 topic hz /ros_robot_controller/imu_raw          # expect 50 Hz
ros2 bag record -o calib_gyro /ros_robot_controller/imu_raw
```

Acceptance: 50 +/- 1 Hz; mean bias within +/-0.02 rad/s (measured +0.006 rad/s [M]); spread within 0.005 rad/s; drift below 1.5 deg over 60 s. Re-measure the axes if the board moves.

**7.2 Servo centre.**

```
ros2 topic pub --once /ros_robot_controller/acker_servo/set_state \
  ros_robot_controller_msgs/msg/SetAckerServoState "{position: 1500, duration: 0.06}"
```

Wheels off the ground, measure both front wheels against the centreline. Acceptance: 1500 us gives 0.0 +/- 0.5 deg and the wheels are parallel within 0.5 deg; re-find the centre if the horn is re-splined.

**7.3 Servo gain and wheel table.** Command 10 and 25 deg equivalents (1683 and 1958 us positive, 1333 and 1083 us negative with 18.33 and 16.67 us/deg [C]), drive three circles, and measure the radius. `delta = atan(L/R)`, `L = 117.4 mm` [M]. Acceptance: repeatability within +/-0.5 deg, left and right recorded separately. The existing table is from the old mounting and is `pending`.

**7.4 Camera field and mount.** The node uses the raw frame: no rotation, undistortion or crop.

```
ros2 topic hz /usb_cam/image_raw                     # expect 6 fps
```

Measure the field on a wall at 1.0 m by the column scale. Acceptance: 66.7 +/- 0.5 deg; a 50 x 100 mm red target at 1.5 m gives at least 650 px of mask and at 1.6 m is rejected; the inverted mount mirrors `cx_norm`, and that mirror is recorded before tuning the nudge.

**7.5 LiDAR angle offset.** Place a 50 mm box at 0.50 m in front, centred on the car axis, and adjust the offset until the front window (180 deg in the node convention) reads 0.50 +/- 0.02 m with edges symmetric within 1 deg.

```
ros2 topic hz /scan                                  # expect 10 Hz
```

Acceptance: range error at most 20 mm, bearing error at most 1 deg, no body returns in the 220 deg sector (the 2 to 8 cm returns beyond +/-110 deg must be filtered [M]). The index comes from `angle_min` and `angle_increment`.

**7.6 Current and autonomy.** Clamp a UNI-T UT210E on the 5 V positive, then the motor positive, wheels off the ground. Run 35 % for 30 s and log minimum, average and maximum; block the steering 0.5 s for the servo peak; repeat three times. Acceptance: repeatability within 5 %, 5 V rail above 4.8 V, pack above 7.2 V before and 7.0 V after. Enter the values in `data/power-budget.csv` and change their status to `measured`.

## 8. Failure modes

| Failure | Detection | Automatic action | Recovery | Test |
|---|---|---|---|---|
| Brownout, 5 V below 4.63 V | Undervoltage flag | Pi resets, STBY low, motor coasts | Power cycle, recalibrate | Ramp 5.0 to 4.0 V at 1 A |
| LiDAR dropout | No `/scan` 0.35 s | Motor 0, servo centre, latch | Button or relaunch | Unplug at 0.39 m/s; stop under 0.30 m |
| IMU drift or wrong axis | Bias above 0.02 rad/s | Motor 0, servo centre | Recalibrate 4 s | Inject 0.05 rad/s; lap aborts |
| Camera glare or occlusion | No image 1.5 s, no colour 3 frames | Keep the last side, continue on LiDAR | Clean, relight, relaunch | Cover lens 30 s; no side change |
| Servo stall | Angle against gyro yaw | 5 V sag browns out the Pi; no servo limit | Free wheel, power cycle | Hold 25 deg for 3 s |
| Motor stall | Above 1.2 A, case hot | TB6612 thermal shutdown | Remove obstruction, cool | Stall 35 % for 5 s |
| USB disconnect | Device node missing | Motor 0, servo centre, latch | Replug, relaunch | Unplug each USB device |

The first two rows are in `robot_node.py`: the 0.20 s IMU and 0.35 s LiDAR timeouts stop the motor, and the branch fix latches the stop [C]. The camera row is the 1.5 s block expiry added in this branch. The servo has no current limit; a sustained stall relies on the 5 V sag (R-06, fixed by a stall timer).

## 9. Iteration evidence

| Version | Change | Reason | Measured or derived effect |
|---|---|---|---|
| v9 | IR sensors and a webcam | First autonomous loop | IR gives a threshold, not a bearing; no wall angle, so wall following was impossible |
| v10 | LiDAR high, L298N, 2000 mAh, 6 V UBEC | Add bearing, more torque | LiDAR looked over the 100 mm wall when the car pitched; L298N drops 1.8 V at 1 A, 24 % of the pack; 14.8 Wh gives 0.93 h at 12.7 W |
| v11 | LiDAR at 30 mm, TB6612, 5000 mAh, one 5 V rail, LED ring | Fix wall returns, driver loss, colour stability | Plane 70 mm below the wall top; TB6612 drop 0.3 to 0.5 V at 1 A, 4 to 7 %; 37 Wh gives 2.3 h; LED adds 0.30 A typical |

The LiDAR height is proven by geometry (30 mm inside 0 to 100 mm); the TB6612 loss is derived against the L298N's 1.8 V at 1 A; the 5000 mAh pack is 2.5 times the energy. The LED ring's before-and-after false-positive count is `pending` [R:307].

## 10. Wiring diagram notes

`schemes/electrical-wiring-diagram.png` is pin-level: GPIO12/5/6/13 to PWMA/AIN1/AIN2/STBY, RRC 5 V to the Pi header and LED ring, servo on the RRC Lite, LiDAR and camera on USB, TB6612 VM on the pack. It omits the 5 A fuse and master switch, which are in the harness.

`schemes/visual-wiring-diagram.png` is the assembly view (component images, cable routes, no pin numbers). `schemes/power-distribution-diagram.png` is the v10 drawing, superseded: 6 V UBEC for the servo, sensors fed from the Pi 5 V pins.

Pack and harness use XT60; 18 AWG for pack, RRC Lite and TB6612 VM; 5 A blade fuse 30 mm from the XT60 in the pack positive; ferrules on the TB6612; 20 AWG twisted pair to the motor; 22 AWG for 5 V to the Pi header, servo and LED ring; 26 AWG for GPIO; USB elsewhere. The fuse is the only overcurrent device: the 5 V branches rely on the RRC Lite and Pi USB limits, and the motor on the TB6612's 1.2 A rating.
