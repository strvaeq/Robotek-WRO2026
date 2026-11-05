# 05 - Systems Engineering and Engineering Decisions

This document records the decomposition, the constraints, the decisions and their evidence, the versions, and what can still fail. Every number is tagged `measured`, `derived`, `declared` or `pending` as in `data/measurements.csv`; a change with no measurable effect says `not measured`.

---

## 1. The vehicle as five subsystems

```
+-------------------------------------------------------------------------------+
| POWER   7.4 V 2S Li-Po 5000 mAh 20C, 37 Wh, 100 A declared                     |
| RRC Lite 5 V rail: Pi 5, servo, sensors, ring                                  |
| TB6612FNG bridge: motor, 1 kHz PWM, STBY disable                               |
+-----+--------------------+--------------------+--------------------+----------+
      | 5 V                | 7.4 V              | 5 V                | 5 V
      v                    v                    v                    v
+----------------+   +----------------+   +----------------+   +----------------+
| SENSING        |   | ACTUATION      |   | COMPUTE        |   | OPERATIONS     |
| LD19 LiDAR     |   | servo 15 kg.cm |   | Pi 5           |   | start button   |
| IMU 50 Hz      |   | TB6612FNG      |   | Ubuntu 24.04   |   | buzzer         |
| camera 6 fps   |   | 25 mm motor    |   | ROS 2 Jazzy    |   | WS2812 ring    |
|                |   |                |   | PD + FSMs      |   |                |
+----------------+   +----------------+   +----------------+   +----------------+
      |                    |                    |                    |
 data /scan, /imu_raw, image      commands /motor_vel, servo set_state
      v                    v                    v                    v
+-------------------------------------------------------------------------------+
| MECHANICAL   plate 189 x 90 mm, top at 45 mm, wheelbase 117.4 mm               |
| track 133.5 / 123.5 mm, LiDAR 128 mm ahead at 30 mm, camera 56.5 mm inverted   |
+-------------------------------------------------------------------------------+
```

**Figure 1.** Five subsystems, four interface families; every geometric constraint comes from the mechanical one.

### 1.1 Interface register

| ID | Interface | Type | Value | Failure if it breaks |
|---|---|---|---|---|
| I-01 | pack -> TB6612FNG | power | 7.4 V, peak pending | Brownout, Pi reboot |
| I-02 | RRC -> Pi, servo, sensors | power | 5 V, current per rail pending | Pi undervoltage, servo reset |
| I-03 | LiDAR -> Pi, `/scan` | data | 10 Hz, 500 points/rev | Wall estimate lost |
| I-04 | IMU -> Pi, `/imu_raw` | data | 50 Hz, sigma 0.0019 rad/s | Heading drift |
| I-05 | camera -> Pi | data | 6 fps, 66.7 deg HFOV | Colour missed |
| I-06 | Pi -> RRC -> servo | data, timing | 1000 to 2050 us, 60 ms | Linkage pushed into its stop |
| I-07 | Pi -> TB6612FNG | data, timing | duty %, 1 kHz, 0.30 s watchdog | Stale command, run-away |
| I-08 | Pi clock -> all nodes | timing | 1.5 s camera expiry, 0.35 s LiDAR timeout | A dead sensor keeps steering |
| I-09 | plate -> sensors | physical | plate 45 mm, plane 30 mm | Occlusion, misalignment |

Timing cost the most: a sensor that stops publishing is harmless until a consumer keeps the last value. Hence I-08.

---

## 2. Constraint register

| ID | Class | Constraint | Value | Unit | Source | Design consequence |
|---|---|---|---|---|---|---|
| C-01 | rule | Autonomous, no external compute | 0 off-board processes | count | 2026 rules | All control on the Pi 5 |
| C-02 | rule | Three laps per round | 3 | laps | 2026 rules | Corner-based lap counter, finish beep |
| C-03 | rule | Parallel parking after lap 3 | 1 | manoeuvre | 2026 rules | `parking.py`; reverse required |
| C-04 | rule | Inside the start envelope | 300 x 200 (car 230 x 150 x 160) | mm | docs/01, rules | Single plate, vertical stacking |
| C-05 | rule | English text | 100 | % | 2026 rules | No Spanish in committed text |
| C-06 | rule | One driven rear axle | 1 axle | count | v4 build note | No skid steering, so Ackermann |
| C-07 | field | Wall height | 100 | mm | field geometry | Scan plane at 30 mm |
| C-08 | physical | Mass with pack | 1.39 (0.10) | kg | modelled + estimate | 0.042 N m at 1.0 m/s^2 |
| C-09 | physical | Top speed | 0.42 | m/s | measured, 50 % duty | Lap time has a floor |
| C-10 | physical | Accel / decel limits | 1.0 / 0.6 | m/s^2 | configured, IMU verified | 0.39 m/s in 0.39 s; braking 0.127 m |
| C-11 | physical | Servo rate and delay | 70, 60 | deg/s, ms | measured 8 Oct 2026 | 30 deg step costs 0.43 s, 0.17 m |
| C-12 | physical | LiDAR field of view | 360, 0.05 to 3.3 | deg, m | declared + config | Beyond 3.3 m or 55 deg is rejected |
| C-13 | physical | Camera | 66.7 HFOV, 6 fps | deg, fps | measured 1 Nov 2026 | 0.065 m per frame; 3 frames cost 0.20 m |
| C-14 | physical | Gyro bias | 0.006 | rad/s | measured | 9.6 deg drift in a 28 s run |
| C-15 | physical | Wheel | 61.0 diameter, 0.192 | mm, m/rev | measured 21 Oct 2026 | 131 rpm at 0.42 m/s |
| C-16 | physical | Minimum radius | 0.37 | m | 0.1174 / tan(17.6 deg) | 90 deg corner is a 0.58 m arc, 1.5 s |
| C-17 | project | Team | 3 students, 1 coach | people | team | One owner per subsystem |
| C-18 | project | Build window | 6 | weeks | team plan | No second chassis concept |
| C-19 | project | Budget | 1646 PEN / 469 USD | PEN, USD | Section 7, estimated | Prefer the part already owned |

C-14 reshaped the software: 0.006 rad/s accumulates 9.6 deg in 28 s, more than the 10 deg lap margin, so a lap is counted from corner events confirmed by yaw. The 4 s calibration reduces the bias but does not remove it.

---

## 3. Decision log

| D | Context and options | Criteria | Decision | Evidence | Reversal trigger |
|---|---|---|---|---|---|
| D01 L298N to TB6612FNG | v10 stalled at 50 % duty under the Darlington drop. Options: higher duty, heatsink, TB6612, BTS7960. | Low-duty torque, heat, size | TB6612FNG, 1 kHz, chip 4, PWM 12, AIN1 5, AIN2 6, STBY 13 | 35 % gives 0.39 m/s; deadband 6.5 %; signed reverse; STBY disables | Stall torque below 0.042 N m |
| D02 PLA to polycarbonate | Housings near the servo and pack deformed; PLA softens near 60 C. Options: thicker PLA, PETG, PC, MDF. | Heat, impact, printability | PC on the housings that deformed in v6; PLA elsewhere | `not measured`; the parts table in docs/01 lists PLA for the current set | A PC part moves the LiDAR plane 1 mm |
| D03 LiDAR high to low | On top, the plane sat above the 100 mm walls. Options: tilt, raise, camera-only. | Wall return, plane stability | Front mount, plane 30 mm, 128 mm ahead | 70 mm margin; valid line in corners; tilt biases distance | Reflections inside 0.15 m |
| D04 2000 to 5000 mAh | v10 had 14.8 Wh and sag on start. Options: two packs, 3300, 5000, 3S. | Energy, mass, current | 2S 5000 mAh 20C, 37 Wh, 100 A declared | Energy +150 %; autonomy `pending`; 0.10 kg adds 0.003 N m | A round uses under 30 % of 37 Wh |
| D05 PID to PD | No integral in code; it would wind up in the 1.5 s cooldown. Options: clamped PID, PD, PI, P. | Overshoot, offset, tuning | PD 1.8/0.22 and 1.6/0.28, 4 s calibration | D on gyro rate, not differentiated theta; offset `not measured` | Offset above 0.05 m on a straight |
| D06 mean to median | One pillar ray biases a mean by tens of cm. Options: rejection band, median, trimmed mean, Kalman. | Robustness, latency, code | Median, 10 deg window, 29 rays | Tolerates 14 bad rays; pillars rejected by 55 deg and 0.9 m | Lags a corner by over one scan |
| D07 "2DOF camera" to fixed | README claimed pan/tilt; code has none. Options: build the mount, wide lens, second camera. | Latency, coverage, mass | Fixed camera, 56.5 mm ahead, inverted | 3 frames at 6 fps cost 0.50 s and 0.20 m; a pan adds 0.43 s; the inverted mount mirrors cx (Section 9) | A sign missed in over 1 run in 10 |
| D08 RRC M4 vs TB6612 | The RRC M4 port was considered: closed-loop rps with its own PID. v11 drives the motor from GPIO through the TB6612; the RRC carries servo, IMU, button and buzzer only. Options: M4, TB6612, both. | One controller, units, reverse | TB6612 over GPIO via lgpio; RRC out of the motor path | Law is in duty because v(pct) was measured; M4 adds a PID in series | Duty speed fails when traction drops |
| D09 30 deg steering clamp | Linkage reaches 19.7 / 17.6 deg at a 25 deg command. Options: to the stop, clamp 30, clamp 25. | Pulse range, stall current | Clamp 30 deg, gains 550/30 and 500/30 us | Stays inside 1000 to 2050 us; radius 0.37 m | A corner needs under 0.37 m: rework the linkage |
| D10 Ackermann vs differential | Corners are 90 deg and the run ends in a park. Options: Ackermann kit, differential plus castor, tricycle. | Radius, parking envelope, parts | Ackermann, rear axle on one gear pair | Kit linkage on hand, one current path, radius 0.37 m | Slot shorter than the two-arc envelope |

---

## 4. Iteration log

| V | Change | Reason | Measured effect | Cost |
|---|---|---|---|---|
| 1 | Cardboard Ackermann prototype | Learn the geometry | not measured (no drive) | Cardboard only |
| 2 | Acrylic chassis, printed housings, stepper, IR, webcam | First drivable car | not measured | Acrylic, 3 prints, stepper |
| 3 | Smaller power bank, two levels, IR to the front | Runtime, space, early detection | not measured | 1 pack, 4 wheels |
| 4 | HiWonder chassis, LiDAR, mono camera, servo, ROS | 360 deg perception, one framework | not measured (no log) | 1 LiDAR, 1 servo, 2 motors |
| 5 | Bases joined, Pi inside, LiDAR on top | Packaging, clearance | not measured; saw over the walls | 1 chassis, 2 prints |
| 6 | Lower wheels, PLA to PC | LiDAR above the 100 mm walls | Wall return recovered, qualitative | 4 wheels, 3 prints |
| 7 | Smaller chassis, single motor, single level | Space, airflow | First sign detected, qualitative | 1 chassis, 1 motor, 2 prints |
| 8 | Slimmer front wheels, vertical servo, tilted LiDAR | Width, steering travel | Envelope 230 x 150 mm | 2 wheels, 2 prints |
| 9 | MDF base, Ackermann moved, wheel groove | Rack hit the servo, wheels slipped | 20 s per lap, first timed lap | 1 MDF cut, 1 print |
| 10 | LiDAR housing removed, rear wheels printed, big gear on motor shaft | Less obstruction, wheel speed | Under 10 s per lap, 28 s for 3 laps | 4 prints, gear rework |
| 11 | LED ring, LiDAR to the front, TB6612 | Colour stability, wall detection, low-duty torque | 0.39 m/s at 35 %, 0.42 top, deadband 6.5 %, reverse available | 20 PEN driver, 18 PEN ring, 1 print |

The clearest result is v9 to v10: 20 s per lap to under 10 s, 3 laps in 28 s instead of a derived 60 s, a 53 % reduction. It came from the gear swap, not software: the drivetrain was the binding constraint.

Two entries keep `not measured`. v6 recovered the wall return with no scan recorded before and after. v8 was expected to widen the steering angle, measured only on 8 Oct.

---

## 5. Risk register

P and I run 1 to 5; score is their product.

| ID | Risk | P | I | Score | Detection | Mitigation | Owner | Residual |
|---|---|---|---|---|---|---|---|---|
| R-01 | Brownout on start | 4 | 4 | 16 | Power status, Pi undervoltage flag | Separate motor path, 1.0 m/s^2 ramp, 5000 mAh pack | Power | 8 |
| R-02 | LiDAR dropout over 0.35 s | 3 | 5 | 15 | `/scan` timeout | Watchdog latches the stop until reset; no last-value reuse | Software | 5 |
| R-03 | Glare or false colour | 4 | 3 | 12 | Decision log, confirmation counter | LED ring, dual red HSV range, 3-frame confirmation, area and fill filters | Vision | 6 |
| R-04 | Servo stall on the stop | 3 | 3 | 9 | Yaw not following for 0.5 s | 30 deg clamp, 1000 to 2050 us pulse clamp, asymmetric gains | Mechanical | 4 |
| R-05 | Lap miscount | 3 | 5 | 15 | Lap and finish beeps, corner counter | Corner events confirmed by yaw, 1.5 s cooldown, 10 deg margin | Software | 6 |
| R-06 | Parking overshoot | 4 | 4 | 16 | Front clearance, yaw per phase | Reverse capped at 25 %, 0.25 m/s; abort below 0.10 m; 0.03 m target | Software | 8 |
| R-07 | USB disconnect | 3 | 4 | 12 | Topic rate drop, missing udev path, 1.5 s expiry | Strain relief, udev symlink by ID, fixed port map | Integration | 6 |
| R-08 | Rule non-compliance | 2 | 5 | 10 | Pre-run checklist | Size, autonomy, English and video checks | Team | 2 |

The top residuals are measurement gaps: a clamp meter and a measured slot would close both.

---

## 6. Verification matrix

| Req | Requirement | Test | Result | Evidence |
|---|---|---|---|---|
| V-01 | 0.39 m/s at 35 % duty | 1 s straight pulses | pass | `data/measurements.csv` rows 20 to 22 |
| V-02 | Steering inside 30 deg, 1000 to 2050 us | Sweep both limits, read the pulse | pass | `config.py`, `delta_to_us()` |
| V-03 | Radius at or below 0.40 m | 3 circle tests | pass, 0.37 m | `data/measurements.csv` rows 15 to 17 |
| V-04 | IMU at 50 Hz, bias removed | 4 s calibration, `ros2 topic hz` | pass | `data/measurements.csv` rows 40 to 42 |
| V-05 | Wall held at 0.18 m, error at or below 0.05 m | 3 laps with dL, dR logged | pending | run log under `data/` |
| V-06 | Stop within 0.35 s of LiDAR loss | Unplug the LiDAR on a straight | pending | `docs/06` |
| V-07 | Colour confirmed within 0.5 s | 10 approaches per colour | pending | video, notebook |
| V-08 | 3 laps counted in 10 runs | Field test against manual count | pending | `data/runs.csv` |
| V-09 | Park within 30 mm lateral and 5 deg | 10 manoeuvres | pending | `data/runs.csv` |
| V-10 | 3 laps plus parking inside round time | Stopwatch on a full run | pending | video |

Five are closed; five need a run log, recorded as a gap.

---

## 7. Budget

Prices are planning values, not receipts: none are archived. USD uses 3.50 PEN per USD, fixed 4 November 2026; the BCRP rate on the purchase date replaces it.

| Item | Qty | Unit (PEN, estimated) | Total (PEN) | Total (USD) |
|---|---|---|---|---|
| Raspberry Pi 5 | 1 | 380 | 380 | 109 |
| RRC Lite controller | 1 | 260 | 260 | 74 |
| LD19 / STL-19P LiDAR | 1 | 320 | 320 | 91 |
| LiDAR USB adapter | 1 | 45 | 45 | 13 |
| 15 kg.cm servo | 1 | 75 | 75 | 21 |
| 25 mm gear motor | 1 | 55 | 55 | 16 |
| Wheel protectors | 4 | 8 | 32 | 9 |
| Monocular camera | 1 | 70 | 70 | 20 |
| TB6612FNG driver | 1 | 20 | 20 | 6 |
| Jumper cables | 8 | 2 | 16 | 5 |
| USB cables | 3 | 12 | 36 | 10 |
| Li-Po 7.4 V 5000 mAh 20C | 1 | 190 | 190 | 54 |
| WS2812 16-LED ring | 1 | 18 | 18 | 5 |
| MDF plate and laser cut | 1 | 40 | 40 | 11 |
| Filament, PLA and PC | 0.6 kg | 90 /kg | 54 | 15 |
| Fasteners, bearings, spacers | 1 | 35 | 35 | 10 |
| **Total** | | | **1646** | **469** |

| Resource | Value | Status |
|---|---|---|
| Vehicle versions, part families | 11, 5 | counted |
| Iterations with a measured effect | 3 (v9 to v11) | counted |
| Printing hours | pending | slicer estimate per STL |

Printing hours are the one line with no data: slicer estimate per STL, summed into `data/measurements.csv`.

---

## 8. Post-mortem

**What worked.** The cascade split, LiDAR decides where and IMU decides how, made tuning separable: wall distance was fixed in the outer loop without touching the heading gains. The kit Ackermann linkage, once the servo was vertical, gave repeatable steering, and 0.37 m is enough for the corners. Lowering the LiDAR fixed wall detection more than any software change. The TB6612FNG turned a car that stalled below 50 % duty into one that cruises at 35 % in the Obstacle Challenge, where the validated speed band ends; the Open Challenge runs at 75 % and its speed is still `pending measurement`.

**What failed.** The documentation described a different car from the one that ran: PID with an integral, a 2DOF camera, an L298N, a 2000 mAh pack. Each was true once; none was updated when the part changed. Nothing was logged until 21 October, so eight versions have no measured effect. With no rig, every check needs the field and two people.

**What the next team should do first.** Instrument before designing. One CSV line per run with lap time, mean and worst lateral error, colour decisions, corner count and pack voltage turns every later argument into a comparison, and costs one afternoon. Second, a 1 m straight rig with a fixed wall so gains can be swept without booking a track. Third, write the paragraph for a part on the day it is installed.

---

## 9. Known limitations and the honest gap

| Gap | Why | Cost | How to close it |
|---|---|---|---|
| No logged lateral error | No run logger until late | V-05 cannot be claimed | Log dL, dR, time per run |
| Per-rail current pending | No clamp meter in build weeks | No peak figure | UT210E on both rails |
| Autonomy pending | No discharge test | No runtime margin | Discharge at race pace |
| Parking untested on a field | `parking.py` is new | The last manoeuvre is a design | Measure the slot, then 10 runs |
| Traction and stall torque pending | No incline or stall test | Braking and torque margins modelled | Incline and stall tests |
| Centre of gravity pending | No balance test | Rollover margin unknown | Balance test on blocks |
| 6 fps camera | USB camera limit | 0.065 m per frame | Slower near signs |
| Inverted camera mirrors cx | Mount is upside down and the frame is not rotated | Pillar nudge can steer toward the block | Rotate 180 deg before detection, or negate cx |
| Mass 1.39 kg, 0.10 kg | Pack not weighed | 7 % torque uncertainty | Weigh the pack |

Three numbers a judge asks first, lateral error, per-rail current and parking accuracy, are still `pending`. A perfect run needs three laps without wall contact, every sign read on the first approach, and a park inside 30 mm lateral and 5 deg. The design supports all three; the evidence supports speed, steering geometry, sensor rates and stop logic, and the rest only partly.

The reasoning is defensible; the measurement record for five requirements is not, and needs Section 6.
