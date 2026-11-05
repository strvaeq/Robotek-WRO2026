# 01. Mobility and Mechanical Design

Mechanical design of Renkay, vehicle version 11: rule constraints, measured geometry, steering geometry, drivetrain sizing, traction, manufactured parts, assembly, and the tests that changed the design. Every value is measured, derived, or marked `pending measurement`.

Sources: `[M]` is a row of `data/measurements.csv`, `[R:line]` a line of `README.md`, `[MOD:folder]` a model README, `[RULE:x.y]` a WRO 2026 rule.

## 1. Requirements and constraints

| Constraint | Value | Rule | Consequence |
|---|---|---|---|
| Envelope | 300 x 200 mm | 9.17, 11.1 | Plate is 189 x 90 mm [M] |
| Mass | 1.5 kg max | 11.2 | 1.39 kg, 0.11 kg margin [M] |
| Steering | One actuator, one driving axle, 4 wheels | 11.3 | No two-motor skid steer |
| Differential drive | Forbidden, including one motor per side | 11.3, 11.5 | Forces a linkage |
| Omni wheels, casters | Forbidden | 11.4 | Scrub cannot be hidden |
| Drive motors | Max 2, coupled to the axle | 11.13 | One motor, one gear pair, one solid axle |
| Corridor | 1000 or 600 mm, +/- 100 mm | Ch. 8 | Sets maximum corner sweep |
| Parking lot | 200 mm wide, 1.5 x car length | Ch. 5 | 25 mm per side for a 150 mm car |
| Walls | 100 mm high | 13.3, 13.5 | Sets scan height budget |

The mass budget drives everything below: 1.11 kg modelled from URDF link masses plus a 0.28 kg pack estimate gives 1.39 kg [M], so the margin is 0.11 +/- 0.10 kg, 93 % of the limit.

## 2. Geometry

| Parameter | Symbol | Value | Unit | Source |
|---|---|---|---|---|
| Wheelbase | L | 117.4 +/- 0.2 | mm | caliper, 21 Oct 2026 [M] |
| Front track, outer to outer | t_f | 133.5 +/- 0.2 | mm | caliper [M] |
| Rear track, outer to outer | t_r | 123.5 +/- 0.2 | mm | caliper [M] |
| Track at steering centres | t | 119.0 +/- 0.3 | mm | t_f - wheel width [M] |
| Wheel diameter | D | 61.0 +/- 0.1 | mm | caliper, 3 readings per wheel [M] |
| Wheel width | b | 14.5 +/- 0.1 | mm | caliper [M] |
| Chassis plate | | 189 x 90 | mm | caliper [M] |
| Plate top height | h_p | 45.0 +/- 0.5 | mm | caliper [M] |
| Ground clearance | h_g | pending measurement | mm | not measured [M] |
| CG height | h_cg | pending measurement | mm | balance test not run [M] |
| LiDAR axis ahead of base_link | | 128 | mm | URDF |
| LiDAR scan plane above ground | | 30 | mm | URDF |
| Wheelbase / front track | L/t_f | 0.88 | - | derived |

The 133.5 mm front track is outer to outer, which is how a judge with a caliper checks the 200 mm width limit; it is not the track used in steering. The steering-centre track is 133.5 - 14.5 = 119.0 mm, and using 133.5 mm would overstate the required steering differential by 12 %. The front track is 10.0 mm wider than the rear so the hubs clear the plate, which makes the front outer wheel the widest point of the car.

## 3. Ackermann geometry

### 3.1 Formula

A car turning without tyre slip rotates about a point on the extension of the rear axle, so the front wheels run on different radii:

```
cot(delta_outer) - cot(delta_inner) = t / L
delta_inner = atan( L / (R - t/2) )
delta_outer = atan( L / (R + t/2) )
R = L / tan(delta)
```

### 3.2 Numbers at the measured minimum radius

The circle test of 8 Oct 2026 measured the inner wheel at 19.7 deg turning left and 17.6 deg turning right at a 25 deg command [M]. The tighter case gives

```
R_min = L / tan(17.6) = 117.4 / 0.3172 = 370 mm
```

so `R_min = 0.37 +/- 0.03 m` [M]. The left side gives `117.4 / tan(19.7) = 328 mm`: the car turns 11 % tighter left than right, a linkage asymmetry treated in section 10.

| Quantity | Calculation | Result |
|---|---|---|
| Inner angle required | atan(117.4 / (370 - 59.5)) | 20.7 deg |
| Outer angle required | atan(117.4 / (370 + 59.5)) | 15.3 deg |
| Differential required | 20.7 - 15.3 | 5.4 deg |
| Ackermann check | cot(15.3) - cot(20.7) | 1.01 |
| t / L | 119.0 / 117.4 | 1.014 |
| Agreement | | 0.5 % |

The linkage delivers 19.7 deg against 20.7 deg required on the left (95 %) and 17.6 deg on the right (85 %). The outer wheel angle was not recorded, so the delivered differential is `pending measurement`.

### 3.3 Does the radius suit the field

The outer front wheel centre runs on `R + t/2 = 430 mm`, giving a 505 mm swept band and 95 mm of margin in the 600 mm corridor. The parking lot is the opposite case: `1.5 x 0.19 = 0.285 m`, smaller than the 0.37 m radius, so parking must be a two-arc reverse manoeuvre.

### 3.4 Why Ackermann and not skid steer

Rules 11.3 and 11.5 forbid differential drive and 11.4 forbids omni wheels and casters, so skid steer is a disqualification before any performance argument. The engineering reasons agree: rule 9.20 allows touching a pillar only inside an 85 mm circle around its seat, and a skid-steer car scrubs laterally by an amount that depends on surface and load; rear encoder rate is a valid speed signal only if the wheels roll without slip; and the rear wheels of an Ackermann car track inside the front wheels, so the swept band in the 600 mm corridor is the front outer path only. The cost is four pivot joints, a linkage and a servo in a 90 mm plate.

## 4. Steering system

| Item | Value | Source |
|---|---|---|
| Actuator | 15 kg.cm servo, 5 V rail from RRC Lite | BOM [R:662] |
| Torque at the horn | 1.47 N.m declared | BOM [R:662] |
| Command range | +/-30 deg, software clamp | [M], `DELTA_MAX_DEG` |
| Pulse centre, limits | 1500 us, 1000 to 2050 us | `config.py` |
| Gains | 18.33 us/deg positive, 16.67 us/deg negative | `config.py` |
| Slew | 70 deg/s | step response, 8 Oct 2026 [M] |
| Reaction delay | 60 ms | [M] |
| Linkage | HiWonder set, bearings at both ends | [R:443] |

Slew matters more than torque. A 30 deg swing takes `30/70 = 0.43 s`; at 0.39 m/s and `R = 0.37 m` the yaw rate is `v/R = 1.05 rad/s = 60 deg/s`, so a 90 deg corner takes 1.5 s and the servo needs 29 % of that time. Servo torque margin is `pending measurement`: kingpin inclination, scrub radius and horn-to-wheel ratio are not recorded.

### 4.1 Four iterations

| # | Problem | Change | Result |
|---|---|---|---|
| 1 | Printed gear and rack: imprecise teeth, backlash | Enlarged the teeth, then abandoned the concept | Too big for the plate; replaced by the HiWonder linkage [R:441-443] |
| 2 | Linkage turned less left than right, hurting corner exit | Servo was horizontal and its arm ran out of travel | 19.7 vs 17.6 deg: reduced, not removed [M] |
| 3 | Horizontal servo cost height and limited swing | Servo vertical, output down, free linkage travel | Wider angle, shorter chassis, space for the battery housing [R:452], [R:895] |
| 4 | Rack collided with the servo in version 8 | Ackermann moved a few millimetres; wheel grooves, printed cylinders | 20 s per lap in version 9, under 10 s in version 10 [R:282], [R:294] |

Iteration 3 allowed the angle behind the 0.37 m radius; iteration 4 removed a hard failure. The 11 % radius asymmetry remains, mitigated by asymmetric gains and an outer-wheel limit taken from the weaker side.

## 5. Drivetrain

The car is **rear-wheel drive with front-wheel steering**: the motor drives the rear axle through a gear pair, and the front axle carries only the steering linkage. Rule 11.3 requires one of the three drive layouts and rule 9.8 requires declaring the front axle, which carries the servo and the end of the car with the LiDAR (128 mm) and camera (56.5 mm ahead of base_link).

| Item | Value | Source |
|---|---|---|
| Motor | 25 mm metal gear DC motor with encoder | BOM [R:663] |
| Gear train | Large gear on the motor shaft drives a smaller gear on the axle | [R:489-491] |
| Gear ratio | pending measurement | tooth counts not recorded |
| Driver | TB6612FNG, 1 kHz PWM | BOM, [BRIEF] |
| Pins | GPIO chip 4, PWM A 12, AIN1 5, AIN2 6, STBY 13 | [BRIEF] |
| Encoder scale | 4.8 rps per m/s, `+rps` drives backward | [BRIEF] |
| Speed constant | v = 0.0137 x (pct - 6.5) m/s at 7.3 V | 1 s pulses [M] |
| Deadband | 6.5 % | first movement [M] |
| Top speed | 0.42 m/s at 50 % duty | [M] |
| Accel / decel | 1.0 / 0.6 m/s^2 | [M] |

The gear pair is a speed increase: version 10 swapped the gears and took the lap from 20 s to under 10 s [R:294]. The trade is start-up torque, paid for in the driver: with the L298N the car stopped at 50 % duty or below, and the TB6612FNG removed that floor so the car holds torque from 35 % [R:504-507].

### 5.1 Speed

| Duty | Model v = 0.0137 x (pct - 6.5) | Measured | Note |
|---|---|---|---|
| 6.5 % | 0.00 m/s | 0.00 m/s | deadband [M] |
| 25 % | 0.25 m/s | not measured | parking reverse cap |
| 35 % | 0.39 m/s | 0.39 m/s | lowest usable cruise [M] |
| 50 % | 0.60 m/s | 0.42 m/s | top speed [M] |

The 35 % figure is the Obstacle Challenge cruise. The Open Challenge is configured at 75 % duty, above the band where the speed constant was fitted, so its speed is not quoted here and is listed as `pending measurement`. The model is used only between 15 % and 35 %, where it was fitted.

Wheel speed at 35 % is `0.39 / (pi x 0.061) x 60 = 122 rpm`; at the record, 131 rpm.

The model predicts 0.60 m/s at 50 % against 0.42 m/s measured, 30 % lower, because the constant was fitted on 1 s pulses in the 15 to 35 % band and is not extrapolated in the code. Separately, 4.8 rps per m/s implies 66.3 mm effective diameter against 61.0 mm measured; that 8.7 % offset is `pending measurement`, so the speed loop uses the encoder scale.

### 5.2 Torque

```
T = m x a x R_wheel = 1.39 x 1.0 x 0.0305 = 0.042 N.m at the axle
```

with `T_req = 0.042 +/- 0.004 N.m` [M]. Both rear wheels come off one solid axle, so each transmits 0.021 N.m: 0.69 N of thrust per wheel and 1.38 N total, matching `m x a`.

| Check | Value | Meaning |
|---|---|---|
| Thrust for 1.0 m/s^2 | 1.39 N | 10 % of the car's weight |
| Torque per driven wheel | 0.021 N.m | Wheel radius is 30.5 mm |
| Axle speed at cruise | 12.8 rad/s | 122 rpm |
| Axle power during the ramp | 0.54 W | Upper bound |
| Motor shaft torque | 0.042 / (i x eta) | `pending measurement` |
| Stall torque | `pending measurement` | stall test not run [M] |

The 0.54 W bound is the useful result: the drivetrain is limited by traction, low-duty motor behaviour and the 6.5 % deadband, not by power. Stall torque must not be quoted before it is measured.

## 6. Traction and stability

Total weight is `1.39 x 9.81 = 13.6 N`. The load split has not been measured, so the friction requirement is bounded:

| Mass on driven axle | Normal load | Required mu for 1.0 m/s^2 |
|---|---|---|
| 50 % | 6.8 N | 0.20 |
| 40 % | 5.4 N | 0.26 |
| 25 % | 3.4 N | 0.41 |
| 20 % | 2.7 N | 0.51 |

The car needs `mu >= 0.20` at best and 0.51 in a bad case; a rubber band on a printed mat is well above 0.5, which is why the traction problem appeared as stalling at low duty, not wheelspin. `mu` is `pending measurement`: on an adjustable board with the driven axle downhill, raise the angle until the wheels slip at 35 % duty, and report `mu = tan(angle)` (10 deg = 0.18, 20 deg = 0.36, 30 deg = 0.58). Lateral demand at the radius is `0.39^2/0.37 = 0.41 m/s^2`, needing `mu >= 0.042`, so grip is not the constraint; reaching the angle in time is.

### 6.1 Why the battery sits where it sits

The pack sits in the compartment under the Pi and RRC Lite in the rear housing, behind the servo and over the rear axle area [MOD:raspberry_housing], [R:895]. At 0.28 kg of 1.39 kg (20 %) it is the heaviest item and the only free way to load the driven axle; it sits at plate level rather than above it, which is the mechanism intended to keep `h_cg` low, and it balances the LiDAR and camera at the front.

### 6.2 Stability margins

| Check | Value | Verdict |
|---|---|---|
| Wheelbase / track | 0.88 | Quick rotation, acceptable at 0.42 m/s |
| Car in a 200 mm slot | 150 mm | 25 mm per side, tightest in the rules |
| Braking at cruise | 0.39^2/1.2 = 0.127 m | Exceeds the 0.10 m parking abort |
| Braking at 25 % duty | 0.25^2/1.2 = 0.053 m | Fits with 47 mm to spare |

Parking aborts below 0.10 m of clearance, and the car needs 127 mm to stop from cruise but 53 mm at the 25 % reverse cap, so the 25 % figure is a mechanical consequence.

## 7. Printed and laser-cut parts

| Part | Process, material | Layer | Infill | Mass | Function | Source |
|---|---|---|---|---|---|---|
| Base plate | Laser cut MDF, thickness `pending` | n/a | n/a | `pending` | Servo, motor mount, axle supports, housings | [MOD:vehicle_base] |
| Wheels, 4 off | FDM PLA | 0.20 mm | 25 % | `pending` | 61.0 x 14.5 mm, tyre groove | [MOD:vehicle_wheels] |
| Raspberry and battery housing | FDM PLA | 0.16 mm | 20 % | `pending` | Pi 5, RRC Lite, pack, driver over the servo | [MOD:raspberry_housing] |
| Camera housing | FDM PLA | 0.16 mm | 25 % | `pending` | Fixed forward camera, inclined | [MOD:camara_housing] |
| LiDAR housing | FDM PLA | 0.24 mm | 25 % | `pending` | Low front mount, scan at 30 mm | [MOD:lidar_housing] |
| Axle spacers and wheel cylinders | FDM PLA | `pending` | `pending` | `pending` | Remove drivetrain play, keep screws off the tyre | [R:497], [R:282] |
| Tyre bands, 4 off | Purchased rubber | n/a | n/a | `pending` | Friction surface | [R:664] |

The plate is laser cut rather than printed because a 189 x 90 mm PLA plate warps and loses hole positions; it was prototyped in cardboard and acrylic before being cut from Fusion 360. The wheels are printed because HiWonder wheels were wider than the parking slot tolerates, and printed without a friction border the car could not move.

## 8. Assembly sequence

| Step | Action | Check |
|---|---|---|
| 1 | Fixed Ackermann bracket to plate | Square within 0.5 mm |
| 2 | Steering arms with bearings, hole nearest the wheel | Free rotation, no axial play |
| 3 | Long linkage between the arms | No binding at either extreme |
| 4 | Short linkage to arm and horn, servo vertical, output down | 1000 to 2050 us hits both stops |
| 5 | Hubs to the arms | Wheel plane vertical within 1 deg |
| 6 | Motor mount and motor | Mesh backlash 0.1 to 0.2 mm |
| 7 | Axle, two bearings, two locking rings | End float under 0.1 mm |
| 8 | Motor to axle, then spacers | Spacers clear the motor screws |
| 9 | Housings: LiDAR front, camera behind, battery and Pi rear | LiDAR clear through 360 deg |
| 10 | Electronics and tyre bands | Circumference equal within 0.5 mm |

No torque driver is available, so these are design intent verified by behaviour:

| Joint | Fastener | Design intent | Verification |
|---|---|---|---|
| Steering pivots | M3 and bearing | Snug plus 1/8 turn | Arm falls under its own weight |
| Linkage bearings | M3 shoulder screw | No preload on the races | No perceptible play |
| Motor mount | M3 into PLA | 0.3 N.m | No movement after a 30 s run |
| Axle locking rings | Grub screw | Contact plus 1/4 turn | No axial movement by hand |
| Housing to plate | M3 into MDF | 0.2 N.m | Does not rock when lifted |

Strip torque for M3 in PLA and MDF is `pending measurement`; 0.3 and 0.2 N.m are design intent, not test results. Bearings are hand-pressed, the mesh is set by feel and no bore dimension is recorded. To rebuild, buy the motor, servo, TB6612FNG, RRC Lite, Pi 5, STL-19P with adapter, camera, 7.4 V 5000 mAh 20C pack, four tyre bands and the HiWonder linkage set [R:656-670], and make the eight parts in section 7.

## 9. Test log

| Test | Setup | Result | Decision |
|---|---|---|---|
| Circle test, 8 Oct 2026 | 25 deg command, LiDAR odometry plus gyro | 19.7 deg left, 17.6 deg right; radius 0.370 and 0.328 m | Fits the 600 mm corridor; asymmetry compensated in gains |
| Speed calibration, 8 Oct 2026 | 1 s pulses, 7.3 V | 0.0137 m/s per %, deadband 6.5 %, 0.39 m/s at 35 % | Cruise set from measurement, model kept to 15 to 35 % |
| Top speed, 8 Oct 2026 | 50 % duty, straight | 0.42 m/s | 70 % cap, because 6 fps vision needs time |
| Straight-line drift | 3 m, steering centred | `pending measurement` | Drift above 30 mm would justify a trim |
| Hill start | Inclined board, 35 % duty | `pending measurement`; the field is level, so this is robustness | Would confirm `mu` and start-up margin |
| 3-lap endurance | Full field, version 10 | 28 s for 3 laps, under 10 s per lap, not instrumented [R:294] | Gear swap kept; lap time `pending measurement` [M] |
| Low-duty behaviour | Reduce duty until stall | Stalled below 50 % with L298N, holds from 35 % with TB6612FNG [R:504-507] | Driver replaced |
| Wheel friction | Wheels with and without the border | Without it the car could not move | Border adopted, four bands in the BOM |
| Parking abort margin | From the 0.6 m/s^2 limit | 0.127 m at cruise, 0.053 m at 25 % | Reverse capped at 25 % |

## 10. Known limitations and next iteration

1. **Steering asymmetry, 11 %.** 0.328 m left, 0.370 m right, cause unmeasured because the arm lengths were never recorded; next, record them and re-cut the linkage to match within 1 deg.
2. **Stall torque and gear ratio unknown**, so only the requirement is computable; next, a spring-scale stall test and a tooth count.
3. **No bore, axle, plate thickness or part mass recorded**, so a second team cannot reproduce the fit and the budget carries 0.10 kg of uncertainty; next, dimension the bores and export STEP beside the STL.
4. **`mu` and CG height not measured**, so the friction and battery arguments are bounds and intent; an incline test and a balance test close both.
5. **Speed model invalid above 35 %** and the encoder scale disagrees with the wheel diameter by 8.7 %; next, roll the car 3 m by hand and re-fit above 40 % duty.
6. **No CAD of the drivetrain and no compliance in the front axle**, so a rebuild starts from photographs and kerb strikes reach a rigid linkage at the widest point of the car.

The geometry fits the 600 mm corridor with 95 mm of margin, 0.042 N.m at the axle is inside what the motor can deliver, and traction needs `mu >= 0.20` at an even load split. The measurement set that turns bounds into margins is still missing; items 2 and 4 close most of it.
