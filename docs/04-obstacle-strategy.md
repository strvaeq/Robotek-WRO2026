# 04 Obstacle Strategy

The Obstacle Challenge is three laps with randomly placed red and green pillars, then a parallel
park. Rules 5 and 9.19: a red pillar marks the side of the lane to keep on the right, a green pillar
the side on the left, and after three laps the car has to find the parking lot. The corridor between
borders is 1000 mm +-10 mm (rules 8a), the round lasts 3 minutes, and the pillars are 50 x 50 x 100 mm
(rules 13.19). Touching a marker of the lot ends the parking attempt (rules 9.24.7).

The strategy is not a waypoint follower. It has one continuous job (hold the followed wall at
0.18 m), one discrete job (turn 90 deg at each of the four corners) and two event jobs (take the side
from the last confirmed colour, park after the twelfth corner). Every constant below is in
`src/renkay_obstacle/renkay_obstacle/config.py`, cited by line.

| Requirement | Rule | Mechanism |
|---|---|---|
| Keep right of a red pillar | 9.19 | red sets `wall_side = right`, car hugs the right wall, pillar passes on its left |
| Keep left of a green pillar | 9.19 | green sets `wall_side = left` |
| Three laps | 5 | 12 corner events (`MAX_LAPS = 3`, `config.py:86`) |
| Park, parallel, fully inside | Appendix A.6 | `parking.py`, section 6 of [document 03](03-software-architecture.md) |
| No contact with walls or markers | 9.20, 9.24.7 | abort thresholds and 0.18 m standoff |

## 1. Colour to side

The camera finds the nearest block, takes its colour and its horizontal position. Only the colour
changes the side; the position only adds a transient nudge, so a mislocated block cannot invert the
rule.

| Colour | `wall_side` | Meaning in the corridor | Source |
|---|---|---|---|
| red | `right` | follow the right wall, pillar on the left | `control.py:39-45` |
| green | `left` | follow the left wall, pillar on the right | `control.py:39-45` |
| none | unchanged | keep the last confirmed side | `robot_node.py:189-190` |

The mapping is applied only when the colour has been confirmed for 3 consecutive frames
(`CONFIRM_FRAMES = 3`, `config.py:52`) and only when no turn is in progress (`robot_node.py:184`);
a colour seen mid-corner cannot change the wall the car is about to follow out of the corner. The
first side of a run is `right` (`robot_node.py:67`), so a car that never sees a pillar still completes
the lap on a valid, predictable line. There is no hysteresis beyond the 3-frame confirmation, because
the rule is absolute: once a pillar is passed, the side it dictates is the side for that section.

## 2. Pillar avoidance geometry

The car does not steer at the pillar. It keeps the wall and adds a temporary heading bias with the
sign that opens clearance to the block while the block is close:

```
nudge = clamp(-cx_norm * 14 deg, -12 deg, 12 deg)   if base_norm >= 0.5 else 0
```

`PILLAR_GAIN_DEG = 14`, `PILLAR_MAX_DEG = 12` (`config.py:50-51`), near threshold
`PILLAR_NEAR_BASE = 0.5` (`config.py:49`). `cx_norm` is -0.5 at the left edge of the image and +0.5 at
the right, so a block at positive `cx_norm` produces a negative nudge. These are image coordinates:
the inverted mount mirrors the horizontal axis ([document 03](03-software-architecture.md), section 4),
so the mapping from image side to car side is carried by the sign inside `pillar_nudge_deg`
(`-cx_norm`, `control.py:143`) and was fixed by driving the manoeuvre, not derived. The bias is added
to the wall-follow request and the sum is clamped to +-(7 + 12) = +-19 deg (`robot_node.py:232-233`).

The clearance requirement is geometric. To pass a 50 mm pillar without contact the car centre must
clear the pillar centre by 25 mm (half pillar) + 66.75 mm (half of the 133.5 mm front track) + 15 mm
of margin, 107 mm. At 0.39 m/s a 19 deg heading gives 0.127 m/s of lateral rate, so about 0.8 s of
travel builds that offset; the nudge starts at `base_norm = 0.5`, roughly 1 m before the block leaves
the frame, so the offset is reached before the closest approach. The bias is not a timed pulse: it
stays on while the block is near and the wall loop keeps pulling back to 0.18 m, which makes the
manoeuvre self-limiting. A timed pulse would need the distance to the pillar, and the camera has no
depth.

## 3. Wall following at 0.18 m

`TARGET_M = 0.18` (`config.py:40`). Three constraints set it:

* **LiDAR floor.** The two beams used for the wall fit leave at 60 and 30 deg from the front
  (`config.py:41-42`). At a perpendicular 0.18 m they travel 0.208 m and 0.360 m, both above the
  0.12 m self-return floor. At 0.10 m the 60 deg ray would be 0.115 m and be rejected.
* **Body clearance.** With a 66.75 mm half track the outer wheel sits 113 mm from the wall, well
  outside the 14.5 mm wheel width plus any yaw the follow loop allows.
* **Corridor margin.** In a 1000 mm corridor the far wall is 0.82 m away, so a 0.32 m lateral error
  still leaves 0.43 m between the outer wheel and the far wall while the outer loop corrects at
  0.471 rad/m.

Following at a constant distance rather than centring is deliberate. Centring in a 1000 mm corridor
puts the car at 0.50 m from each wall, which is outside the reliable fit range of the 30 deg beam for
a clean two-point line and leaves the pillar side undefined. A fixed 0.18 m wall gives one hard
reference and a repeatable lane, and it is what makes the pillar rule decidable at all: the car knows
which side is "its" side.

## 4. Corner detection and the 90 deg turn

A corner is four conditions in series (`control.py:171-180`):

| Condition | Value | Why |
|---|---|---|
| front wall closer than | 0.65 m (`config.py:73`) | 1.67 s of warning at 0.39 m/s |
| no turn in the last | 1.5 s (`config.py:74`) | stops a double trigger on the same corner |
| heading within | 15 deg of the straight (`config.py:75`) | only turn from a settled heading |
| one side open beyond | 1.20 m (`config.py:79`) | corridor is 1000 mm, so this is an opening |

The warning time is matched to the manoeuvre: a 90 deg arc at the 0.37 m minimum radius is 0.581 m,
1.49 s at cruise, so the corner is detected just before the turn-in point. The first corner fixes the
direction for the whole round, because the field has a single driving direction: if only the right
side is open the car turns right, if only the left side is open it turns left, and if both are open it
turns toward the longer one (`control.py:183-200`). From then on the base heading steps by
`direction * 90` at every corner (`control.py:197`), and the turn ends when the heading is within
4 deg of the new base and the yaw rate has fallen below 20 deg/s (`config.py:76-77`). The rate
condition matters: without it the car leaves the turn while still rotating and overshoots by the
amount the gyro had left to travel.

The motor does not slow for the corner in this challenge: cruise and turn are both 35 %
(`config.py:82-83`), 0.39 m/s. The 1000 mm corridor and the 0.37 m radius leave enough room at that
speed, and a speed change would add a deceleration transient exactly where the gyro is doing the most
work.

## 5. Re-anchoring and lap counting

Two events change the heading base. At every corner it steps by 90 deg (`control.py:197`). When the
followed wall changes side, the base is set to the measured heading and `theta_ref` is zeroed, so the
car does not inherit a heading offset that belonged to the other wall. Without that, a red to green
change leaves the car tracking a wall with up to 7 deg of wall-follow bias until the next corner.

Laps are counted by corner events, not by integrated yaw. Four completed corners are one lap, and
`MAX_LAPS = 3` (`config.py:86`) means the twelfth corner ends the racing phase. The old counter
derived the lap from `abs(theta)` (`control.py:218-230`), which drifts with the 0.34 deg/s bias and
counts up even if the car is pushed backwards. A corner event is already validated by the state
machine: it required a wall ahead, an opening on one side, a settled heading, and a completed 90 deg
rotation. The cooldown of 1.5 s prevents one physical corner from counting twice, which is the only
way a lap count can run ahead.

## 6. Return to lane and parking approach

The nudge and the side decision are independent. When the block leaves the near zone or expires
(1.5 s without a detection), only the nudge goes to zero; `wall_side` stays, because the rule it
implements is still in force for the rest of that section. The car therefore returns to the 0.18 m
line with the same wall, over about 2 m, the outer loop's time constant of 5.4 s at 0.39 m/s.

After the twelfth corner the car is in the starting straight, which is where the lot is placed
(rules 5). Parking starts there, not after an extra lap, and not on a timer. `APPROACH` creeps
forward at 12 percent (0.08 m/s) with the servo centred and watches the rear range: the slot is
beside the car once that range falls to 0.60 m. The bay side is chosen by `PARK_STEER_SIGN`
(default -1.0), a field calibrated constant that encodes the servo direction and the side the lot
is on. `ALIGN`, `REVERSE_ARC` and `STRAIGHTEN` are described in
[document 03](03-software-architecture.md), section 6; the acceptance test is the rule's own:
completely inside the parking area and parallel, with the final gap and yaw logged per run.

## 7. The full three-lap sequence

| Phase | Trigger | Car state | Heading base | Duty | Vision |
|---|---|---|---|---|---|
| start | button, 4 s calibration | `FOLLOWING` | 0 | 35 % | first colour |
| straight 1 | none | wall follow, nudge if near | 0 | 35 % | confirm side |
| corner 1 | front < 0.65 m, side open | `TURNING` | +-90 | 35 % | frozen |
| straight 2 to 4 | turn complete | wall follow | +-90, +-180 | 35 % | confirm side |
| corners 2 to 4 | as corner 1 | `TURNING` | steps of 90 | 35 % | frozen |
| laps 2 and 3 | corner count 4, 8 | repeat | 360, 720 | 35 % | repeat |
| lap 3 end | corner 12 | `PARKING` | 1080 | 0 to 12 % | none |
| park | rear <= 0.60 m | `APPROACH` to `DONE` | lot heading | 0 to 25 % | none |
| stop | acceptance, or abort | `DONE` / `ABORT` | - | 0 % | - |

The heading base is not normalised, so after three laps it reads 1080 deg; the inner loop works on
the error to that base and the wrap is handled by `wrap180` where a relative angle is needed
(`robot_node.py:207`).

## 8. Decision table

| Measurement | Decision | Why |
|---|---|---|
| red confirmed 3 frames, not turning | follow the right wall | rule 9.19 |
| green confirmed 3 frames, not turning | follow the left wall | rule 9.19 |
| no colour yet | follow the right wall | deterministic start, `robot_node.py:67` |
| colour seen mid-turn | keep it, switch after the turn | a side change would fight the 90 deg base |
| block `base_norm` >= 0.5 | nudge, max 12 deg | clearance without depth |
| block lost for 1.5 s | nudge 0, side unchanged | block has passed or camera stalled |
| wall fit valid | follow it | 2-point fit inside 0.12 to 0.90 m |
| wall fit rejected | hold the last `theta_ref` for one scan | better than a value from a pillar |
| front < 0.65 m and a side > 1.20 m | turn 90 deg | corner |
| both sides open | turn toward the longer side | first corner only, direction is then fixed |
| 12 corner events | stop racing, start parking | 3 laps |
| turn not complete in 3.3 s | stop the car | something is blocking the arc |

## 9. Failure recovery

| Failure | Symptom | Recovery | Residual risk |
|---|---|---|---|
| missed pillar | car stays on the previous side | none: the next pillar re-decides | a missed pillar means a wrong-side pass, a round penalty |
| false colour | side flips without a pillar | 3-frame confirmation makes this a 500 ms event, not a frame | a large red object in the background |
| pillar read as wall | heading request jumps | angle and ratio gates reject the pair | one scan of held heading |
| blocked corner | turn timeout | stop, do not continue blind | the round is lost, the car is not damaged |
| wall lost | no valid fit | hold the request, keep driving on the gyro | drift until the wall returns, 0.34 deg/s |
| LiDAR or IMU lost | watchdog | latch stop until an explicit reset | the round is lost |
| marker contact in parking | front clearance < 0.10 m | stop, no retry after the retry limit | no parking points |

The strategy is deliberately conservative at the two points where a wrong decision cannot be undone:
the side decision needs 3 frames, and a turn that does not finish stops the car instead of guessing.
Everywhere else the car keeps driving on the gyro and re-acquires the wall, because a stopped car
scores nothing while a car on the wrong line still completes the lap.

## 10. What is not measured yet

The nudge timing assumes the block reaches `base_norm = 0.5` about 1 m before the closest approach;
the camera has no depth, so that distance is an estimate from the 6 fps geometry, not a measurement.
The parking lateral target is 0.03 m. A 200 mm lot with a 133.5 mm track leaves 33.25 mm per side,
so the final setpoint keeps 3 mm of margin; the field run validates it, and document 03 derives it. The lap and lateral error figures that would validate the
whole strategy are not logged yet: `data/test-runs.csv` has a header and no rows.
