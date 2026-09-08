# Architecture

## The one decision everything else follows from

**PX4 cannot do this project's obstacle avoidance.**

PX4 Collision Prevention runs only in Position mode with `MPC_POS_MODE` set to
acceleration-based. It is a pilot-assist velocity limiter and is *not* active in
Offboard mode — which is the mode an autonomous follower has to fly in.
Separately, `PX4-Avoidance` (local_planner / global_planner) was archived
read-only on 1 Aug 2024 and is ROS 1 Noetic only.

So avoidance lives in the trajectory the companion computer hands down. On
VOXL 2 that means `voxl-mapper`'s ESDF, reached by setting `voxl-vision-hub`'s
`offboard_mode` to `trajectory`.

Two consequences run through the whole codebase:

1. The follower emits a **goal**, never a path. `follow_core` computes where the
   vehicle should be; `voxl-mapper` owns how to get there without hitting
   anything. If no valid plan exists, the correct output is HOLD — never a
   straight line nobody checked.
2. `voxl-mapper` is shipped by ModalAI as a *reference* package, not production
   autonomy. It is the highest-risk dependency in the project, which is why
   Gate C (week 9) proves it standalone before any follow logic depends on it.

## Layering

```
             +-----------------------------+
  hardware   |  band firmware (ESP32)      |  gestures, hardware E-stop, UWB tag
             +-----------------------------+
                          | BLE / UWB TWR
             +-----------------------------+
  adapters   |  ros2_ws/  |  crazyflie/    |  ROS 2 nodes, cflib driver
             +-----------------------------+
                          | plain Python objects
             +-----------------------------+
  logic      |        follow_core          |  EKF, standoff, governor,
             +-----------------------------+  arbiter, supervisor
```

`follow_core` imports nothing from ROS, cflib, or any hardware library. That is
not tidiness — it is the schedule. The same tested code runs on the Crazyflie in
week 6 and on the Starling in week 12, so the control law, arbiter, governor and
supervisor are all validated in the mocap volume long before real perception
exists.

This directly targets the documented failure mode of comparable projects: one
MSc thesis reached its final follow-flight validation with a non-functional
tracker and had to feed the controller a *simulated* bounding box. Here the
mocap volume plays that role deliberately and early, via
`Follower.on_mocap_target()`.

## Estimator

State is the operator in the world frame: `[px, py, pz, vx, vy, vz]`, constant
velocity with acceleration noise. The vehicle moves, so a body-frame target
state would have to model the vehicle's own motion as target motion; in world
frame the process model is just a person walking and the vehicle pose enters
through the measurement model where it belongs.

| Sensor | Supplies | Rate | Survives occlusion |
|---|---|---|---|
| UWB two-way ranging | range + **identity** | ~5 Hz | yes |
| Vision detection | bearing (az/el) | 30 Hz | no |

The two fail in opposite places, which is the entire argument for carrying both.

### Three failure modes the simulation surfaced, and what was done about them

**1. Range-only updates are consistent with a wrong position.**
With vision gone, a range measurement constrains only the radial direction; the
tangential error is unobservable and grows with process noise. Crucially the
filter reports *no rejections* while this happens, so freshness stamps look
perfectly healthy. Fix: `max_position_std_m` — when the filter's own horizontal
1-sigma exceeds it, health becomes LOST regardless of the clock. Trust the
covariance over the timestamps.

**2. Gate-to-death spiral.** Once the state drifts far enough, every measurement
falls outside the chi-square gate, nothing updates, and because freshness stamps
only advance on *accepted* measurements the supervisor reports LOST while the
sensors are working perfectly. Fix: `divergence_reject_limit` consecutive
rejections inflates the covariance and deliberately reopens the gate.

**3. Follow bearing derived from instantaneous course.** Course steps
discontinuously when the operator turns a corner or reverses, teleporting the
goal to the far side of them; the vehicle then flies a chord straight past the
person. Fix: the commanded bearing is slew-limited to `speed_cap / standoff`,
the angular rate the vehicle can actually sustain at that radius.

## Camera geometry — a real constraint, not a detail

At the default 2.5 m standoff and 2.0 m height, the operator sits about **39°
below horizontal**. With a 50° vertical field of view and only a 15° camera
down-pitch they ride the very bottom edge of the frame, and any disturbance
drops the bearing measurement entirely. The simulation defaults to a **25°
down-pitch** for this reason.

Before flying, check this against the actual mount:

    required_pitch ≈ atan2(height_m, standoff_m) − FOV_V/2

Get it wrong and vision drops out constantly while every log looks nominal.

## Safety, in layers

Each layer is independent, so a bug in one does not defeat the others.

1. **Hardware E-stop** on the band, in an ISR, bypassing the gesture classifier
   entirely and latching until an explicit 2-second clear.
2. **Arbiter precedence** — E-stop > tablet > gesture > internal, with STOP
   latching until a command that *post-dates* it authorises resume.
3. **Speed governor** — cap falls with range to the operator; a commanded stop
   is immediate and never slew-limited.
4. **Hard standoff clamp** — the goal is pushed out to `min_standoff_m`
   regardless of what the control law asked for. A bug in the law cannot defeat
   this.
5. **PX4 failsafes** — geofence (Hold, with margin, because the fence is
   reactive), offboard loss, RC loss. See `docs/SAFETY.md`.

## Commands must be re-asserted

The arbiter expires any command older than `command_timeout_s` (default 2 s) and
falls back to HOLD. This is deliberate: a ground station or band that stops
talking must not leave the vehicle following. **Every command source needs a
heartbeat** — the tablet, the band, and the simulator all re-send at 2 Hz. A
consumer that sends FOLLOW once and stops will watch the vehicle hold, and that
is correct behaviour, not a bug.
