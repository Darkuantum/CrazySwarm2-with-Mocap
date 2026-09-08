# crazyflie_follow

Person-following on the Crazyflie, for the prototyping phase of the follower
drone project. Drop-in package for the vendored crazyswarm2 + OptiTrack rig.

## Why this phase exists

The point is **not** to build a Crazyflie product. It is to validate the follow
control law, command arbiter, speed governor and track supervisor in the mocap
volume *before* any real perception exists on the Starling — with OptiTrack
playing the part of a perfect target sensor.

That targets a documented failure mode: a comparable MSc thesis reached its
final follow-flight validation with a non-functional tracker and had to feed the
controller a *simulated* bounding box. Here the substitution is deliberate,
early, and scheduled.

All the logic is in `follow_core/` at the repo root — the same tested code that
later flies on the Starling. This package is the adapter.

## Install

```bash
cp -r crazyflie_follow ~/CrazySwarm2-with-Mocap/src/
cd ~/CrazySwarm2-with-Mocap
colcon build --packages-select crazyflie_follow
source install/setup.bash
```

`follow_core` must be importable — add the repo root to `PYTHONPATH`, or
`pip install -e` it.

## Motive setup

You need **two** rigid bodies streaming on `/poses`
(`motion_capture_tracking_interfaces/msg/NamedPoseArray`, 50 Hz):

| Rigid body | Default name | Notes |
|---|---|---|
| The aircraft | `cf1` | must match `crazyflies.yaml` |
| The operator | `operator` | markers on a helmet or a hard hat — **not** loose on clothing |

Set both names in `config/follow.yaml`. Put the helmet markers in an asymmetric
pattern so Motive cannot flip the body's orientation mid-flight.

The parent repo documents a known upstream bug where `motion_capture_tracking`
ignores the `interface_ip` parameter — see its CLAUDE.md. That is a prerequisite
of the rig, not something this package fixes.

## Run

```bash
# 1. ALWAYS prove it in sim first
ros2 launch crazyflie launch.py backend:=sim
ros2 launch crazyflie_follow follow_launch.py

# 2. Offline safety proof — no drone, no ROS graph needed
ros2 run crazyflie_follow check_safety

# 3. Commands, from a second terminal
ros2 run crazyflie_follow keyboard_cmd
#    f = FOLLOW   s = STOP/HOLD   o = ORBIT   l = LAND
#    space = E-STOP   c = clear E-stop
```

## Safety

This rig has **no onboard collision avoidance**, and the whole point of the
exercise is to fly near a person. Every limit is enforced in `follow_node`, not
assumed:

- 8 x 8 m soft boundary around `ROOM_CENTER`, plus an altitude ceiling and floor
- 1.5 m minimum standoff from the operator, enforced **twice**: once by
  `follow_core`'s governor, and again by `SetpointGuard` in `safety.py` from
  this node's own reading of the operator position
- hard velocity, acceleration and yaw-rate caps, so a backup pilot can take over
- `safety.py` proves the geometry offline, before anything arms

`SetpointGuard` re-asserts the standoff clamp *after* the speed and accel slew.
That ordering is load-bearing: the slew interpolates back toward the previous
setpoint, so clamping only before it silently does nothing whenever the operator
walks toward a drone holding station. See
`tests/test_setpoint_guard.py` at the repo root — remove the post-slew clamp and
two tests fail immediately.

## Abort

1. **Space** on `keyboard_cmd` — E-stop, latches until `c`.
2. The parent repo's preflight GUI has a broadcast e-stop on `e`.
3. Backup pilot takes manual control.
4. Kill the node. The Crazyflie will hold its last commanded setpoint, so this
   is the *worst* option — use 1–3 first.

Commands must be re-asserted; the arbiter expires anything older than 2 s and
falls back to HOLD. `keyboard_cmd` heartbeats for you.
