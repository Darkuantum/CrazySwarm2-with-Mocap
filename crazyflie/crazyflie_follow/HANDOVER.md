# Handover — crazyflie_follow

## State

Written, compiles, **never flown**. Not built against `crazyflie_py` either,
because that lives in the crazyswarm2 workspace and not here.

| Piece | State |
|---|---|
| `follow_core` integration | logic covered by 72 tests at the repo root |
| `safety.py` offline proofs | pass for the shipped defaults |
| `SetpointGuard` | 6 tests, including two that fail without the post-slew clamp |
| `follow_node` | compiles; never run against a real `Crazyswarm` object |
| `mocap_target` | compiles; never run against a real `/poses` stream |
| `keyboard_cmd` | compiles; never exercised on a terminal |

## First session — do it in this order

1. `ros2 run crazyflie_follow check_safety`. No drone, no ROS graph. If the
   offline proof fails, the config is wrong and flying it will not help.
2. `backend:=sim`. Watch the goal track the operator body in RViz. Do not skip
   this because sim "isn't the real thing" — every bug found here is one not
   found at 1.5 m from someone's face.
3. Real flight, **operator standing still**, standoff only. Confirm the drone
   holds station and that E-stop works before anyone walks anywhere.
4. Operator walks slowly in a straight line.
5. Operator turns.

## Things to check before believing anything

- **Weigh the standoff.** `min_standoff_m` is 1.5 m by default. Decide whether
  that is right for your room and your comfort, and set it before flight rather
  than during.
- **Does the operator body ever drop out of Motive?** It will, at the edges of
  the volume. `mocap_target` tolerates a few missing frames; confirm what
  happens on a long dropout before you find out by accident.
- **Yaw.** The Crazyflie must yaw to face the operator. Confirm the mocap yaw
  convention matches what `follow_core` expects, in sim, with a printout.

## Known gaps

- The 8 x 8 m boundary and `ROOM_CENTER` (0.0467, -0.1037) are inherited from
  the parent repo. **Measure your own room.**
- No orbit-direction control; `ORBIT` always goes one way.
- `keyboard_cmd` uses a non-blocking terminal read, not curses. It will behave
  oddly inside `tmux` with mouse mode on.
- Battery state is not checked by this node. The parent repo's preflight GUI is
  still the go/no-go authority.

## Where to look when it misbehaves

`follow_node` logs a reason for every mode change and every clamp it applies.
If the drone is not moving, read those first — `HOLD` with reason
`target lost` and `HOLD` with reason `cmd:internal` (a stale command) look
identical from outside and mean completely different things.
