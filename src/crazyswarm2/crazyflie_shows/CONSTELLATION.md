# CONSTELLATION — a 5-drone, 78 s shape show on a beat

**Written 2026-09-20.** The second show in this package. `SHOW_GUIDE.md`
describes the first one (the carousel show) *and* everything rig-specific —
config, mocap address, the scan-every-address rule, the tracking margin. All
of that applies here unchanged and is **not** repeated. This file covers only
what is different about the constellation show.

**Status. Verified offline, flown end to end on `backend:=sim`, never on
hardware.** Every number below is measured in sim or on the ground. §5 is a
procedure to follow, not a report of something that worked on real drones.

---

## 0. The short version

```bash
./scripts/build.sh crazyflie_shows
source install/setup.bash

ros2 run crazyflie_shows plan_constellation          # no radio, no mocap, no drones
ros2 launch crazyflie_shows show_launch.py backend:=sim
ros2 run crazyflie_shows constellation_show --ros-args -p use_sim_time:=true

# hardware, after SHOW_GUIDE §5d's checklist and a fresh position sync:
ros2 run crazyflie_shows constellation_show --ros-args -p dry_run:=true   # rehearsal
ros2 run crazyflie_shows constellation_show
```

---

## 1. What it is, and why it is not the carousel show

The carousel show keeps one pentagon for 62 s and varies what the pentagon
*does*. This show changes the **shape itself** — pentagon, arrow, pyramid,
spiral staircase — and uses figures to animate whichever shape is standing.
The vocabulary comes from `reference/complex-shows-report.html`.

Three rules make the difference safe to fly:

1. **Plan view is the invariant.** Clearance is enforced on the horizontal
   projection (`report_extras`, 0.99 m against a 0.90 m floor), not just in
   3D. Height is decorative: no drone is ever above another, so a lost
   altitude estimate cannot turn into a collision, and downwash never stacks.
2. **Shape changes are assigned, not scripted.** `safety.assign_makespan`
   picks which drone takes which slot of the next shape by *bottleneck*
   distance (the longest single leg), preferring assignments with comfortable
   clearance over merely legal ones. Slots are not tied to drone names —
   re-syncing `initial_position` re-solves the whole show.
3. **The show is an arch, not a loop.** Measured against the real start
   positions, no one-way route through ring → arrow → pyramid → staircase
   keeps plan-view clearance on every leg: the staircase can only be entered
   from the pyramid and left back to it. So the second half retraces the
   first — which also makes the reversed turbine free in trajectory memory.

### The timeline

| bar.beat | t | dur | phase | what you see |
|---|---|---|---|---|
| 1.1 | 0.0 s | 2.8 s | takeoff | all five to 1.00 m over their own start marks |
| 2.3 | 3.0 s | 1.8 s | settle | hold while the Kalman filter settles |
| 3.3 | 5.0 s | 2.8 s | gather | converge into a regular pentagon, R = 1.10 m |
| 5.1 | 8.0 s | 7.8 s | **swashplate** | the ring spins while riding a tilted plane — it looks like a wobbling disc |
| 9.1 | 16.0 s | 2.8 s | morph → arrow | the ring folds into an arrowhead pointing 205° |
| 10.3 | 19.0 s | 2.8 s | **dart** | the whole arrow thrusts forward, rigidly |
| 12.1 | 22.0 s | 2.8 s | **recoil** | and draws back |
| 13.3 | 25.0 s | 2.8 s | morph → pyramid | four arms and a raised centre |
| 15.1 | 28.0 s | 6.8 s | **turbine** | the arms revolve around the still centre |
| 18.3 | 35.0 s | 4.8 s | morph → staircase | the widest single move of the show (1.30 m makespan) |
| 21.1 | 40.0 s | 9.8 s | **staircase** | a rising helix of five steps, turning one full circle |
| 26.1 | 50.0 s | 3.8 s | retrace → pyramid | the staircase leg run backwards — same paths, same clearance |
| 28.1 | 54.0 s | 6.8 s | **turbine (reversed)** | the same figure backwards; costs no extra memory |
| 31.3 | 61.0 s | 2.8 s | morph → pentagon | an intermediate chosen so the flight home is also safe |
| 33.1 | 64.0 s | 6.8 s | **starburst** | finale: fling out, alternately up and down, spinning |
| 36.3 | 71.0 s | 2.8 s | return home | back over each drone's own `initial_position` |
| 38.1 | 74.0 s | 3.8 s | land | 0.25 m/s descent, then disarm |

73.8 s of motion, **78.0 s wall clock, exactly 39 bars at 120 BPM.**

### Measured budgets (`plan_constellation`, five drones at the 2026-09-16 marks)

```
separation   1.00 m  min 0.90 m    90%    (3D; cf1/cf5 at t=68.6 s)
plan-view    0.99 m  min 0.90 m    90%    (cf3/cf8 at t=48.4 s)  <- the binding one
speed        1.81 m/s  max 2.00    90%
accel        2.39 m/s2 max 3.00    80%
radius       2.10 m  max 2.50 m    84%
height       1.45 m  max 2.00 m    73%
floor        0.60 m  min 0.30 m    50%
pieces         25     max 31       81%
downwash     2.26x the measured ellipsoid          (reported, not enforced)
```

**This show has 10 cm more separation margin than the carousel show** (1.00 m
vs 0.91 m against the same 0.90 m floor), because the plan-view rule forbids
the vertical stacking the counterflow figure relies on. Run
`plan_constellation` after every `sync_initial_positions.py` anyway — the
assignment is re-solved from the live marks, and it can fail loudly rather
than silently degrade.

---

## 2. What is new in the code

| file | what it adds |
|---|---|
| `constellation.py` | the whole plan: palette, shapes (`pyramid`/`arrow`/`staircase`), beat grid, `build_plan`, `report_extras`. Pure — no ROS, no radio |
| `constellation_show.py` | the ROS executor: beat-locked scheduling, light cues, staged abort |
| `safety.py` | `pairs_transit_min` (vectorised leg clearance), `assign_makespan`, `downwash_clearance` |
| `figures.py` | `plateau` easing, `swashplate_path` |
| `choreography.py` | `Phase.lights` and `Phase.slot` (both optional; `swarm_show` ignores them) |

`plan_show` now takes `--show {swarm,constellation}`; `plan_show`'s output for
the old show is byte-identical to before this work (regression-checked).

### Beat-locked scheduling

Each phase is commanded at an absolute time `t0 + Σ slots`, not "after the
previous sleep", so command latency cannot accumulate and a music track stays
in sync. A phase that starts more than 50 ms late says so on the console. The
plan is padded to a whole bar, and the pad is only ever added to a `goto`
phase — never to a figure, whose duration is fixed by its polynomial.

### Light cues

Sent on `colorLedBot.wrgb8888` **after** the motion command, because `setParam`
is fire-and-forget and a slow cue must not delay a drone. Cue failures are
caught and printed; they can never stop a show. On by default on hardware, off
under `use_sim_time` (`lights:=false` to disable). The palette avoids
green-dominant colours so a cue is never mistaken for the server's "connected"
green, and ends on red, the rig convention for "a script has the drones".

### The staged abort — and what it does *not* cover

If the script stops after arming — an exception, Ctrl-C, or the console's Stop
button — the drones are told to land where they are and are then disarmed,
rather than left flying the last figure.

**This took a fix worth knowing about.** `rclpy` installs its own SIGINT
handler inside `Crazyswarm()`, and it shuts the ROS context down *before* the
exception reaches the script — so the landing call failed with *"the given
context is not valid"* at exactly the moment it was needed. Verified failing
in sim on 2026-09-20, then fixed: `take_signals()` takes SIGINT and SIGTERM
back after `Crazyswarm()` has initialised, so the context is still alive when
the landing is commanded. A second Ctrl-C during the abort restores the
default handler and kills the process outright.

Measured in sim, interrupted mid-figure at 0.60–1.45 m: `landed and disarmed`,
all five at z ≈ 0, exit code 130. **The E-STOP remains the answer to a drone
out of control** (console, or `e` in the preflight GUI) — the staged abort is
the answer to a *script* that stopped, which is a different failure.

---

## 3. Parameters

```
dry_run          false   plan, report, exit. Never arms. The rehearsal
lights           true*   light cues (*default false under use_sim_time)
bpm              120.0   tempo of the beat grid — re-plans the whole show
check_placement  true    compare live pose to initial_position before arming
placement_tol    0.25    m, how far off a drone may be
scale            1.0     grow/shrink the whole show
arena_radius     2.5     m
ceiling          2.0     m
```

`bpm` is the interesting one: it retimes every phase to a new beat grid and
re-verifies. Slower is always safe; faster raises speed and accel, and
`build_plan` raises rather than flying an over-budget show.

---

## 4. Sim results (2026-09-20, `backend:=sim`, Humble)

- Full run: landed at **t+78.0 s** against a predicted 78.0 s, **0 late phases**.
- Flown minimum 3D separation **0.995 m** vs **0.997 m** planned — a 2 mm loss,
  against 49 mm for the carousel show's counterflow. Nothing in this show
  depends on two drones passing close.
- Tracking error 7.8 cm mean / 11.8 cm max, after fitting a 0.7 s alignment
  offset — consistent with the 0.10 m `TRACKING_MARGIN` the budgets assume.
- Abort: interrupted mid-turbine, landed and disarmed, exit 130.

Sim is not flight: it has no radio, no mocap latency and no wind. The numbers
above bound the *planning*, not the rig.

---

## 5. Before the first hardware flight

Everything in `SHOW_GUIDE.md` §5d applies unchanged — mocap address, `/poses`
alive, **scan every enabled address**, overlay check. On top of it, for this
show specifically:

1. `python3 scripts/sync_initial_positions.py` with the drones on their marks,
   then **restart the server** (the yaml is read only at launch).
2. `ros2 run crazyflie_shows plan_constellation` — it must print `PLAN OK`.
   If the assignment cannot be solved it raises; do not fly around it.
3. `ros2 run crazyflie_shows constellation_show --ros-args -p dry_run:=true`
   — same plan, through the real server, nothing armed.
4. Fly it once at `bpm:=100` (≈94 s, everything slower) before the 120 BPM run.
   Nothing about the geometry changes; only the time you have to react does.
5. Have the E-STOP in reach. The staircase is the first figure in this package
   where drones sit at genuinely different heights; watch the plan view, which
   is where the clearance actually lives.
