# CONSTELLATION — a 5-drone, 76 s shape show on a beat

**Written 2026-09-20. Refitted to the measured arena 2026-10-02** (§1a): the
figures are now built at the centre of the *tracked volume*, and the staircase
switches back instead of running out towards the wall.

The second show in this package. `SHOW_GUIDE.md`
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
   projection (`report_extras`, 1.00 m against a 0.90 m floor), not just in
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

### 1a. Fitting the volume that was actually measured

On 2026-10-01 the room was measured three ways (carried surveys, a flown
spiral, a flown climb — `safety.ARENA_RADIUS_TESTED` carries the provenance).
The arena the shows had inherited, 2.5 m, is **not** this room: a flying drone
lost tracking at **2.24 m**, and 2.00 m is the largest radius flown clean at
both 1.20 m and 1.95 m altitude. Two things followed for this show.

**The envelope is now measured from the volume's centre, not the formation's.**
`check_show` used to measure radius from the centroid of the drones' own start
marks, which cannot see the dangerous case: every figure fitting a small
circle, the whole circle sitting off to one side, one arm reaching into the
corner the cameras do not cover. It now measures from `safety.ARENA_CENTRE`
and the show builds its figures there (`room_center`), so the marks only have
to be *reachable* — and a drone parked outside the volume is rejected by name,
because that is a chalk line to move, not a figure to shrink.

**The staircase was the figure that reached the wall.** A straight line of
`n` drones has to be `(n-1) × 0.90 m` long, so it spent ±1.80 m of radius at
five drones and grew with every drone added — 2.62 m at six, past where
tracking was lost. It now **switches back**: alternate steps step sideways by
0.70 m, so the clearance sits on the diagonal (`hypot(0.72, 0.70) = 1.00 m`)
and the footprint does not. Measured over n = 4…6, worst footprint fell from
**2.62 m to 1.43 m**, and the figure reads better — a switchback stair rather
than a ramp seen edge-on.

| | before | after |
|---|---|---|
| worst radius, 5 drones | 2.10 m | **1.53 m** |
| worst radius, 4 drones | 1.64 m | **1.51 m** |
| staircase footprint, 6 drones | 2.62 m | 1.43 m (arrow still refuses — see below) |
| checked against | 2.5 m, from the mark centroid | **1.90 m, from the measured centre** |
| spare to where tracking was lost | 0.14 m | **0.73 m** |

The dart was also shortened, 0.60 → 0.50 m, which bought 0.10 m of radius for
no visible loss. **Six drones does not fit this room** and is refused: the
arrow needs 1.98 m, and shrinking it enough to fit breaks the morph into it.
That is a room limit, not a bug to tune around.

### The timeline

As `plan_constellation` prints it. **This show is written for five drones**;
it plans and flies with four (the escort demo's fleet) but the shapes are
designed around five, and the n-gon, arrow, pyramid and stair all follow the
fleet size. Figure durations are fixed and the legs between them are stretched
onto the beat grid, so the wall clock is **76.0 s, 38 bars at 120 BPM** either
way.

| bar.beat | t | dur | phase | what you see |
|---|---|---|---|---|
| 1.1 | 0.0 s | 2.8 s | takeoff | all of them to 1.00 m over their own start marks |
| 2.3 | 2.8 s | 1.8 s | settle | hold while the Kalman filter settles |
| 3.3 | 4.5 s | 2.8 s | gather | converge into a regular pentagon, R = 1.10 m |
| 5.1 | 7.2 s | 7.8 s | **swashplate** | the ring spins while riding a tilted plane — a wobbling disc |
| 9.1 | 15.0 s | 2.8 s | morph → arrow | the ring folds into an arrowhead |
| 10.3 | 17.8 s | 2.8 s | **dart** | the whole arrow thrusts 0.50 m forward, rigidly |
| 12.1 | 20.5 s | 2.8 s | **recoil** | and draws back |
| 13.3 | 23.2 s | 2.8 s | morph → pyramid | four arms out, one drone raised at the centre |
| 15.1 | 26.0 s | 6.8 s | **turbine** | the arms revolve around the still, raised centre |
| 18.3 | 32.8 s | 2.8 s | morph → staircase | into the switchback stair |
| 20.1 | 35.5 s | 9.8 s | **staircase** | five steps climb 0.60 → 1.40 m, turning a full circle |
| 25.1 | 45.2 s | 3.8 s | retrace → pyramid | the staircase leg run backwards — same paths, same clearance |
| 27.1 | 49.0 s | 6.8 s | **turbine (reversed)** | the same figure backwards; costs no extra memory |
| 30.3 | 55.8 s | 2.8 s | morph → pentagon | an intermediate chosen so the flight home is also safe |
| 32.1 | 58.5 s | 6.8 s | **starburst** | finale: fling out, alternately up and down, spinning |
| 35.3 | 65.2 s | 2.8 s | return home | back over each drone's own `initial_position` |
| 37.1 | 68.0 s | 3.8 s | land | 0.25 m/s descent, then disarm |

### Measured budgets (`plan_constellation`, five drones)

```
separation   1.00 m  min 0.90 m    90%    (3D; cf3/cf8 at t=24.2 s)
plan-view    1.00 m  min 0.90 m    90%           <- the binding separation
speed        1.80 m/s  max 2.00    90%
accel        2.39 m/s2 max 3.00    80%
radius       1.53 m  max 1.90 m    81%    about the MEASURED volume centre
height       1.45 m  max 2.32 m    63%    centre column; 1.85 m at full radius
floor        0.60 m  min 0.30 m    50%
pieces         25     max 31       81%
downwash     2.26x the measured ellipsoid          (reported, not enforced)
```

At four drones it is the same show one drone lighter: radius 1.51 m,
separation 1.02 m.

The radius budget is 1.90 m because tracking held to 2.00 m and the drones fly
the plan with up to `TRACKING_MARGIN` (0.10 m) of error — the same reasoning
that makes the separation budget 0.90 m rather than 0.80 m. The height limit is
a cone, not a number: 2.32 m is what the centre column has flown, while a drone
out at full radius is held to 1.85 m.

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

## 4. Sim results (`backend:=sim`, Humble)

2026-10-02, after the refit (four drones, the fleet currently enabled):

- Full run: landed at **t+76.0 s** against a predicted 76.0 s, **0 late phases**.
- **Flown max radius 1.516 m** about the measured volume centre, against 1.51 m
  planned — 6 mm of overshoot, and **0.72 m of spare to the 2.24 m where a
  drone actually lost tracking**.
- Flown minimum separation **1.016 m** vs 1.02 m planned. Flown ceiling 1.38 m.

2026-09-20, before the refit (five drones):

- Landed at t+78.0 s against 78.0 s predicted, 0 late phases; flown separation
  0.995 m vs 0.997 m planned — a 2 mm loss, against 49 mm for the carousel
  show's counterflow. Tracking error 7.8 cm mean / 11.8 cm max after fitting a
  0.7 s alignment offset, consistent with the 0.10 m `TRACKING_MARGIN`.
- Abort: interrupted mid-turbine, landed and disarmed, exit 130.

Sim is not flight: it has no radio, no mocap latency and no wind. The numbers
above bound the *planning*, not the rig.

---

## 5. Before the first hardware flight

Everything in `SHOW_GUIDE.md` §5d applies unchanged — mocap address, `/poses`
alive, **scan every enabled address**, overlay check. On top of it, for this
show specifically:

0. **Stand five drones on five marks ≥1 m apart, inside the tracked volume**
   (1.90 m of `safety.ARENA_CENTRE`). The yaml enables cf1/cf2/cf3/cf5/cf8;
   **cf8's mark is a planned one, not a measured one** — its old recorded spot
   sat 0.36 m from where cf3 now stands, so it was replaced with
   (−0.27, +1.66), verified to plan clean. (−0.22, −1.15) also works. Any
   well-spread spot does, because the figures are built at the arena centre,
   not at the marks.
1. `ros2 launch crazyflie launch.py server:=False` (mocap and the GUIs, no
   radio), then `python3 scripts/sync_initial_positions.py` with the drones on
   their marks. Start the real stack afterwards — the yaml is read only at
   launch. There is ONE
   `crazyflies.yaml` — `crazyflie/config/` — and it is what the server, the
   planners and the sync all use. The sync records a mark for **every drone
   mocap is streaming**, enabled or not, so the parked spares (cf4) are kept
   current too and swapping fleets between this show and the escort needs no
   re-sync.
2. `ros2 run crazyflie_shows plan_constellation` — it must print `PLAN OK`.
   If the assignment cannot be solved it raises; do not fly around it.
3. `ros2 run crazyflie_shows constellation_show --ros-args -p dry_run:=true`
   — same plan, through the real server, nothing armed.
4. Fly it once at `bpm:=100` (≈94 s, everything slower) before the 120 BPM run.
   Nothing about the geometry changes; only the time you have to react does.
5. Have the E-STOP in reach. The staircase is the first figure in this package
   where drones sit at genuinely different heights; watch the plan view, which
   is where the clearance actually lives.
6. Nothing here has flown on hardware, and the arena numbers it is checked
   against were measured on 2026-10-01 with cf1/cf3/cf5. If the net, cameras or
   Motive calibration move, re-measure (`scripts/measure_arena.py`, then
   `scripts/arena_flight_sweep.py`) before trusting the 1.90 m budget.
