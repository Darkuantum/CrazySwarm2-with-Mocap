# SHOW GUIDE — a 5-drone, ~62 s swarm show

**There are two shows in this package.** This guide covers the carousel show
(`swarm_show`) and, in §2 and §5, everything rig-specific that both shows
need. The second show — shape changes on a beat, with lights — is
`constellation_show`, documented in `CONSTELLATION.md`.

**Written 2026-09-07.** Read `HANDOVER.md` first if you have not — this guide
assumes the rig facts in it, especially §4a (the mocap address) and §6 (the
safety rules). This file covers the show itself: what it does, what you must
fill in, how to verify it on a laptop, and how to deploy and fly it.

**Status.** The plan is verified offline, the package builds in the workspace,
and **the whole show has flown end to end on `backend:=sim`** — all twelve
phases, landing at t+62.3 s against a predicted 62.3 s. **Nothing has flown on
hardware.** Every number below is measured, in sim or on the ground; treat §5d
and §5e as a procedure to follow, not a report of something that worked.

Getting the sim run to work required fixing a pre-existing bug in the
workspace's `crazyflie_sim` — see §7 and HANDOVER §4d. It is unrelated to this
package and affects `demo_show` identically.

---

## 0. The short version

```bash
# 1. build (the package lives in the workspace, src/crazyswarm2/crazyflie_shows)
cd ~/CrazySwarm2-with-Mocap && ./scripts/build.sh crazyflie_shows
source install/setup.bash
ros2 pkg prefix crazyflie          # MUST print this workspace (HANDOVER 4b)

# 2. verify the show on the ground — no radio, no mocap, no drones
ros2 run crazyflie_shows plan_show
ros2 run crazyflie_shows plan_show --plot   # writes show_plan.png

# 3. sim
ros2 launch crazyflie_shows show_launch.py backend:=sim
ros2 run crazyflie_shows swarm_show --ros-args -p use_sim_time:=true

# 4. hardware — only after §5's checklist
sudo ip addr add 141.23.110.162/32 dev wlp131s0f0     # >>> your NIC
ros2 run crazyflie scan --address 0xE7E7E7E701        # ... and every other one
ros2 launch crazyflie_shows show_launch.py
ros2 run crazyflie_shows swarm_show
```

If you only read one command, read `plan_show`. It builds the *same* plan
object `swarm_show` flies and checks every budget against it. If it refuses,
the show would have been unsafe.

---

## 1. What the show does

Twelve phases, 60.3 s of motion, ~63.3 s wall clock. It opens with the
formation barely moving and ends with the fastest, widest figure in the piece.

| # | t | dur | phase | what you see |
|---|---|---|---|---|
| 1 | 0.0 s | 2.5 s | takeoff | all five rise to 1.00 m over their own start marks |
| 2 | 2.5 s | 1.5 s | settle | hold — lets the Kalman filter settle before any lateral motion |
| 3 | 4.0 s | 2.5 s | gather | they converge into a regular pentagon, radius 1.10 m |
| 4 | 6.5 s | 5.5 s | **breathe** | the ring pulses in and out twice. Nothing rotates. Deliberately dull |
| 5 | 12.0 s | 7.3 s | **carousel ×1.12** | the pentagon turns one slow, full revolution as a rigid body |
| 6 | 19.3 s | 6.5 s | **wave** | it keeps turning while a vertical wave runs around the ring, ±0.35 m |
| 7 | 25.8 s | 6.0 s | **carousel reversed ×0.92** | the same turn, backwards, and faster — a visible snap |
| 8 | 31.8 s | 10.5 s | **counterflow** | the showpiece. Three drones fan out to 1.45 m, two drop in to 0.48 m, and the two groups counter-rotate **through each other** |
| 9 | 42.3 s | 5.5 s | **carousel ×0.85** | the fastest turn of the show |
| 10 | 47.8 s | 6.5 s | **bloom** | finale: the ring flings out to 1.45 m and up to 1.45 m while turning, then snaps home |
| 11 | 54.3 s | 2.5 s | return home | back over each drone's own `initial_position` at 0.75 m |
| 12 | 56.8 s | 3.5 s | land | 0.20 m/s descent, then disarm |

The arc the show is built around is **simple → complex**, then **slow →
fast**, and the two are deliberately offset. Complexity rises first (a static
pulse, then rotation, then rotation plus a wave, then two counter-rotating
groups); speed rises later, and mostly through one figure — the carousel is
flown three times at 1.12×, 0.92× reversed, and 0.85×. Reusing one figure is
the clearest way an audience reads *"the same thing, faster"*, and it costs 6
pieces of firmware memory instead of 18.

### Measured budgets

```
separation   0.91 m  min 0.90 m    99%     (= 0.80 m floor + 0.10 m tracking margin)
speed        1.87 m/s  max 2.00     93%
accel        2.58 m/s2 max 3.00     86%
radius       1.45 m  max 2.50 m     58%
height       1.45 m  max 2.00 m     73%
floor        0.65 m  min 0.30 m     46%
pieces         28     max 31        90%
```

Separation sits at 99% of budget because the counterflow's clearance is
*designed*, not accidental — it is a static radial gap. The budget itself is
0.80 m of hard floor plus a 0.10 m allowance for controller lag, which was not
guessed: see §4e.

---

## 2. What you must fill in

Everything rig-specific is tagged. Find it all with:

```bash
grep -rn 'FILL IN' crazyflie_shows/
```

### 2a. `config/crazyflies.yaml` — radios and start positions

This is the file that decides *which* drones fly and *where the plan thinks
they are*. Per drone:

```yaml
  cf1:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E701     # <-- channel/datarate MUST match the drone
    initial_position: [-1.025, -1.028, 0.004]   # <-- metres, world frame
    type: cf21
```

* **`uri`** — the datarate in the URI must match what the drone actually
  answers on. A 2M URI for a 1M drone makes the C++ server hang forever with
  no error (HANDOVER §6, and the `cf11` comment in the yaml).
* **`initial_position`** — take it from `/poses`, **never** from `/cfX/pose`
  (HANDOVER §6). Each drone at its own position, ≥1 m apart. This is what
  the whole plan is derived from, so it has to be right *before* you run
  `plan_show`, not after.
* **`enabled`** — the show plans for 3 or more. If a drone fails its scan,
  set `enabled: false`, re-run `plan_show`, and fly the rest; the pentagon
  becomes a square or triangle and every budget is re-checked. Do **not**
  leave an unreachable drone enabled — one silent drone wedges the entire
  server (HANDOVER §6).

Changing this file needs **no rebuild**, but does need a **launch restart** —
the yaml is read once at launch (HANDOVER §2).

### 2b. `config/motion_capture.yaml` — the Motive server

```yaml
    hostname: "192.168.9.124"    # <-- literal dotted quad, NOT a hostname
    type: "optitrack"            # <-- exactly this; see HANDOVER 4a
```

`interface_ip` in this file is **inert** in `motion_capture_tracking` 1.0.9 —
do not spend time on it. The node always joins multicast on the hardcoded
`141.23.110.162`, which is why §5's `ip addr add` is not optional.

### 2c. `ShowConfig` in `crazyflie_shows/choreography.py` — the room

Three fields, all marked `>>> FILL IN`:

```python
room_center: tuple = None       # None = centroid of initial_position
arena_radius: float = 2.5       # m, horizontal half-extent — MEASURE THIS
ceiling: float = 2.0            # m — MEASURE THIS
```

* **`room_center`** — leave it `None` and the show centres itself on the
  centroid of the fleet's start positions, which is usually what you want and
  self-corrects when you move the drones. Set it explicitly if the fleet does
  *not* start symmetrically about the room centre you want to fly around.
  HANDOVER §3 records a hand-measured `(0.0467, -0.1037)` next to a computed
  centroid of `(0.068, -0.109)`.
* **`arena_radius` / `ceiling`** — these two are inherited from `safety.py`
  and **have never been checked against the actual mocap volume.** They are
  the only defaults in this package that are guesses. Measure them. The show
  currently uses 1.45 m of radius and 1.45 m of height, so anything above
  roughly 1.6 m and 1.6 m will pass — but a wrong value here is a check that
  silently permits a wall.

### 2d. Nothing else is rig-specific

The rest of `ShowConfig` is choreography. It is safe to leave alone until the
show has flown once.

---

## 3. How the pieces fit together

```
crazyflie_shows/
├── figures.py        geometry + trajectory fitting   PURE — no ROS
├── safety.py         separation / envelope checking   PURE — no ROS
├── choreography.py   THE SHOW: config, plan, verify   PURE — no ROS
├── plan_show.py      offline verifier + report        entry point
├── swarm_show.py     the flight script                entry point
└── demo_show.py      the original starter show        entry point, unchanged
```

The important line is between `choreography.py` and `swarm_show.py`.
`choreography.build_plan()` produces a `Plan` — every trajectory, every phase,
every position at every instant — and verifies it as it builds, raising rather
than warning. `plan_show` and `swarm_show` both call it. That is what makes
the offline check meaningful: **the laptop verifies the same object the swarm
flies**, not a model of it.

`swarm_show.py` is deliberately thin. It reads the fleet, calls `build_plan`,
checks placement, uploads, and then walks `plan.phases` dispatching on
`phase.kind`. There is no geometry in it at all, and no path from it to an
armed drone flying an unverified plan.

`figures.py`, `safety.py` and `choreography.py` import numpy and two classes
from `crazyflie_py.uav_trajectory`, nothing else — `figures.py` even loads
that module around `crazyflie_py`'s rclpy import so the whole planner runs on
a laptop with no ROS installed:

```bash
python3 -m crazyflie_shows.plan_show --yaml config/crazyflies.yaml
```

---

## 4. Why the show is safe

The firmware has **no collision avoidance enabled on this rig** (HANDOVER §6),
so every metre of clearance has to come from the plan. Three mechanisms do it.

### 4a. Every figure is rigid, radial, or lane-separated

| figure | why it cannot collide |
|---|---|
| `breathe` | purely radial and identical for every drone — the ring stays a regular n-gon, so the worst case is just `2R' sin(180/n)` at the smallest radius |
| `carousel` | rigid rotation: pairwise distance is **constant** for the whole figure, at any speed |
| `wave` | rigid in the horizontal plane; the vertical stagger can only *increase* 3D distance |
| `counterflow` | two groups on concentric circles 0.97 m apart |
| `bloom` | rigid in the horizontal plane *and* expanding — separation only grows |

The one figure where drones genuinely fly through each other is
`counterflow`, and it is worth being precise about why that is acceptable:
**nothing about its clearance depends on timing.** The two groups are on
circles of radius 1.45 m and 0.48 m. Every head-on pass has 0.97 m of
clearance whatever the phase and whatever the timescale. A crossing figure
whose safety came from two drones arriving at the same point at different
*times* would be one radio dropout away from a collision; this one is not.

### 4b. Every figure returns each drone to its own slot

`startTrajectory(relative=True)` shifts an uploaded trajectory so its
`eval(0)` lands on the drone's current setpoint — per drone, with each drone's
own shift. For a rotation figure that is a trap. If a drone has drifted one
slot round since upload, its shift depends on its own start angle, so the five
drones get five *different* translations and the ring stops being a ring —
while the offline check, run on the unshifted geometry, sees nothing wrong.

The rule that removes the failure mode entirely: **every sweep is a whole
multiple of 360°**. Then the shift is always ~0, figures compose in any order,
at any timescale, forwards or reversed. `build_plan` asserts it
(`figures.composes_in_place`).

### 4c. The whole show is sampled and checked, not just the legs

`safety.check_show` evaluates all five drones through all twelve phases at
200 Hz and checks minimum pairwise separation, peak speed, peak acceleration,
radius, ceiling and floor. Two details make that check mean something:

* **The `goTo` legs are modelled exactly, not approximately.** The firmware
  fits a degree-7 polynomial from the current state to the goal; when the
  drone starts and ends at rest with zero acceleration — which every `goTo`
  here does, because every figure is rest-to-rest — that polynomial is the
  same scalar time-warp on all three axes, so the path is *exactly* the
  straight segment. Issue a `goTo` while a drone still has velocity and the
  path bows away from the line by an amount nothing here models.
* **200 Hz, not 50.** Acceleration is a second difference, so a coarse grid
  smooths away exactly the short jolts worth catching. A boundary
  discontinuity that reads as 17 m/s² at 400 Hz reads as 4 m/s² at 50 Hz and
  slips past a 3 m/s² budget.

### 4d. Two things the check does *not* cover

* **Where the drones physically are.** The plan is built from
  `initial_position`; a drone in the wrong corner makes the plan irrelevant
  rather than wrong. `swarm_show` therefore refuses to arm unless every
  drone's live `/cfX/pose` is within `placement_tol` (0.25 m) of its yaml
  position. That tolerance is not arbitrary: `takeoff` climbs from wherever
  the drone actually is and `settle` then pulls it laterally onto the yaml
  position — a leg the plan models as a no-op. Two drones displaced 0.25 m
  towards each other shrink the 1.36 m hover separation to 0.86 m, still over
  budget. Raising it eats that margin directly.
* **Controller tracking** — see §4e. Every number in the check is about the
  *planned* path, and a drone that lags its setpoint is not where the check
  thinks it is. That is the real reason `MAX_SPEED` and `MAX_ACCEL` matter:
  they are not about what the airframe can survive, they are about where it
  still tracks a polynomial closely.

### 4e. The tracking margin — why the budget is 0.90 m, not 0.80 m

An early version of this show was checked against the bare 0.80 m floor and
passed at 0.85 m. Flown in sim, it reached **0.793 m** — under the floor.

Measured, `backend:=sim`, full show, positions from `/tf`:

| | planned | flown |
|---|---|---|
| min separation | 0.85 m | **0.793 m** (counterflow, t+41.4 s) |
| peak speed | 1.87 m/s | 1.95 m/s |

The gap is controller lag, and the important part is that **it does not cancel
where it matters**. Drones holding a rigid formation lag together, so their
separation is unaffected — which is why `carousel`, `wave` and `bloom` flew
exactly as planned. The counterflow's two groups move in *opposite* directions
at different radii, so their lags subtract and eat the static radial clearance
directly. The one figure designed to have drones fly through each other is
also the one where the plan is optimistic.

So `safety.PLAN_SEPARATION` is `MIN_SEPARATION + TRACKING_MARGIN` = 0.80 + 0.10,
and that is what `check_show` enforces. The lanes were widened from
1.30/0.45 m to 1.45/0.48 m to clear it, and `dur_counterflow` went from 9.5 s
to 10.5 s to keep the wider outer lane inside the speed budget.

Re-flown in sim to confirm the fix:

| | planned | flown |
|---|---|---|
| min separation | 0.91 m | **0.861 m** — 0.061 m clear of the floor |

The lag reproduced at 0.049 m, so the 0.10 m margin is roughly 2x what the
counterflow actually costs. That is the intended amount of conservatism for a
figure nobody has flown yet; revisit it with hardware data, not before.

Acceleration could **not** be measured this way: `/tf` carries float-quantised
positions, and a second difference of quantised data is dominated by rounding
— the same run reports 3.6 m/s² at 50 Hz and 130 m/s² at 2 kHz. Speed and
position are trustworthy; the acceleration budget remains checked only against
the plan.

---

## 5. Deploying and flying

### 5a. Build

```bash
cd ~/CrazySwarm2-with-Mocap
./scripts/build.sh crazyflie_shows
source install/setup.bash
ros2 pkg prefix crazyflie          # MUST print this workspace, never /opt/ros/humble
```

That last line is not optional — HANDOVER §4b. Apt ships every package in this
stack, and an unsourced overlay silently runs `/opt/ros/humble`'s binaries
with `/opt/ros/humble`'s config, which reads as a mocap fault.

The package is relocatable by construction (every resource resolved through
`get_package_share_directory`, no absolute paths). When to rebuild, from
HANDOVER §2:

| change | rebuild? | restart launch? |
|---|---|---|
| body of an existing registered `.py` | no | no |
| `config/*.yaml` | no | **yes** |
| new `.py` + new `setup.cfg` entry point | **yes** | no |

### 5b. Verify on the ground

```bash
ros2 run crazyflie_shows plan_show
ros2 run crazyflie_shows plan_show --plot
```

Read three things in the output:

1. **the fleet line** — are those the drones you expect, in that order?
2. **the timeline** — the `<-- tightest` marker shows which phase holds the
   minimum separation. It should be `counterflow`.
3. **the budgets** — anything at 100% is a rejection, and `plan_show` will
   have refused. Anything above ~95% is worth a second look.

The plot is the fastest sanity check: the left panel shows the paths from
above and the right shows closest-pair distance across the whole show, with
the 0.80 m budget as a red line. The counterflow reads as a flat 0.85 m
plateau with visible ripples — those ripples are the head-on passes.

### 5c. Sim

```bash
ros2 launch crazyflie_shows show_launch.py backend:=sim
# in a second terminal:
ros2 run crazyflie_shows swarm_show --ros-args -p use_sim_time:=true
```

`use_sim_time` is **required**: the sim clock runs ~4× slower than wall time,
so a 62 s show takes ~4 minutes and without the flag the script races ahead of
the physics. `rviz:=False` skips the visualiser if you only want the printout.

The placement check auto-skips in sim: the topic is advertised there but
nothing is ever published on it, so there is nothing to compare against. On
hardware it is the firmware's own pose log and the check is real.

### 5d. Hardware pre-flight

Everything here is from HANDOVER §4a and §6. None of it is optional.

```bash
# 1. the mocap address — PER SESSION, and again after any Wi-Fi reconnect
sudo ip addr add 141.23.110.162/32 dev wlp131s0f0     # >>> your NIC
ip -o -f inet addr show dev wlp131s0f0                # expect BOTH addresses

# 2. mocap actually publishing
ros2 topic hz /poses                                  # want ~50 Hz
python3 scripts/sync_initial_positions.py --dry-run   # body names vs fleet (from the workspace root)
ss -uanp | grep :1511                                 # two sockets = an orphan is starving it

# 3. scan EVERY enabled address — one silent drone wedges the whole server
ros2 run crazyflie scan --address 0xE7E7E7E701
ros2 run crazyflie scan --address 0xE7E7E7E702
ros2 run crazyflie scan --address 0xE7E7E7E703
ros2 run crazyflie scan --address 0xE7E7E7E710
ros2 run crazyflie scan --address 0xE7E7E7E712

# 4. overlay check
source install/setup.bash && ros2 pkg prefix crazyflie
```

Then bring the stack up and check the preflight GUI **before** anything arms:

```bash
ros2 launch crazyflie_shows show_launch.py
```

* `err.yaw`: **±5° fly, 5–15° fix, >20° do not fly.** `locSrv.extPosStdDev`
  is `1e-3`, which force-fuses the mocap position — a yaw offset is invisible
  at rest and a fly-away in flight.
* Batteries above the 3.8 V warning. The show is ~62 s plus upload time.
* A mid-session Wi-Fi reconnect withdraws `141.23.110.162` and puts you in the
  fly-away case, not the loud case. If the Wi-Fi drops, re-check the address
  and restart the stack before arming.

### 5e. Fly

```bash
# dress rehearsal: prints the full plan and exits without arming
ros2 run crazyflie_shows swarm_show --ros-args -p dry_run:=true

# the show
ros2 run crazyflie_shows swarm_show
```

`swarm_show` will refuse, before arming, if:

* any budget in the plan is violated,
* any drone has never published `/cfX/pose` (nothing has confirmed it is
  tracked),
* any drone is more than `placement_tol` from its `initial_position`.

Keep a hand on the emergency stop for all 62 s. Nothing in this package has
flown.

### Parameters

```bash
ros2 run crazyflie_shows swarm_show --ros-args \
    -p dry_run:=false \
    -p check_placement:=true \
    -p placement_tol:=0.25 \
    -p scale:=1.0 \
    -p arena_radius:=2.5 \
    -p ceiling:=2.0
```

---

## 6. Changing the show

### Retiming — the cheapest change

The tempo lives in one table in `choreography.build_plan()`:

```python
script = [
    ('breathe',     1.00, False, '...'),
    ('carousel',    1.12, False, '...'),
    ('wave',        1.00, False, '...'),
    ('carousel',    0.92, True,  '...'),   # reversed
    ('counterflow', 1.00, False, '...'),
    ('carousel',    0.85, False, '...'),
    ('bloom',       1.00, False, '...'),
]
```

`(figure, timescale, reversed, note)`. Timescale <1 is faster. Reorder, drop,
repeat — any figure can follow any other, because they all return each drone
to its own slot (§4b). Then re-run `plan_show`; it will tell you which budget
you broke and which phase broke it.

To make the show **punchier**, the honest lever is not the timescales — they
are already near the envelope. It is `safety.MAX_ACCEL`, currently 3.0 m/s²,
which is a conservative first-flight value and about a 17° tilt. A Crazyflie
2.1 has roughly 2.2:1 thrust-to-weight and can do far more. Raising it to ~5
would let every rotation tighten considerably. **That is a decision to make
after the show has flown once, not a fix for a rejected plan.**

### Adding a figure

Watch the piece budget: **31 pieces per drone, total, for all trajectories
resident at once** (4096 B of firmware trajectory memory ÷ 132 B per piece).
The show uses 28. Adding a figure means taking pieces from another one, not
hoping. `figures.pack_offsets` raises rather than letting two trajectories
overwrite each other in firmware memory — which they would otherwise do
silently.

A new figure is a function returning `t -> (x, y, z, yaw)`; `figures.py` has
`polar_path` for anything ring-shaped and `bezier_point` / `bowed_chord` for
free-form curves. Two rules:

1. **Rest-to-rest.** Drive shape parameters through `smoothstep` (or `ramped`
   for rotations, which peaks at 1.33× mean rate instead of 1.875× and is what
   makes a full turn fit the accel budget). For a there-and-back excursion use
   `bump`, **not** `sin(pi*u)` — the latter has the right values at the ends
   and the wrong derivative, which reads as a 7 m/s² spike at the phase
   boundary and is a real jolt on the drone.
2. **Sweep a whole multiple of 360°**, or the composition rule in §4b breaks.

`fit_trajectory` handles the rest, including pinning position, velocity *and*
acceleration at every piece boundary so the assembled trajectory is exactly
C². That constraint costs about 7× in position error — to 1.9 mm, well below
what the mocap resolves — and removes phantom accelerations of 17 m/s² that
a plain per-piece `polyfit` leaves behind. Do not "optimise" it back out.

### Making the show fit a smaller room

`--scale` shrinks every radius and speed but **not** the separation budget,
which is a physical floor plus a measured tracking allowance (§4e) — neither is
a property of the choreography. So scaling down eats clearance directly.

Measured: **the floor is `scale 0.99`.** At 0.98 the counterflow's inner pair
falls to 0.89 m and `plan_show` rejects the show. This choreography is already
at its minimum size for five drones; `scale` is a knob for growing it.

For five drones at the 0.90 m plan budget the geometry needs roughly a **1.40 m
working radius — a 2.8 × 2.8 m clear floor** — before takeoff and landing
margins. A smaller room needs fewer drones or different figures, not a smaller
scale. Dropping to four drones is genuinely cheaper than shrinking: the n-gon
opens up and every budget is re-checked automatically.

---

## 7. Troubleshooting

| symptom | cause | fix |
|---|---|---|
| `No executable found` | added a file, skipped the rebuild | `./scripts/build.sh crazyflie_shows` |
| `plan_show` rejects the plan | a budget is violated | it names the phase and the lever; see §6 |
| launch hangs, no `/all/*` services | an enabled drone did not answer | scan every address; `enabled: false` the dead one (HANDOVER §6) |
| mocap node aborts ~1.1 s, `set_option: No such device` | `141.23.110.162` missing | `sudo ip addr add …` (HANDOVER §4a) |
| mocap node aborts ~1.1 s, `Unknown motion capture type!` | `type` is not `"optitrack"` | fix `motion_capture.yaml` |
| stack starts, `/poses` never publishes | node hung in `recv` | check the **topic**, never the process (HANDOVER §4a) |
| your yaml edits appear to do nothing | overlay not sourced | `ros2 pkg prefix crazyflie` (HANDOVER §4b) |
| **sim** server dies at the first figure, `plan_start_trajectory() takes 5 positional arguments but 7 were given` | `crazyflie_sim` targets a newer cffirmware binding than the one installed | fixed in `crazyflie_sil.py`; HANDOVER §4d. Hardware is unaffected |
| `NO POSE from: …` | that drone is not being tracked | `ros2 topic hz /poses`, then `/cfX/pose` |
| `PLACEMENT CHECK FAILED` | drone is not on its mark | move it, or re-take `initial_position` from `/poses` |
| show runs but far too fast in sim | `use_sim_time` not set | add `--ros-args -p use_sim_time:=true` |

---

## 8. Open items for this show specifically

These are additions to HANDOVER §8, not replacements.

1. **Nothing here has flown on hardware.** Sim first, then a single hover, then
   the show.
2. **`arena_radius` and `ceiling` are guesses.** They are the only unmeasured
   defaults in the package, and they are exactly the check that would stop the
   show hitting a wall. Measure them (§2c).
3. **Mocap rate.** HANDOVER §8 item 11 is unresolved: `/poses` was measured at
   ~27 Hz against ~42 Hz at the socket. This show asks for 1.87 m/s, so a
   dropped frame is ~7 cm of stale position. That is inside the margins here,
   but it is the reason to re-measure before trusting the faster figures.
4. **Battery draw at speed.** 62 s of continuous motion with a 9.5 s
   counterflow is more demanding than a hover test. Watch the 3.8 V warning on
   the first full run.
5. **Trajectory upload time on hardware.** 5 figures × 5 drones = 25 blocking
   round trips over one radio, before arming. Unmeasured on this rig. If it is
   slow, the lever is fewer figures, not fewer pieces.
