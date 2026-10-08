# Writing your own show

How to add a show of your own to `src/crazyswarm2/crazyflie_shows/`.

The ROS API is the easy part. This guide is about the **contract** — the things
a show must do before it is allowed to arm a drone. Every item in it was paid
for: the three shows that exist each cost an incident, a near-miss or an
afternoon on the floor to get right, and this file is where that transfers to
the fourth.

**If this file and the code disagree, the code wins — then fix this file.**
Where a number can be printed by a command, the command is given instead of the
number: prose copies of live values have drifted six times in this repo and
caused real incidents. Everything below either cites the source file it was read
in, or the date it was measured.

Read first, in this order: `CLAUDE.md` (the Gotchas section), then the module
docstrings of `abort.py`, `preflight.py` and `safety.py` — they are the primary
sources for sections 1 and 2 here.

### Which existing show to copy

| Start from | When your show is… | Design doc |
|---|---|---|
| `demo_show.py` (196 lines) | anything. Its docstring is literally *"Template for new choreography"*, and it is the smallest complete correct show | — |
| `swarm_show.py` + `choreography.py` + `plan_show.py` | a planned, phase-table show with uploaded polynomial trajectories | `crazyflie_shows/SHOW_GUIDE.md` |
| `constellation_show.py` + `constellation.py` | beat-locked, with shape changes and light cues | `crazyflie_shows/CONSTELLATION.md` |
| `escort_show.py` + `escort.py` + `plan_escort.py` | **reactive** — setpoints computed per tick, not choreographed | `crazyflie_shows/ESCORT.md` |

Operating any of them (the cards an operator reads on the day) is
`runbooks/` — `CAROUSEL.runcard.md`, `CONSTELLATION.runcard.md`,
`ESCORT.runcard.md`. Rig-level knowledge that predates this package is in
`crazyflie_shows/HANDOVER.md`. None of those are repeated here.

---

## 1. The contract

A show may arm drones only if it does all of the following. Each item exists
because a show without it has already failed on this rig, or would have.

### 1.1 Take the signals back — `abort.py`

```python
from crazyflie_shows.abort import ShowAborted, abort_land, take_signals
```

**Why.** `rclpy.init` (called inside `Crazyswarm()`) installs its own SIGINT
handler that shuts the ROS context down *before* your `KeyboardInterrupt`
handler runs. `allcfs.land()` then dies with `failed to initialize wait set:
the given context is not valid` — at exactly the moment it is needed — and the
drones keep flying their last command. **Verified failing in sim 2026-09-20**
(`abort.py` module docstring).

This machinery lived inside `constellation_show.py` until **2026-10-06**, which
means `swarm_show` (the carousel — the show that has actually flown on
hardware) and `demo_show` had **no abort path at all**. Importing the shared
module instead of copying its body is the whole point.

The six rules, each with its reason:

1. `take_signals()` goes **after** `Crazyswarm()` has returned, and it claims
   SIGINT **and SIGTERM**. The mission console's Stop button sends SIGTERM and
   escalates to SIGKILL a few seconds later
   (`console/mission_console/procs.py`), so an abort that only handles SIGINT
   never starts.
2. Set `armed = True` **before** the arm loop, not after. A signal arriving
   between the first and the last `cf.arm(True)` would otherwise raise with
   `armed` still `False`, skip the abort entirely, and leave part of the fleet
   armed. Disarming an already-disarmed drone is harmless; the reverse is not.
3. Clear `armed = False` only after the final `cf.arm(False)`.
4. `abort_land()` restores `SIG_DFL` for **both** signals before it lands
   (it does this for you). Two consequences, both deliberate: the console's
   SIGINT→SIGTERM escalation at 6 s cannot interrupt a slow landing's own
   disarm loop, and a second Ctrl-C kills the process outright — the operator
   must always be able to give up on the script and reach for the E-STOP.
5. `ShowAborted` returns 130 with no traceback; **any other exception lands the
   drones and re-raises**, so a real fault stays visible.
6. `abort_land(allcfs, cfs, timeHelper, land_height, why)` takes a **float**
   height, not a config object. Every caller used to pass a whole config for
   that one field and `escort_show` had to invent a stub class to satisfy it.

The abort is the answer to a *script* that stopped. A drone that is out of
control is still the E-STOP's job (console E-STOP, or `e` in the preflight GUI);
those are on a different callback group and keep working even when the server is
wedged (`CLAUDE.md`).

**Test your abort** with `kill -INT $(pgrep -f my_show)`. `ros2 run` does **not**
forward a signal sent to the wrapper alone; a terminal Ctrl-C works only because
it goes to the whole foreground process group. Signalling the wrapper looks
exactly like "my abort does nothing", and is not.

### 1.2 Ask the firmware whether it will fly — `preflight.py`

```python
from crazyflie_shows.preflight import report_supervisor   # or check_supervisor
if not sim and not report_supervisor(node, names):
    return 1
```

**Why.** An E-STOP latches the firmware supervisor into LOCKED (`/cfX/status`
`supervisor_info` with `IS_LOCKED` set, `CAN_BE_ARMED` clear). The state machine
has exactly one transition out of locked — back to locked — and **only a battery
pull clears it**. Measured 2026-10-02: five drones sat locked with healthy
batteries after an e-stop, and the only symptom was that nothing flew.

It is invisible without this check because **`arm()` is fire-and-forget**:
`crazyflie_py/crazyflie.py:498-511` calls `self.armService.call_async(req)` and
never inspects the response. In the original show the failure surfaced only
after a ~55 s trajectory upload, which the operator read as a hang and killed.

**Where it goes:** before the upload, before arming, and **before** the
`dry_run` exit — it is read-only, so a rehearsal should exercise it. Check only
the drones that will actually fly.

**What it does not prove.** A silent `/cfX/status` is reported as a NOTE, not a
refusal, because old firmware does not publish it — but a silent status is also
exactly what the telemetry stall looks like (`CLAUDE.md`, *"Link alive, log data
dead"*). So the **only** positive confirmation that a drone actually flew is its
mocap altitude after takeoff. If your show can check that, check it.

Battery gates are `BATTERY_REFUSE_V` / `BATTERY_WARN_V` at the top of
`preflight.py`; a 0.00 V reading is a log block that has not delivered yet, not
a flat battery, and is reported as a note.

### 1.3 Plan against the measured arena — `safety.py`

`safety.py` is pure numpy — no ROS, no radio — so a planner can run on a laptop
with nothing powered on. Print the live values rather than trusting this guide:

```bash
python3 -c "from crazyflie_shows import safety as s; \
  print('centre', s.ARENA_CENTRE, 'plan r', s.ARENA_RADIUS_PLAN, \
        'sep', s.PLAN_SEPARATION, 'v', s.MAX_SPEED, 'a', s.MAX_ACCEL)"
```

What each constant is, and the evidence behind it (read the comments in
`safety.py` for the full version):

| Name | Meaning, and how it is known |
|---|---|
| `ARENA_CENTRE` | centroid of the **tracked volume**, not of the room. Surveyed; re-expressed into the recalibrated Motive frame on 2026-10-08 (§5.6) |
| `ARENA_RADIUS_TESTED` / `ARENA_RADIUS_LOST` | radius flown clean / radius where a flying drone actually lost tracking. Flown 2026-10-01. The volume is anisotropic and roughly a truncated cone |
| `ARENA_RADIUS_PLAN` | `TESTED` minus `TRACKING_MARGIN`. **This is the number a plan must stay inside**, because the drones fly the plan with up to that much error |
| `CEILING_*` / `ceiling_at(r)` | the radius-dependent height limit. A single ceiling number for the whole room either forbids the centre climb or blesses an untested corner |
| `MIN_SEPARATION` / `PLAN_SEPARATION` | the hard floor, versus what a *plan* must clear. `PLAN_SEPARATION = MIN_SEPARATION + TRACKING_MARGIN` |
| `TRACKING_MARGIN` | how much closer the drones actually get than planned. Measured in sim 2026-09-07: planned 0.85 m, flown 0.793 m; re-flown with widened lanes, planned 0.91, flown 0.861 |
| `MAX_SPEED` / `MAX_ACCEL` | where the controller still tracks a polynomial instead of lagging it. Exceeding them does not fail loudly — the drone lags, which silently invalidates every separation number you checked. Treat it as a collision risk, not a comfort setting |
| `DOWNWASH_RXY` / `DOWNWASH_RZ` | the measured anisotropic downwash ellipsoid (0.12 m horizontal, 0.30 m vertical; Preiss/Hoenig et al., IROS 2017). **Reported, not enforced** |

The gates you will actually call:

* `check_envelope(positions, label=…)` — every position inside the radius *and*
  under `ceiling_at`, measured from `ARENA_CENTRE`.
* `check_leg(src, dst, label)` — one set of simultaneous straight-line `goTo`s,
  closed-form minimum separation against `PLAN_SEPARATION`. Exact rather than
  sampled, because a rest-to-rest `goTo` **is** a straight line
  (`rest_to_rest_line`). It is `demo_show`'s only separation gate.
* `check_show(sample_fn, total_time, n_drones, …)` — for polynomial
  trajectories, where drones are *not* on straight lines: samples the whole show
  at `CHECK_RATE` and checks separation, speed, acceleration, radius, ceiling
  and floor, raising `ValueError` so a show script cannot arm a failing plan.
  Pass `phase_bounds` and the failure names the figure; pass `floor_window` so
  the floor is enforced only between takeoff and landing.
* `assign_makespan` / `assign_min_distance` / `best_phase_ngon` — slot
  assignment that is *safe*, not merely short. Brute force, so n ≤ 8.
* `scaled_duration(src, dst, avg_speed, minimum)` — a `goTo` duration scaled to
  the longest leg in the set. A fixed duration on a long move demands
  accelerations the drone cannot produce, and nothing downstream corrects it.
* `downwash_clearance(samples)` — how many times over the downwash ellipsoid the
  closest pair stays. Informational headroom for a denser show; do not treat a
  pass as permission to stack drones until the lag is re-measured on hardware.

**The envelope is set by where the drones are PARKED, not by the figure.** This
is the most repeated lesson in the repo. The constellation planned to 1.76 m of
its 1.90 m budget purely because one drone's mark sat 1.76 m from the arena
centre, while its figures never reach past 1.53 m. The multi-trajectory
formation demo's hover columns alone reach 1.76 m, leaving 0.14 m for every
figure — so it refuses on those marks, correctly. **`check_envelope` the hover
columns over the marks, the home columns, and the figure.**

**The tracking margin is never optional.** Never pass `min_sep=MIN_SEPARATION`
to a gate to make a plan pass. A leg planned at exactly the floor is flown under
it — that is the 2026-09-07 measurement that created `TRACKING_MARGIN`, and the
reason `check_leg`'s default was raised on 2026-10-06.

---

## 2. The offline planner — no planner, no flight

Every show has a planner: `plan_show`, `plan_constellation`, `plan_escort`. A
new show gets one too. The pattern, as `plan_show.py` implements it:

1. **The geometry, timing and verification live in a pure module** with no ROS
   and no radio — `choreography.py`, `constellation.py`, `escort.py`.
2. That module exposes **`build_plan(...)` which verifies as it builds and
   raises** on any budget violation. There is then no code path from a script to
   an armed drone flying an unchecked plan.
3. **The flight script calls that same function** with the live fleet; the
   planner calls it with the yaml fleet. Both verify the object that flies. A
   planner that rebuilt its own copy of the numbers would be verifying something
   else — which is precisely how the escort's duplicated arena centre went
   unnoticed (§5.6).
4. The planner reads `crazyflies.yaml` itself (`plan_show.load_fleet`: enabled
   drones, sorted by name, which is the order the C++ server connects in), so it
   needs no ROS graph at all.

**Re-run the planner after every position sync. This is not optional.** The plan
is a function of `initial_position`, so moving one mark changes every separation
number, and the margins are thin — the carousel sits at ~99 % of its separation
budget (`CLAUDE.md`; re-run `plan_show` for today's number rather than quoting
that one). Re-run it also after any edit to `safety.py`, any Motive
recalibration, and any change to the enabled fleet.

```bash
ros2 run crazyflie_shows plan_show                 # default --show swarm
ros2 run crazyflie_shows plan_constellation        # = plan_show main_constellation
ros2 run crazyflie_shows plan_escort --sweep
python3 -m crazyflie_shows.plan_show --yaml path/to/crazyflies.yaml   # no ROS at all
```

Give a new planner the same shape: an `argparse` main with `--yaml`, a printed
budget bar per number (`plan_show.bar`), and a `hints(msg, cfg)` function that
turns a failure message into *what to change*. A `ValueError` that names the
phase and the time is the difference between a two-minute fix and an afternoon
with five drones on the floor.

---

## 3. Skeleton of a new show

Order matters: each step is the cheapest gate that can stop the next, more
expensive or more dangerous, one. Save as
`src/crazyswarm2/crazyflie_shows/crazyflie_shows/my_show.py`.

This is a complete show — every enabled drone rises, translates rigidly 0.4 m in
+x, comes back, and lands. It is deliberately trivial so that the *structure* is
what you read; `demo_show.py` is the same structure with a real figure, and is
the one to diff against.

> **Status of this skeleton:** it compiles (`python3 -m py_compile`) and every
> symbol in it was checked against the current source. It has **not** been run,
> in sim or on hardware. Treat it as a structure to fill, not as a tested show.

```python
#!/usr/bin/env python3
"""My show: rigid shuffle 0.4 m in +x and back.

    ros2 run crazyflie_shows my_show --ros-args -p use_sim_time:=true   # sim
    ros2 run crazyflie_shows my_show                                    # hardware
    ros2 run crazyflie_shows my_show --ros-args -p dry_run:=true        # rehearsal
"""
import sys

import numpy as np
from crazyflie_py import Crazyswarm

from crazyflie_shows import safety
from crazyflie_shows.abort import ShowAborted, abort_land, take_signals
from crazyflie_shows.preflight import report_supervisor
from crazyflie_shows.swarm_show import _param, check_placement

HOVER = 1.0                     # m above each drone's own initial_position
SHIFT = np.array([0.4, 0.0, 0.0])
TAKEOFF_S, LEG_S = 2.5, 3.0     # s
LAND_H, LAND_S = 0.04, 4.5      # m, s -- ~0.21 m/s descent
MARGIN = 0.5                    # s of slack after every command


class _Starts:                  # check_placement only reads .starts
    def __init__(self, starts):
        self.starts = starts


def main():
    swarm = Crazyswarm()
    timeHelper, allcfs = swarm.timeHelper, swarm.allcfs
    cfs = allcfs.crazyflies
    names = [cf.prefix.lstrip('/') for cf in cfs]

    # 1. parameters --------------------------------------------------------
    sim = bool(_param(allcfs, 'use_sim_time', False))
    dry_run = bool(_param(allcfs, 'dry_run', False))
    tol = float(_param(allcfs, 'placement_tol', 0.25))

    # 2. plan and verify on the ground: pure geometry from the YAML marks ---
    if len(cfs) < 2:
        raise SystemExit('need >= 2 enabled drones (crazyflies.yaml `enabled:`)')
    starts = [np.array(cf.initialPosition, float) for cf in cfs]
    up = [s + [0, 0, HOVER] for s in starts]
    out = [u + SHIFT for u in up]
    safety.check_envelope(up, label='hover columns over the marks')
    safety.check_envelope(out, label='shifted figure')
    safety.check_leg([u[:2] for u in up], [o[:2] for o in out], 'out')
    safety.check_leg([o[:2] for o in out], [u[:2] for u in up], 'back')
    print(f'plan ok: {len(cfs)} drones, budget {safety.PLAN_SEPARATION:.2f} m',
          flush=True)

    # 3. is each drone physically where the plan thinks it is? --------------
    #    Skipped in sim: backend:=sim publishes no /cfX/pose at all (section 5.2).
    if not sim:
        check_placement(allcfs, cfs, names, _Starts(starts), tol)

    # 4. supervisor go/no-go -- read-only, so it runs before the dry_run exit
    if not sim and not report_supervisor(allcfs, names):
        return 1

    # 5. a rehearsal stops here. Everything past this line touches a drone --
    if dry_run:
        print('dry_run: checks done, nothing uploaded, nothing armed')
        return 0

    # 6. (upload trajectories HERE, before arming, if your show has any)

    # 7. arm -> takeoff -> figure -> land -> disarm, all under the abort ----
    armed = False
    try:
        take_signals()
        armed = True                      # BEFORE the loop -- see 1.1 rule 2
        for cf in cfs:
            cf.arm(True)
        timeHelper.sleep(1.0)

        allcfs.takeoff(targetHeight=HOVER, duration=TAKEOFF_S)
        timeHelper.sleep(TAKEOFF_S + MARGIN)

        for goal_set in (out, up):        # your figure goes here
            for cf, g in zip(cfs, goal_set):
                cf.goTo(g, 0.0, LEG_S)    # absolute; allcfs.goTo is relative only
            timeHelper.sleep(LEG_S + MARGIN)

        allcfs.land(targetHeight=LAND_H, duration=LAND_S)
        timeHelper.sleep(LAND_S + MARGIN)
        for cf in cfs:
            cf.arm(False)
        armed = False
        return 0
    except ShowAborted as e:              # deliberate stop: quiet
        if armed:
            abort_land(allcfs, cfs, timeHelper, LAND_H, str(e))
        return 130
    except BaseException as e:            # noqa: BLE001 -- real fault: land, then re-raise
        if armed:
            abort_land(allcfs, cfs, timeHelper, LAND_H, repr(e))
        raise


if __name__ == '__main__':
    sys.exit(main())
```

Notes on the skeleton, each traceable to an existing show:

* **Plan from `cf.initialPosition` (the yaml), never from live pose.** The
  onboard estimate is seeded *from* the yaml, so reading it back is circular
  (`CLAUDE.md`). `check_placement` is the one thing that compares live pose with
  the yaml; it waits up to `POSE_WAIT_S` (25 s) for every drone's first
  `/cfX/pose`, because unicast telemetry on one radio drops out routinely, and
  it refuses with the fix steps spelled out. Its 0.25 m default tolerance is not
  arbitrary — two drones displaced `tol` toward each other eat `2*tol` of the
  hover separation.
* **Upload before arm.** An upload saturates the radio (~55 s for the carousel's
  figures across five drones) and can wedge the server outright (`CLAUDE.md`,
  *"An UPLOAD can wedge the server permanently"*). Never upload to armed drones,
  and print progress — an operator who reads silence as a hang kills the show,
  which has happened.
* **`cf.goTo` for absolute goals, per drone;** `allcfs.goTo` is a broadcast but
  hard-codes `relative=True`.
* **`allcfs.startTrajectory(...)` is one broadcast,** so a rigid formation starts
  on the same radio frame; five unicast starts would smear it.
* **Sleep `duration + margin` after every command,** and keep `goTo` legs
  rest-to-rest. `rest_to_rest_line` — and therefore `check_leg` — is only exact
  when the drone starts and ends at rest. Issue a `goTo` mid-motion and the path
  bows away from the straight line by an amount nothing in `safety.py` models.
* **Flying a subset of the fleet** (as `escort_show` does): arm and
  supervisor-check only the drones that fly, but let the abort path address them
  all. Landing a drone that was not flying is never wrong.
* **Beat-locked timing:** schedule phase *k* at `t0 + sum(durations before k)`
  rather than "after the previous sleep", so command latency cannot accumulate
  (`constellation_show.py`).
* **Light cues must never be able to stop a show:** wrap `setParam` in
  try/except and send it *after* the motion command (`constellation_show.cue`).

### If your show is reactive (streaming setpoints)

When setpoints are computed per tick rather than planned (as in `escort_show`),
the offline proof is replaced by a **clamp at flight time**, and extra rules
apply — all from `ESCORT.md` and `escort_show.py`:

* Stream `cmdFullState`, never `cmdPosition` (§5.3).
* Call `notifySetpointsStop()` before any `goTo` or `land`, **including on the
  abort path** (`escort_show._stop_stream`). Otherwise the high-level commander
  is still being preempted by a stale stream and fights the landing.
* **Staleness is fatal.** A target built from mocap must hold position and then
  land when its input goes stale. Landing next to a confused operator beats
  orbiting a guess.
* Read live poses from `/poses`, never `/cfX/pose` (the escort's `PoseCache`),
  and subscribe **BEST_EFFORT** (`qos_profile_sensor_data`). A RELIABLE
  subscription is incompatible with the mocap publisher and silently receives
  nothing — that shipped broken once and read as the expected "no pose in sim".
* A live check that cannot distinguish *"the drones are too close"* from *"I
  cannot see the drones"* is worse than no check. Degrade explicitly
  (`escort_show._live`).
* Clamp every commanded position into the envelope on the way out, and
  timestamp the clamp so the LEDs can be matched to it afterwards.

---

## 4. Registering the show

1. Add the entry point in `src/crazyswarm2/crazyflie_shows/setup.cfg` under
   `[options.entry_points] console_scripts`:

   ```
   my_show = crazyflie_shows.my_show:main
   ```

   Register your planner the same way. A planner can share `plan_show`'s main
   with a different default, as `plan_constellation =
   crazyflie_shows.plan_show:main_constellation` does.
2. To get `plan_show --show my_show`, add an entry to the `SHOWS` dict in
   `plan_show.py`: `key -> (ConfigClass, build_plan, report_extras_or_None)`.
3. Rebuild and re-source:

   ```bash
   ./scripts/build.sh crazyflie_shows
   source install/setup.bash
   ros2 pkg prefix crazyflie            # must print THIS workspace -- apt can shadow it
   ros2 run crazyflie_shows my_show --ros-args -p dry_run:=true
   ```

**What the symlink install does and does not give you.** `build.sh` runs colcon
with `--symlink-install`. Checked in the install tree on this box (2026-10-08):
the installed `crazyflie_shows/*.py` are symlinks back into `src/`, so **editing
an existing `.py` is live with no rebuild**. But:

* a **new module file** gets its symlink only at build time — `abort.py`'s link
  carries the date of the build that followed its addition — so rebuild after
  adding one;
* a **new console-script entry point** is a generated shim, also created at build
  time: rebuild after editing `setup.cfg`. (Inferred from the generated scripts
  under `build/`; not tested by adding one.)
* data, launch and config files are installed per `CMakeLists.txt` — check it if
  you add one;
* `.msg`/`.srv` edits need the **dependents** rebuilt, not just the interface
  package (`CLAUDE.md`, Conventions).

A show that is registered but not rebuilt fails with `No executable found`,
which reads like a typo in the name and is not.

---

## 5. Traps a new author will hit

### 5.1 Sim needs `use_sim_time:=true`; hardware must not have it

```bash
ros2 launch crazyflie_shows show_launch.py backend:=sim
ros2 run crazyflie_shows my_show --ros-args -p use_sim_time:=true
```

The sim clock runs roughly 4× slower than wall time (`CLAUDE.md`), so without
the flag the script races ahead of the physics and every phase is cut short. On
hardware the flag must be **absent**, or `timeHelper` follows a clock the drones
are not on. Read it back the way the shows do —
`sim = bool(_param(allcfs, 'use_sim_time', False))` — and gate every sim-only
skip on that one variable.

### 5.2 Sim has no `/cfX/pose` and no `/cfX/status`: live checks must degrade

Read in `crazyflie_sim/crazyflie_server.py` (2026-10-08): the sim server creates
exactly one publisher per drone — `robot_description` — plus the services and
the `cmd_vel_legacy` / `cmd_hover` / `cmd_full_state` subscriptions. There is
**no pose publisher, no `/poses`, and no `/status`**. So `get_position()` returns
`[0, 0, 0]` and a live-position check reads the whole fleet as stacked on the
origin; a supervisor check would time out and report every drone silent. That is
why the shows skip both in sim — not because the checks are optional.

There is also no `arm` service in the sim server. That is harmless rather than a
hang only because `arm()` is `call_async` and the response is never awaited
(`crazyflie_py/crazyflie.py:498-511`) — the same property that makes a *refused*
arm invisible on hardware (§1.2).

Do what `swarm_show` does (skip, and say so) or what `escort_show._live` does
(reject a pose that disagrees and report that it did). Never let a check refuse
a good sim run: that teaches the operator to ignore it.

### 5.3 `cmdFullState`, not `cmdPosition`

`CrazyflieSIL.cmdPosition` is commented out (`crazyflie_sim/crazyflie_sil.py`
line 247) and the sim server subscribes only to `cmd_full_state`. A
`cmdPosition` show therefore **cannot be flown in sim at all** — it takes off,
hovers, and ignores every setpoint. Hardware accepts both. Use
`cmdFullState(pos, vel, acc, yaw, omega)`; feeding the target's own velocity
forward is better control than position alone anyway.

Related: `plan_start_trajectory`'s binding arity differs between cffirmware
builds, and `crazyflie_sil.py` probes it with `inspect.signature` at import. Never
hard-code either signature (`CLAUDE.md`).

### 5.4 LED convention

Green = connected and ready for a command; red = in flight / a script is in
control. The server sets green on connect; every `crazyflie_py` script sets red
for its lifetime and restores green at exit via `atexit` (it survives exceptions
and Ctrl-C; if rclpy is already shut down the deck keeps its last colour). Do
not fight this with your own colour at start or end.

Cues mid-show are fine: parameter `colorLedBot.wrgb8888`, absent in sim. Build
the values with `constellation.wrgb()`, which enforces the two rules the
escort's hand-written constants used to break — a full white channel washes
every other colour out on video. `setParam` is fire-and-forget; a failed cue
must never stop a show.

### 5.5 Things that look like a bug in your show and are not

Each of these has its full account in `CLAUDE.md`; this is only enough to
recognise it:

* **Fresh rclpy processes and `/parameter_events`.** A node's own parameter
  events never loop back on this rig, and `Crazyswarm()` in a fresh process can
  hang waiting on `all/emergency`. Run your show the way the others are run; do
  not work around it inside the show.
* **Telemetry is lossy by design on one radio.** Multi-second silences are
  normal with five drones, and flight does not depend on telemetry — mocap poses
  reach the drones as broadcasts with no ACK. Gate on telemetry once, with a
  long timeout, and name the drones you are still waiting for.
* **A drone that freezes mid-show** is most likely the log-block stall and the
  server's telemetry watchdog, not your show. The watchdog deliberately refuses
  to act on a drone that may be airborne.
* **The server owns the radio.** Never open a second `Crazyflie` link from a show.
* **Parameters** are declared on first use by `_param()`. Name yours the way the
  others are named, and document them in the module docstring table.
* **Do not name drones in prose or in constants.** Take the list from `allcfs`,
  and print the fleet with `./scripts/scan_fleet.sh --list`.

### 5.6 Frame-dependent constants must be re-expressed when Motive is recalibrated

This bit the escort on **2026-10-08**. Recalibrating Motive redefines the world
frame; the room, the cameras and the drones did not move. Anything written down
in world coordinates is now wrong, in one of three different ways:

| Kind of number | Example in this repo | Correct transform |
|---|---|---|
| a **point** | `safety.ARENA_CENTRE`, `initial_position` marks | **rotate and translate** |
| a **direction / offset** | `EscortConfig.vip_offset` ("0.60 m toward the operator") | **rotate** only |
| an **absolute bearing** | `AdversaryScript.legs` | **add the rotation angle** |

Distances and radii (`ARENA_RADIUS_*`, ceilings, ring radii) are invariant.

The 2026-10-08 transform, fitted on three undisturbed drones (max residual
0.0159 m): rotate **+91.59°**, translate **[+0.043, −0.098] m**. It was trusted
because those three drones' pairwise distances still agreed with the yaml to
≤ 21 mm, which proves the *frame* moved rather than the drones.

What went wrong, in order, so you can recognise the shape of it:

1. `safety.ARENA_CENTRE` was carried into the new frame — but `escort.py` kept
   its own literal copy of the old centre. Every escort geometry then used a
   centre **0.41 m** from the real one while `check_envelope` used the right one,
   so the check passed and the figure was wrong. Fixed by *deriving*
   (`room_center = tuple(safety.ARENA_CENTRE)`). **Never write a surveyed
   physical place down twice.**
2. Carrying the centre over was only half the job. `vip_offset` is a *direction*
   and still pointed at +0.0° while the operator and the DJI had been measured
   at +86.5° — the VIP mark was 90° away from the aircraft it describes. Rotated,
   it points at +91.6°, which doubles as a check on the frame fit.
3. The adversary's bearings are *absolute* and coupled to `vip_offset` — the
   invariant is "the adversary stands opposite the VIP offset", not "the number
   is near 180". Left at 145°, the 2.30 m stand-off landed at **2.701 m** from
   the arena centre: outside the 1.90 m plan clamp and outside even the 2.24 m
   radius where tracking was measured to fail. Rotated, 1.841 m — identical to
   what had flown before.

**Procedure after any Motive recalibration or rigid-body rename:**

1. Do not fly. Confirm the physical drones did not move, then re-sync the marks
   with `scripts/sync_initial_positions.py` (mocap up, server **not** started —
   `ros2 launch crazyflie launch.py server:=False`).
2. Fit the old→new transform from drones that did not move, and check the
   residual before you trust it.
3. Update `safety.ARENA_CENTRE` (the only copy), then **grep your own show for
   every literal coordinate, offset and angle** and transform each one *by its
   kind*. Prefer deriving from `safety` over writing a number down.
4. Re-run every planner.
5. Re-run `scripts/arena_flight_sweep.py` before trusting the arena **edges**
   again. A transformed survey is not a survey.

### 5.7 Two ways to be outside the envelope without noticing

* **A rigid formation spinning about its own centroid** reaches
  `|centroid − ARENA_CENTRE| + radius`, not `radius`. `demo_show` computes that
  point explicitly (`spin_worst`) rather than checking the slots.
* **A show that orbits a point *plus* each drone's own offset** adds the offset
  to the sweep. The formation demo's old `ROOM_CENTER` (0.359 m off the measured
  centre) plus the triangle offsets reached **2.418 m** — outside even the
  lost-tracking radius. Do not quote a fixed excursion for any show whose
  geometry depends on `initial_position`; compute it from the live marks.

---

## 6. Checklist before a new show flies

Nothing arms real drones until every line above it is ticked. Each level must
pass before the next is attempted.

**Offline — laptop, nothing powered on**
- [ ] `python3 -m py_compile` clean; no new warnings anywhere in the build.
- [ ] A planner exists, builds the **same** plan object the flight script flies,
      and passes on the current yaml.
- [ ] `./scripts/scan_fleet.sh --list` printed the fleet you think you planned for.
- [ ] Hover columns over the marks, home columns, every leg **and** the figure
      are inside `ARENA_RADIUS_PLAN` and `ceiling_at` (§1.3).
- [ ] Every simultaneous leg clears `PLAN_SEPARATION` — not `MIN_SEPARATION`.
- [ ] Speed and acceleration under `MAX_SPEED` / `MAX_ACCEL` (`check_show`).
- [ ] No literal world coordinate, offset or bearing that is not derived from
      `safety` (§5.6).
- [ ] Trajectory memory fits (`figures.MAX_PIECES`, asserted by the piece-offset
      allocator; the firmware has ~4 KB in total).

**Contract**
- [ ] `take_signals()` after `Crazyswarm()`; `armed = True` **before** the arm
      loop; `armed = False` only after the final disarm; `abort_land` in both
      `except` branches; the non-`ShowAborted` exception is re-raised.
- [ ] `report_supervisor` runs before any upload, on the drones that fly, skipped
      in sim, and **before** the `dry_run` exit.
- [ ] `dry_run:=true` exits after all read-only checks and before any upload or arm.
- [ ] Live checks degrade explicitly in sim (§5.2).
- [ ] Reactive show only: `cmdFullState`; `notifySetpointsStop` before every
      `goTo`/`land` including the abort; stale input → hold then land; a
      flight-time envelope clamp.

**Sim**
- [ ] `backend:=sim` with `-p use_sim_time:=true`: a full run, takeoff to disarm.
- [ ] **Abort tested three times** with `kill -INT $(pgrep -f my_show)`: during
      takeoff, mid-figure, and inside the arm loop (widen it with a temporary
      sleep). The drones must land and disarm every time.
- [ ] `kill -TERM` tested as well — that is what the console's Stop sends.
- [ ] An injected exception inside the figure lands the drones **and** prints the
      traceback.
- [ ] Peak speed and minimum separation measured in sim and compared against the
      plan. The planner is optimistic by controller lag; that is what
      `TRACKING_MARGIN` is for, and this is where you find out it is not enough.

**Hardware, first power-up — props OFF**
- [ ] `./scripts/scan_fleet.sh` reports GO for every enabled address (the server
      blocks forever and silently on the first unreachable enabled drone).
- [ ] Mocap alive: `ros2 topic hz /poses`, and `ss -uanp | grep :1511` shows one
      owner.
- [ ] `ROS_DOMAIN_ID` set, so a colleague's `/all/takeoff` cannot reach your fleet.
- [ ] Preflight GUI green for every drone that will fly; yaw banner clean.
- [ ] Drones on their own marks; planner re-run **after** the last position sync.
- [ ] `dry_run:=true` passes on hardware, not just in sim.

**Hardware, first flight**
- [ ] One person on the E-STOP (console, or `e` in the preflight GUI) and nothing
      else; everyone else outside the arena.
- [ ] Fly reduced first if the show supports it — fewer drones, or `--scale`.
- [ ] Confirm each drone's **mocap altitude** after takeoff. It is the only proof
      the arm was accepted (§1.2).
- [ ] Do the first abort deliberately: low, early, before the first full run.
- [ ] Afterwards, compare flown separation and envelope against the plan, and
      write the measured numbers **with the date** next to the constants they
      justify. That is how every number in `safety.py` got there.
- [ ] Add an operator runcard in `runbooks/`, a design doc beside the show, and a
      line in `CLAUDE.md`'s layout section.

---

## 7. Keeping the show separable

`src/crazyswarm2/crazyflie_shows/` is meant to stay a clean, donatable subtree:
design docs live beside the shows, operator runcards live in `runbooks/`, and
nothing in `src/` outside the package depends on `console/`.

Keep that property:

* Put your show's design notes **beside the show**, its runcard in `runbooks/`.
* **Import** the shared machinery — `abort`, `preflight`, `safety` — rather than
  copying it. Copying is exactly how `swarm_show` and `demo_show` ended up with
  no abort path for three weeks.
* Do not reach into `console/` from a show, and do not make anything in `src/`
  import from a show.

---

## What this document does not know

Stated so the next editor does not mistake confidence for verification:

* The skeleton in §3 compiles and its symbols were checked against the current
  source, but it has never been run in sim or on hardware.
* §2's planner pattern generalises from `plan_show.py` and from what the show
  scripts import. `plan_escort`'s and `plan_constellation`'s internals, and
  `choreography.py` / `constellation.py` in full, were not read line by line for
  this guide.
* The carousel's "~99 % of its separation budget" is quoted from `CLAUDE.md`,
  not re-measured here. Run `plan_show` for the current number.
* The entry-point rebuild requirement in §4 is inferred from the generated
  scripts under `build/`; the module-symlink behaviour was confirmed directly
  from the install tree. Adding a new entry point was not tested end to end.
* §6's trajectory-memory line relies on `figures.MAX_PIECES` and `CLAUDE.md`;
  the byte budget was not recomputed.
* How to donate a show (or `console/`) upstream to AI-DA-STC is deliberately out
  of scope here — §7 only says how to keep that option open.
