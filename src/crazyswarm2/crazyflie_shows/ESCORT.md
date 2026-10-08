# ESCORT — three defenders, one VIP, one adversary

A reactive demo, not a choreographed show: three drones hold a ring around a
VIP and turn that ring to put a defender between the VIP and an approaching
adversary. Built for the area-denial demo of **8 Oct 2026**, which took place:
see [What flew, and what broke](#what-flew-and-what-broke-as-of-2026-10-08).

> **Frame warning.** The 2026-10-08 Motive recalibration rotated the world frame
> by +91.59 degrees ([MOCAP.md 2c](../../../docs/MOCAP.md#2c-the-world-frame-the-measured-arena-and-what-a-recalibration-breaks)).
> Some worked numbers below were written in the OLD frame and say so where they
> are known to be stale; **for any current mark, offset or radius, print it**
> (`plan_escort --marks`, `plan_escort`, the `EscortConfig` source) rather than
> trusting this prose.

| File | What it is |
|---|---|
| `crazyflie_shows/escort.py` | the whole control law, pure Python, no ROS |
| `crazyflie_shows/plan_escort.py` | offline verification — simulates the encounter and refuses bad numbers |
| `crazyflie_shows/escort_show.py` | the flight script (mocap + radio) |
| `crazyflie_shows/escort_teleop.py` | keyboard-steered virtual VIP or adversary, for testing with nobody in the room |
| `crazyflie_shows/escort_viz.py` | RViz markers for the ring and the threat |
| `scripts/place_drones.py` | floor-placement helper for the marks |
| `scripts/watch_escort.py` | read-only run monitor (see "Monitoring") |

```bash
python3 -m crazyflie_shows.plan_escort --marks       # where to stand the drones
python3 -m crazyflie_shows.plan_escort --sweep       # no ROS needed at all
ros2 run crazyflie_shows plan_escort --plot /tmp/escort.png

ros2 launch crazyflie launch.py backend:=sim
ros2 run crazyflie_shows escort_show --ros-args -p use_sim_time:=true \
    -p vip_mode:=point -p check_placement:=False
```

## The law

Three slots, evenly spaced on a circle of radius `R` centred on the VIP, at a
fixed altitude. `psi` is the ring's phase; slot *i* sits at `psi + 2*pi*i/3`.

```
v̂_V  = lowpass_4Hz(d/dt p_V)                     VIP velocity, from /poses
phi_A = atan2(p_A - p_V)                          threat bearing
psi  <- psi + clamp(wrap(phi_A - psi), ±phase_rate*dt)
p_i* = p_V + R*[cos(psi + 2*pi*i/3), sin(psi + 2*pi*i/3)] + h*ẑ  (+ v̂_V/rate)
```

Each slot target then goes through `SetpointGuard`: slew by `a_max` and
`v_max`, **then** re-assert the separations, **then** the room. Blocking
engages when the adversary is within `alert_radius` of the VIP and releases at
`release_radius` (hysteresis, so it cannot chatter).

Three choices worth knowing:

* **The ring rotates; the drones never swap slots.** Slot order is fixed once,
  at gather, by current bearing. Reassignment would have two drones trading
  places through the middle of a ring that contains a person, and this rig has
  no onboard collision avoidance.
* **The blocker stays on the ring.** It does not pursue. The pure-pursuit
  defender law is in the docstrings as the alternative if a future version
  wants it, but chasing an adversary next to a person spends the entire
  separation budget to look marginally more aggressive.
* **Bad poses are rejected, not acted on.** A defender's live pose that
  disagrees with its commanded setpoint by more than a ring radius is treated
  as a bad input (stale estimate, flipped rigid body, fallback source) and the
  commanded setpoint is used instead, with a line in the log. Acting on it
  instead made the guard shove setpoints around on a ring whose slots are
  2.6 m apart — found in sim, and the same input is the fly-away case on
  hardware.
* **Clamp order is load-bearing.** Slew first, then separations, and always
  finish with the VIP clamp. Both halves of that were found by running
  `plan_escort`, not by reasoning: a single pass let the push away from the
  adversary shove a defender to 1.27 m from the VIP against a 1.50 m minimum.

## What the sources support, and what is ours

Backed by published work: the escort ring itself (circular formation around a
leader that holds while the leader moves, arXiv 2212.12554, built on
Olfati-Saber), encirclement of a moving target at a stand-off distance
(IET CTA 2022), and the vertical-separation rule — downwash from a drone above
costs the one below enough thrust to matter, negligible only above
`dz/l > 19`, about 0.62 m for a Crazyflie (arXiv 2507.09463, arXiv 1704.04852).

Ours, not from any source: **using three defenders at all.** Every defender
paper found in the 2026-09-22 survey is one defender against one attacker. The
ring is therefore the thing that gets verified by `plan_escort` rather than
something with a citable guarantee behind it.

Deliberately not used: the equal-spacing orbit law flown on Crazyflies under
Vicon (arXiv 2103.11574). It assumes the agents are much faster than the
target, which for a walking person means defender speeds above ~3 m/s.

## What flew, and what broke (as of 2026-10-08)

Sourced from commit messages, the code comments they left, the runbooks and
the maintainer's notes; **there is no flight log document**, so treat dates
and outcomes as the best available record, not a data set.

* **2026-10-07, hardware.** The escort flew, with a hand-flown DJI as the VIP. That day's fixes name what went
  wrong: `be27585` "engage earlier, because the attacker was getting through"
  (`alert_radius` 1.88 -> 2.10 m, `release_radius` 2.00 -> 2.22 m); `bc24892` added
  per-drone containment after a **fly-away of the attacker**: the VIP had a geofence and a
  staleness watchdog, the Crazyflies had neither, and `self_stale_s` was dead
  config (declared, never read). The geofence was timestamped so the LED
  colour changes could be matched to events afterwards, which had been
  impossible after that run.
* **2026-10-08, hardware, shown to guests.** Several runs; the last was clean.
  The same morning Motive was recalibrated and the frame rotated 91.59
  degrees, which silently broke `vip_offset`, the adversary's absolute bearings
  and a private copy of the arena centre ([WRITING-A-SHOW §5.6](../../../docs/WRITING-A-SHOW.md)).
  The fixes are `f6ab833` and `c38f0ab` (both 2026-10-08).
* **Not cleared:** the walking VIP. With the shipped numbers `plan_escort`
  measures the ring holding only to a 0.20 m/s walk (and the runcard states the
  failing margins). The demo flew with a hand-piloted DJI as the VIP.
* **Open:** adversary tracking near the arena edge. The adversary's waiting
  station is about 1.84 m from the arena centre, the setpoint clamp is
  1.90 m and tracking was lost at 2.24 m (`ARENA_RADIUS_LOST`); the
  maintainer's notes list four candidate explanations that are confounded with
  each other, and none has been isolated.
* **Never measured on hardware:** a separation margin for the escort; mocap
  occlusion rate of a head-worn rigid body.

## Reactive adversary, geofence, containment, lights

Added after this document's first draft; the code comments in `escort.py` and
`escort_show.py` are the detailed source and the runcard is the operator view.

* **`ReactiveAdversary`** (default adversary) replaces the fixed script with a
  potential field: pulled toward the VIP, pushed by the nearest defender, held
  inside the room, and it STANDS DOWN when blocked. `v_max` 0.80 m/s is
  deliberately FASTER than a defender's 0.60: the race is angular and the
  defenders are on the inside track, so the ring (turning at
  `engaged_phase_rate` 0.55 rad/s) only loses beyond 0.55 x 1.80 = 0.99 m/s.
  Making it slower made the demo look unaggressive. It cannot be proven like a
  script; `SetpointGuard` still enforces the separations. `defenders_yield`
  (default False) and `defender_yield_floor` govern whether defenders give way
  to it.
* **Geofence.** The VIP is held inside `arena_radius - ring_radius` of room
  centre (with hysteresis, `FENCE_HYST`); beyond it the ring HOLDS and the
  defenders go CYAN until the VIP comes back. Every episode is printed with
  wall clock at the end so the LEDs can be laid against the video.
* **Containment, every drone** (`escort.contain_check`, `bc24892`): per-drone
  stale watchdog (`self_stale_s` / `self_stale_land_s`) and a trip when a
  drone's ACTUAL pose passes `arena_radius + contain_margin` (0.25 m), or when
  it stays `track_error_m` off its own command for `track_error_s`. It
  covers the adversary and an `adversary:=external` drone too.
* **Lights.** Colours are `LED_*` constants in `escort_show.py`; the legend an
  audience is told is in `runbooks/ESCORT.narrative.md`. Cyan is reserved for
  the geofence hold. Cues are sent after motion commands, may never stop the
  show, and are off under `use_sim_time`.
* **Monitoring.** `scripts/watch_escort.py` is read-only by construction
  (subscriptions only) and reports mocap gaps per body including the DJI,
  radius against clamp / containment / lost-tracking, commanded versus actual
  position, minimum pair separation and any defender at or below the DJI's
  altitude.

## Decisions still open

Each of these has a working default, and each is someone's call, not the
code's. `plan_escort` prints the consequences of whatever is chosen.

1. **Speed cap vs walking speed.** `v_max = 0.6 m/s` is inherited from the
   follow-drone prototype. Turning the ring costs `R * phase_rate` of it, so
   the worst-case budget left for following the VIP is
   `max_vip_speed = 0.6 - 1.0*0.3 = 0.30 m/s` — that is the number to quote
   when the ring is turning and the VIP is walking the same way at once.
   (It was 0.15 m/s at `R = 1.5`; shrinking the ring to 1.0 m doubled it, which
   is the one place the smaller ring pays rather than costs.)
   `plan_escort --sweep` measures the stricter, end-to-end case — every
   separation held for a whole run, not just the ring's shape — and with the
   shipped numbers it **clears up to a 0.20 m/s walk** (2026-10-01). The
   binding constraint is that `min_vip_dist` equals `ring_radius`, so any
   inward lag at all is a violation: at 0.30 m/s the defenders sit 0.94 m from
   a 1.00 m floor. Brief the pilot at **~0.2 m/s** — a hover with small
   drifts. A normal walk is 1.0–1.4 m/s. Options,
   in order of how much they cost elsewhere: have the VIP walk deliberately
   slowly; slow the ring (a lazier block); shrink `R` (less room between the
   drones and the person); raise `v_max` (a real safety decision about flying
   faster next to a human, and the one that needs a written justification).
2. **Where the VIP stands.** `vip_offset` is a *direction and distance* from
   room centre, shipped as `(-0.017, +0.600)` since the 2026-10-08 frame
   rotation (it was `(+0.60, 0)`, "the +x side, nearest the operator", in the
   old frame; rotated by +91.59 degrees it points at +91.6 degrees and still
   points at the operator and the DJI, which were measured at +86.5 degrees).
   The adversary starts on the opposite side, so it runs at the VIP across the
   far half of the room and the encounter happens facing the audience. In the
   new frame that is from -y, not -x. Do not reason from "+x" or "-x" here:
   print `plan_escort --marks`. Centring the VIP does not work: it leaves
   `arena_radius - (ring_radius + min_adv_sep)` of stand-off, so the arena clamp
   drags the adversary inside the alert radius immediately and the demo
   begins already blocked (that is exactly what the first sim run did). The
   offset also constrains the adversary's approach bearings to the open side —
   `AdversaryScript.check` verifies every leg against the arena and refuses
   the ones that would end up in a wall. The cost of the offset is the pilot
   box: the geofence measures the VIP's stray from **room centre**, so the DJI
   gets `arena - ring - offset` further toward the operator and
   `arena - ring + offset` away from them (with `vip_offset` 0.60 m: 0.30 m
   and 1.50 m at the current arena and ring; derived, not printed by a tool).
3. **Ring radius and altitude.** `R = 1.0 m`, down from 1.5 via 1.2. Each step
   was paid for by the measured volume (see *The arena* below) and the last one
   was asked for on sight: a 1.2 m ring reads as a loose circle rather than an
   escort, and the 0.2 m it frees went into `vip_offset`, which moved the
   encounter toward the audience and took the adversary's mark off the arena
   edge (2.00 m from room centre at `R = 1.2`, 1.50 m at `R = 1.0`).
   `min_vip_dist` tracks `R` exactly, so the guard sits on its own boundary
   with no margin — that is what caps the walk at 0.20 m/s. `h = 1.2 m` is
   chest height on a standing adult, and the altitude the DJI is flown at.
   **If a HUMAN ever stands in as the VIP, put `R` back to 1.5 m** and accept
   that the demo then does not fit this room.
4. **The arena — MEASURED 2026-10-01, no longer open.**
   The tested radius is 2.00 m. The escort PLANS to
   `arena_radius = safety.ARENA_RADIUS_PLAN` (1.90 m, tested minus the tracking
   margin), `ceiling = 1.85 m`, and `room_center = safety.ARENA_CENTRE`,
   derived rather than copied (a private copy of the old centre sat 0.41 m
   from the real one after the 2026-10-08 recalibration and the planner
   passed anyway). The centre is the centroid of the tracked space, not the
   room's middle. (The 2026-10-01 survey was expressed in the frame of that
   date; the radii are frame-invariant, the centre was carried across.) Walking cf1 through the volume
   (`scripts/measure_arena.py`, 8067 samples) then flying it
   (`scripts/arena_flight_sweep.py`) put the first sustained dropout at 2.24 m
   and verified 72 waypoints inside 2.00 m with zero stale poses at both
   1.20 m and 1.95 m altitude. Tracking is not isotropic: by 45° sector the
   first dropout sits at 2.15 m (270–315°), 2.25 m (0–45°), 2.31 m (225–270°)
   and never in 180–225°, which held to 2.46 m. The worst sector binds. The
   numbers live in `crazyflie_shows.safety` so the other shows can use them.
5. **Which drones.** Defaults are now chosen by GEOMETRY, not name order: the
   three drones nearest the VIP mark defend, the farthest one attacks
   (`escort.pick_roles`). Override with
   `-p defenders:=cf1,cf2,cf3 -p adversary_drone:=cf5` (a comma-separated
   string, not a list: an empty-list ROS parameter default is inferred as
   BYTE_ARRAY and rejects string values) — but run it with `dry_run:=true`
   first, because an override is exactly how you get an adversary parked
   inside the ring.
6. **The DJI adversary.** Out of scope for the 8 Oct demo as an ADVERSARY (the
   DJI flew as the VIP, hand-piloted). Nothing in the survey
   verified its prop-wash risk to 30 g drones, its indoor stability without
   GPS, or the netting it would need. A Tello tracks as an ordinary rigid body
   and would arrive on `/poses` like any other, so `adversary:=external`
   already supports it whenever someone decides to try.

## The VIP: a point, a person, or a DJI

Three ways to be the thing that gets escorted:

| Mode | What the VIP is | Used for |
|---|---|---|
| `vip_mode:=point` | a fixed coordinate | sim, and the first hardware stage |
| `vip_mode:=manual` | a point you steer with the keyboard | sim and dry runs |
| `vip_mode:=mocap` | a rigid body — a person's hat, or **a hand-flown DJI** | the live demo |

A DJI needs no config here. `/poses` carries every rigid body Motive reports
(the driver publishes `mocap->rigidBodies()`, not just the ones listed in
`crazyflies.yaml`), so it is enough to build the body in Motive with an
asymmetric marker set and pass `-p vip_name:=<that name>`.

**An airborne VIP changes the altitude rule.** `vip_airborne` (true by default
for `vip_mode:=mocap`) makes the ring hold level with the VIP instead of at a
fixed height. A fixed height would let the DJI climb over the defenders or
drop beneath them, and a Crazyflie under a DJI is under a downwash far beyond
the one that already costs a Crazyflie its thrust at 0.62 m of separation.
Level is the only safe relative altitude, and it is the one the pilot does not
have to think about. If the pilot leaves the altitude band the defenders stay
inside it and the gap is reported, not chased.

### What the pilot has to stay inside

Two numbers, both printed by `plan_escort --vip-z <height>` and by the flight
script's banner:

* **Keep-in radius** = `arena_radius - ring_radius`. Outside it the ring no
  longer fits in the arena and the far slots get clamped.
* **VIP speed** = `max_vip_speed` = `v_max - ring_radius * phase_rate`.

With the shipped config that is **`arena_radius - ring_radius` = 0.90 m of
room centre** (the runcard's "pilot box"; the earlier 1.00 m here was computed
at arena 2.0) **and 0.30 m/s** in the worst case, and `--sweep` clears the whole run only to **0.20 m/s** — a hover
with small drifts, not a flight. That is the honest state of it, and the knobs
trade against each other (keep-in = `arena - R`, VIP max = `v_max - R*rate`):

| arena | ring R | phase_rate | v_max | keep-in | VIP max |
|---|---|---|---|---|---|
| 1.9 m | 1.0 m | 0.30 | 0.6 | 0.90 m | 0.30 m/s | ← shipped arena and ring
| 1.9 m | 1.0 m | 0.15 | 0.6 | 0.90 m | 0.45 m/s |
| 1.9 m | 1.2 m | 0.30 | 0.6 | 0.70 m | 0.24 m/s |
| 1.9 m | 1.5 m | 0.30 | 0.6 | 0.40 m | 0.15 m/s |
| 1.9 m | 1.0 m | 0.15 | 1.0 | 0.90 m | 0.85 m/s |

(`phase_rate` and `v_max` rows are the plain formulas; the table was re-based
from arena 2.0 to 1.9 on 2026-10-08 by recomputing `arena - R`. The shipped
`phase_rate` is 0.30 rad/s while blocking is idle and `engaged_phase_rate`
0.55 rad/s once it engages -- see the code.)

Halving `phase_rate` buys the most for the least: the wall then closes in
about 8 s instead of 4 s, which is slower to watch but costs no separation.
Raising `v_max` is the one that needs a written safety argument, because it is
the speed the defenders fly at next to whatever they are escorting.

**The arena lever has now been pulled, and it went the wrong way.** The volume
measures 2.24 m to first dropout, not the 3.5 m that would have made this
comfortable, so `arena_radius = 2.0 m` and every other number was fitted down
to it rather than up. There is no more room to find by measuring; the next
0.1 m would have to come from a verification sweep out to ~2.15 m.

**`ring_radius` is also the chord the wall closes to.** The lead and each wing
stand `2*R*sin(wall_half_angle/2)` apart, so the angle cannot be set without
knowing the radius: 50° held 1.01 m at `R = 1.2` and only 0.85 m at `R = 1.0`,
under the 0.90 m plan budget, which is why `wall_half_angle` opened to 58°
(0.97 m) when the ring shrank. **Change one and re-run `plan_escort`.**

**A DJI standoff is still unset.** `min_vip_dist` is 1.0 m, and it is now a
number chosen to fit a room rather than measured against a DJI. Nothing has
verified what a DJI's prop wash does to a 30 g Crazyflie at that range. The
mitigation in place is vertical, not horizontal — see `vip_airborne` and
`vip_height_offset` in `escort.py` — and the horizontal figure still wants a
measurement rather than a guess.

## Operator-paced: the show waits for you

The encounter involves three defenders, a scripted attacker and a human flying
a DJI. Pacing that on a wall clock asks every party to be ready at a time none
of them chose, which is how coordination accidents happen.

```bash
ros2 run crazyflie_shows escort_show --ros-args -p paced:=true
```

The adversary then holds its current leg until you press Enter, and arming
waits for you as well. Each press announces what the attacker is about to do
(`leg 2/6: PROBE 1 - run at the VIP`), so you commit to the next move knowing
what it is. `q` lands.

**What pacing does NOT slow down.** Show time still runs. The ring still turns
at `phase_rate`, every `SetpointGuard` clamp is still enforced per setpoint,
and the mocap staleness watchdogs are real-time (the VIP's: 0.4 s hold,
2.0 s land; each drone's own, `self_stale_s` 0.3 s hold and
`self_stale_land_s` 1.0 s land, added 2026-10-07). Only
the attacker's place in its script is yours. The defence stays automatic
because it is the thing being demonstrated and the thing that must not wait
for a human.

Input is read as a LINE, not a keypress, so it works from a plain terminal and
from the mission console's stdin box -- no termios, and nothing that breaks if
stdout is not a tty.

## Manual control: testing with nobody in the room

`escort_teleop` publishes a point you steer with the keyboard, so the demo can
be exercised with no person and no mocap hat. Two independent targets:

```bash
# terminal 2 -- a virtual VIP for the defenders to escort
ros2 run crazyflie_shows escort_teleop --ros-args -p target:=vip
ros2 run crazyflie_shows escort_show   --ros-args -p vip_mode:=manual

# or terminal 2 -- fly the adversary drone by hand instead of the script
ros2 run crazyflie_shows escort_teleop --ros-args -p target:=adversary
ros2 run crazyflie_shows escort_show   --ros-args -p adversary:=manual
```

Keys are `wasd` or the arrows in **plan view** — the orientation
`plan_escort --plot` draws, x right and y up — plus `q`/`e` for the
adversary's altitude, `space` to stop, `c` to re-centre, `x` to quit. Hold a
key and the terminal's auto-repeat keeps it moving; the velocity decays
`key_timeout` (0.35 s) after the last keystroke, because a terminal has no
key-release event.

Four properties worth knowing, each of which is a deliberate refusal:

* **A VIP target defaults to `escort.max_vip_speed`** (0.30 m/s as shipped;
  this page said 0.15 m/s, which was the value at the old `R = 1.5` ring --
  see §"VIP speed" above, where the same document gets it right),
  not to something that feels responsive. That is the speed budget the ring
  actually has after turning; steering a virtual person faster tests whether
  the defenders fall behind, which is a fine thing to test **on purpose** with
  `-p speed:=...`, and a misleading thing to do by accident.
* **The demo refuses to arm until the teleop is publishing.** A manual target
  that never arrives is a flight that takes off, holds, and lands.
* **A stale manual target is treated exactly like a lost mocap body**: the VIP
  going quiet holds the ring at 0.4 s and lands it at 2.0 s, so closing the
  teleop is a legitimate way to end a test. A stale *adversary* only freezes
  the adversary — there is no threat moving, but the person is still fine.
* **The teleop clamps to the arena itself**, so what you steer is what the
  flight script will try to fly.

`duration` (seconds) bounds the run; it defaults to the script's own length
for `adversary:=scripted` and 120 s when a human is driving.

## Where to stand the drones

**The VIP mark is not inferred from the fleet.** The shows centre themselves on
the centroid of `initial_position`; the escort does not — `room_center` and
`vip_offset` are fixed in `EscortConfig`, so syncing positions moves the
*drones* in the plan and never moves the mark. Place the drones around the
mark, not the other way round.

`plan_escort --marks` prints the marks and then checks the yaml against them:

```
VIP mark        [x, y]   room centre + vip_offset -- NOT inferred
defender 1..3   [x, y]   on the ring, ring_radius from the VIP mark
adversary       [x, y]   adversary_start_dist out, on the far side of the VIP
```

(Placeholders on purpose. Marks moved twice in this document's life: when
`vip_offset`/`ring_radius` changed, and again on 2026-10-08 when the world
frame rotated, which left every number that used to be printed here wrong.
**Run `plan_escort --marks`; do not copy numbers from prose.**)

Three rules behind those numbers:

1. **Defenders stand on the ring they will hold** (1.00 m from the mark, 120°
   apart). The gather is then a lift rather than a march, and the slot
   assignment — which goes by bearing from the mark — is unambiguous. Three
   drones bunched in one corner share almost the same bearing, which makes the
   assignment arbitrary and the legs long.
2. **The adversary starts outside `ring_radius + min_adv_sep`** (1.80 m, and
   the mark is placed 0.3 m further out still). Any
   closer and it begins the demo already inside the ring, and its first leg out
   crosses the defenders.
3. **Everything stays inside the arena and at least 1 m apart** — the sync tool
   refuses to write marks closer than 1 m, and the phase `--marks` picks is the
   one that keeps the ring furthest from the walls (it prints
   how far the ring reaches; the figure quoted here earlier was in the old
   frame).

## The gather is checked, as of 2026-09-24

The defenders and the adversary fly onto the ring TOGETHER, on straight goTo
legs, with no onboard collision avoidance — the same shape as the 2026-08-04
collision. That leg used to be unverified. It is now checked twice against
`safety.PLAN_SEPARATION` (0.90 m), by the same closed-form routine the shows
use:

* **before arming**, from the yaml marks, so a bad placement can still be
  fixed by moving a drone;
* **after takeoff**, from the live poses, where a refusal lands instead of
  gathering.

This was not theoretical. With the marks of 2026-09-24 and the OLD default
roles (first three names defend, fourth attacks), the four gather legs close
to **0.00 m** — cf5 outbound and cf3 inbound cross head-on. Checking only the
three defenders misses it entirely: they clear at 1.27 m. The adversary flies
the same leg at the same time and has to be in the check.

`escort.adversary_start_problem` is the second half: an adversary inside
`ring_radius + min_adv_sep` has already won before the demo starts, and its
first leg out goes through the defenders' slots.

## Before any hardware flight

1. `plan_escort` — it refuses configurations, and it is the whole point.
2. Sim, with `vip_mode:=point`.
3. Scan every enabled address. The server blocks forever, silently, on the
   first enabled drone that does not answer.
4. Preflight GUI, per-drone go/no-go, including `err.yaw`.
5. `ROS_DOMAIN_ID` set — domain 0 is shared with the lab.
6. Only then a person in the volume, and only at stage 3 below.

## Staging

| Stage | What runs | Gate to the next one |
|---|---|---|
| 1 | sim, static VIP point, scripted adversary | the block reads clearly in RViz |
| 2 | hardware, static VIP point, no person in the volume | separations match `plan_escort` |
| 3 | hat on a pole, carried, then worn; walk slowly | hat never drops out of Motive for > 0.4 s |
| 4 | adversary Crazyflie (reactive by default, or scripted), red LED | the blocker gets between them every time |
| 5 | teleoperated Crazyflie adversary (`adversary:=external`) | operator can fly it without chasing the ring |
| 6 | dress rehearsals | — |

## Sim gotcha: one run per server

`crazyflie_sim` logs `Notify setpoint stop not yet implemented`, so a drone
that has been given streamed setpoints stays in low-level mode forever. The
*next* run's gather `goTo` then kills the sim server outright with
`ValueError: goTo from low-level modes not yet supported` — it looks like the
escort hanging at "gathering onto the ring", and the reason is only in the
launch log. **Restart `ros2 launch` between sim runs.** Hardware implements
`notifySetpointsStop` properly and does not have this problem.

## Sim does not exercise the pose path at all

`backend:=sim` publishes no `/poses`, and `cf.get_position()` there returns
`[0, 0, 0]`. Both trust checks fire on every sim run and say so:

```
cf1: live pose [0.0, 0.0, 0.0] is not near its slot [-0.95, -1.6, 1.2] -- seeding from the slot instead
[t+  0.0s] ignoring the live pose of cf1, cf3 -- too far from what was commanded to be believable
```

That is the correct behaviour (it is what stops a setpoint being seeded at
floor level), but it means **sim verifies the geometry, the sequencing and the
clamps, and nothing about reading real poses.** The defender-defender clamp
and the `_live` path get their first real exercise on hardware, at stage 2.

## Known gaps

* **Hardware status is in the next section, not here.** The sim sequence of
  2026-09-23 (takeoff, gather, 40 s of streamed escort with a scripted
  adversary, land, disarm) ran clean and found the gather/resync bug and the
  arena squeeze; `plan_escort`'s drone model is a first-order lag, not the real
  controller. `plan_show` carries a measured 0.10 m `TRACKING_MARGIN`; the
  escort now has `contain_margin` and real flight data, but no separately
  measured separation margin.
* **Mocap occlusion of the hat is unaddressed.** The stale-pose behaviour
  (hold at 0.4 s, land at 2.0 s) is implemented, but how often a head-worn
  rigid body actually drops out here is unknown — and it is the most likely
  thing to end a demo.
* **The radio budget** was assumed on 2026-09-23 (no Crazyradio plugged in
  then). Since then it has been measured on the rig: a healthy drone shows
  ~175 packets/s sent and ~168 unicast received (CLAUDE.md, 2026-10-05, a
  Crazyradio 2.0), so four drones at 20 Hz of setpoints on top is a real
  fraction of the link, not a rounding error. Whether the setpoint stream
  contributed to any telemetry stall is unmeasured.
* **Yaw is commanded as 0 for every drone.** Defenders do not turn to face
  the VIP or the threat. It would look better if they did; it is also more to
  go wrong, so it was left out.
