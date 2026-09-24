# ESCORT — three defenders, one VIP, one adversary

A reactive demo, not a choreographed show: three drones hold a ring around a
VIP and turn that ring to put a defender between the VIP and an approaching
adversary. Built for the area-denial demo due **8 Oct 2026**.

| File | What it is |
|---|---|
| `crazyflie_shows/escort.py` | the whole control law, pure Python, no ROS |
| `crazyflie_shows/plan_escort.py` | offline verification — simulates the encounter and refuses bad numbers |
| `crazyflie_shows/escort_show.py` | the flight script (mocap + radio) |

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

## Decisions still open

Each of these has a working default, and each is someone's call, not the
code's. `plan_escort` prints the consequences of whatever is chosen.

1. **Speed cap vs walking speed.** `v_max = 0.6 m/s` is inherited from the
   follow-drone prototype. Turning the ring costs `R * phase_rate` of it, so
   the worst-case budget left for following the VIP is
   `max_vip_speed = 0.6 - 1.5*0.3 = 0.15 m/s` — that is the number to quote
   when the ring is turning and the VIP is walking the same way at once.
   `plan_escort --sweep` measures the looser, average case: **the ring still
   holds its shape (worst slot error under 0.5 m) up to a 0.90 m/s walk**, but
   by 0.3 m/s the separation guard is already firing on a third of the steps,
   i.e. the geometry has stopped doing the work and the clamps have started.
   Treat 0.15 m/s as designed-for, 0.3 m/s as demonstrated-with-clamping, and
   0.9 m/s as the edge of the cliff. A normal walk is 1.0–1.4 m/s. Options,
   in order of how much they cost elsewhere: have the VIP walk deliberately
   slowly; slow the ring (a lazier block); shrink `R` (less room between the
   drones and the person); raise `v_max` (a real safety decision about flying
   faster next to a human, and the one that needs a written justification).
2. **Where the VIP stands.** `vip_offset = (-1.0, 0)` from room centre, so the
   adversary has the far half of the room to approach through. Centring the
   VIP does not work: `arena_radius - (ring_radius + min_adv_sep) = 0.2 m` of
   stand-off, so the arena clamp drags the adversary inside the alert radius
   immediately and the demo begins already blocked (that is exactly what the
   first sim run did). The offset also constrains the adversary's approach
   bearings to the open side — `AdversaryScript.check` verifies every leg
   against the arena and refuses the ones that would end up in a wall.
3. **Ring radius and altitude.** `R = 1.5 m` equals the minimum VIP standoff,
   so the guard sits exactly on its boundary with no margin, and `h = 1.2 m`
   is chest height on a standing adult. Neither has been measured against
   this room or agreed with whoever is going to stand inside the ring.
4. **The arena.** `arena_radius = 2.5 m`, `ceiling = 2.0 m` are inherited from
   `crazyflie_shows.safety` and have never been checked against the volume
   Motive actually covers (the same gap HANDOVER.md section 8 flags for the
   shows).
5. **Which drones.** Defaults are now chosen by GEOMETRY, not name order: the
   three drones nearest the VIP mark defend, the farthest one attacks
   (`escort.pick_roles`). Override with
   `-p defenders:=cf1,cf2,cf3 -p adversary_drone:=cf5` (a comma-separated
   string, not a list: an empty-list ROS parameter default is inferred as
   BYTE_ARRAY and rejects string values) — but run it with `dry_run:=true`
   first, because an override is exactly how you get an adversary parked
   inside the ring.
6. **The DJI adversary.** Out of scope for 8 Oct. Nothing in the survey
   verified its prop-wash risk to 30 g drones, its indoor stability without
   GPS, or the netting it would need. A Tello tracks as an ordinary rigid body
   and would arrive on `/poses` like any other, so `adversary:=external`
   already supports it whenever someone decides to try.

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

* **A VIP target defaults to `escort.max_vip_speed`** (0.15 m/s as shipped),
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
defender 1   [+0.55, -0.10]      on the ring, 1.50 m from the VIP mark
defender 2   [-1.70, +1.20]
defender 3   [-1.70, -1.40]
adversary    [+1.65, -0.10]      2.60 m out, on the far side of the VIP
```

Three rules behind those numbers:

1. **Defenders stand on the ring they will hold** (1.50 m from the mark, 120°
   apart). The gather is then a lift rather than a march, and the slot
   assignment — which goes by bearing from the mark — is unambiguous. Three
   drones bunched in one corner share almost the same bearing, which makes the
   assignment arbitrary and the legs long.
2. **The adversary starts outside `ring_radius + min_adv_sep`** (2.30 m). Any
   closer and it begins the demo already inside the ring, and its first leg out
   crosses the defenders.
3. **Everything stays inside the arena and at least 1 m apart** — the sync tool
   refuses to write marks closer than 1 m, and the phase `--marks` picks is the
   one that keeps the ring furthest from the walls (2.18 m of the 2.50 m arena
   with the shipped config, against 2.50 m — i.e. touching — at the worst
   phase).

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
| 4 | scripted adversary Crazyflie, red LED | the blocker gets between them every time |
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

* **Flown in sim, never on hardware.** The full sequence (takeoff, gather,
  40 s of streamed escort with a scripted adversary, land, disarm) ran clean
  against `backend:=sim` on 2026-09-23. That run is also what found the
  gather/resync bug and the arena squeeze above — but sim is not a substitute:
  `plan_escort`'s drone model is a first-order lag, not the real controller.
  `plan_show` carries a measured 0.10 m `TRACKING_MARGIN` because flown
  separations came out tighter than planned; the escort has no such measured
  margin yet. Fly it in sim, compare `/tf` against the plan, and add one.
* **Mocap occlusion of the hat is unaddressed.** The stale-pose behaviour
  (hold at 0.4 s, land at 2.0 s) is implemented, but how often a head-worn
  rigid body actually drops out here is unknown — and it is the most likely
  thing to end a demo.
* **The radio budget is assumed, not measured.** Four drones at 20 Hz is
  80 packets/s of setpoints, comfortable against ~600–1200 packets/s shared
  across the link, but which dongle and firmware this rig has was never
  confirmed (no Crazyradio was plugged in on 2026-09-23).
* **Yaw is commanded as 0 for every drone.** Defenders do not turn to face
  the VIP or the threat. It would look better if they did; it is also more to
  go wrong, so it was left out.
