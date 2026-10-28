# Flight-test safety protocol

Scope: indoor, netted-mocap-volume flight testing of the person-follower,
on both the Crazyflie 2.1 Brushless prototyping platform and the ModalAI
Starling 2 Max, per the committed 15-week internship scope (8 Sep -
20 Dec 2026; see `config/px4_starling_indoor.params`). Outdoor flight
(`config/px4_starling_outdoor.params`) is explicitly out of scope and is
not covered by the procedures below beyond the notes in that file.

This project's operator is supervised by licensed pilots, so the
regulatory/permitting side is handled elsewhere. Everything in this
document exists to protect the aircraft and, above all, the person the
aircraft is following -- that job does not go away just because permits
are sorted.

---

## 1. Pre-flight checklist

Run through this checklist, in order, before every powered session (not
just every day -- every time the vehicle is about to be armed).

**Aircraft**
- [ ] Frame, arms, and motor mounts inspected for cracks or play (flex the
      frame gently by hand; no creaking or visible flex).
- [ ] All four propellers/props on correct rotation, tightened, undamaged
      (no nicks, delamination, or chips at the tips).
- [ ] Prop guards fitted and secure (see Section 3).
- [ ] Battery: voltage checked and within range (Section 8), securely
      strapped/seated, connector inspected for heat damage or looseness.
- [ ] All companion-computer/FMU cabling seated (camera ribbon cables,
      ToF, radios) -- a loose connector is a common cause of an in-flight
      failsafe that looks like a software bug.
- [ ] `voxl-inspect-services` (Starling) shows all core services running
      with sane CPU%, per `docs/BRINGUP.md` Stage 1.
- [ ] qVIO confirmed `OKAY` and held for 5+ seconds per
      `docs/BRINGUP.md` Stage 4 -- **do not arm on a fresh/unconfirmed VIO
      lock**.
- [ ] `voxl-tflite-server` delegate-bind confirmed per `docs/BRINGUP.md`
      Stage 6 if perception is in the loop for this session.
- [ ] Current `config/px4_starling_indoor.params` (or
      `config/crazyflie_optitrack.yaml` limits) loaded and verified against
      the vehicle's actual parameter values, not assumed from a previous
      session.

**Environment**
- [ ] Netting fully closed with no gaps a departing vehicle could exit
      through; net anchor points inspected.
- [ ] Mocap volume calibrated and rigid bodies (`cf_follow_drone` /
      `operator_helmet` for Crazyflie; Starling equivalent) visible and
      correctly labeled in Motive/tracking software -- confirm the drone's
      rigid body is not accidentally mislabeled as the operator's or
      vice versa.
- [ ] Software boundary (Section 4) loaded and matches the physical net
      clearance measured for this specific room setup.
- [ ] Area inside the net clear of people other than the operator and
      essential crew; onlookers outside the net line.
- [ ] Fire extinguisher (LiPo/electrical rated) and a metal or fireproof
      bin/bag accessible for battery incidents (Section 8).

**People**
- [ ] Safety pilot present, radio bound, and confirmed able to command a
      mode change / take control (Section 2).
- [ ] Operator (the person being followed) briefed on: what the vehicle
      will do, the minimum standoff distance (Section 5), the E-stop
      gesture and hardware button on the band, and what to do if the
      vehicle behaves unexpectedly (stop moving, do not run, let the
      safety pilot act).
- [ ] E-stop test completed for this session, per Section 6, before
      anyone stands inside the working volume with the vehicle armed.
- [ ] Roles for this session assigned and confirmed out loud (Section 2)
      -- do not assume yesterday's assignments carry over.

---

## 2. Roles

| Role | Responsibility | Non-negotiable rule |
|---|---|---|
| **Safety pilot** | Holds the RC transmitter for the entire powered session. Watches the vehicle, not a screen. Can command a mode change (to manual Position control) or disarm at any moment. | **Always has a live RC link with mode-override authority**, for every second the vehicle is powered, regardless of whether the vehicle is currently flying autonomously (Offboard) or under test software control. This is the backstop underneath every other layer in this document. |
| **Operator** | The person being followed. Wears the gesture band (and, on Crazyflie sessions, the OptiTrack helmet). Can trigger the ESTOP_BUTTON on the band at any time. | Stays outside the minimum standoff distance (Section 5) whenever able to do so, and never turns their back on / loses awareness of the vehicle's position while it is powered. |
| **Test conductor** | Runs the test script/sequence, operates the tablet app, calls out state transitions ("arming", "entering follow", "holding", "landing") so everyone in the room has a shared model of what the vehicle is doing. | Never the same person as the safety pilot -- these are separate attention budgets, and the safety pilot must be free to watch the aircraft, not a laptop. |
| **Log/data crew (optional)** | Runs recording, timestamps events for post-session review. | Does not have authority to command the vehicle; reports observations to the test conductor and safety pilot. |

The safety pilot's authority is absolute: any of the safety pilot, the
operator (via ESTOP_BUTTON), or the test conductor calling a stop ends the
test immediately, no discussion first.

---

## 3. Netting and prop guards

- **Netting**: the full flight volume must be enclosed by mocap-rated
  netting with no gap large enough for the airframe to pass through at any
  point along its perimeter, including where net panels meet at corners
  and doorways. Inspect anchor points and panel seams before every
  session, not just on initial rig-up -- netting sags and anchor points
  loosen with repeated use.
- **Prop guards**: fitted for every indoor session in this project's scope,
  no exceptions. They do not make a collision with a person safe, but they
  materially reduce laceration risk from a prop strike and reduce the
  chance of a prop shattering on frame/net contact and becoming a
  secondary projectile.
- Prop guards and netting are **independent, layered mitigations** for the
  same failure mode (an out-of-control or off-course vehicle) -- the
  software boundary and PX4/Crazyflie failsafes in Sections 4 and 7 are
  meant to keep the vehicle from ever reaching the net or a person in the
  first place; guards and netting are what happens when those layers are
  beaten. Do not treat either layer as making the other optional.

---

## 4. Software boundary inside the mocap volume

In addition to (not instead of) PX4's geofence (Section 7), a
software-enforced boundary runs in the ROS 2 supervisor for both
platforms, since PX4's geofence is **reactive** (see Section 7's notes) and
the stock Crazyflie firmware has no geofence of its own at all.

| Platform | Boundary | Capped velocity | Capped acceleration | Capped yaw rate |
|---|---|---|---|---|
| Crazyflie 2.1 Brushless (OptiTrack) | 8 x 8 m box, `x/y ∈ [-4, 4] m`, `z ∈ [0.2, 2.5] m` -- see `config/crazyflie_optitrack.yaml` | 1.0 m/s | 2.0 m/s² | 90 deg/s |
| Starling 2 Max (PX4 geofence, reactive) | 8 m radius cylinder, 3 m height -- see `config/px4_starling_indoor.params` | 1.0 m/s (`MPC_XY_VEL_MAX`) | governed by `MPC_POS_MODE`/`MPC_TILTMAX_AIR`, not a direct accel param | n/a at the PX4 layer; bounded by the follower's own command smoothing |

**MEASURE/CONFIRM for your room**: both boundaries above assume a specific
tracked-volume size and ceiling height. Before the first session in a new
room, measure the actual usable tracked radius/box and ceiling height, and
set both the software boundary and the PX4 geofence with margin inside the
smaller of (tracked volume, net clearance) -- not just inside whichever one
happens to be larger. The whole point of the margin is that the vehicle
will have already crossed the boundary line by the time any reactive action
fires (Section 7); the margin is what keeps that overshoot off the net.

The velocity/acceleration/turn-rate caps exist specifically so that a
**backup (safety) pilot can take over smoothly** -- a vehicle moving at
1 m/s with a 2 m/s² acceleration cap is something a human can safely grab
control of mid-motion; a vehicle at full aggressive tuning is not.

---

## 5. Minimum standoff distance from a person

**Default: 1.5 m minimum horizontal standoff**, matching
`follow_core.FollowParams.min_standoff_m` (the follower's own commanded
position is never allowed inside this radius) and mirrored into
`config/crazyflie_optitrack.yaml`'s `min_standoff_from_operator_m` so the
prototyping phase never asks the vehicle to fly closer to the operator than
the eventual deployment will.

This number is a **software governor limit on the commanded target
position**, not a guaranteed physical clearance -- it assumes the position
estimate (mocap, or UWB+vision on the Starling) is accurate and the vehicle
is tracking its commanded position within normal control error. Treat 1.5 m
as the floor to design the follower's behaviour around, not as a number
that alone makes closer approaches safe; the other layers in this document
(prop guards, safety pilot, E-stop) are what cover the case where tracking
or control error puts the real vehicle closer than the commanded 1.5 m
says it should be.

**Why speed is capped near a person** (`FollowParams.speed_cap_near_m_s =
0.6 m/s` at `min_standoff_m`, ramping up to `speed_cap_far_m_s = 3.0 m/s`
beyond `governor_far_m = 4.0 m`): kinetic energy scales with the square of
speed, and the distance available for a person to react and step back
shrinks as standoff shrinks. Capping speed specifically in the near band is
what keeps the worst-case "vehicle continues in a straight line for the
person's reaction time" outcome survivable, at the exact range where there
is the least room for that reaction to happen. This is a deliberate,
range-dependent trade: the follower is allowed to move quickly when it is
already far from the person, and is forced to slow down precisely as it
gets close, rather than flying at one constant speed regardless of range.

If a session ever needs a closer standoff than 1.5 m (e.g. testing tight
indoor following in a genuinely small room), that is a deliberate protocol
deviation requiring: prop guards confirmed fitted, speed cap reduced
further than the 0.6 m/s default for that session, and explicit sign-off
from the safety pilot before the session starts -- not a config value
changed quietly mid-session.

---

## 6. E-stop test procedure (before every session)

Run this with props off first, then once more with props on at idle/hover
in a clear area before any person stands inside the working volume. All
three E-stop paths below must be tested -- they are independent, and a
prior session passing does not mean today's hardware/firmware combination
still does.

1. **Hardware ESTOP_BUTTON on the band**: with the follower stack running
   and the vehicle in a FOLLOW-equivalent mode (or its ground-test
   analogue), press the band's hardware E-stop button. Confirm:
   - The command is observed by the supervisor as `CommandSource.
     ESTOP_BUTTON` (highest-precedence source in the arbiter).
   - The vehicle transitions to a stop/land state within the expected
     latency budget (target: under `FollowParams.estop_timeout_s`'s
     1.5 s window end-to-end, ideally well under -- this timeout is the
     supervisor's *staleness* bound, not the target response time; a
     genuinely fast E-stop should act in a small fraction of a second,
     not wait out most of that window).
2. **Gesture E-stop (the "chop" gesture, `kGestureStop`)**: perform the
   sharp downward chop gesture defined in the band firmware
   (`firmware/band/src/gestures.h`). Confirm the same stop behaviour as
   above, and separately confirm the gesture is **not** confused with
   normal follow-mode motion during ordinary walking/movement (a false
   E-stop trigger during normal use is itself a hazard to characterize,
   not just a nuisance).
3. **Safety-pilot RC override**: with the vehicle in autonomous
   (Offboard-equivalent) mode, have the safety pilot flip to manual
   Position-mode control on the transmitter. Confirm:
   - The vehicle responds to stick input within one control cycle -- no
     perceptible lag from the mode switch.
   - On the Starling, confirm this transition is the one that also
     re-arms Collision Prevention (Section 7), since CP is inert in
     Offboard mode and only becomes active once Position mode (with
     `MPC_POS_MODE` acceleration-based) is engaged.

**Acceptance**: all three paths must independently produce a safe stop
before the session proceeds to arming with a person in the volume. Any
path that fails, is sluggish, or is ambiguous must be fixed (or the session
must proceed without whichever capability failed, with the other two
covering for it and everyone briefed on the gap) before continuing.

---

## 7. Failsafe matrix

This matrix is the core of this document: every condition that can trigger
an automated safety response, what layer and parameter(s) govern it, and
the action actually configured for the **indoor, committed-scope profile**
(`config/px4_starling_indoor.params` / `config/crazyflie_optitrack.yaml`).

Two facts govern how to read this table:

- **PX4 executes the single MOST SEVERE action** when more than one
  failsafe condition is true at once. Severity order (least to most
  severe): `None < Warning < Hold < Return < Land < Disarm < Flight
  termination`. Where two rows below could both be true simultaneously,
  the more severe one is what actually happens.
- **PX4 Collision Prevention (`CP_*`) is only active in Position mode**,
  and only when `MPC_POS_MODE` is set to an acceleration-based mode. It is
  **NOT active in Offboard mode** -- the mode the autonomous follower flies
  in. Anywhere this table says "Offboard," read Collision Prevention as
  inert; the rows explicitly covering autonomous-flight obstacle
  proximity say what actually covers that gap instead.

| # | Condition | Layer | Governing parameter(s) (indoor value) | Configured action | Notes |
|---|---|---|---|---|---|
| 1 | RC transmitter signal lost | PX4 failsafe | `COM_RC_LOSS_T = 0.5 s`, `NAV_RCL_ACT = 3 (Land)` | **Land** in place | Land, not Return, so the vehicle doesn't fly laterally across the netted volume with no RC override available. |
| 2 | Offboard setpoint stream lost, RC still present | PX4 failsafe | `COM_OF_LOSS_T = 0.5 s`, `COM_OBL_RC_ACT = 0 (Position)` | Fall back to **Position mode** | This is the normal "companion computer glitch, pilot has RC" path. Falling into Position mode is also what re-arms Collision Prevention (row 5/6), since `MPC_POS_MODE` is already acceleration-based. |
| 3 | Offboard **and** RC lost simultaneously | PX4 failsafe arbitration | Rows 1 + 2's params together | **Land** wins | Per the severity rule above: Land (row 1) outranks the milder Position-mode fallback (row 2), so Land is what actually happens if both conditions are true at once. |
| 4 | Geofence cylinder (8 m horizontal / 3 m vertical) breached | PX4 geofence (**reactive**) | `GF_MAX_HOR_DIST = 8`, `GF_MAX_VER_DIST = 3`, `GF_ACTION = 2 (Hold)` | **Hold** in place | Reactive: the vehicle has already crossed the fence line before this fires. Hold (not Return) is used specifically because a strict indoor boundary should not have the vehicle then flying further, per PX4's own guidance for tight geofences. The 8 m/3 m sizing must include margin to the actual net -- see Section 4. |
| 5 | **Position mode only**: obstacle within `CP_DIST` and sensor data present | PX4 Collision Prevention | `CP_DIST = 1.0 m`, `MPC_POS_MODE = 4` (required for CP to be active at all) | Velocity toward the obstacle limited/stopped | Piloted-testing safety net. **Never active while the vehicle is following autonomously in Offboard mode** -- see row 7. |
| 6 | **Position mode only**: no range data in some direction | PX4 Collision Prevention | `CP_GO_NO_DATA = 0`; watchdog: 0.5 s no data blocks movement that way, 5 s forces Hold | Zero velocity into the blind direction; **Hold** after 5 s | Same Position-mode-only caveat as row 5. `CP_GO_NO_DATA = 0` is the conservative choice: "no data" defaults to "don't move that way," which matters with a person in the same small volume. |
| 7 | **Offboard mode (autonomous following)**: obstacle close to the vehicle | **Not Collision Prevention** -- CP is inert here | n/a at the PX4 layer | **No PX4-level velocity limiting.** Coverage comes from: (a) `voxl-mapper`'s ESDF-based path planning refusing to route through mapped obstacles (`config/voxl-mapper.conf`), (b) the software boundary (Section 4), (c) the safety pilot's standing ability to flip to Position mode at any instant (Section 6), which both stops autonomous control immediately and re-arms Collision Prevention. | **This is the single most important row in this table.** Anyone testing autonomous following must understand that the piloted-mode obstacle safety net (rows 5/6) simply does not apply while it's actually doing its job. |
| 8 | Target (operator) estimate lost | follow_core supervisor (software, not PX4) | `FollowParams.lost_timeout_s = 2.0 s` | Mode -> **Hold**, then Land if not reacquired | Independent of the PX4-layer failsafes; this is the autonomy stack noticing it has nothing to follow, not a vehicle-health condition. |
| 9 | Tablet/gesture command link stale | follow_core supervisor (software) | `FollowParams.command_timeout_s = 2.0 s` | Mode -> **Hold** | Same layer as row 8; protects against a dead tablet/band link leaving the vehicle executing a stale command indefinitely. |
| 10 | E-stop invoked (band hardware button or chop gesture) | follow_core supervisor, highest-precedence command source | `CommandSource.ESTOP_BUTTON`; staleness bound `FollowParams.estop_timeout_s = 1.5 s` | Immediate **Estop -> Land/Disarm** per supervisor state machine | See Section 6 for the required pre-session test of this path. The safety pilot's RC override (row "n/a", Section 2) remains available underneath this at all times regardless of supervisor state. |
| 11 | Software boundary breached (Section 4) | ROS 2 supervisor (software, not an autopilot geofence on Crazyflie) | `config/crazyflie_optitrack.yaml: safety.boundary`, or Starling's PX4 geofence (row 4) | **Hold/Land** commanded by supervisor | On Crazyflie there is no autopilot-level geofence at all -- this software check is the only boundary enforcement on that platform, making it a harder requirement to test (Section 6-style dry runs) than on the Starling, which has both this and row 4. |
| 12 | Operator standoff violated (Section 5) | follow_core governor (software) | `FollowParams.min_standoff_m = 1.5 m`, `speed_cap_near_m_s = 0.6 m/s` | Commanded target clamped to ≥ 1.5 m; speed capped in the near band | A commanded-position governor, not a hard physical guarantee -- see Section 5's caveat about tracking error. |
| 13 | Battery critical | PX4 battery failsafe | `COM_LOW_BAT_ACT` (**not part of this project's tuned parameter set** -- verify and set explicitly before flight) | Should be **Land** | Flagged as a gap: this project's verified parameter set (fact list this repo's configs are built from) does not include a battery-failsafe value. Confirm `COM_LOW_BAT_ACT` and the associated `BAT_*` capacity/voltage params are sanely set for the actual 2x 2S 18650 packs (Section 8) before the first flight of each session, rather than trusting factory defaults tuned for a different battery. |
| -- | Multiple failsafes true at once (general rule) | PX4 arbitration | n/a | **Most severe applicable action wins** | Restated from the top of this section because it is easy to reason about each row in isolation and forget that a real in-flight event is often two or more of these at once. |

---

## 8. Battery handling (2x 2S 18650 series packs)

This project's Starling 2 Max power system uses **two 2S 18650-cell packs
wired in series**, giving a combined 4S-equivalent pack (nominal 14.8 V,
2 x 7.4 V nominal per sub-pack). 18650 cylindrical cells behave differently
from pouch-cell LiPos in ways that matter for handling:

- **Lower continuous discharge rating (C-rate) than pouch LiPo** for a
  given capacity -- respect the pack manufacturer's continuous/burst
  discharge current rating; do not assume LiPo-style high C-rates apply.
- **Voltage sag under load is more pronounced** near end of charge -- a
  pack that reads acceptable at rest can sag further under peak draw
  (aggressive maneuvers, high thrust) than a comparable-capacity LiPo
  pouch pack would. Treat the pre-flight voltage check as a floor, not a
  guarantee of in-flight headroom.
- **Series-pairing matters**: because two separate 2S packs are wired in
  series to make the 4S string, an imbalance in capacity or internal
  resistance between the two packs causes the weaker pack to be
  over-discharged while the stronger pack still reads fine at the
  string level. **Pair packs by capacity and cycle count**, and check
  each 2S pack's own voltage individually (not just the combined
  string voltage) before and after every flight.
- **Charging**: balance-charge each 2S pack on its own balance leads
  (standard 2S balance charging), not as a single undifferentiated 4S
  charge, unless your charger and pack wiring are specifically set up
  for true 4S balance charging across both packs' cell taps. Charge in
  or on a fireproof LiPo bag/bin, attended, away from combustibles.
- **Storage voltage**: store packs not immediately flown at their
  storage voltage (per the cell manufacturer's spec, typically ~storage
  charge around 3.6-3.8 V/cell for common 18650 chemistries -- confirm
  against the actual cell datasheet you're using) rather than left at
  full charge for extended periods.
- **Pre-flight inspection**: check both packs individually for swelling,
  case damage, connector heat-discoloration, or a burnt smell before
  every session. Any pack showing swelling or damage is retired
  immediately, not "used for one more test."
- **Post-flight**: check both packs' individual voltages again
  immediately after landing. A pack that reads meaningfully lower than
  its partner after a flight is a sign of the imbalance problem above --
  pull that pair from service and re-check pairing before reuse.
- **Temperature**: 18650 cells lose usable capacity and discharge
  headroom in cold conditions more than pouch LiPo does at the same
  temperature. Indoor testing at room temperature is the expected
  condition for this project's committed scope; if a session is ever run
  in a cold space, budget for reduced flight time and re-check voltage
  sag under load before trusting the same flight-time numbers from a
  warm session.
- **Disposal**: fully discharge (to the storage/disposal voltage
  recommended by the cell manufacturer, not to 0 V) before disposing of
  any retired pack, and dispose of via a battery-recycling stream rated
  for lithium cells, never general waste.

---

## 9. Incident log template

Log every incident -- not just crashes. A failsafe firing unexpectedly, an
E-stop test that didn't behave as expected, a near-miss with the operator,
or a battery reading outside its expected range are all worth a log entry;
patterns across "minor" entries are often the first sign of a real problem.

```
INCIDENT LOG ENTRY
-------------------
Date / time:
Session / test ID:
Platform:                  [ ] Crazyflie 2.1 Brushless   [ ] Starling 2 Max
Location / room:

People present:
  Safety pilot:
  Operator:
  Test conductor:
  Others:

Flight mode at time of incident:
  [ ] Offboard (autonomous follow)   [ ] Position (manual)   [ ] Hold
  [ ] Land   [ ] Other: ________________

--- What happened ---
Describe the sequence of events in plain language, in order. Include what
the vehicle was doing, what the operator was doing, and what (if
anything) the safety pilot / supervisor / autopilot did in response.


--- Which failsafe(s) fired, if any ---
Reference the row number(s) from Section 7's matrix, or note "none fired"
if an unsafe condition occurred WITHOUT any configured failsafe engaging
(this is its own important category -- it means a gap exists in Section 7
that needs a new row, not just a retuned parameter).
  Row(s):
  Configured action taken:
  Did it match the expected action from the matrix?  [ ] Yes  [ ] No

--- Distances / speeds involved ---
Closest approach to operator (m), if relevant:
Vehicle speed at time of incident (m/s), if known:
Standoff / boundary parameters in effect at the time (attach config used):

--- Damage / injury ---
Vehicle damage:
Prop/frame/net damage:
Any injury (however minor), to whom:

--- Immediate action taken ---


--- Root cause (if known yet) ---


--- Follow-up actions before next flight ---
[ ] Config change (specify file/param):
[ ] Hardware repair/replacement:
[ ] Procedure change (specify section of this document):
[ ] No action -- explain why:

--- Sign-off ---
Logged by:
Reviewed by (safety pilot):
Date closed:
```

Keep completed logs together (chronologically) so patterns across sessions
are easy to spot -- e.g. three "row 7 gap" near-misses in different
sessions is a signal to change the matrix or the mitigation, not three
unrelated one-off events.
