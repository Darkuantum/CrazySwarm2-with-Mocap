# Escort demo and rig: technical briefing (prep for the 2026-10-13 meeting)

Everything below comes from the code as of `8d8b28b`, not from the prose docs. Where the
docs, runbooks or code comments disagree with the code, see **§9 Corrections**. Read that
section before you quote any number.

Numbers marked *[measured today]* came from running `plan_escort` on 2026-10-12.

---

## 0. The 30-second version

Three Crazyflies hold a ring of radius 1.0 m around a VIP (in the live demo, a hand-flown
DJI tracked by OptiTrack). A fourth Crazyflie attacks. The defenders never trade places.
Instead the whole ring **rotates** (one angle, rate-limited) so the defender already nearest
the threat bearing ends up between the threat and the VIP. The other two close in beside it
to form a "wall". Every setpoint goes through a per-drone guard (slew limit, then separation
projections, then geofence) before it is sent. Everything runs on the laptop: mocap comes in
at 50 Hz, and `cmdFullState` setpoints go out at 20 Hz per drone over one Crazyradio. The
whole control law is pure Python with no ROS (`escort.py`). The flight script and the offline
verifier (`plan_escort`) run the *same objects*.

---

## 1. System architecture

```
OptiTrack/Motive ──NatNet multicast 50 Hz──► motion_capture_tracking ──/poses (NamedPoseArray, BEST_EFFORT)
                                                     │
          escort_show.py (rclpy, laptop) ◄───────────┘   reads /poses only, never /cfX/pose
              │  EscortController.step()  @ 20 Hz
              ▼
          crazyflie_server (C++) ──Crazyradio, radio://0/80/2M──► 4 Crazyflie 2.1
                                    (one dongle; mocap pose also broadcast to every drone)
```

* **Onboard:** each Crazyflie runs its own EKF, fed with the mocap position as an external
  position measurement (`extPosStdDev = 1e-3`, so mocap is effectively trusted), plus the
  stock PID position/attitude controller. We command **position + velocity**, not attitude.
* **Two command paths.** Takeoff, the gather and the landing use the **high-level commander**
  (`takeoff`, `goTo`, `land`: onboard-planned smooth polynomials). The escort itself uses
  **low-level streaming** (`cmdFullState(pos, vel, acc=0, yaw=0, omega=0)`). After streaming
  you must call `notifySetpointsStop()` before any high-level command, or the stale stream
  fights the landing.
* **Why `cmdFullState` and not `cmdPosition`:** the simulator (`crazyflie_sil`) only
  implements full-state. The velocity field carries the guard's own velocity as a feedforward
  term.
* **Yaw is always 0.** The defenders do not turn to face anything.
* **Why the VIP pose comes from `/poses` and never from `/cfX/pose`:** `/cfX/pose` is the
  onboard estimate, seeded from the yaml, so it agrees with the plan even when the drone is
  in the wrong place.

---

## 2. The escort control law (`escort.py`, `EscortController.step`)

### 2.1 Roles: who defends and who attacks (`pick_roles`)
Sort the enabled drones by plan-view distance from the **VIP mark**. The 3 nearest defend and
the farthest attacks. Override with `-p defenders:=cf1,cf2,cf3 -p adversary_drone:=cf5`.

The **VIP mark** is fixed config, not inferred from the drones:
`room_center + vip_offset = (-0.213, -0.072) + (-0.017, +0.600) = (-0.230, +0.528)` in the
post-2026-10-08 world frame. The offset points toward the operator and audience, so the attacker
comes in across the far half of the room.

### 2.2 Slot assignment and initial phase (`assign`)
* Compute each defender's bearing from the VIP. **Slot order = the counter-clockwise order of
  those bearings** (`argsort`). This is fixed once and never changes, so drones cannot swap
  places mid-air. That matters because the rig has no onboard collision avoidance (the
  2026-08-04 collision was crossing paths).
* Initial ring phase ψ₀ = **circular mean** of `(bearing of the drone in slot k − 2πk/3)`.
  This is the phase that minimises total angular travel to the slots.
* Slot *k* sits at angle `ψ + 2πk/3`, at radius R = 1.0 m from the VIP.

### 2.3 Gather (high-level `goTo`), verified twice
* All four drones fly straight-line `goTo`s onto their slots / the attacker's first point
  **at the same time**, with duration `max(3 s, longest_leg / 0.4 m/s)`.
* Before that move, `safety.transition_min_sep` computes the **exact closed-form minimum
  pairwise distance** over the simultaneous straight-line move. For each pair, minimise
  `|d0 + s·(d1−d0)|` over s ∈ [0,1]. It must be ≥ `PLAN_SEPARATION` = 0.80 + 0.10 = **0.90 m**.
* This is checked **before arming** from the yaml marks, and **again after takeoff** from the
  live poses. A refusal after takeoff lands the drones.
* `adversary_start_problem` refuses if the attacker's mark is within `R + min_adv_sep = 1.80 m`
  of the VIP, because its first leg out would cross the ring.
* After the gather, `resync()` re-seeds each guard from the live pose. Otherwise the first
  streamed setpoint would slew back from the *pre-gather* position (bug found in sim on
  2026-09-23).

### 2.4 Each 20 Hz step, in order

1. **VIP velocity estimate:** a finite difference passed through a first-order low-pass with
   a 4 Hz corner (`α = 1 − e^(−2π·4·dt)` ≈ 0.72 at dt = 0.05 s).
2. **Threat latch, with hysteresis.** Engage when attacker–VIP distance ≤ **2.10 m**. Release
   when it is > **2.22 m**. These were raised from 1.88/2.00 m on 2026-10-07 after the attacker
   got through on a live run. The fix was to engage *earlier* (a time budget), not to change
   the geometry.
3. **Lead selection (who blocks).** While engaged, pick the slot whose *resting* angle is
   nearest the threat bearing φ = atan2(p_adv − p_vip). This is re-evaluated every step, and
   the lead switches only if another slot is better by more than **18°** (`lead_swap_hyst`).
   Because the nearest slot always leads, the ring never has to rotate more than half the
   slot spacing, i.e. **≤ 60°**.
4. **Phase target:** `ψ_target = φ − 2π·lead/3`, which puts the lead slot exactly on the
   threat bearing. When not engaged, **hold the current ψ** (`hold_phase_on_clear`). Winding
   back to a rest angle made the ring oscillate. A bearing-rate lead term exists
   (`threat_lead_s`) but is set to **0**: measured, it made facing worse (8° median error
   with it off, 23–24° with it on).
5. **Rate-limited rotation (`PhaseTracker`):** ψ moves toward its target by at most
   `rate·dt` along the shortest arc. The rate is **0.30 rad/s at rest** and **0.55 rad/s while
   engaged**. Worst-case time to bring a slot onto the threat bearing: 60° / 0.55 ≈ **1.9 s**.
6. **Wall closing.** A scalar `close` ramps 0 → 1 over **4 s** while engaged and back down
   when clear. The two wing slots blend along the *shortest arc* from their ±120° rest posts
   to **±58°** either side of the lead (`slot_offsets`). Shortest arc matters: interpolating
   the raw angle values once sent a wing through the lead's post, and separation dropped to
   0.26 m.
   * Lead-to-wing chord when shut: `2R·sin(29°)` = **0.97 m**. It must be ≥ 0.90 m, which
     sets the floor for this angle (53.5° at R = 1.0).
   * Ring chord at rest: `2R·sin(60°)` = **1.73 m**. Three defenders do not seal the circle.
     The ring rotates to meet the threat instead.
7. **Slot targets:** `p_vip + R·[cos θ_k, sin θ_k]`, at the ring altitude (§2.6), plus
   `v_vip / rate_hz`. That last term is a one-tick (50 ms) position lead, about 1 cm at
   0.2 m/s. It is small, so do not call it a velocity feedforward in the meeting. The
   feedforward that does matter is the guard velocity sent in `cmdFullState`.
8. **Live-pose sanity check:** if a defender's mocap pose is more than R (1.0 m) from its own
   commanded setpoint, treat the pose as bad (stale, flipped rigid body, fallback source) and
   use the setpoint instead. These live poses only feed the defender–defender clamp.
9. **`SetpointGuard`, per drone (the safety core):**
   1. **Slew:** limit the change in velocity to `a_max·dt` (1.0 m/s²), then limit speed to
      `v_max` (0.6 m/s). `sp = sp_prev + v·dt`.
   2. **Separations, applied AFTER the slew:** 3 iterations of radial *projections* out of
      discs, in the order attacker → other defenders → **VIP last**. Each projection is "if
      closer than d_min, move radially out to d_min", with a 1 mm deadband. Iterating makes
      the projections converge. Finishing on the VIP means the person/DJI separation is never
      traded away.
      Floors: defender–defender 0.80 m, defender–VIP **1.00 m** (= R, so a slot sits *on*
      the boundary), defender–attacker **0.70 m** (`defender_yield_floor`, see below).
   3. **Room:** clamp the horizontal radius to **1.90 m** about the arena centre, and the
      altitude to [0.30, 1.85] m.
   4. Recompute `v` from the setpoint actually committed, so the next acceleration limit is
      honest.

   *Why the order matters:* if you clamp first and slew second, the slew drags the drone back
   inside the floor on the next step. The follow-drone prototype shipped that bug. With a
   single pass and no iteration, the attacker projection once shoved a defender to 1.27 m
   from the VIP against a 1.50 m floor.

**Who yields.** `defenders_yield = False`. The defenders give way to the attacker only below
0.70 m. The *attacker* owns the 0.80 m floor through its own hard projection (§3.2). Before
that change (2026-10-06), the defenders yielded at 0.80 m, the ring could never be a real
obstacle, and an attacker driving straight in reached 0.34 m from the VIP.

### 2.5 The speed budget (the central trade-off)
A defender has `v_max` = 0.6 m/s in total. Rotating the ring costs `R·ω` of that, and
following the VIP costs the VIP's speed.
* At rest: `0.6 − 1.0·0.30` = **0.30 m/s** left over for VIP motion (`max_vip_speed`).
* **While engaged: `0.6 − 1.0·0.55` = 0.05 m/s.** That is why *any* VIP motion during an
  encounter compresses the margins (§6). The design intent is that the VIP hovers during the
  attack.

### 2.6 Altitude
* Floor VIP (a point or a person): the ring sits at a fixed 1.20 m.
* Airborne VIP (`vip_mode:=mocap`, the DJI): ring altitude = VIP z **+ 0.30 m**, clamped to
  [0.30, 1.85]. The defenders ride *above* the DJI. Below it, the DJI's downwash would hit
  them. The Crazyflie-on-Crazyflie downwash threshold in the literature is dz/l > 19, about
  0.62 m. Level flight was flown on 10-06 and rejected: coplanar with a hand-flown DJI, a
  lateral slip becomes a collision.
* While the DJI takes off and lands (paced + mocap only): the ring is pinned at **1.70 m**
  (`dji_clear_height`) and does not follow the VIP. The DJI goes up last and comes down
  first.

---

## 3. The adversary

### 3.1 Scripted (`AdversaryScript`)
Three legs, each `(duration, absolute bearing, radius from the VIP)`, all at bearing
**236.59°**: 6 s to 2.30 m (take station), 8 s to **1.80 m** (the probe = R + min_adv_sep,
so it never commands closer than one separation outside the ring), then 6 s back to 2.30 m.
`check()` proves every leg stays inside the arena. The chain that has to hold is
`probe 1.80 < alert 2.10 < release 2.22 < stand-off 2.30`. In paced mode each leg is anchored
to where the VIP was when the leg started, so a moving DJI genuinely evades it instead of
towing it along.

The bearing is frame-dependent. It was rotated by +91.59° after the 2026-10-08 Motive
recalibration. Left unrotated, the attacker's station landed 2.70 m out, outside the arena.
`vip_offset`, the script bearing and `ReactiveAdversary.bearing` are three separate copies of
the same direction.

### 3.2 Reactive (`ReactiveAdversary`): saturated potential field + gap seeking + give-up timer
This is a stateful virtual point that the attacker drone is commanded to follow. At each step:

* **Aim (gap seeking, `seek_gaps=True`):** take the defenders' bearings about the VIP, list
  the gaps, and discard gaps a defender is within 42° of. Choose the gap that maximises
  `width + 0.9·|angle from my current bearing|`, so it prefers far gaps and flanks. Then
  **commit** to that gap until a defender covers it, which stops it dithering 20 times a
  second.
* **Velocity:**
  `v = 1.0·min(1, d_goal/0.6)·û_goal`, where the goal is the aimed bearing at the shell
  radius, and the attraction tapers linearly inside 0.6 m,
  `+ Σ_defenders 3.5·(1 − d/1.70)·û_away` (linear repulsion, range 1.70 m),
  `+ 2.0·(1 − margin/0.40)·(−r̂)` within 0.40 m of the arena wall.
  Then saturate to **0.80 m/s** (but see §9, item 2).
* **Hard projections after integrating**, the same pattern as the guard: 3 iterations of
  "push out to 0.80 m from each defender", finishing with "push out to the **0.80 m shell
  around the VIP**", then the 1.90 m arena clamp. Why hard projections: with force-only
  keep-out, the three defenders' pushes summed *toward the centre* inside the ring and drove
  the attacker onto the VIP (closest approach 0.01–0.06 m over 18 gain combinations).
* **Give-up:** track the closest-ever distance to the VIP, which *relaxes outward* at
  0.01 m/s. If the attacker fails to beat it by more than 0.08 m/s of closing speed for
  **12 s**, it stands down and returns to station. The relaxation means repeated pressing
  counts as trying. Without it, it gave up mid-attack. Measured: it presses for about 31 s
  before standing down.
* **Phases:** station → attack → stand down. Paced mode: advanced by the operator. Unpaced:
  every 40/3 s.
* **Angular race argument** (`check()`): pushed out to 1.80 m, the attacker's angular rate is
  `v/1.80`. The ring's is min(0.55, v_max/R) = 0.55 rad/s. Break-even is 0.55·1.80 =
  **0.99 m/s**. Caveat: this assumes the defenders hold it at 1.80 m. Its hard floor is
  0.80 m, where it could out-turn the ring.

### 3.3 Other modes
`external` tracks a rigid body (a human-flown attacker) and commands nothing. `manual` steers
a keyboard target (`escort_teleop`). `none` runs no attacker.

---

## 4. Runtime safety layers, in the order they act

| Layer | Rule | Where |
|---|---|---|
| Config proof | `check_config`: ring ≥ VIP floor, slot chord ≥ 0.80, wall chord ≥ 0.90, alert > R+0.8, release > alert, ring fits the arena, `v_max > R·phase_rate` | before arming |
| Gather proof | closed-form min-separation ≥ 0.90 m, from the yaml marks AND the live poses | before arming, after takeoff |
| Mocap + placement | every flying drone on `/poses`, within 0.25 m of its yaml mark | before arming |
| Supervisor | `/cfX/status`: refuse if LOCKED (only a power cycle clears it, which an e-stop leaves behind), not armable, tumbled, or < 3.7 V | before arming |
| **Per-drone containment** (`contain_check`), on the ACTUAL pose | no pose for 0.3 s → hold; 1.0 s → land all; actual radius > 1.90+0.25 = **2.15 m** → land (tracking was lost at 2.24 m); > 0.60 m from its own command for 0.8 s → land. A drone never seen on `/poses` → land (fixed 10-09) | every step, including inside operator gates |
| VIP staleness | 0.4 s → defenders hold; 2.0 s → land | every step |
| VIP geofence | VIP > 0.90 m from the arena centre (= 1.90 − R) → ring **parks** at the last good point and holds its *current* altitude, LEDs cyan. It resumes only when the VIP is ≤ 0.65 m from the centre (0.25 m hysteresis) **and** inside the parked ring | every step |
| Setpoint guard | §2.4 step 9 | every setpoint |
| Abort | SIGINT/SIGTERM taken back from rclpy (`take_signals`), raising `ShowAborted` (a BaseException, so `except Exception` cannot swallow it), then `notifySetpointsStop`, then broadcast land over 4 s, then disarm. A second signal kills the process | any time |
| E-stop | `/all/emergency` broadcast. Separate callback group, so it works even when per-drone services are wedged | console / preflight GUI `e` |

Why containment exists: on 2026-10-07 the attacker flew away. The VIP had a watchdog and the
Crazyflies did not (`self_stale_s` was declared but never read). Clamping the *setpoint* proves
nothing about the aircraft, so containment checks the actual pose.

---

## 5. Operator pacing and the live sequence (`paced:=true`, `vip_mode:=mocap`)

Stdin is read a **line** at a time, with a non-blocking `select()`, so it works in a terminal
and in the console's stdin box. While a gate waits, `_hold()` **keeps the control loop
running**: the ring holds, the guards and watchdogs keep running, and the attacker keeps being
fed. A sleeping gate once froze pose timestamps, so the operator's reaction time was read as
an 8 s mocap loss and the show landed. Keystrokes queued before a gate opens are drained, so
a stale Enter cannot skip the "DJI is down" confirmation.

1. ARM gate → arm (the `armed` flag is set *before* the arm loop) → take off to 0.6 m →
   gather.
2. The ring climbs to 1.70 m; whole fleet **amber** ("your move"). The pilot takes off the DJI
   and settles it. Enter.
3. The ring drops to DJI z + 0.30 and tracks it. Enter starts leg 1 (attacker **amber**,
   taking station).
4. Enter starts leg 2: attacker **red**, attacking. Blocker **white**, wings **violet**, ring
   at rest **deep blue**. "Leg complete" is announced automatically (reactive: when it stands
   down).
5. Enter starts leg 3: attacker **magenta**, withdrawing.
6. The ring climbs to 1.70 m, all amber, and the pilot lands the DJI. Enter, then
   `notifySetpointsStop`, land, disarm, and print the geofence table (show time + wall clock,
   for syncing with video).

LED cues are sent only on transitions, after the motion commands, can never raise, and are off
in sim. A `setParam` every step would be radio traffic the link cannot spare.

---

## 6. Verification: `plan_escort`

* A closed-loop simulation of the **same** `EscortController` at 20 Hz. Each drone is modelled
  as a **first-order lag** (τ = 0.25 s) toward its setpoint. That is a stand-in for the real
  tracking dynamics, not a measured value.
* It runs a static VIP case and a walking VIP case (the VIP bounces off the keep-in boundary).
  It measures min pair, VIP and attacker separation, peak speed, radius, altitude,
  **formation error while settled** (≤ 0.5 m), median facing error, and how many steps a
  separation clamp fired.
* `--sweep` finds the fastest walk that passes. `--marks` prints where to stand the drones
  (the ring phase with the most wall clearance) and checks the yaml.

*[measured today]* with the defaults (reactive attacker):

| case | def–def | def–VIP | def–attacker | peak v | engaged | verdict |
|---|---|---|---|---|---|---|
| static VIP | 0.96 | 1.00 | 1.04 | 0.55 | 38 % | **pass** (but 7 separation-clamp steps) |
| walk 0.05 m/s | 0.80 | 0.99 | **0.70** | 0.59 | 66 % | fail: def–att |
| walk 0.20 m/s | 0.81 | 0.96 | 0.67 | 0.68 | 70 % | fail |
| walk 0.30 m/s | 0.81 | 0.93 | 0.69 | 0.67 | 78 % | fail: VIP, att, speed |

The scripted attacker passes up to **0.15 m/s** and fails at 0.20 m/s (0.72 m def–att). The
static facing error is a median of 0°, and the wall closes to 100%.

What it does **not** prove: real controller lag (no hardware tracking margin has been
measured for the escort), real pose noise and occlusion (the sim publishes no `/poses`), or
radio behaviour.

---

## 7. What flew, and what broke (sources: commits and code comments; there is no flight-log dataset)

| date | event |
|---|---|
| 09-23/24 | Built (`a8434de`). Sim runs. Found and fixed the gather/resync bug and the "arena squeeze" (a centred VIP starts the demo already engaged). Gather verification and geometric role picking added (`c881d6f`): the old name-order roles had gather legs closing to **0.00 m**. |
| 10-01 | Arena **measured**: walked survey (8067 samples) plus flown spiral/climb sweeps. Tracking lost at 2.24 m, clean to 2.00 m, ceiling 1.95 m at full radius. Ring shrunk 1.5 → 1.2 → 1.0 m to fit. |
| 10-06 | First hardware flights. Reactive attacker and "defenders don't yield" (`dd8e6b7`), stage LEDs. Blocker cue moved from amber (unreadable against red) to white. Defenders raised to +0.30 m above the DJI. Fixed: gate drained queued keys; the gate pump fed the attacker too (it had **dropped out of the sky** when starved during the DJI-down gate). |
| 10-07 | Hardware with the DJI as VIP. **Attacker fly-away** → per-drone containment (`bc24892`). Attacker punched through → engage earlier, 2.10/2.22 m (`be27585`). Geofence: own colour (cyan), keeps current altitude (it had **dropped the ring to DJI level**), wall-clock timestamps. |
| 10-08 | Motive recalibrated: the world frame rotated +91.59°. Fitted on cf1/cf3/cf5 (residual ≤ 16 mm). Re-expressed the arena centre and rotated the three bearings. **Demo shown to guests, several runs, the last one clean.** |
| 10-09 | Arena moved into `arena.yaml` (`measured: false` refuses to fly). Untracked drone → land (C1). Abort no longer swallowable. |

---

## 8. Supporting work on the rig (beyond the escort)

* **Self-contained fork** of AI-DA-STC/CrazySwarm2-with-Mocap: vendored source, distro-aware
  setup (Humble/Jazzy), 73 commits ahead of upstream (`9ed4bae`), and about 14.5k lines added
  under `src/`.
* **Mocap:** vendored `motion_capture_tracking`. The apt 1.0.9 build hard-codes a foreign IP,
  and its NatNet 4.1/4.2 model-definition parser segfaulted (patched). Auto-discovery of the
  Motive host via a NatNet ping on UDP 1510. Diagnosed UDP 1511 `SO_REUSEPORT` starvation.
* **Telemetry stall, root-caused in the firmware:** in `radiolink.c`, a drone that *receives*
  nothing for 1 s concludes it is disconnected, and `log.c` then runs
  `logReset(); crtpReset()`. It deletes its own log blocks silently and keeps acking, so every
  host counter looks healthy. Localised by adding packet tracing. **Fix: a server telemetry
  watchdog** (tier 1 stop/start, tier 2 destroy + `logReset` + rebuild, all waits bounded,
  only when mocap shows the drone on the floor). Verified on hardware: recovery in about 1 s.
  *Still open:* what causes the ≥ 1 s receive gap in the first place (suspect: the
  broadcast-priority skip in `CrazyradioThread.cpp`).
* **Upload wedge fix:** a trajectory upload during radio saturation could block the server
  forever. Memory-write waits are now bounded and replies are matched by identity (memory id +
  address) rather than shape. A failed upload marks the id bad and `start_trajectory` refuses
  it, because a silent 24-byte hole would fly a hybrid polynomial.
* **Other shows:** carousel (flown on hardware) and constellation (shape changes on a 120 BPM
  grid), each with an offline planner that refuses unsafe plans. Shared `abort.py` and
  `preflight.py`.
* **Mission console:** a browser GUI that drives the `ros2` CLI, so every button shows its exact
  command. One-click e-stop with a confirmed answer. Per-demo mission windows defined by YAML.
* Forward-looking (untracked): `docs/research/2026-10-11-cbf-commander-feasibility.md`, on
  putting the CFs under the AirStack CBF commander.

---

## 9. Corrections: where the docs were wrong

**Status 2026-10-12:** items 1–6 are now corrected in ESCORT.md, the three
runbooks, CLAUDE.md, CONTRIBUTING-UPSTREAM.md and the code comments (comment
and docstring changes only, checked by an AST comparison). `adversary:=reactive`,
which is what flew on 10-08, is now the default in the console mission window too.
Item 7 is documented in a code comment. Item 8 (the bug) is deliberately left
open, and item 9 still stands. The list below records what was wrong.

1. **Walking-VIP speed.** ESCORT.md, the runcard and the Q&A say "`--sweep` clears 0.20 m/s".
   That figure predates the reactive attacker. **Today the sweep returns 0.00 m/s** with the
   default reactive attacker (it already fails at 0.05 m/s, def–attacker 0.70 m), and
   **0.15 m/s** with the scripted one. The reactive failure is largely *by construction*: the
   defenders only yield at 0.70 m, while the verdict wants ≥ 0.75 m. So the honest statement
   is: "static VIP verified; any VIP motion during an encounter is not cleared".
2. **The attacker is not faster than the defenders on hardware.** The reactive point
   integrates at 0.80 m/s, but `escort_show` passes its setpoint through a `SetpointGuard` built
   with the *same* `cfg` (`escort_show.py:827`), so the commanded attacker is capped at
   **0.60 m/s** and 1.0 m/s². The narrative and Q&A ("it is faster, still cannot get through")
   do not describe what flew. `plan_escort` feeds the attacker unguarded, so it simulates a
   0.8 m/s attacker that never flies. Also, the attacker's own 0.80 m separation projection is
   computed on the *virtual* point. The guarded setpoint lags behind it.
3. **Slot assignment is *not* "bottleneck distance"** (Q&A, Tier 2). That is the
   constellation show. The escort uses bearing order plus a circular-mean phase (§2.2).
4. **Defender–attacker floor is 0.70 m in practice, not 0.80** (the Q&A numbers card says
   0.8). The attacker owns 0.80, and the defenders give way only at 0.70.
5. **Defaults disagree.** `escort_show` defaults to `adversary=reactive`. The console's mission
   window, the runcard and the script's own docstring say `scripted`. Defender and attacker
   defaults are chosen *by distance from the VIP mark*, not "first 3 / 4th enabled" as the
   docstring and runcard said. **10-08 flew the reactive attacker**, which is now the
   default everywhere.
6. **Stale code comments:** containment trips at **2.15 m** (comment says 2.25).
   `max_vip_speed` docstring says 0.15 (it is 0.30). `aim()` docstring quotes 0.45 m/s / 0.30
   rad/s. `seek_gaps` comment says "OFF by default" (it is True). `LED_BLOCKING` comment says
   cyan (it is white). `shell()` docstring says R + 0.8 (it returns 0.8). `attack_reach = 0`, so
   the soft VIP keep-out force never fires; only the hard projection bounds it. The `__init__`
   comment says the lead is "held until release" (it is re-selected every step with 18°
   hysteresis). The `alert_radius` comment says "engaged 58%" (38% today).
7. **Lead switches mid-wall make slot targets jump.** When the lead changes while `close > 0`,
   the wing posts are recomputed relative to the new lead instantly. ψ is rate-limited but the
   offsets are not. Only the guard's slew limit smooths it. Probably benign, but it is a
   discontinuity.
8. **Possible bug, VIP dropout starves the attacker.** In the VIP-stale hold branch
   (`escort_show.py:1067-1078`) only the defenders are streamed. The attacker gets no
   setpoints from VIP age 0.4 s until the 2.0 s land, which is up to about 1.6 s. Firmware
   2025.02 `supervisor.c`: setpoint age > 0.5 s puts it in *WarningLevelOut* (attitude levelled,
   uncommanded drift), and > 2.0 s means *free-fall* (motors off). It lands just before the
   2.0 s limit, so expect a brief drift, not a fall. Note that this is the same mechanism as the
   10-06 attacker drop, in a different branch. Not verified on hardware.
9. **Never measured on hardware:** an escort tracking/separation margin, the occlusion rate of
   a head- or DJI-mounted rigid body, and how much the 20 Hz × 4 setpoint stream contributes to
   the radio budget. "Radio RTT 28–87 ms" in the Q&A comes from one morning and is not
   reproducible from the repo.

---

## 10. Likely questions → precise answers

* **"Why rotate instead of reassigning?"** Reassignment means two drones crossing through a
  ring that contains a person, with no onboard collision avoidance. Rotation keeps the order
  fixed, keeps the spacing at 2π/3 by construction, and reduces the decision to one
  rate-limited angle.
* **"Is it a formation control law from the literature?"** The ring geometry follows
  circular-formation-around-a-leader work (arXiv 2212.12554, Olfati-Saber lineage), but with
  the potential fields replaced by slot targets plus a saturated, slew-limited setpoint, since
  mocap gives exact positions. The 3-defender rotation is ours. Every defender paper in the
  survey was 1-vs-1. The orbit law of arXiv 2103.11574 was rejected because it needs
  defenders much faster than the target.
* **"Is there any guarantee?"** Separations are enforced per setpoint by projection, but there
  is no formal invariance proof (no CBF). Defence success is empirical (`plan_escort`
  plus flights). The CBF route is the 10-11 research note.
* **"What's the hardest constraint?"** Room size. A 1.90 m planning radius forces R = 1.0,
  which forces the 0.30 m/s (rest) / 0.05 m/s (engaged) speed budget and the 0.90 m DJI box.
* **"What breaks first?"** The VIP leaving its box (geofence), or VIP motion during an
  encounter (margins compress). On hardware so far: the attacker fly-away (fixed with
  containment) and the attacker getting through (fixed with earlier engagement).

To print any live number: `ros2 run crazyflie_shows plan_escort [--marks|--sweep]`, or the
banner `escort_show` prints before it arms.
