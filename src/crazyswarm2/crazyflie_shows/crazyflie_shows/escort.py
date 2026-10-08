#!/usr/bin/env python3
"""Three defenders hold a ring around a VIP and turn it to face an adversary.

Pure geometry and control law -- no ROS, no radio, no mocap. ``plan_escort``
verifies *this* module offline and ``escort_show`` flies it, so what is proven
on a laptop is what is commanded to the drones.

The demo
--------
Three drones hold evenly spaced slots on a circle centred on the VIP. The
circle translates with the VIP (a fixed point first, then a person wearing a
mocap hat). A fourth drone in a different colour plays the adversary; when it
comes inside ``alert_radius`` the whole ring *rotates* so slot 0 sits on the
VIP-adversary bearing, putting a defender between the two.

Why rotate the ring instead of reassigning drones to slots
----------------------------------------------------------
Reassignment is the obvious move and it is the wrong one here: two drones
swapping slots fly through each other, and this rig has no onboard collision
avoidance (that is the 2026-08-04 collision, HANDOVER.md section 6). Rotating
the ring keeps the slot *order* fixed forever -- the drones never exchange
places, the spacing stays 2*pi/n by construction, and the only thing that
changes is one angle.

Where the parts come from
-------------------------
* The escort ring is the circular-formation-around-a-leader result
  (arXiv 2212.12554, built on Olfati-Saber): a circle that holds whether the
  leader is stationary or moving. We keep its geometry and drop its potential
  fields -- with mocap we know every position exactly, so a slot target plus
  a saturated P law does the same job with nothing to tune.
* The equal-spacing *orbit* law flown on Crazyflies under Vicon
  (arXiv 2103.11574) is deliberately NOT used: it assumes the agents are much
  faster than the target (V_Tmax << V_Amin), which for a walking person means
  defender speeds above ~3 m/s. That is not a speed to fly next to a human.
  Holding slots needs only the VIP's own speed plus a margin.
* The blocking layer's fallback is the pure-pursuit defender law
  u_d = (y - x_d)/||y - x_d|| (arXiv 2203.15872). Note that paper -- and every
  other defender paper found in the 2026-09-22 survey -- is ONE defender
  against one attacker. Spreading the job over three defenders is ours, so the
  ring is the thing that gets verified, not a cited guarantee.

The blocker stays ON the ring. It does not charge the adversary: an intercept
law next to a person buys nothing for a demo and spends the whole separation
budget.

Numbers that are still open
---------------------------
See ESCORT.md "Decisions still open". In short: ``ring_radius``, ``height``
and ``v_max`` are defaults inherited from the follow-drone prototype
(min 1.5 m standoff, 0.6 m/s cap), not measured on this rig, and
``v_max = 0.6`` bounds how fast the VIP may walk -- see
:func:`max_vip_speed`, which is the number to quote when someone asks how
fast the person may move.
"""

from dataclasses import dataclass

import numpy as np

from crazyflie_shows import safety

TWO_PI = 2.0 * np.pi


@dataclass
class EscortConfig:
    """Every number the escort needs. Checked by :func:`check_config`."""

    # -- the ring ---------------------------------------------------------
    n_defenders: int = 3
    #: 1.00 m, down from 1.50 m (inherited from the follow-drone prototype and
    #: flagged in ESCORT.md as "not measured on this rig") via 1.20 m. Every
    #: step was paid for by the measured volume: flying cf1 a slow spiral on
    #: 2026-10-01 lost mocap at 2.24 m radius, and the arena a ring needs for a
    #: working approach/retreat cycle is ring + min_adv_sep + 0.3 m of run-up
    #: minus the VIP offset -- 2.14 m at 1.50, 1.84 m at 1.20, 1.50 m at 1.00.
    #: The last step was asked for on sight (2026-10-01, marks chalked on the
    #: floor): a 1.20 m ring reads as a big loose circle rather than an escort,
    #: and the 0.30 m it frees goes into vip_offset, moving the whole encounter
    #: toward the audience AND taking the adversary mark off the arena edge
    #: (2.00 m from room centre at 1.20, 1.50 m at 1.00).
    #: The standoff this gives up is to a DJI, not a person; if a HUMAN ever
    #: stands in as the VIP, put this back to 1.50 and accept that the demo
    #: then does not fit this room. min_vip_dist tracks this value -- the ring
    #: IS the separation, so changing one without the other means the guard
    #: either fights the ring or stops enforcing anything.
    ring_radius: float = 1.0       # m, horizontal distance from the VIP
    #: Defender altitude when the VIP is on the floor (a person, or the
    #: fixed point). Ignored when vip_airborne -- see ring_height.
    height: float = 1.2

    #: True when the VIP is itself airborne -- the live demo flies a DJI,
    #: hand-piloted, as the thing being protected. The ring then holds
    #: ``vip_height_offset`` above the VIP's OWN altitude instead of a fixed
    #: height, because a fixed height means the DJI can climb over the
    #: defenders or drop under them, and neither is survivable: a Crazyflie
    #: under a DJI is under a downwash an order of magnitude stronger than the
    #: one that already costs a Crazyflie its thrust below 0.62 m
    #: (arXiv 2507.09463). Level is the only safe relative altitude, and it is
    #: the one geometry the pilot does not have to think about.
    vip_airborne: bool = False
    #: +0.30: the defenders ride ABOVE the DJI, not level with it.
    #:
    #: This was 0.0 and the comment above argued level was "the only safe
    #: relative altitude". That reasoning only ever ruled out the DJI being
    #: ABOVE the Crazyflies (downwash). It does not follow that LEVEL is best,
    #: and level is the one geometry where a lateral mistake by either
    #: aircraft is a collision rather than a near miss. FLOWN 2026-10-06:
    #: downwash was not the problem; being coplanar with a hand-flown DJI was,
    #: both while following it and when the ring re-formed for the landing leg.
    #:
    #: 0.30 m, sized against the ceiling: the ring reaches 1.60 m from the
    #: arena centre, where safety.CEILING_TESTED is 1.95 m, so a DJI at 1.60 m
    #: still puts the ring at 1.90 m. It also makes the handover seamless --
    #: with the DJI at 1.40 m the tracked ring sits at exactly
    #: dji_clear_height, so the defenders do not visibly drop when tracking
    #: turns on. Brief the pilot to hover at or below 1.50 m.
    vip_height_offset: float = 0.30  # m, ring altitude relative to the VIP

    #: Where the ring waits while the DJI takes off or lands, in the paced
    #: mocap-VIP show. NOT ``height``: pinning the ring at the show altitude
    #: parks it exactly where a hand-flown DJI is least precise and most
    #: likely to be -- measured on the shipped config, a DJI climbing to its
    #: 1.2-1.5 m hover passes through 1.20 m with a 0.00 m vertical gap, and
    #: if it settles at 1.5 m the defenders end up 0.30 m UNDERNEATH it, in
    #: the downwash this file calls unsurvivable. The same happens in reverse
    #: on the landing leg. So the ring goes UP and out of the way instead, and
    #: only comes down to the VIP once the pilot says the DJI is settled.
    #:
    #: 1.70 m clears a 1.5 m DJI by 0.20 m and leaves 0.25 m under
    #: safety.CEILING_TESTED (1.95 m at full radius), which is the binding
    #: limit because the ring reaches 1.60 m from the arena centre. Raise it
    #: only against that budget: 1.90 m leaves 0.05 m and is not worth it.
    #: Above the DJI is also the RIGHT side to be on -- a Crazyflie over a
    #: DJI is out of its wash; under it is the case nobody has survived.
    dji_clear_height: float = 1.70   # m, ring altitude during the DJI legs

    # How fast the ring may turn to face a new threat bearing. This is not a
    # comfort setting: a slot moving round the ring travels at
    # ring_radius * phase_rate, and that comes straight out of the same speed
    # budget as following the VIP (see max_vip_speed).
    # 0.3 rad/s puts a slot at 1.5*0.3 = 0.45 m/s, inside v_max with 0.15 m/s
    # left for the P term. It also means a 120 deg swing takes ~7 s, which is
    # slower than it sounds -- check it against the adversary's probe legs.
    phase_rate: float = 0.3        # rad/s

    #: When blocking, the wings leave their 120 deg posts and close up beside
    #: the blocker, so the three of them stand as a wall across the threat
    #: bearing instead of a ring with one drone on it. This is the half-angle
    #: they close to: slot 0 stays on the bearing, the other two sit at
    #: +/- this.
    #:
    #: It is bounded from below by the drones themselves. The tight pair is
    #: LEAD-to-wing, an angular gap of wall_half_angle, so they stand
    #: 2*R*sin(wall_half_angle/2) apart -- a CHORD, so it shrinks with the ring
    #: and this angle cannot be set without knowing ring_radius. At the old
    #: 1.20 m ring, 50 deg held 1.01 m against the 0.90 m plan budget; at
    #: 1.00 m the same 50 deg holds only 0.85 m and plan_escort refuses.
    #: The floor is 2*R*sin(t/2) >= 0.90, i.e. 53.5 deg at R = 1.00; 58 deg
    #: holds 0.97 m, which is the 0.07 m of margin the 1.20 m ring had.
    #: It still reads as a wall: the three gather from the 240 deg arc they
    #: rest on into 116 deg, facing the threat.
    wall_half_angle: float = np.radians(58.0)

    #: Seconds to close the wall, and to open back out. A wing travels
    #: R*(120-50) deg = 1.8 m to take its post; 4 s asks 0.46 m/s of a
    #: 0.60 m/s cap, and fits inside the 8 s probe leg so the wall is shut
    #: before the adversary arrives. Slower is calmer (6 s holds 1.25 m
    #: instead of 1.13 m) and later.
    wall_ramp_s: float = 4.0

    # -- when blocking engages (hysteresis, so it cannot chatter) ----------
    # Not the ring radius plus a bit: at 0.3 rad/s the ring needs seconds to
    # swing onto a new bearing, so it has to START turning while the adversary
    # is still repositioning, not when it commits. MEASURED (plan_escort,
    # default script, 2026-09-23): engaging at 2.5 m left the ring a median
    # 51 deg off the threat bearing when the probe arrived; 2.6 m gives 0 deg.
    # 2.6 also keeps the ring at REST for the first 30% of the run, so the
    # demo shows a clear->blocking transition instead of starting blocked --
    # which is the whole thing an audience is there to see.
    #: RE-FITTED 2026-10-01 to the measured volume. These were 2.6/2.9 m, set
    #: against a 2.5 m arena. The real volume cannot hold a 2.9 m retreat: the
    #: adversary ends up outside tracking, so the retreat legs were cut -- and
    #: that put the adversary permanently INSIDE a 2.6 m alert radius, which is
    #: the "engaged at t+0.0 s and never disengaged" failure this file already
    #: warns about for a centred VIP. Verified in sim: 1 BLOCKING event, zero
    #: clears, the whole demo spent blocked.
    #:
    #: The chain that has to hold is
    #:     probe 1.80 < alert 1.88 < release 2.00 < retreat 2.30
    #: with 1.80 fixed (ring_radius + min_adv_sep -- the guard will not let the
    #: adversary closer) and 2.46 m the furthest it can retreat on the 145/215
    #: deg bearings while staying inside the 2.00 m arena, given the VIP's
    #: 0.60 m offset. The whole chain shifted down 0.20 m with ring_radius, and
    #: the hysteresis widened with the room the smaller ring freed: 0.12 m of
    #: gap between release and retreat where it used to be 0.07 m.
    #: How far AHEAD the ring aims, in seconds of the threat's own angular
    #: motion. The ring turns at phase_rate, so chasing the attacker's CURRENT
    #: bearing is always late: measured 36 deg of median lag against a moving
    #: attacker, which is a blocker visibly trailing rather than planted.
    #: Leading by the measured bearing rate cancels most of it. Too much and
    #: the ring overshoots every time the attacker changes direction, so this
    #: is bounded by lead_max_rad below.
    #: 0.0 -- OFF, by measurement. Leading looked right and measured wrong:
    #: once the blocker is re-selected every step (lead_swap_hyst) the ring is
    #: already on the threat, and aiming further ahead only walks it off.
    #: Measured 2026-10-06 against a circling attacker, median angle between
    #: the nearest defender and the attacker: 8 deg at lead 0.0, 23 deg at
    #: 0.6, 24 deg at 1.2. Re-selecting WHICH slot blocks beats predicting
    #: where the threat will be. Kept as a knob for a fast-moving VIP, where
    #: the ring is translating as well as turning, but prove it before using it.
    threat_lead_s: float = 0.0
    lead_max_rad: float = np.radians(55.0)
    #: On DISENGAGE, hold the ring where it is instead of winding back to the
    #: resting orientation. Returning to rest is what made the formation
    #: oscillate: the attacker backs off past release_radius, the ring spins
    #: home, the attacker returns, the ring spins out again. Holding costs
    #: nothing -- the slots are still evenly spaced, just at a different
    #: phase -- and the next engagement starts from wherever it already is.
    hold_phase_on_clear: bool = True

    #: 2.10 / 2.22, raised from 1.88 / 2.00 on 2026-10-07 after the attacker
    #: punched through to the DJI on a live run.
    #:
    #: The lever is TIME, not geometry. The ring has to translate after a
    #: hand-flown DJI and rotate a defender onto the threat bearing at the
    #: same time, and engaging later leaves it too little of the second.
    #: Measured against a DJI drifting at 0.20 m/s, closest the attacker got
    #: to it: 0.98 m at alert 1.88 (inside the ring -- through), 1.74 m at
    #: 2.00, 1.82 m at 2.10. A STATIC DJI holds it at 1.68 m either way,
    #: which is why this never showed up in sim until the VIP was moved.
    #:
    #: Things that measured as NOT the answer, so they are not tried again:
    #: closing the wall tighter (46 to 58 deg -- identical 1.68/1.69 m, and
    #: engagement drops), spreading the ring out instead (0.98 m, worse), and
    #: turning the ring faster (0.55 -> 0.80 rad/s made it WORSE at 0.80 m,
    #: apparently by overshooting the bearing).
    #:
    #: 2.10 and not higher: it must stay below ReactiveAdversary.stand_off
    #: (2.30 m) or the attacker is engaged while still parked at its station
    #: and the demo never shows a clear->blocking transition at all. At 2.10
    #: the ring is engaged 58% of the run; at 2.25 it is 79% for 0.04 m.
    alert_radius: float = 2.10     # m, adversary-to-VIP distance that engages
    release_radius: float = 2.22   # m, and the larger one that disengages

    # -- speed / acceleration ---------------------------------------------
    v_max: float = 0.6             # m/s, hard cap on a defender setpoint
    a_max: float = 1.0             # m/s^2

    # -- separations, all enforced AFTER the slew (see SetpointGuard) ------
    min_vip_dist: float = 1.0      # m, horizontal, defender to VIP
                                   #   (= ring_radius; move them together)
    min_pair_sep: float = 0.8      # m, defender to defender
    min_adv_sep: float = 0.8       # m, defender to adversary

    #: WHO gives way when the ring and the attacker converge.
    #:
    #: False, and that inverts the original design deliberately. The guard
    #: used to push DEFENDERS away from the adversary, which meant the ring
    #: could never be an obstacle: measured 2026-10-06, an attacker driving
    #: straight in ended 0.34 m from the VIP with the blocker shoved out to
    #: 1.14 m, behind it. The separation was always honoured and the defence
    #: was always theatre.
    #:
    #: That asymmetry was correct while the adversary flew a VERIFIED script
    #: that never came inside ring_radius + min_adv_sep -- the clamp was then
    #: a backstop that should never fire. It stopped being correct when the
    #: attacker started choosing its own path. The attacker now carries a HARD
    #: separation projection of its own (ReactiveAdversary.target), so the
    #: floor is still enforced -- by the party that is trying to close the
    #: distance, which is the only one that can give way without undoing the
    #: thing it is there to do.
    #:
    #: Set True to restore the old behaviour for a scripted adversary.
    defenders_yield: bool = False
    #: Last-resort separation at which a defender gives way EVEN when
    #: defenders_yield is False. Smaller than min_adv_sep on purpose: the
    #: attacker owns the 0.80 m floor and yields first, so the ring can be a
    #: real obstacle -- but the attacker is SLOWER than a defender (0.45 vs
    #: 0.60 m/s), so a ring translating with a moving VIP can run it down
    #: faster than it can retreat. Measured before this existed: a walking VIP
    #: produced 0.31 m of defender-to-adversary separation. This only fires
    #: once the geometry has already failed; at rest it never fires at all.
    #: 0.70, chosen off a measured trade (plan_escort, 2026-10-06). It is the
    #: knob that sets how much of a real obstacle the ring is:
    #:     0.55 -> static held at 1.76 m, walking adv-def 0.52 m
    #:     0.70 -> static held at 1.76 m, walking adv-def 0.61 m
    #:     0.80 -> static held at 1.52 m, walking adv-def 0.79 m   (= old behaviour)
    #: At 0.80 the defenders yield exactly as they always did and the block
    #: weakens; below 0.70 the walking case buys nothing further. Raise it to
    #: 0.80 to trade the block back for margin against a MOVING VIP.
    defender_yield_floor: float = 0.70

    #: Ring turn rate WHILE BLOCKING. phase_rate is what it uses at rest.
    #:
    #: The ring could turn at v_max / ring_radius = 0.60 rad/s; phase_rate
    #: caps it at 0.30, half its capability, because R * phase_rate is spent
    #: out of the same budget as following the VIP. That trade is worth making
    #: at rest and NOT during an encounter: the attacker out-turns a 0.30
    #: rad/s ring as soon as it is inside 1.4 m, and a ring that cannot hold a
    #: bearing cannot block. While engaged the VIP is meant to be hovering, so
    #: spend the budget on turning instead.
    engaged_phase_rate: float = 0.55
    #: How much nearer in bearing another slot must be before it takes over as
    #: the blocker. Without it a threat sitting midway between two slots flips
    #: the lead every step and the ring judders.
    lead_swap_hyst: float = np.radians(18.0)

    # -- the room ----------------------------------------------------------
    #: MEASURED 2026-10-01, and then FLOWN: cf1 lost tracking at 2.24 m on a
    #: slow spiral at ring height, so 2.0 m leaves 0.24 m to a demonstrated
    #: failure rather than to a hand-carried estimate. Tracking is also NOT
    #: isotropic:
    #: by 45 deg sector the first sustained dropout sits at 2.15 m (270-315),
    #: 2.25 m (0-45), 2.31 m (225-270) and never in 180-225, which held to
    #: 2.46 m. 2.1 is the worst sector, so it is the one that binds.
    #: ARENA_RADIUS_PLAN (1.90 m), not the tested edge.
    #:
    #: This clamps every commanded setpoint, and it used to sit at
    #: ARENA_RADIUS_TESTED (2.00 m) -- the radius a drone was FLOWN to
    #: cleanly, with no margin at all, while the ceiling in this same config
    #: already kept the 0.10 m tracking margin. The radius should keep it too:
    #: a setpoint is commanded, the aircraft follows it with error, and
    #: tracking was actually lost at 2.24 m.
    #:
    #: It costs almost nothing, because the clamp was never the binding
    #: constraint. Measured (plan_escort, 2026-10-07) the geometry only ever
    #: reaches 1.60 m for a defender and 1.84 m for the attacker, and the
    #: encounter is unchanged: held at 1.68 m vs 1.67, separations 0.78 vs
    #: 0.79, engaged 39% vs 40%. The whole price is 0.10 m of the DJI pilot
    #: box (vip_keep_in = arena - ring, so 0.90 m instead of 1.00).
    #:
    #: What it buys: containment now trips at arena + contain_margin = 2.15 m,
    #: which is INSIDE the 2.24 m where tracking was lost. The show reacts
    #: while it can still see the drone, instead of after.
    arena_radius: float = safety.ARENA_RADIUS_PLAN   # 1.90 m from room centre
    #: 1.85 m, not 2.0: this clamps ring_height, and the ring reaches 1.60 m
    #: from the arena centre where safety.CEILING_TESTED is 1.95 m. 2.0 would
    #: let a high-hovering DJI drag the ring above altitude anyone has proven
    #: tracking at. 1.85 keeps the usual 0.10 m margin under 1.95.
    ceiling: float = 1.85          # m
    floor: float = 0.3             # m
    #: MEASURED 2026-10-01 by walking cf1 through the volume (scripts/
    #: measure_arena.py, 8067 samples): the centroid of the tracked space is
    #: NOT the room centre the other shows use. The escort is the only show
    #: that puts a drone near the edge on purpose, so it is centred on what
    #: Motive can SEE rather than on the room.
    #:
    #: DERIVED from safety.ARENA_CENTRE, not written out again. It was a
    #: literal (0.033, 0.255) until 2026-10-08, when Motive was recalibrated
    #: and the surveyed centroid had to be re-expressed in the new world
    #: frame: safety.py was updated and this copy was not, so every escort
    #: geometry -- the geofence keep-in, the arena clamp, the VIP mark, role
    #: assignment -- silently kept using a centre 0.41 m from the real one
    #: while safety.check_envelope used the right one. Two sources of truth
    #: for a surveyed physical place is the bug; there is now one.
    room_center: tuple = tuple(safety.ARENA_CENTRE)

    #: Where the VIP stands for the static stages, and the centre the walking
    #: stages start from. NOT the room centre, deliberately: a VIP in the
    #: middle leaves the adversary only arena_radius - (ring_radius +
    #: min_adv_sep) = 0.2 m of stand-off before the arena clamp drags it in,
    #: so the "approach" starts already blocked (measured in sim,
    #: 2026-09-23 -- the demo engaged at t+0.0 s and never disengaged).
    #: Offsetting the VIP 1.0 m gives the adversary the far half of the room
    #: to run at, and still leaves ring_radius + 1.0 = 2.5 m for the ring.
    #: Chosen against the measured volume (walks of 2026-10-01), not symmetry.
    #: +0.30 m along 0 deg: the VIP sits on the +x side, NEAREST the operator,
    #: who stands on +x looking down -x. The adversary therefore runs at it
    #: from -x, across the far half of the room, so the encounter happens
    #: facing the audience rather than behind the VIP.
    #:
    #: 0.60 m, restored once ring_radius came down to 1.00 m. The geofence
    #: measures the VIP's stray from ROOM CENTRE and keep-in is
    #: arena - ring, so the pilot box is arena - ring - offset: at ring 1.20
    #: an 0.60 m offset left a hand-flown DJI just 0.20 m before the ring
    #: stopped following it, which is why this sat at 0.30 m. At ring 1.00 the
    #: same 0.60 m offset gives 0.40 m, and the box is a CIRCLE of radius
    #: 1.00 m about room centre -- so the DJI has 0.40 m further toward the
    #: operator and 1.60 m away from them, which is the asymmetry you want
    #: when the far side is the only place it should be flown.
    #:
    #: (Previously 0.60 m along 180 deg, keeping the direction the first offset
    #: used.)
    #: That matters: AdversaryScript's legs are absolute +-35 deg bearings, so
    #: they are only symmetric about the open floor while the VIP sits on the
    #: 180 deg side. Offsetting the VIP anywhere else silently makes one half
    #: of the probe reach further from the volume centre than the other -- at
    #: 200 deg the -35 deg retreat came out at 2.31 m against a 2.10 m arena
    #: while the +35 deg one fitted, which is how the demo ended up refusing
    #: on only half its own script. Distance (0.60 m) is set by the volume:
    #: the ring reached vip_offset + ring_radius = 2.10 m toward 180 deg,
    #: inside the 2.46/2.70 m both walks measured there -- and with ring 1.00
    #: it reaches only vip_offset + ring_radius = 1.60 m that way. The offset
    #: is also retreat room: the adversary has to get back past
    #: release_radius for the demo to show a clear->blocking transition at
    #: all, and every centimetre the VIP moves toward the operator is a
    #: centimetre more room on the far side for it to do that in.
    #: Symmetric placement does not fit at all: with the VIP at the centre the
    #: adversary mark lands at ring + min_adv_sep + 0.3 = 2.10 m from room
    #: centre, outside the 2.00 m arena and in the weakest sector (2.15 m).
    vip_offset: tuple = (0.60, 0.0)

    # -- loop / estimation -------------------------------------------------
    rate_hz: float = 20.0          # setpoint stream rate per drone
    vip_lowpass_hz: float = 4.0    # corner of the VIP velocity filter

    # -- staleness (mocap dropout) ----------------------------------------
    vip_stale_hold_s: float = 0.4  # no VIP pose this long -> stop moving
    vip_stale_land_s: float = 2.0  # ...this long -> land, uninvited
    #: Per-drone mocap watchdog. Until 2026-10-07 self_stale_s was DEAD
    #: CONFIG -- defined, documented as "hold its setpoint", read nowhere --
    #: so the only drone with any mocap protection was the VIP. A Crazyflie
    #: that lost tracking kept being commanded from a model that could not
    #: see it, while its own estimator drifted; that is how the attacker flew
    #: away on 2026-10-07.
    self_stale_s: float = 0.3      # no pose for a drone -> hold its setpoint
    self_stale_land_s: float = 1.0  # ...this long -> land, uninvited

    #: CONTAINMENT, checked against each drone's ACTUAL pose rather than the
    #: setpoint. Clamping a setpoint proves nothing about the aircraft: the
    #: commanded point stayed inside the arena for the whole fly-away.
    #:
    #: contain_margin is how far past arena_radius a real drone may be before
    #: the show lands everything. 0.25 m puts the trip at 2.25 m, which is
    #: essentially safety.ARENA_RADIUS_LOST (2.24 m) -- past there tracking is
    #: gone anyway, so there is nothing to be gained by waiting.
    contain_margin: float = 0.25   # m beyond arena_radius

    #: A drone that is NOT FOLLOWING is the earliest symptom there is, and it
    #: shows up while the drone is still inside the room -- before staleness,
    #: before containment. Both must hold for track_error_s so a single bad
    #: frame or a hard slew cannot trip it.
    track_error_m: float = 0.60    # m, actual vs commanded
    track_error_s: float = 0.80    # s it must persist



def vip_home(cfg):
    """The static VIP mark: room centre plus ``vip_offset``."""
    return np.array([cfg.room_center[0] + cfg.vip_offset[0],
                     cfg.room_center[1] + cfg.vip_offset[1], 0.0])


def wrap_pi(a):
    """Wrap an angle to (-pi, pi]."""
    return (a + np.pi) % TWO_PI - np.pi


def slot_offsets(n, close, half_angle, lead=0):
    """Angular offsets of each slot from the ring phase.

    ``close`` blends from the resting ring (evenly spaced, 2*pi/n apart) to
    the wall: the LEAD slot holds the threat bearing and the others gather to
    +/- ``half_angle`` beside it. Blending rather than switching matters --
    the slots are what the drones chase, and a slot that teleports across the
    ring is a drone commanded to fly there as fast as its cap allows.

    ``lead`` is why this takes an index at all. Making slot 0 the blocker
    every time forces the whole ring to spin until slot 0 reaches the threat
    -- up to 180 deg, and measured at 155 deg on the default script, which no
    drone can fly at 0.6 m/s while the wings are also closing (the wing that
    had to go the long way round lagged its slot by 1.55 m). Leading with the
    slot ALREADY nearest the threat caps that rotation at half the slot
    spacing: 60 deg for three drones. It is also the behaviour you actually
    want described in one line -- the drone that meets the adversary first is
    the one the others reinforce.

    Slot ORDER is untouched: each wing closes toward the lead from the side
    it already rests on, so nobody crosses anybody.
    """
    base = wrap_pi(TWO_PI * np.arange(n) / n)
    rel = wrap_pi(base - base[lead])
    wall = base[lead] + np.sign(rel) * half_angle
    # Blend along the SHORTEST arc to the wall post, not by interpolating the
    # angle values. Interpolating the values sends a wing the long way round:
    # measured (plan_escort, lead=1, 2026-09-24) a wing swept from -120 deg up
    # through 0 to +170, passing straight through the lead's post, and the
    # commanded separation fell to 0.26 m mid-close. Every clamp downstream
    # then spent the blend fighting the geometry.
    return base + close * wrap_pi(wall - base)


def slot_angles(psi, n, close=0.0, half_angle=0.0, lead=0):
    """The ``n`` slot bearings of a ring whose slot 0 sits at ``psi``."""
    return psi + slot_offsets(n, close, half_angle, lead)


def ring_height(cfg, p_vip):
    """Altitude the ring holds, given where the VIP is.

    A floor VIP (a person, or the fixed point) gets the fixed ``height``. An
    airborne VIP gets matched, clamped into the altitude band -- if the pilot
    flies below the floor or above the ceiling the defenders stay in the band
    and the vertical gap opens, which is reported rather than chased.
    """
    if not cfg.vip_airborne:
        return cfg.height
    z = float(np.asarray(p_vip, float)[2]) + cfg.vip_height_offset
    return float(np.clip(z, cfg.floor, cfg.ceiling))


def contain_check(cfg, p_live, age, sp, dt, state):
    """One drone's containment verdict: ``(action, reason)``.

    ``action`` is ``'ok'``, ``'hold'`` or ``'land'``; ``reason`` is None when
    ok. ``state`` is a per-drone dict the caller keeps between calls (timers).

    Pure: no ROS, no drones. The order is deliberate -- staleness first,
    because a stale pose makes the other two meaningless, and there is no
    point reporting "outside the arena" from a reading that is a second old.
    """
    bad = float(age) > cfg.self_stale_land_s
    if bad:
        return 'land', f'no mocap for {age:.1f} s'
    if float(age) > cfg.self_stale_s:
        state['drift'] = 0.0          # cannot judge tracking without a pose
        return 'hold', f'no mocap for {age:.1f} s'

    p = np.asarray(p_live, float)
    r = float(np.linalg.norm(p[:2] - np.array(cfg.room_center)))
    if r > cfg.arena_radius + cfg.contain_margin:
        return 'land', (f'{r:.2f} m from room centre, outside the '
                        f'{cfg.arena_radius + cfg.contain_margin:.2f} m '
                        'containment')
    if sp is not None:
        err = float(np.linalg.norm(p[:2] - np.asarray(sp, float)[:2]))
        if err > cfg.track_error_m:
            state['drift'] = state.get('drift', 0.0) + float(dt)
            if state['drift'] >= cfg.track_error_s:
                return 'land', (f'{err:.2f} m from where it was commanded for '
                                f'{state["drift"]:.1f} s -- not following')
        else:
            state['drift'] = 0.0
    return 'ok', None


def vip_keep_in(cfg):
    """How far the VIP may stray from room centre before the ring will not fit."""
    return cfg.arena_radius - cfg.ring_radius


def vip_problems(cfg, p_vip, v_vip=None, defenders=()):
    """Live complaints about where the VIP is and how fast it is going.

    None of these can be fixed by commanding anything -- the VIP is a person
    or a hand-flown DJI. They exist to be SAID, early and in plain words,
    because each one degrades the escort silently otherwise.
    """
    out = []
    p_vip = np.asarray(p_vip, float)
    c = np.array(cfg.room_center)
    stray = float(np.linalg.norm(p_vip[:2] - c))
    if stray > vip_keep_in(cfg):
        out.append(f'the VIP is {stray:.2f} m from room centre, past the '
                   f'{vip_keep_in(cfg):.2f} m where a {cfg.ring_radius:.2f} m '
                   'ring still fits inside the arena -- the far slots are '
                   'being clamped and the ring is no longer a ring')
    if v_vip is not None:
        speed = float(np.linalg.norm(np.asarray(v_vip, float)[:2]))
        if speed > max_vip_speed(cfg):
            out.append(f'the VIP is moving at {speed:.2f} m/s, over the '
                       f'{max_vip_speed(cfg):.2f} m/s the ring can follow while '
                       'also turning -- the formation is lagging, not holding')
    if cfg.vip_airborne:
        for i, p in enumerate(defenders):
            p = np.asarray(p, float)
            dz = float(p_vip[2] - p[2])
            if abs(dz) > DOWNWASH_DZ and float(np.linalg.norm(p[:2] - p_vip[:2])) < DOWNWASH_RXY:
                out.append(f'the VIP is {dz:+.2f} m vertically from defender '
                           f'{i} and only '
                           f'{float(np.linalg.norm(p[:2] - p_vip[:2])):.2f} m '
                           'away horizontally -- one is flying over the other')
    return out


#: Vertical gap below which one aircraft over another is a downwash problem,
#: and the horizontal radius over which it matters. 0.62 m is the Crazyflie
#: figure (dz/l > 19, arXiv 2507.09463); a DJI throws far more air than that,
#: so treat this as the floor of the safe gap, not a measured one for the DJI.
DOWNWASH_DZ = 0.62
DOWNWASH_RXY = 1.0


def ring_targets(p_vip, psi, cfg, close=0.0, lead=0):
    """Slot positions (n, 3) for a ring centred on ``p_vip`` at phase ``psi``.

    ``p_vip`` may be 2D or 3D; only x and y are used. The ring is always flat
    and at ``cfg.height`` -- the VIP is a person on the floor, and a defender
    directly above anyone (or anything) is the one geometry this rig must
    never fly: downwash from an upper drone costs the lower one enough thrust
    to fall out of the sky, and the measured threshold is dz/l > 19, about
    0.62 m for a Crazyflie (arXiv 2507.09463, Preiss et al. arXiv 1704.04852).
    """
    ang = slot_angles(psi, cfg.n_defenders, close, cfg.wall_half_angle, lead)
    out = np.empty((cfg.n_defenders, 3))
    out[:, 0] = p_vip[0] + cfg.ring_radius * np.cos(ang)
    out[:, 1] = p_vip[1] + cfg.ring_radius * np.sin(ang)
    out[:, 2] = ring_height(cfg, p_vip)
    return out


def max_vip_speed(cfg):
    """How fast the VIP may move and still be escorted, m/s.

    A defender has ``v_max`` of budget. Translating with the VIP costs the
    VIP's own speed; turning the ring costs ``ring_radius * phase_rate`` on
    top, and the two add when a slot is moving round the ring in the
    direction the VIP is walking. What is left is the margin the P term needs
    to actually close an error, so this is an upper bound on the walk speed,
    not a target.

    With the shipped defaults: 0.6 - 1.5*0.3 = 0.15 m/s. That is a slow
    deliberate walk and nothing more, and it is the number to quote when
    someone asks how fast the person may move. Buying more of it means
    raising v_max (a safety decision about flying faster next to a person),
    slowing the ring (a slower block), or shrinking the radius (less room
    between the drones and the person) -- see ESCORT.md.
    """
    return cfg.v_max - cfg.ring_radius * cfg.phase_rate


def check_config(cfg, moving_vip=True):
    """Offline proof of the geometry. Returns a list of problems (empty = OK).

    Deliberately returns rather than raises, so ``plan_escort`` can print all
    of them at once instead of one per run.
    """
    bad = []
    if cfg.n_defenders < 2:
        bad.append(f'n_defenders={cfg.n_defenders}: a ring needs at least 2')
    if cfg.ring_radius < cfg.min_vip_dist:
        bad.append(f'ring_radius {cfg.ring_radius:.2f} m is inside '
                   f'min_vip_dist {cfg.min_vip_dist:.2f} m -- the ring itself '
                   'would violate the standoff')
    chord = 2.0 * cfg.ring_radius * np.sin(np.pi / max(cfg.n_defenders, 2))
    if chord < cfg.min_pair_sep:
        bad.append(f'adjacent slots are {chord:.2f} m apart, under '
                   f'min_pair_sep {cfg.min_pair_sep:.2f} m -- widen the ring '
                   f'or drop a defender')
    # The tight pair in a wall is LEAD-to-wing, separated by wall_half_angle
    # -- not wing-to-wing, which spans twice that. Checking the wide pair is
    # how 35 deg looked fine on paper (1.72 m) and flew at 0.90 m.
    wall_chord = 2.0 * cfg.ring_radius * np.sin(cfg.wall_half_angle / 2.0)
    if wall_chord < safety.PLAN_SEPARATION:
        bad.append(f'closed up as a wall, the lead and each wing stand '
                   f'{wall_chord:.2f} m apart, under PLAN_SEPARATION '
                   f'{safety.PLAN_SEPARATION:.2f} m -- open wall_half_angle '
                   f'(now {np.degrees(cfg.wall_half_angle):.0f} deg) or widen '
                   f'the ring')
    if cfg.vip_airborne and vip_keep_in(cfg) <= 0.3:
        bad.append(f'an airborne VIP would have only {vip_keep_in(cfg):.2f} m '
                   'of room to fly in before the ring stops fitting in the '
                   'arena -- shrink ring_radius, or the demo is a hover')
    if not (cfg.floor < cfg.height < cfg.ceiling):
        bad.append(f'height {cfg.height:.2f} m is outside the band '
                   f'({cfg.floor:.2f}, {cfg.ceiling:.2f}) m')
    if cfg.release_radius <= cfg.alert_radius:
        bad.append('release_radius must exceed alert_radius, or blocking '
                   'chatters on the threshold')
    if cfg.alert_radius < cfg.ring_radius + cfg.min_adv_sep:
        bad.append(f'alert_radius {cfg.alert_radius:.2f} m is inside the ring '
                   f'plus min_adv_sep ({cfg.ring_radius + cfg.min_adv_sep:.2f} m) '
                   '-- blocking would only engage once the adversary is '
                   'already through the defenders')
    if cfg.ring_radius >= cfg.arena_radius:
        bad.append(f'ring_radius {cfg.ring_radius:.2f} m >= arena_radius '
                   f'{cfg.arena_radius:.2f} m -- the ring cannot fit')
    reach = float(np.hypot(*cfg.vip_offset)) + cfg.ring_radius
    if reach > cfg.arena_radius:
        bad.append(f'the ring around the offset VIP reaches {reach:.2f} m from '
                   f'room centre, outside arena_radius {cfg.arena_radius:.2f} m '
                   '-- move the VIP closer to the centre or shrink the ring')
    if cfg.v_max <= cfg.ring_radius * cfg.phase_rate:
        bad.append(f'a slot travels {cfg.ring_radius * cfg.phase_rate:.2f} m/s '
                   f'when the ring turns, at or over v_max {cfg.v_max:.2f} m/s '
                   '-- the ring can never catch a new threat bearing')
    elif moving_vip and max_vip_speed(cfg) <= 0.1:
        bad.append(f'max_vip_speed {max_vip_speed(cfg):.2f} m/s: after turning '
                   'the ring there is no speed budget left to follow a moving '
                   'VIP. Fine for the static-point stage, not for a walking '
                   'person -- see ESCORT.md "Decisions still open"')
    return bad


class VelocityEstimator:
    """Low-passed finite difference of a mocap position.

    A raw difference of two mocap samples is dominated by jitter at 27 Hz, and
    this value is fed FORWARD into the setpoint, so its noise lands directly on
    the drones. The 4 Hz corner is inherited from the follow-drone prototype's
    mocap_target for the same reason.
    """

    def __init__(self, corner_hz=4.0):
        self.corner_hz = corner_hz
        self._p = None
        self._v = np.zeros(3)

    def update(self, p, dt):
        p = np.asarray(p, float)
        if self._p is None or dt <= 0.0:
            self._p = p.copy()
            return self._v.copy()
        raw = (p - self._p) / dt
        self._p = p.copy()
        alpha = 1.0 - np.exp(-TWO_PI * self.corner_hz * dt)
        self._v += alpha * (raw - self._v)
        return self._v.copy()

    @property
    def value(self):
        return self._v.copy()


class ThreatLatch:
    """Is the adversary close enough to block? Hysteretic, so it cannot chatter."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.engaged = False

    def update(self, p_vip, p_adv):
        if p_adv is None:
            self.engaged = False
            return False
        d = float(np.linalg.norm(np.asarray(p_adv, float)[:2] - np.asarray(p_vip, float)[:2]))
        if self.engaged:
            self.engaged = d <= self.cfg.release_radius
        else:
            self.engaged = d <= self.cfg.alert_radius
        return self.engaged


class PhaseTracker:
    """The ring's phase, slew-rate-limited towards a target bearing.

    The rate limit is what makes the block *readable* to an audience and safe
    to fly: without it a threat bearing that jumps (adversary crossing behind
    the VIP, or a mocap glitch) would command three drones to teleport round
    the ring.
    """

    def __init__(self, cfg, psi0=0.0):
        self.cfg = cfg
        self.psi = float(psi0)

    def update(self, psi_target, dt, rate=None):
        """``rate`` overrides phase_rate -- the ring turns harder while it is
        blocking, because that is when the VIP is meant to be hovering and the
        speed budget is better spent on facing the threat than on following."""
        r = self.cfg.phase_rate if rate is None else float(rate)
        step = np.clip(wrap_pi(psi_target - self.psi), -r * dt, r * dt)
        self.psi = wrap_pi(self.psi + step)
        return self.psi


class SetpointGuard:
    """Clamp one drone's setpoint. The ORDER of the clamps is load-bearing.

    Slew first, then re-assert the separations. Doing it the other way round
    silently does nothing: the slew interpolates from the previous setpoint
    towards the new one, so a target that was pushed out to the standoff
    distance gets dragged back inside it by the very next step. The
    follow-drone prototype shipped that bug and two of its tests exist purely
    to keep it fixed (crazyflie_follow/README.md, "SetpointGuard re-asserts
    the standoff clamp *after* the slew").

    Every clamp here is a *last* line. The geometry is supposed to be safe
    already -- check_config proves the ring is -- and a guard that fires in
    flight means the plan was wrong, so it says so.
    """

    def __init__(self, cfg, p0):
        self.cfg = cfg
        self.sp = np.asarray(p0, float).copy()
        self.v = np.zeros(3)
        self.reasons = []

    def step(self, target, dt, p_vip=None, p_adv=None, p_others=(),
             vip_dist=None, adv_dist=None):
        """``vip_dist`` overrides ``cfg.min_vip_dist`` for THIS drone.

        It exists for the adversary. A defender's floor to the VIP is the
        ring it holds; the attacker's job is to get inside that ring, so it
        needs a smaller floor -- but it does need one, because under
        ``adversary:=manual`` nothing else bounds it. Measured in sim
        (2026-10-02) before this existed: holding one direction key flew the
        commanded adversary from -1.47 m straight THROUGH the VIP point,
        passing 0.036 m from it, and stopped only at the arena wall.
        """
        cfg = self.cfg
        self.reasons = []
        target = np.asarray(target, float)

        # 1. speed + acceleration slew, in that order
        want_v = (target - self.sp) / max(dt, 1e-3)
        dv = want_v - self.v
        dv_max = cfg.a_max * dt
        n = float(np.linalg.norm(dv))
        if n > dv_max > 0.0:
            dv *= dv_max / n
            self.reasons.append('accel')
        v = self.v + dv
        n = float(np.linalg.norm(v))
        if n > cfg.v_max:
            v *= cfg.v_max / n
            self.reasons.append('speed')
        sp = self.sp + v * dt

        # 2. separations, re-asserted AFTER the slew (see the docstring).
        #
        # Each push is a projection onto one "outside this disc" set, and the
        # LAST one applied wins -- so a single pass let the push away from the
        # adversary shove the defender inside the VIP standoff (measured:
        # 1.27 m against a 1.50 m minimum, plan_escort 2026-09-23). Two fixes,
        # both needed: iterate, so the projections converge on a point outside
        # all of them, and always finish with the VIP. The person is the one
        # separation that is never traded away.
        for _ in range(3):
            if p_adv is not None:
                sp = _push_out(sp, p_adv,
                               cfg.min_adv_sep if adv_dist is None else adv_dist,
                               self.reasons, 'adversary')
            for q in p_others:
                sp = _push_out(sp, q, cfg.min_pair_sep, self.reasons, 'defender')
            if p_vip is not None:
                sp = _push_out(sp, p_vip,
                               cfg.min_vip_dist if vip_dist is None else vip_dist,
                               self.reasons, 'vip')

        # 3. the room
        c = np.array([cfg.room_center[0], cfg.room_center[1]])
        r = sp[:2] - c
        n = float(np.linalg.norm(r))
        if n > cfg.arena_radius:
            sp[:2] = c + r * (cfg.arena_radius / n)
            self.reasons.append('arena')
        if not (cfg.floor <= sp[2] <= cfg.ceiling):
            sp[2] = float(np.clip(sp[2], cfg.floor, cfg.ceiling))
            self.reasons.append('altitude')

        # the slew's velocity must reflect what we actually committed to, or
        # the next accel clamp is computed against a setpoint that never was
        self.v = (sp - self.sp) / max(dt, 1e-3)
        self.sp = sp
        return sp.copy()


#: Deadband on every separation clamp, m. The ring radius equals the VIP
#: standoff by design, so a slot sits exactly ON the boundary and float noise
#: alone would otherwise report the guard "firing" every single step, burying
#: the real ones. 1 mm is far below anything the drones can hold.
PUSH_DEADBAND = 1e-3


def _push_out(sp, other, dmin, reasons, tag):
    """Move ``sp`` radially out (in xy) until it is ``dmin`` from ``other``."""
    d = sp[:2] - np.asarray(other, float)[:2]
    n = float(np.linalg.norm(d))
    if n >= dmin - PUSH_DEADBAND:
        return sp
    if n < 1e-6:                       # exactly on top: pick a direction
        d, n = np.array([1.0, 0.0]), 1.0
    sp = sp.copy()
    sp[:2] = np.asarray(other, float)[:2] + d * (dmin / n)
    reasons.append(tag)
    return sp


@dataclass
class AdversaryScript:
    """The adversary's path: a scripted probe, or nothing at all.

    Scripted until the teleop stage, on purpose. The defenders' reaction is
    the thing being demonstrated *and* the thing that can go wrong, so it is
    the only part that should be free to vary; a scripted attacker also means
    ``plan_escort`` can prove the whole encounter on the ground.

    A leg is (duration_s, bearing_deg, radius_m): the adversary flies to a
    point at that bearing and distance from the VIP's *current* position, so
    the probe still makes sense when the VIP is walking. ``closest`` is the
    nearest radius any leg commands, and it is what check_script measures.
    """

    height: float = 1.2
    #: Closest approach is ring_radius + min_adv_sep, not less: the blocker
    #: holds the ring and the adversary is stopped a clear gap outside it.
    #: Probing closer does not look bolder, it just makes the guard fight the
    #: script for the whole leg (measured, plan_escort, 2026-09-23).
    #: Bearings sit either side of the open side of the room (+x, away from
    #: the VIP's offset), NOT all the way round: a leg behind the VIP puts the
    #: adversary through the wall. check() verifies every leg against the
    #: actual arena, which is the part to trust when these are edited.
    #: Retreat radii cut from 2.9/3.0 m to 2.55/2.60 m on 2026-10-01: measured
    #: against the real volume, a retreat to 3.0 m put the adversary 2.42 m
    #: from the volume centre on a bearing whose first sustained dropout is at
    #: 2.25 m -- i.e. the "give up and back off" legs flew it out of tracking.
    #: The probe radius is always ring_radius + min_adv_sep exactly -- it is
    #: the one number here that is a safety limit rather than staging, so it
    #: moves whenever the ring does and never independently.
    #: Bearings are ABSOLUTE and sit either side of 180 deg, because the VIP
    #: moved to +x: the adversary must approach across the far half of the
    #: room, not through the operator. The radii moved with ring_radius on
    #: 2026-10-01: probe 2.00 -> 1.80 m (still exactly ring + min_adv_sep) and
    #: standoff 2.20 -> 2.30 m, which the smaller ring makes affordable. At
    #: 145 deg with the VIP 0.60 m out on +x, a 2.30 m standoff leaves the
    #: adversary 1.78 m from the volume centre, 0.22 m inside the 2.00 m
    #: arena -- so the retreat genuinely clears release_radius instead of
    #: being clamped back in with the block still engaged.
    #: ONE approach, then it backs off. Cut from six legs to three on
    #: 2026-10-06: the second run at the other bearing doubled the length
    #: without showing anything the first had not, and a demo you have to
    #: narrate twice is harder to narrate once. The remaining three are the
    #: whole story -- arrive, commit, be turned away.
    legs: tuple = ((6.0, 145.0, 2.30),   # rise and sit off to one side
                   (8.0, 145.0, 1.80),   # the probe = ring_radius + min_adv_sep
                   (6.0, 145.0, 2.30))   # turned away, backs off past release

    #: One label per leg, for the operator-paced mode: what the NEXT press of
    #: Enter is about to make the adversary do. Kept beside the legs rather
    #: than in the show, so editing a leg and forgetting its label is a visible
    #: mismatch in one file instead of a lie printed at the operator.
    labels: tuple = ('rise and sit off to one side',
                     'ATTACK - run at the VIP',
                     'turned away - backs off and stands down')

    def target(self, t, p_vip, defenders=(), cfg=None):
        """Where the adversary should be at show time ``t``.

        ``defenders``/``cfg`` are accepted and ignored so this and
        :class:`ReactiveAdversary` are interchangeable at every call site.
        """
        p_vip = np.asarray(p_vip, float)
        t0 = 0.0
        last = len(self.legs) - 1
        # Compare by INDEX, not by value. `leg == self.legs[-1]` matched any
        # leg that happened to be EQUAL to the last one, so a script whose
        # first and last legs are the same tuple -- which is exactly what
        # "arrive, attack, go back where you came from" looks like -- returned
        # leg 0 for all t and the adversary never moved. Found 2026-10-06 when
        # the script was cut to three legs: plan_escort reported "blocking
        # engaged 0% of the run", which is the only reason it was caught.
        for i, (dur, bearing, radius) in enumerate(self.legs):
            if t <= t0 + dur or i == last:
                a = np.radians(bearing)
                return np.array([p_vip[0] + radius * np.cos(a),
                                 p_vip[1] + radius * np.sin(a),
                                 self.height])
            t0 += dur
        raise AssertionError('unreachable')      # pragma: no cover

    @property
    def duration(self):
        return sum(d for d, _, _ in self.legs)

    def leg_start(self, i):
        """Show time at which leg ``i`` begins."""
        return sum(d for d, _, _ in self.legs[:max(0, min(i, len(self.legs)))])

    def leg_mid(self, i):
        """A time safely INSIDE leg ``i``.

        The operator-paced mode holds the adversary on one leg until Enter, and
        ``target()`` selects the leg by time, so it needs a time that cannot sit
        exactly on a boundary and select the neighbour.
        """
        i = max(0, min(i, len(self.legs) - 1))
        return self.leg_start(i) + self.legs[i][0] * 0.5

    def label(self, i):
        if 0 <= i < len(self.labels):
            return self.labels[i]
        return f'leg {i + 1}'

    @property
    def n_legs(self):
        return len(self.legs)

    def check(self, cfg, p_vip=None):
        """Problems with this script against ``cfg`` (empty list = OK).

        ``p_vip`` is where the VIP will actually be; the legs are relative to
        it, so containment can only be checked once it is known. A walking
        VIP narrows this further -- what is verified here is the start.
        """
        bad = []
        p_vip = vip_home(cfg) if p_vip is None else np.asarray(p_vip, float)
        c = np.array(cfg.room_center)
        for dur, bearing, radius in self.legs:
            a = np.radians(bearing)
            p = p_vip[:2] + radius * np.array([np.cos(a), np.sin(a)])
            d = float(np.linalg.norm(p - c))
            if d > cfg.arena_radius:
                bad.append(f'leg ({bearing:+.0f} deg, {radius:.1f} m) puts the '
                           f'adversary {d:.2f} m from room centre, outside '
                           f'arena_radius {cfg.arena_radius:.2f} m -- the guard '
                           'would drag it inside and the block would engage '
                           'before the approach even starts')
        if abs(self.height - cfg.height) > 0.3:
            bad.append(f'adversary height {self.height:.2f} m differs from the '
                       f'defenders\' {cfg.height:.2f} m by more than 0.3 m. '
                       'Keep them level: dz under ~0.62 m is where downwash '
                       'from the upper drone starts costing the lower one '
                       'thrust, and a drone crossing OVER another is the one '
                       'geometry this rig never flies.')
        closest = min(r for _, _, r in self.legs)
        if closest <= cfg.ring_radius:
            bad.append(f'the script commands the adversary to {closest:.2f} m '
                       f'from the VIP, inside the ring at {cfg.ring_radius:.2f} m '
                       '-- it would fly through the defenders rather than be '
                       'blocked by them')
        if closest < cfg.ring_radius + cfg.min_adv_sep:
            bad.append(f'closest approach {closest:.2f} m leaves under '
                       f'min_adv_sep {cfg.min_adv_sep:.2f} m between the '
                       f'adversary and a defender holding the ring at '
                       f'{cfg.ring_radius:.2f} m; the guard would be fighting '
                       'the script for the whole probe')
        return bad


def check_gather(src, dst, labels, cfg, min_sep=None):
    """Problems with the simultaneous move onto the ring (empty list = OK).

    The shows verify every leg they fly (``safety.check_leg``); this one did
    not, and got away with it on geometry alone -- with the marks of
    2026-09-24 the gather happens to clear at 1.27 m. "Happens to" is not a
    safety argument: the drones move to the ring TOGETHER, the firmware has no
    collision avoidance, and wrong-corner placement is the 2026-08-04 collision
    (HANDOVER.md section 6). ``src``/``dst`` include the adversary when it is
    one of ours, because it flies the same leg at the same time.
    """
    min_sep = safety.PLAN_SEPARATION if min_sep is None else min_sep
    bad = []
    sep = safety.transition_min_sep(src, dst)
    if sep < min_sep:
        bad.append(f'gather legs close to {sep:.2f} m, under {min_sep:.2f} m '
                   f'({", ".join(labels)}) -- move the drones apart on the '
                   'floor, or pick roles so nobody has to cross the ring')
    return bad


def adversary_start_problem(p_adv, p_vip, cfg):
    """Why this adversary position cannot start the demo, or None.

    An adversary sitting INSIDE the ring has already won before the show
    starts, and its first leg out crosses the defenders' slots. With the
    default roles (first three enabled drones defend, the fourth attacks) this
    is not hypothetical: on 2026-09-24 the fourth drone stood 1.21 m from the
    VIP mark, inside a 1.50 m ring.
    """
    d = float(np.linalg.norm(np.asarray(p_adv, float)[:2]
                             - np.asarray(p_vip, float)[:2]))
    need = cfg.ring_radius + cfg.min_adv_sep
    if d < need:
        return (f'the adversary starts {d:.2f} m from the VIP mark, inside the '
                f'ring plus min_adv_sep ({need:.2f} m). It would fly out '
                'through the defenders. Stand it further out, or name a '
                'different drone with adversary_drone.')
    return None


def pick_roles(names, positions, cfg, n_defenders=None):
    """Default roles by geometry: the nearest drones defend, the farthest attacks.

    Taking the first three names instead (what this used to do) is arbitrary
    -- it depends on lexicographic order, not on where anything is standing,
    so it can hand the adversary a mark inside the ring and send a defender
    across the whole room.
    """
    n = cfg.n_defenders if n_defenders is None else n_defenders
    p_vip = vip_home(cfg)
    order = sorted(names, key=lambda k: float(np.linalg.norm(
        np.asarray(positions[k], float)[:2] - p_vip[:2])))
    return order[:n], (order[-1] if len(order) > n else None)


class EscortController:
    """The whole law: ring slots, threat-facing rotation, per-drone guards.

    Slot order is decided ONCE, by :meth:`assign`, and never changes again --
    that is what keeps the defenders from swapping places in mid-air.
    """

    def __init__(self, cfg, p_defenders, p_vip, psi0=None):
        self._prev_phi = None
        self._phi_rate = 0.0
        self.cfg = cfg
        p_defenders = [np.asarray(p, float) for p in p_defenders]
        self.n = len(p_defenders)
        # ALWAYS assign: it is what sets self.order (the slot mapping), and
        # skipping it when a phase was supplied left the controller without
        # one -- slot_of() then raises AttributeError on the first step. No
        # caller in the repo passes psi0 today, which is why this never fired.
        auto_psi0 = self.assign(p_defenders, p_vip)
        psi0 = auto_psi0 if psi0 is None else psi0
        self.phase = PhaseTracker(cfg, psi0)
        self.latch = ThreatLatch(cfg)
        self.vip_vel = VelocityEstimator(cfg.vip_lowpass_hz)
        self.guards = [SetpointGuard(cfg, p) for p in p_defenders]
        self.rest_psi = psi0
        self.engaged = False
        self.untrusted = []
        #: 0 = resting ring, 1 = closed up as a wall. Ramped, never switched.
        self.close = 0.0
        #: Which slot holds the threat bearing. Chosen when blocking engages
        #: (the one already nearest the adversary) and held until it releases,
        #: so the wall cannot flip sides while it is closing.
        self.lead = 0

    def assign(self, p_defenders, p_vip):
        """Pick the ring phase that suits where the drones already are.

        Each drone keeps the slot matching its current bearing from the VIP
        (slot i = the i-th drone counter-clockwise), so nobody has to cross
        the ring to reach its slot. The returned phase is the one that
        minimises the total angular travel to get there.
        """
        p_vip = np.asarray(p_vip, float)
        bearings = np.array([np.arctan2(p[1] - p_vip[1], p[0] - p_vip[0])
                             for p in p_defenders])
        order = np.argsort(bearings)
        self.order = order                     # drone index -> slot index
        # slot k sits at psi + 2 pi k / n; choose psi as the circular mean of
        # (bearing_of_drone_in_slot_k - 2 pi k / n)
        offs = bearings[order] - TWO_PI * np.arange(self.n) / self.n
        return float(np.arctan2(np.mean(np.sin(offs)), np.mean(np.cos(offs))))

    def resync(self, p_defenders):
        """Re-seed the guards from where the drones actually are, now.

        MUST be called after any leg the high-level commander flew (the gather
        goTo), and after a long hold. Each guard slews from its OWN last
        setpoint, so if the drones moved while something else was commanding
        them, the first streamed setpoint jumps back to wherever the guard
        last left off -- which is the pre-gather position, on the far side of
        the VIP. Measured in sim, 2026-09-23: without this the defenders were
        dragged back across the ring and the VIP clamp fired for seconds on
        end, which is exactly the separation the demo exists to respect.
        """
        for g, p in zip(self.guards, p_defenders):
            g.sp = np.asarray(p, float).copy()
            g.v = np.zeros(3)

    def slot_of(self, i):
        """Which slot drone ``i`` holds (set once by :meth:`assign`)."""
        return int(np.where(self.order == i)[0][0])

    def step(self, dt, p_vip, p_adv=None, p_defenders=None, p_threat=None):
        """One control step. Returns (setpoints (n,3), info dict).

        ``p_threat`` is the bearing the ring TURNS TO COVER, when that is not
        simply where the adversary is now. A reactive attacker aims at the
        ring's widest gap, so turning to its current position is always one
        move behind; passing its aim point makes the ring cover the gap it is
        going for instead. Engagement and every separation still use the real
        ``p_adv`` -- only the rotation target is anticipated.
        """
        cfg = self.cfg
        p_thr = p_adv if p_threat is None else np.asarray(p_threat, float)
        p_vip = np.asarray(p_vip, float)
        v_vip = self.vip_vel.update(p_vip, dt)

        was_engaged = self.engaged
        self.engaged = self.latch.update(p_vip, p_adv)
        if self.engaged:
            # WHICH defender takes the threat bearing -- re-checked every step,
            # not just at first contact.
            #
            # Picking it once meant that when the attacker flanked, the
            # original lead stayed the designated blocker and the whole ring
            # had to rotate it round -- up to 180 deg at phase_rate, which it
            # cannot do before the attacker arrives. Handing the post to the
            # slot ALREADY nearest the threat caps the rotation at half the
            # slot spacing (60 deg for three).
            #
            # This does NOT swap slots: order is fixed by assign() and never
            # changes, so nobody crosses anybody. It only changes which slot
            # the ring aims at the threat, which is a smaller rotation, not a
            # different formation. The hysteresis stops it dithering between
            # two slots at the midpoint, where a tie would otherwise flip the
            # whole ring back and forth every step.
            phi = float(np.arctan2(p_thr[1] - p_vip[1], p_thr[0] - p_vip[0]))
            here = slot_angles(self.phase.psi, self.n)
            off = np.abs(wrap_pi(here - phi))
            best = int(np.argmin(off))
            if not was_engaged or off[best] < off[self.lead] - cfg.lead_swap_hyst:
                self.lead = best
        # Bearing rate of the threat about the VIP, low-passed. This is what
        # the ring is actually chasing, and knowing its RATE is what lets the
        # ring stop trailing it.
        phi_now = (float(np.arctan2(p_thr[1] - p_vip[1], p_thr[0] - p_vip[0]))
                   if p_thr is not None else None)
        if phi_now is not None and self._prev_phi is not None:
            raw = wrap_pi(phi_now - self._prev_phi) / max(dt, 1e-3)
            self._phi_rate += (raw - self._phi_rate) * min(dt / 0.25, 1.0)
        self._prev_phi = phi_now

        if self.engaged:
            # slot 0 goes onto the VIP-adversary bearing: a defender ends up
            # between the two, which is the whole demo. AIM AHEAD of it by the
            # measured bearing rate, bounded, or the blocker arrives where the
            # attacker used to be.
            base = wrap_pi(TWO_PI * np.arange(self.n) / self.n)
            lead = float(np.clip(self._phi_rate * cfg.threat_lead_s,
                                 -cfg.lead_max_rad, cfg.lead_max_rad))
            psi_t = float(phi_now + lead - base[self.lead])
        elif cfg.hold_phase_on_clear:
            psi_t = self.phase.psi          # hold, do not wind back to rest
        else:
            psi_t = self.rest_psi
        psi = self.phase.update(
            psi_t, dt,
            cfg.engaged_phase_rate if self.engaged else cfg.phase_rate)

        # Reinforce: once a defender is on the threat bearing, the other two
        # leave their posts and close up beside it, so the adversary faces a
        # wall rather than one drone with 120 degrees of daylight either side.
        # It opens back out when the threat leaves -- an escort permanently
        # bunched on one side is not escorting anybody.
        step_close = dt / max(cfg.wall_ramp_s, 1e-3)
        self.close = float(np.clip(self.close + (step_close if self.engaged
                                                 else -step_close), 0.0, 1.0))

        slots = ring_targets(p_vip, psi, cfg, self.close, self.lead)
        # feedforward the VIP's velocity so the ring translates *with* the
        # person instead of forever chasing them by a lag-shaped error
        slots[:, :2] += v_vip[:2] / max(cfg.rate_hz, 1.0)

        # Live positions feed the defender-defender clamp only. A pose that
        # disagrees with what we commanded by more than a ring radius is not
        # a drone that drifted, it is a bad input -- a stale estimate, a
        # fallback source, a rigid body that flipped -- and acting on it makes
        # the guard shove setpoints around for no reason (observed in sim,
        # 2026-09-23: "defender" clamps on a ring whose slots are 2.6 m
        # apart). Fall back to the commanded setpoint, which is at least
        # somewhere the drone was told to be.
        self.untrusted = []
        if p_defenders is None:
            live = [g.sp for g in self.guards]
        else:
            live = []
            for i, (p, g) in enumerate(zip(p_defenders, self.guards)):
                p = np.asarray(p, float)
                if np.linalg.norm(p - g.sp) > cfg.ring_radius:
                    self.untrusted.append(i)
                    p = g.sp
                live.append(p)
        out = np.empty((self.n, 3))
        clamped = {}
        for i, g in enumerate(self.guards):
            others = [live[j] for j in range(self.n) if j != i]
            out[i] = g.step(
                slots[self.slot_of(i)], dt, p_vip, p_adv, others,
                adv_dist=(cfg.min_adv_sep if cfg.defenders_yield
                          else cfg.defender_yield_floor))
            if g.reasons:
                clamped[i] = list(g.reasons)
        return out, {'psi': psi, 'engaged': self.engaged, 'v_vip': v_vip,
                     'close': self.close, 'lead': self.lead, 'clamped': clamped,
                     'untrusted': list(self.untrusted)}


#: Phases a reactive attacker moves through, in order. Same shape as
#: AdversaryScript.labels so the operator-paced machinery is unchanged.
REACTIVE_PHASES = ('take station off to one side',
                   'ATTACK - it finds its own way in',
                   'stand down and withdraw')


@dataclass
class ReactiveAdversary:
    """An attacker that plans its own approach and gives up when blocked.

    Replaces a fixed script with a gradient: pulled toward the VIP, pushed by
    whatever defender is nearest, and held inside the room. Three consequences
    that are the point of it:

    * its path is never the same twice, because it depends on where the
      defenders actually are;
    * moving the VIP genuinely evades it, instead of towing it around at a
      fixed radius (the scripted version resolved every leg against the VIP's
      CURRENT position, which read as the attacker flying formation with it);
    * when it cannot make progress it STANDS DOWN, which is a thing the
      audience can see happen rather than a leg ending on a timer.

    What it is not: an optimiser, or a guarantee. It is a saturated gradient
    with a give-up timer. ``check`` verifies its PARAMETERS, and
    ``plan_escort`` runs it -- but a reactive attacker cannot be proven the
    way a script can, so the separations are enforced by SetpointGuard exactly
    as before and the ring is what keeps it honest.

    It flies at the DEFENDERS' altitude, not the VIP's: the ring rides above
    an airborne VIP, and an attacker left at the VIP's height would approach
    underneath the drones meant to be facing it.
    """

    height: float = 1.2             # overwritten per step with the ring height
    #: FASTER than a defender, and that is not a mistake.
    #:
    #: This was 0.45 against the defenders' 0.60 "so the ring can get ahead of
    #: it", which compared the wrong quantity. The race is ANGULAR and the
    #: defenders are on the inside track: they sit at ring_radius while the
    #: attacker sits at ring_radius + min_adv_sep, so to match the ring's
    #: angular rate the attacker must cover 1.8x the distance. The ring turns
    #: at engaged_phase_rate = 0.55 rad/s, so the attacker only wins beyond
    #: 0.55 * 1.80 = 0.99 m/s.
    #:
    #: Making it slower in LINEAR terms made it slower in angular terms too,
    #: which is why it was trivially contained and read as unaggressive. 0.80
    #: is visibly faster than the defenders, still 0.19 m/s short of
    #: out-turning them, and it also fixes a hazard the yield inversion
    #: created: an attacker slower than the ring could be RUN DOWN by a ring
    #: translating with a moving VIP.
    v_max: float = 0.80             # m/s
    #: Where it waits before committing and after standing down.
    stand_off: float = 2.30         # m from the VIP
    bearing: float = 145.0          # deg, which side it enters from
    #: Gradient weights. Only their RATIO matters -- the sum is saturated to
    #: v_max -- but the repulsion must win inside ``repel_range`` or the
    #: attacker drives through the ring and leans on the guard instead.
    #: MEASURED by sweep (plan_escort, 2026-10-06), not guessed. At
    #: k_repel 2.2 / range 1.30 the attacker walked straight through the ring
    #: to 0.01 m of the VIP -- the defenders YIELD to it, so their push fades
    #: exactly as it closes and nothing stops it. 3.5 / 1.70 turns it away at
    #: 1.76 m (inside alert_radius, so the ring does engage) while holding
    #: 0.86 m from the nearest defender against a 0.80 m floor.
    k_attract: float = 1.0
    k_repel: float = 3.5
    k_wall: float = 2.0
    #: How much of the repulsion is turned sideways rather than straight back.
    #: 0 reproduces the passive bounce-in-place this replaced.
    k_tangent: float = 0.0          # only meaningful with seek_gaps
    #: How deep it tries to get, as a radius from the VIP.
    #:
    #: It used to aim INSIDE the ring (0.8 * ring_radius) and that is
    #: unfixable by tuning: three defenders 120 deg apart leave 1.73 m gaps,
    #: so a gap-seeker always walks through. Measured over 18 gain
    #: combinations on 2026-10-06, EVERY one reached the VIP (0.01-0.06 m) and
    #: every one breached defender separation (0.67-0.77 m against 0.80).
    #: More repulsion does not help, because the hole is real.
    #:
    #: So it presses the ring instead of passing through it: it circles
    #: hunting the widest gap, which is what makes the encounter move, and
    #: the ring has to chase. The defence holding is then a property of the
    #: geometry rather than of a gain nobody can tune.
    attack_reach: float = 0.0       # m; 0 = derive it from the geometry

    def shell(self, cfg=None):
        """Closest the attacker may ever be to the VIP.

        Derived, not chosen: ``ring_radius + min_adv_sep`` puts it one full
        separation outside the ring on EVERY bearing, so it can press a gap
        without ever being adjacent to a defender. It is the same radius the
        old scripted probe used, for the same reason -- the difference is that
        the attacker now picks its own bearing on that shell instead of being
        told one.
        """
        if self.attack_reach > 0.0:
            return self.attack_reach
        # min_adv_sep, NOT ring_radius + min_adv_sep. The larger shell made
        # the attacker orbit a standoff it was never allowed to cross, so the
        # defence could not fail and therefore could not succeed either -- it
        # read as the attacker "being nice". This is a collision floor against
        # the DJI and nothing more: what keeps it off the VIP is supposed to
        # be three drones, and now it has to be.
        return 0.8 if cfg is None else cfg.min_adv_sep
    #: Range over which the attraction tapers to zero as it arrives.
    approach_band: float = 0.60     # m
    #: Strength of the VIP keep-out barrier. Must dominate k_attract or the
    #: attacker pushes through its own limit.
    k_keepout: float = 4.0
    #: How far a defender pushes. Must exceed min_adv_sep, or the attacker
    #: only reacts once it is already inside the separation floor and the
    #: guard has to rescue every approach.
    repel_range: float = 1.70       # m
    #: Blocked for this long -> stand down. Blocked means "not closing on the
    #: VIP by more than progress_eps", which is what being walled actually
    #: looks like: still moving, sliding along the ring, getting no nearer.
    give_up_s: float = 12.0
    progress_eps: float = 0.08      # m/s of closing speed that counts
    #: How fast the "closest I have managed" reference RELAXES back outward.
    #:
    #: Without this it is an all-time ratchet, and an attacker that presses,
    #: is pushed out, and presses again registers no progress at all after its
    #: single best approach -- so it stood down mid-attack. Measured in sim
    #: 2026-10-06: it oscillated 1.76 <-> 2.10 m, engaged the whole time, and
    #: gave up at t+21 s because nothing had beaten 1.76 m since t+16 s.
    #: Relaxing the reference means REPEATED pressure counts as trying, and
    #: the timer only runs when it is genuinely being driven off and kept off.
    #: MEASURED 2026-10-06 against a held ring: 0.01 -> it presses for 31 s
    #: then stands down; 0.02 -> 46 s; 0.03 and above -> it never gives up at
    #: all, which costs the demo its payoff beat (and leaves the paced leg
    #: with no completion to announce). 31 s of probing is the middle act.
    relax_rate: float = 0.01        # m/s
    duration: float = 40.0


    def __post_init__(self):
        self.p = None
        self._gap = None
        self.phase = 0
        self.stood_down = False
        self._best = np.inf
        self._since = 0.0
        self._t = None

    # -- the same surface AdversaryScript exposes, so paced mode is unchanged
    @property
    def n_legs(self):
        return len(REACTIVE_PHASES)

    @property
    def labels(self):
        return REACTIVE_PHASES

    def label(self, i):
        return REACTIVE_PHASES[max(0, min(i, len(REACTIVE_PHASES) - 1))]



    def leg_start(self, i):
        return i * self.duration / self.n_legs

    def leg_mid(self, i):
        return self.leg_start(i) + self.duration / (2 * self.n_legs)

    def station(self, p_vip):
        """Where it waits: ``stand_off`` from the VIP on its entry bearing."""
        a = np.radians(self.bearing)
        return np.array([p_vip[0] + self.stand_off * np.cos(a),
                         p_vip[1] + self.stand_off * np.sin(a), self.height])

    def set_phase(self, i, p_vip):
        """Called when the operator advances a leg. Idempotent.

        It MUST do nothing when the phase is unchanged: plan_escort calls it
        every step, and resetting the give-up timer each time would mean the
        attacker could never stand down.
        """
        i = max(0, min(int(i), self.n_legs - 1))
        if i == self.phase:
            return
        self.phase = i
        if self.phase == 1:                 # committing: restart the timer
            self.stood_down = False
            self._best = float(np.linalg.norm(
                np.asarray(self.p, float)[:2] - np.asarray(p_vip, float)[:2]))
            self._since = 0.0

    def target(self, t, p_vip, defenders=(), cfg=None):
        """Where the attacker wants to be now. Stateful: integrates itself."""
        p_vip = np.asarray(p_vip, float)
        if self.p is None:
            self.p = self.station(p_vip)
        dt = 0.05 if self._t is None else float(np.clip(t - self._t, 1e-3, 0.5))
        self._t = t
        self.p[2] = self.height

        if self.phase != 1 or self.stood_down:
            goal = self.station(p_vip)
            v = (goal[:2] - self.p[:2]) * 1.0
        else:
            v = self._attack(p_vip, defenders, cfg, dt)

        n = float(np.linalg.norm(v))
        if n > self.v_max:
            v *= self.v_max / n
        self.p[:2] = self.p[:2] + v * dt
        # HARD keep-out, applied after integration. The force version alone is
        # not enough: inside the ring each defender's push points away from
        # THAT defender, so three of them sum to a push toward the centre --
        # onto the VIP -- and it out-votes any gain (measured: closest 0.02 m
        # with k_keepout at 4.0). A projection cannot be out-voted, and it
        # turns being pressed into sliding around the bubble, which is the
        # behaviour worth watching anyway.
        # ALWAYS, not just while attacking. The shell is what guarantees
        # separation: at ring_radius + min_adv_sep the attacker is one full
        # min_adv_sep outside the ring no matter which bearing it is on, so it
        # cannot be adjacent to a defender even when it sits in a gap. Applied
        # only during the attack it still crossed the ring on the way out.
        if True:
            d = self.p[:2] - np.asarray(p_vip, float)[:2]
            dn = float(np.linalg.norm(d))
            shell = self.shell(cfg)
            sep = cfg.min_adv_sep if cfg is not None else 0.8
            # HARD collision avoidance, same shape as SetpointGuard: iterate
            # the projections and ALWAYS finish on the VIP shell. The soft
            # repulsion above is what makes it look like it is avoiding; this
            # is what makes it actually avoid. The shell only guarantees
            # separation while the defenders are ON the ring -- a defender
            # pushed off it (or a moving VIP the ring is lagging) breaks that
            # assumption, which is exactly when a hard floor has to exist.
            for _ in range(3):
                for q in defenders:
                    dq = self.p[:2] - np.asarray(q, float)[:2]
                    dqn = float(np.linalg.norm(dq))
                    if dqn < sep:
                        if dqn < 1e-6:
                            dq, dqn = np.array([1.0, 0.0]), 1.0
                        self.p[:2] = (np.asarray(q, float)[:2]
                                      + dq * (sep / dqn))
                d = self.p[:2] - np.asarray(p_vip, float)[:2]
                dn = float(np.linalg.norm(d))
                if dn < shell:
                    if dn < 1e-6:
                        d, dn = np.array([1.0, 0.0]), 1.0
                    self.p[:2] = np.asarray(p_vip, float)[:2] + d * (shell / dn)
        if cfg is not None:
            c = np.array(cfg.room_center)
            r = self.p[:2] - c
            rn = float(np.linalg.norm(r))
            if rn > cfg.arena_radius:
                self.p[:2] = c + r * (cfg.arena_radius / rn)
        return self.p.copy()

    #: Hunt the ring's widest gap instead of pressing head-on.
    #:
    #: OFF by default, and that is a measured decision, not timidity. Gap
    #: seeking makes the encounter move -- the attacker sweeps 460 deg around
    #: the VIP instead of 95, and the ring turns 216 deg instead of 85 -- but
    #: it also gets IN, every time, at every gain tried (18 combinations,
    #: 2026-10-06). That is geometry, not tuning: three defenders 120 deg
    #: apart leave 1.73 m gaps, and closing the wall into a 116 deg arc leaves
    #: 244 deg wide open. Forcing it back out with a keep-out projection then
    #: pushed it into DEFENDERS (0.29 m against a 0.80 m floor).
    #:
    #: The honest fix is the one that cannot be done two days before a demo:
    #: stop the defenders yielding, so the ring is a real obstacle and the
    #: attacker is deflected by bodies rather than by an invented barrier.
    #: Until then this stays off and the attacker presses head-on.
    seek_gaps: bool = True
    #: A committed gap counts as CLOSED once a defender is within this angle
    #: of it; only then does the attacker pick a new one. Without commitment
    #: it re-chose the widest gap every single step, and with three near-equal
    #: gaps that is a coin flip 20 times a second -- it dithered between two
    #: of them and swept 52 deg of arc in a whole encounter ("stuck in the
    #: same quadrant"). Committing makes it actually GO somewhere, and makes
    #: the ring chase it when it does.
    gap_closed_deg: float = 42.0
    #: How much it prefers a gap on the FAR side over the nearest one.
    #:
    #: With this at 0 it takes whichever opening is closest, which is always
    #: the one it is already next to -- it alternated between two gaps 80 deg
    #: apart and swept 74 deg of arc all encounter, while the room actually
    #: allows 201 deg (the +x side is unreachable: the shell there lands
    #: outside the arena). Biasing toward distant gaps makes it break off and
    #: flank, which is the move that forces the ring to travel.
    flank_bias: float = 0.9

    def aim(self, p_vip, defenders):
        """The bearing it is trying to get through: the ring's WIDEST GAP.

        Driving straight at the VIP is why the first version looked passive.
        Attraction pointed in, repulsion pointed straight back out, both along
        the same radial line, so the attacker bounced in and out of one spot
        and the ring never had to rotate -- which made the defenders look
        static too. Hunting the gap makes it travel: the ring turns to cover,
        the gap moves, it chases the gap, and the defence has to keep up.

        The race is deliberately close. At ~1.8 m from the VIP the attacker's
        0.45 m/s is 0.25 rad/s around it, against the ring's phase_rate of
        0.30 rad/s -- so the ring can just outrun it. Tension, with the
        defence winning.
        """
        # VIP-RELATIVE, like the gap branch below. This returned the
        # attacker->VIP bearing, which is 180 deg out, and BOTH callers treat
        # the result as a bearing measured from the VIP. Two consequences,
        # and every symptom chased on 2026-10-06 was one of them:
        #   * _attack aimed at a point on the FAR side of the VIP, so the
        #     attacker drove straight through it -- which is why no value of
        #     attack_reach, k_keepout or k_repel ever bounded the approach;
        #   * escort_show fed it to ctrl.step as the threat bearing, so the
        #     ring turned to face AWAY and the blocker took station at the
        #     opposite end of the circle.
        if not self.seek_gaps or len(defenders) == 0:
            return float(np.arctan2(*(self.p[:2] - p_vip[:2])[::-1]))
        a = sorted(float(np.arctan2(*(np.asarray(q, float)[:2] - p_vip[:2])[::-1]))
                   for q in defenders)
        gaps = []
        for i in range(len(a)):
            lo = a[i]
            hi = a[(i + 1) % len(a)] + (TWO_PI if i + 1 == len(a) else 0.0)
            gaps.append((hi - lo, wrap_pi(lo + (hi - lo) / 2.0)))

        def covered(ang):
            return min(abs(wrap_pi(ang - b)) for b in a) < np.radians(
                self.gap_closed_deg)

        # Stay committed until the ring actually closes the gap we chose.
        if self._gap is not None and not covered(self._gap):
            return self._gap
        open_gaps = [g for g in gaps if not covered(g[1])]
        if not open_gaps:                      # fully covered: press the widest
            open_gaps = gaps
        here = float(np.arctan2(*(self.p[:2] - p_vip[:2])[::-1]))
        self._gap = max(
            open_gaps,
            key=lambda g: g[0] + self.flank_bias * abs(wrap_pi(g[1] - here)))[1]
        return self._gap

    def _attack(self, p_vip, defenders, cfg, dt):
        # aim THROUGH the widest gap, not straight at the VIP
        ang = self.aim(p_vip, defenders)
        goal = p_vip[:2] + self.shell(cfg) * np.array([np.cos(ang), np.sin(ang)])
        to_goal = goal - self.p[:2]
        d_goal = float(np.linalg.norm(to_goal))
        # Attraction must DECAY as it arrives. A unit vector here means full
        # speed at any range, so the attacker could never settle on the goal
        # radius -- it overshot straight through the VIP and oscillated, and
        # no value of attack_reach changed that (measured 2026-10-06: closest
        # 0.01 m at every reach from 1.2 to 1.8). Linear inside approach_band,
        # saturated outside, so it still commits from far away.
        v = (self.k_attract * min(1.0, d_goal / self.approach_band)
             * to_goal / max(d_goal, 1e-6))

        to_vip = p_vip[:2] - self.p[:2]
        d_vip = float(np.linalg.norm(to_vip))

        for q in defenders:
            d = self.p[:2] - np.asarray(q, float)[:2]
            dn = float(np.linalg.norm(d))
            if dn < self.repel_range:
                # linear falloff: full push on contact, nothing at the edge
                w = self.k_repel * (1.0 - dn / self.repel_range)
                u = d / max(dn, 1e-6)
                v += w * u
                # ...plus a TANGENTIAL component, or a pure radial push just
                # bounces it back the way it came and it never gets round the
                # defender. Signed toward the gap it is aiming for, so the
                # push becomes a slide in the direction it already wants.
                tang = np.array([-u[1], u[0]])
                if float(tang @ (goal - self.p[:2])) < 0.0:
                    tang = -tang
                v += self.k_tangent * w * tang

        # KEEP OUT of the VIP's bubble. Without this, attack_reach is only a
        # goal, and nothing stops the attacker slipping inside the ring --
        # where the three defenders' pushes very nearly cancel and the
        # residual drives it onto the VIP. Measured: closest 0.01 m at every
        # reach and every gain. As a barrier it is the thing that makes the
        # defence hold, and it is also what the attacker would do anyway: it
        # wants to threaten the VIP, not collide with it.
        if d_vip < self.attack_reach:
            v += (self.k_keepout * (1.0 - d_vip / self.attack_reach)
                  * (-to_vip / max(d_vip, 1e-6)))

        if cfg is not None:                 # the room pushes back too
            c = np.array(cfg.room_center)
            r = self.p[:2] - c
            rn = float(np.linalg.norm(r))
            margin = cfg.arena_radius - rn
            if margin < 0.40:
                v += self.k_wall * (1.0 - max(margin, 0.0) / 0.40) * (-r / max(rn, 1e-6))

        # progress check: closing on the VIP, or just being herded?
        self._best = min(self._best + self.relax_rate * dt, d_vip + 1e3)
        if d_vip < self._best - self.progress_eps * dt:
            self._best = d_vip
            self._since = 0.0
        else:
            self._since += dt
            if self._since >= self.give_up_s:
                self.stood_down = True
        return v

    def check(self, cfg, p_vip):
        """Problems with the PARAMETERS. A path cannot be checked; these can."""
        bad = []
        # ANGULAR, not linear. The old check compared the two speeds directly
        # and was simply the wrong comparison -- see v_max above.
        # The radius it actually ORBITS at when the block is working -- one
        # separation outside the ring -- NOT shell(), which is the minimum
        # distance it may ever be from the VIP (0.80 m). Dividing by the
        # floor instead of the operating radius made this report 1.00 rad/s
        # for a 0.80 m/s attacker and refuse a configuration that is fine.
        shell = cfg.ring_radius + cfg.min_adv_sep
        w_adv = self.v_max / max(shell, 1e-6)
        w_ring = min(cfg.engaged_phase_rate, cfg.v_max / cfg.ring_radius)
        if w_adv >= w_ring:
            bad.append(
                f'adversary turns at {w_adv:.2f} rad/s about the VIP '
                f'({self.v_max:.2f} m/s at {shell:.2f} m) and the ring only '
                f'manages {w_ring:.2f} -- it can out-turn the defence, so the '
                f'block cannot hold. Cap it below {w_ring * shell:.2f} m/s.')
        if self.repel_range <= cfg.min_adv_sep:
            bad.append(f'repel_range {self.repel_range:.2f} m is inside '
                       f'min_adv_sep {cfg.min_adv_sep:.2f} -- the attacker '
                       'only reacts once the guard is already rescuing it')
        if self.stand_off <= cfg.release_radius:
            bad.append(f'stand_off {self.stand_off:.2f} m does not clear '
                       f'release_radius {cfg.release_radius:.2f} -- it would '
                       'start and end the run already engaged')
        st = self.station(np.asarray(p_vip, float))
        d = float(np.linalg.norm(st[:2] - np.array(cfg.room_center)))
        if d > cfg.arena_radius:
            bad.append(f'its waiting station is {d:.2f} m from room centre, '
                       f'outside the {cfg.arena_radius:.2f} m arena')
        return bad
