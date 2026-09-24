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
    ring_radius: float = 1.5       # m, horizontal distance from the VIP
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
    vip_height_offset: float = 0.0  # m, ring altitude relative to the VIP

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
    #: 2*R*sin(wall_half_angle/2) apart: 0.90 m at 35 deg (on the floor, and
    #: it flew there), 1.27 m at 50 deg. MEASURED (plan_escort, 2026-09-24):
    #: 50 deg holds 1.13 m in the closed wall against a 0.90 m budget, while
    #: still gathering the three of them into a 100 deg arc from the 360 deg
    #: they rest on -- the change reads clearly from outside.
    wall_half_angle: float = np.radians(50.0)

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
    alert_radius: float = 2.6      # m, adversary-to-VIP distance that engages
    release_radius: float = 2.9    # m, and the larger one that disengages

    # -- speed / acceleration ---------------------------------------------
    v_max: float = 0.6             # m/s, hard cap on a defender setpoint
    a_max: float = 1.0             # m/s^2

    # -- separations, all enforced AFTER the slew (see SetpointGuard) ------
    min_vip_dist: float = 1.5      # m, horizontal, defender to VIP
    min_pair_sep: float = 0.8      # m, defender to defender
    min_adv_sep: float = 0.8       # m, defender to adversary

    # -- the room ----------------------------------------------------------
    arena_radius: float = 2.5      # m, from room centre (crazyflie_shows.safety)
    ceiling: float = 2.0           # m
    floor: float = 0.3             # m
    room_center: tuple = (0.0467, -0.1037)

    #: Where the VIP stands for the static stages, and the centre the walking
    #: stages start from. NOT the room centre, deliberately: a VIP in the
    #: middle leaves the adversary only arena_radius - (ring_radius +
    #: min_adv_sep) = 0.2 m of stand-off before the arena clamp drags it in,
    #: so the "approach" starts already blocked (measured in sim,
    #: 2026-09-23 -- the demo engaged at t+0.0 s and never disengaged).
    #: Offsetting the VIP 1.0 m gives the adversary the far half of the room
    #: to run at, and still leaves ring_radius + 1.0 = 2.5 m for the ring.
    vip_offset: tuple = (-1.0, 0.0)

    # -- loop / estimation -------------------------------------------------
    rate_hz: float = 20.0          # setpoint stream rate per drone
    vip_lowpass_hz: float = 4.0    # corner of the VIP velocity filter

    # -- staleness (mocap dropout) ----------------------------------------
    vip_stale_hold_s: float = 0.4  # no VIP pose this long -> stop moving
    vip_stale_land_s: float = 2.0  # ...this long -> land, uninvited
    self_stale_s: float = 0.3      # no pose for a defender -> hold its setpoint


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

    def update(self, psi_target, dt):
        step = np.clip(wrap_pi(psi_target - self.psi),
                       -self.cfg.phase_rate * dt, self.cfg.phase_rate * dt)
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

    def step(self, target, dt, p_vip=None, p_adv=None, p_others=()):
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
                sp = _push_out(sp, p_adv, cfg.min_adv_sep, self.reasons, 'adversary')
            for q in p_others:
                sp = _push_out(sp, q, cfg.min_pair_sep, self.reasons, 'defender')
            if p_vip is not None:
                sp = _push_out(sp, p_vip, cfg.min_vip_dist, self.reasons, 'vip')

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
    legs: tuple = ((6.0, 35.0, 3.0),     # rise and sit off to one side
                   (8.0, 35.0, 2.3),     # probe 1
                   (5.0, 35.0, 2.9),     # pushed back, retreat
                   (7.0, -35.0, 2.9),    # swing round to the other side
                   (8.0, -35.0, 2.3),    # probe 2
                   (6.0, -35.0, 3.0))    # give up

    def target(self, t, p_vip):
        """Where the adversary should be at show time ``t``."""
        p_vip = np.asarray(p_vip, float)
        t0 = 0.0
        for dur, bearing, radius in self.legs:
            if t <= t0 + dur or (dur, bearing, radius) == self.legs[-1]:
                a = np.radians(bearing)
                return np.array([p_vip[0] + radius * np.cos(a),
                                 p_vip[1] + radius * np.sin(a),
                                 self.height])
            t0 += dur
        raise AssertionError('unreachable')      # pragma: no cover

    @property
    def duration(self):
        return sum(d for d, _, _ in self.legs)

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
        self.cfg = cfg
        p_defenders = [np.asarray(p, float) for p in p_defenders]
        self.n = len(p_defenders)
        psi0 = self.assign(p_defenders, p_vip) if psi0 is None else psi0
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

    def step(self, dt, p_vip, p_adv=None, p_defenders=None):
        """One control step. Returns (setpoints (n,3), info dict)."""
        cfg = self.cfg
        p_vip = np.asarray(p_vip, float)
        v_vip = self.vip_vel.update(p_vip, dt)

        was_engaged = self.engaged
        self.engaged = self.latch.update(p_vip, p_adv)
        if self.engaged and not was_engaged:
            # First contact: whoever is closest in bearing leads, and the ring
            # turns the short way to put them on the threat line.
            phi = float(np.arctan2(p_adv[1] - p_vip[1], p_adv[0] - p_vip[0]))
            here = slot_angles(self.phase.psi, self.n)
            self.lead = int(np.argmin(np.abs(wrap_pi(here - phi))))
        if self.engaged:
            # slot 0 goes onto the VIP-adversary bearing: a defender ends up
            # between the two, which is the whole demo.
            base = wrap_pi(TWO_PI * np.arange(self.n) / self.n)
            psi_t = float(np.arctan2(p_adv[1] - p_vip[1], p_adv[0] - p_vip[0])
                          - base[self.lead])
        else:
            psi_t = self.rest_psi
        psi = self.phase.update(psi_t, dt)

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
            out[i] = g.step(slots[self.slot_of(i)], dt, p_vip, p_adv, others)
            if g.reasons:
                clamped[i] = list(g.reasons)
        return out, {'psi': psi, 'engaged': self.engaged, 'v_vip': v_vip,
                     'close': self.close, 'lead': self.lead, 'clamped': clamped,
                     'untrusted': list(self.untrusted)}
