"""Constellation -- the second show. Five drones are five points; draw with them.

Built from the research dossier in ``reference/complex-shows-report.html``
("Beyond the Carousel"). ``swarm_show`` is rings: every figure is a variation
on a regular n-gon. This show is **shapes** -- a pentagon, a pyramid, an arrow,
a spiral staircase -- joined by morphs, with the vertical axis finally in use.
Like :mod:`choreography` it is **pure**: no rclpy, no radio, so ``plan_show
--show constellation`` verifies the exact object ``constellation_show`` flies.

What it takes from the report, and what it deliberately does not
-----------------------------------------------------------------
Adopted -- all verifiable on the ground, none of it spends safety margin:

* **Shape formations with makespan morphs.** A formation change is
  simultaneous, so its duration is set by the *last* drone to arrive. Every
  morph here searches every drone-to-slot assignment *and* every heading of
  the target shape, keeps only the ones whose straight-line leg is provably
  safe, and picks the shortest longest-leg (:func:`safety.assign_makespan`).
  Morphs are ``goTo`` legs, which cost **zero** trajectory pieces -- the
  31-piece budget goes entirely to the four figures that move.
* **The vertical axis, under a stricter rule than the first show's.** Every
  formation and every leg must clear :data:`safety.PLAN_SEPARATION` *in plan
  view alone*. Height is therefore purely visual: no drone ever flies over
  another, so downwash never enters into it. The first show's wave and bloom
  already worked this way; here it is enforced for the whole show.
* **A beat grid.** Every phase boundary lands on a beat of :attr:`bpm`, and
  ``constellation_show`` schedules commands on absolute time, so the show
  does not drift against a music track. ``plan_show`` prints the cue sheet.
* **Light cues** on the bottom Color LED deck every drone on this rig carries
  (the server sets it green on connect). Never green while airborne -- green
  means "connected, safe, ready" -- and back to red for the landing.

Deliberately NOT adopted, and why:

* **The relaxed ellipsoid separation budget.** The report's headline finding
  is that downwash is an ellipsoid (0.12 m across, 0.30 m vertical), so the
  isotropic 0.90 m budget is ~3.75x stricter than needed side by side. It is
  also explicit that this removes conservatism the rig has been flying
  behind, and that the tracking lag must be re-measured on hardware first.
  This show keeps 0.90 m and *reports* the ellipsoid clearance
  (:func:`safety.downwash_clearance`) so the headroom is known.
* **Mid-show re-upload** to escape 31 pieces -- radio activity mid-flight.
* **Onboard collision avoidance (BVCA)** -- it rewrites setpoints, which
  voids every guarantee ``plan_show`` makes.

The show
--------
==  ===================  ======================================================
 #  phase                what you see
==  ===================  ======================================================
 1  takeoff, settle      rise over the start marks
 2  gather               into a pentagon, radius 1.10 m
 3  **swashplate**       the ring spins through a fixed tilted plane: each drone
                         rises and dips once per turn
 4  morph -> pyramid     one drone to the centre, raised; four arms around it
 5  **turbine**          the arms revolve around the still, raised centre
 6  morph -> arrow       a swept V, apex high, wings descending
 7  dart / recoil        the whole arrow thrusts forward and draws back (rigid)
 8  morph -> staircase   a line, stepping up from 0.60 m to 1.40 m
 9  **spiral staircase** the line turns a full circle -- a staircase spinning
10  morph -> pentagon    back to the ring (chosen so the flight home is safe)
11  **starburst**        finale: flings out, alternately up and down, spinning
12  home, land
==  ===================  ======================================================

Why each moving figure is safe: all four are rigid in plan view (a rotation
about the room centre, or a rotation plus a uniform radial expansion), so
their plan-view separation is the formation's own and never shrinks. Height
only adds. The morphs are the only moments drones move relative to each
other, and those are checked exactly, in closed form, before takeoff.
"""

from dataclasses import dataclass, field
from math import atan2, ceil, cos, degrees, radians, sin

import numpy as np

from crazyflie_shows import choreography, figures, safety
from crazyflie_shows.choreography import Figure, Phase, Plan


# =========================================================================
# Lights -- bottom Color LED deck, param colorLedBot.wrgb8888 (0xWWRRGGBB)
# =========================================================================

def wrgb(w=0, r=0, g=0, b=0):
    """Pack a Color LED deck value.

    ``w`` is capped below 0x80: the value travels as a ROS integer parameter,
    and keeping the top bit clear avoids any signed-32-bit handling on the
    way to the firmware's uint32. Brightness is capped at the level the rig
    already uses for its own status colours (single channel at 0xFF).
    """
    for v in (w, r, g, b):
        if not 0 <= v <= 0xFF:
            raise ValueError(f'channel {v} outside 0..255')
    if w >= 0x80:
        raise ValueError('white channel must stay below 0x80')
    return (w << 24) | (r << 16) | (g << 8) | b


#: Show palette. Nothing green-dominant: green is the rig's "ready" colour.
LIGHT = {
    'deep_blue': wrgb(b=0xC0),
    'indigo':    wrgb(r=0x30, b=0xD0),
    'violet':    wrgb(r=0x70, b=0xE0),
    'magenta':   wrgb(r=0xD0, b=0x90),
    'amber':     wrgb(r=0xFF, g=0x50),
    'white':     wrgb(w=0x70),
    'red':       wrgb(r=0xFF),          # = crazyswarm_py LED_SCRIPT_RUNNING
}
#: Low-to-high gradient for the staircase, one colour per step.
STAIR_GRADIENT = [LIGHT['deep_blue'], LIGHT['indigo'], LIGHT['violet'],
                  LIGHT['magenta'], LIGHT['white']]


def light_name(value):
    """Palette name for a value, for the cue sheet."""
    for k, v in LIGHT.items():
        if v == value:
            return k
    return f'0x{value:08X}'


# =========================================================================
# Configuration
# =========================================================================

@dataclass
class ConstellationConfig(choreography.ShowConfig):
    """Tunables. Inherits the rig fields (room centre, arena, scale, takeoff,
    landing, floor, transition speed) from :class:`choreography.ShowConfig`."""

    #: Tempo. Every phase boundary lands on a beat; 120 BPM is 0.5 s a beat,
    #: 2 s a 4/4 bar. Changing it re-times every phase and re-runs every check.
    bpm: float = 120.0

    #: Swashplate: tilt amplitude of the plane the ring rides, metres.
    tilt: float = 0.35
    #: Pyramid: arm radius, and how far the centre drone sits above the arms.
    #: The arm radius is set by the morph INTO the pyramid, not the pyramid:
    #: one drone flies from the ring to the centre between two arms, and the
    #: plan-view gap it passes through is ~0.7 x the arm radius. Measured best
    #: achievable clearance: 1.0 m -> no safe assignment at all, 1.1 -> 0.90,
    #: 1.2 -> 0.95, 1.3 -> 0.99.
    #:
    #: The three shape sizes below were then chosen TOGETHER, by searching them
    #: against the real start positions for the widest worst-case morph (all
    #: four morphs, plan view, 2026-09-20):
    #:
    #:   arm   arrow  stair   ring>arrow  arrow>pyr  pyr>stair  pyr>ring
    #:   1.25  0.95   0.95      0.937      0.910      0.910      0.959
    #:   1.45  1.05   1.05      1.012      0.996      1.016      1.030
    #:
    #: The first row passed, 1 cm over budget on two legs. The second clears
    #: every leg by ~10 cm, which is what a first hardware flight should have.
    #: Re-run plan_show after moving the fleet: the numbers depend on it.
    pyramid_arm: float = 1.45
    pyramid_rise: float = 0.45
    #: Arrow: spacing along each arm, half-angle of the V, height step per row
    #: (apex highest), and how far the dart thrusts.
    arrow_spacing: float = 1.05
    arrow_half_angle: float = 35.0
    arrow_step: float = 0.20
    arrow_dart: float = 0.60
    #: Staircase: spacing along the line, and the bottom / top step heights.
    stair_spacing: float = 1.05
    stair_low: float = 0.60
    stair_high: float = 1.40
    #: Starburst: radial overshoot and the alternating up/down excursion.
    burst_expand: float = 0.35
    burst_rise: float = 0.35

    #: Nominal figure durations, seconds. Rounded up onto the beat grid.
    dur_swashplate: float = 7.75
    dur_turbine: float = 6.75
    dur_staircase: float = 9.75
    dur_starburst: float = 6.75

    #: Heading resolution for the morph search, degrees.
    heading_step: float = 5.0

    pieces: dict = field(default_factory=lambda: {
        'swashplate': 6, 'turbine': 6, 'staircase': 7, 'starburst': 6,
    })


# =========================================================================
# Shapes -- each returns one 3D slot per drone, in no particular order
# =========================================================================

def _rot(v, heading_deg):
    h = radians(heading_deg)
    return np.array([v[0] * cos(h) - v[1] * sin(h), v[0] * sin(h) + v[1] * cos(h)])


def _place(center, local_xy, heights, heading_deg):
    """Local 2D offsets -> world slots: centred on ``center``, rotated."""
    local_xy = np.asarray(local_xy, float)
    local_xy = local_xy - local_xy.mean(axis=0)       # centroid on the centre
    return [np.array([*(np.asarray(center[:2], float) + _rot(p, heading_deg)), z])
            for p, z in zip(local_xy, heights)]


def pyramid(center, n, arm, height, rise, heading_deg):
    """One drone raised at the centre, ``n - 1`` arms on a circle around it."""
    k = n - 1
    xy = [(0.0, 0.0)] + [(arm * cos(2 * np.pi * i / k), arm * sin(2 * np.pi * i / k))
                         for i in range(k)]
    # the apex sits exactly on the centre, so place without re-centring
    slots = [np.array([center[0], center[1], height + rise])]
    for p in xy[1:]:
        q = _rot(p, heading_deg)
        slots.append(np.array([center[0] + q[0], center[1] + q[1], height]))
    return slots


def arrow(center, n, spacing, half_angle_deg, height, step, heading_deg):
    """A swept V pointing along ``heading``: apex highest, rows descending."""
    a = radians(half_angle_deg)
    xy, z = [(0.0, 0.0)], [height + step]
    row = 1
    while len(xy) < n:
        for side in (1, -1):
            if len(xy) < n:
                xy.append((-row * spacing * cos(a), side * row * spacing * sin(a)))
                z.append(height + step - row * step)
        row += 1
    return _place(center, xy, z, heading_deg)


def staircase(center, n, spacing, low, high, heading_deg):
    """A straight line through the centre, stepping up evenly from low to high."""
    offs = [((i - (n - 1) / 2.0) * spacing, 0.0) for i in range(n)]
    zs = list(np.linspace(low, high, n))
    return _place(center, offs, zs, heading_deg)


def _polar(center, p):
    """(radius, angle_deg, z) of a slot about the centre."""
    d = np.asarray(p[:2], float) - np.asarray(center[:2], float)
    return float(np.hypot(*d)), degrees(atan2(d[1], d[0])), float(p[2])


# =========================================================================
# Plan construction
# =========================================================================

def _slot(cfg, duration):
    """Command-to-command time for a phase of ``duration``, on the beat grid."""
    beat = 60.0 / cfg.bpm
    return ceil((duration + cfg.phase_margin) / beat - 1e-9) * beat


def _on_beat(cfg, duration):
    """The motion duration that makes a phase end exactly on a beat."""
    return _slot(cfg, duration) - cfg.phase_margin


#: A morph that clears this in plan view beats any that does not, however
#: much faster the tighter one is. 5 cm over the budget: the budget already
#: carries the measured tracking lag, this is margin on top for a first flight.
COMFORT_SEP = safety.PLAN_SEPARATION + 0.05


def _morph(src, candidates, label, accept=None):
    """Best safe morph over every (heading, assignment). Raises if none is safe.

    Ranked: comfortably clear (>= COMFORT_SEP) first, then shortest longest
    leg (makespan), then widest plan-view clearance.
    """
    best, best_key = None, None
    for heading, slots in candidates:
        res = safety.assign_makespan(src, slots, safety.PLAN_SEPARATION,
                                     safety.PLAN_SEPARATION, accept,
                                     prefer_sep_xy=COMFORT_SEP)
        if res is None:
            continue
        key = (0 if res['sep_xy'] >= COMFORT_SEP else 1,
               round(res['longest'], 2), -round(res['sep_xy'], 3))
        if best_key is None or key < best_key:
            best_key, best = key, dict(res, heading=heading)
    if best is None:
        raise ValueError(
            f'{label}: no assignment and heading keeps every pair >= '
            f'{safety.PLAN_SEPARATION:.2f} m in plan view during the move')
    return best


def build_plan(names, initial_positions, cfg=None):
    """Build and internally verify the constellation show. Pure.

    Raises ``ValueError`` on any budget violation -- the same contract as
    :func:`choreography.build_plan`, plus the plan-view rule described in the
    module docstring.
    """
    cfg = cfg or ConstellationConfig()
    choreography.PHASE_MARGIN_REF[0] = cfg.phase_margin

    starts = [np.asarray(p, float) for p in initial_positions]
    n = len(starts)
    if not 4 <= n <= 8:
        raise ValueError(f'the constellation show needs 4-8 drones, got {n}')

    center = (np.asarray(cfg.room_center, float) if cfg.room_center is not None
              else np.mean([p[:2] for p in starts], axis=0))
    H = cfg.form_height
    R = cfg.scaled(cfg.ring_radius)
    step = cfg.heading_step

    phases, pos = [], [p.copy() for p in starts]

    def add(name, kind, duration, ends, lights=None, **kw):
        if kind != 'figure':
            duration = _on_beat(cfg, duration)      # stretch legs onto the grid
        slot = _slot(cfg, duration)
        phases.append(Phase(name, kind, duration, [p.copy() for p in pos],
                            [np.asarray(e, float) for e in ends],
                            lights=lights, slot=slot, **kw))
        for j in range(n):
            pos[j] = np.asarray(ends[j], float)

    def leg_time(src, dst):
        longest = max(float(np.linalg.norm(np.asarray(d) - np.asarray(s)))
                      for s, d in zip(src, dst))
        return max(cfg.trans_min_duration, longest / cfg.trans_avg_speed)

    # ------------------------------------------------ takeoff and gather
    hover = [np.array([s[0], s[1], cfg.takeoff_height]) for s in starts]
    add('takeoff', 'takeoff', cfg.takeoff_duration, hover, goals=hover,
        lights=LIGHT['deep_blue'],
        note=f'all drones to {cfg.takeoff_height:.2f} m over their own start')
    add('settle', 'goto', cfg.settle_duration, hover, goals=hover,
        note='hold while the Kalman filter settles')

    slots, angles, perm, gather_sep = safety.best_phase_ngon(
        [p[:2] for p in starts], center, R, H, figures.ngon_slots)
    if gather_sep < safety.PLAN_SEPARATION:
        raise ValueError(f'gather: plan-view separation {gather_sep:.2f} m < '
                         f'{safety.PLAN_SEPARATION:.2f} m from these start positions')
    ring_ang = [angles[perm[j]] for j in range(n)]
    ring = [np.asarray(slots[perm[j]], float) for j in range(n)]
    add('gather', 'goto', leg_time(hover, ring), ring, goals=ring,
        lights=LIGHT['indigo'], note=f'form the {n}-gon, min sep {gather_sep:.2f} m')

    fig_specs = []   # (name, per-drone path fns, duration), in upload order

    def to_downbeat():
        """Stretch the previous leg so the next phase starts on beat 1 of a bar.

        The figures are the moments an audience reads as the music, so each
        one starts on a downbeat. Only ever stretches a goTo leg -- a slower
        straight line over the same endpoints follows the same path, so its
        clearance is unchanged (the sampled check below confirms it).
        """
        bar = 4 * 60.0 / cfg.bpm
        t = sum(ph.slot for ph in phases)
        pad = (-t) % bar
        if pad > 1e-6 and bar - pad > 1e-6:
            prev = phases[-1]
            if prev.kind == 'figure':
                raise ValueError(f'cannot bar-align after figure {prev.name!r}')
            prev.duration += pad
            prev.slot += pad

    def figure_phase(name, paths_fn, nominal, note, lights):
        to_downbeat()
        dur = _on_beat(cfg, nominal)
        paths = [paths_fn(j, dur) for j in range(n)]
        fig_specs.append((name, paths, dur))
        add(name, 'figure', dur, [p.copy() for p in pos], figure=name,
            note=note, lights=lights)

    # -------------------------------------------------------- swashplate
    tilt_heading = ring_ang[0]      # any fixed heading; this one is reproducible
    figure_phase('swashplate',
                 lambda j, d: figures.swashplate_path(
                     center, R, ring_ang[j], 360.0, H, d,
                     tilt=cfg.tilt, tilt_heading_deg=tilt_heading),
                 cfg.dur_swashplate,
                 'the ring spins through a tilted plane', LIGHT['violet'])

    # ------------------------------------------------ morph -> arrow
    # The order is not free. Measured against the real start positions, no
    # one-way route through ring, arrow, pyramid and staircase keeps plan-view
    # clearance on every leg: the staircase can only be entered from the
    # pyramid, and cannot be left for anything but the pyramid. So the show is
    # an arch -- out through arrow, pyramid, staircase, and back the way it
    # came. Running a straight-line leg backwards retraces the same paths, so
    # its closest approach is identical; the return is safe by construction.
    sp = cfg.scaled(cfg.arrow_spacing)
    m = _morph(pos, ((h, arrow(center, n, sp, cfg.arrow_half_angle, H,
                                cfg.arrow_step, h))
                     for h in np.arange(0.0, 360.0, step)),
               'morph to arrow')
    arr, heading = m['targets'], m['heading']
    add('morph -> arrow', 'goto', leg_time(pos, arr), arr, goals=arr,
        lights=LIGHT['amber'],
        note=f'heading {heading:.0f} deg, plan-view sep {m["sep_xy"]:.2f} m')

    fwd = np.array([*_rot((cfg.scaled(cfg.arrow_dart), 0.0), heading), 0.0])
    dart = [p + fwd for p in arr]
    add('dart', 'goto', cfg.trans_min_duration, dart, goals=dart,
        note='the whole arrow thrusts forward - a rigid translation')
    add('recoil', 'goto', cfg.trans_min_duration, arr, goals=arr,
        note='and draws back')

    # ------------------------------------------------ morph -> pyramid
    arm = cfg.scaled(cfg.pyramid_arm)
    m = _morph(pos, ((h, pyramid(center, n, arm, H, cfg.pyramid_rise, h))
                     for h in np.arange(0.0, 360.0 / (n - 1), step)),
               'morph to pyramid')
    pyr = m['targets']
    to_pyr = leg_time(pos, pyr)
    add('morph -> pyramid', 'goto', to_pyr, pyr, goals=pyr, lights=LIGHT['violet'],
        note=f'plan-view sep {m["sep_xy"]:.2f} m, makespan {m["longest"]:.2f} m')

    pyr_polar = [_polar(center, p) for p in pyr]
    apex = int(np.argmax([p[2] for p in pyr]))
    turbine_lights = [LIGHT['white'] if j == apex else LIGHT['magenta']
                      for j in range(n)]
    figure_phase('turbine',
                 lambda j, d: figures.carousel_path(
                     center, pyr_polar[j][0], pyr_polar[j][1], 360.0,
                     pyr_polar[j][2], d),
                 cfg.dur_turbine,
                 'the arms revolve round the raised, still centre', turbine_lights)

    # ----------------------------------------------- morph -> staircase
    ss = cfg.scaled(cfg.stair_spacing)
    m = _morph(pos, ((h, staircase(center, n, ss, cfg.stair_low, cfg.stair_high, h))
                     for h in np.arange(0.0, 360.0, step)),
               'morph to staircase')
    stair = m['targets']
    to_stair = leg_time(pos, stair)
    add('morph -> staircase', 'goto', to_stair, stair, goals=stair,
        note=f'plan-view sep {m["sep_xy"]:.2f} m, makespan {m["longest"]:.2f} m')

    stair_polar = [_polar(center, p) for p in stair]
    rank = np.argsort(np.argsort([p[2] for p in stair]))   # 0 = lowest step
    grad = [STAIR_GRADIENT[int(round(r * (len(STAIR_GRADIENT) - 1) / (n - 1)))]
            for r in rank]
    figure_phase('staircase',
                 lambda j, d: figures.carousel_path(
                     center, stair_polar[j][0], stair_polar[j][1], 360.0,
                     stair_polar[j][2], d),
                 cfg.dur_staircase,
                 'the staircase turns a full circle', grad)

    # --------------------------------------------------- the way back
    add('retrace -> pyramid', 'goto', to_stair, pyr, goals=pyr,
        lights=turbine_lights,
        note='the staircase leg run backwards - same paths, same clearance')
    # The same uploaded turbine, flown in reverse: the arms turn the other way.
    # A reversed figure starts at eval(T), which is eval(0) because it sweeps
    # 360 deg -- the composition rule is what makes this free.
    turb_dur = [ph.duration for ph in phases if ph.name == 'turbine'][0]
    to_downbeat()
    add('turbine (reversed)', 'figure', turb_dur, [p.copy() for p in pos],
        figure='turbine', reverse=True, lights=turbine_lights,
        note='the same figure backwards - costs no trajectory memory')

    # ------------------------------------------------ morph -> pentagon
    # The flight home follows the finale with each drone's goal fixed (its own
    # start mark), so there is no assignment freedom left there. Spend this
    # morph's freedom on it: only accept pentagons from which the home leg is
    # safe as well.
    home = [np.array([s[0], s[1], cfg.home_height]) for s in starts]

    def home_clears(limit):
        return lambda targets: (
            safety.pairs_transit_min(targets, home, 3) >= limit
            and safety.pairs_transit_min(targets, home, 2) >= limit)

    def pentagons():
        return ((h, figures.ngon_slots(center, R, n, H, phase_deg=h)[0])
                for h in np.arange(0.0, 360.0 / n, step / 2.0))

    # Comfortable on the way home too if any pentagon allows it; otherwise
    # settle for the budget. (Measured: the budget-only filter picked a
    # pentagon whose flight home cleared 0.91 m -- the tightest leg of the show.)
    try:
        m = _morph(pos, pentagons(), 'morph back to the pentagon',
                   accept=home_clears(COMFORT_SEP))
    except ValueError:
        m = _morph(pos, pentagons(), 'morph back to the pentagon',
                   accept=home_clears(safety.PLAN_SEPARATION))
    ring2 = m['targets']
    add('morph -> pentagon', 'goto', leg_time(pos, ring2), ring2, goals=ring2,
        lights=LIGHT['indigo'],
        note=f'chosen so the flight home is safe too; plan-view sep {m["sep_xy"]:.2f} m')

    ring2_polar = [_polar(center, p) for p in ring2]
    order = np.argsort([p[1] for p in ring2_polar])     # round the ring
    sign = np.empty(n)
    sign[order] = [1.0 if k % 2 == 0 else -1.0 for k in range(n)]
    figure_phase('starburst',
                 lambda j, d: figures.bloom_path(
                     center, R, ring2_polar[j][1], 360.0, H, d,
                     expand=cfg.scaled(cfg.burst_expand),
                     rise=sign[j] * cfg.burst_rise),
                 cfg.dur_starburst,
                 'finale: fling out, alternately up and down, spinning',
                 LIGHT['white'])

    # --------------------------------------------------- home and land
    add('return home', 'goto', leg_time(pos, home), home, goals=home,
        lights=LIGHT['deep_blue'], note="back over each drone's own start mark")
    ground = [np.array([s[0], s[1], cfg.land_height]) for s in starts]
    add('land', 'land', cfg.land_duration, ground, goals=ground,
        lights=LIGHT['red'],
        note='red again: the rig convention for "a script has the drones"')

    # ------------------------------------------------------ fit figures
    offsets, total_pieces = figures.pack_offsets(
        [cfg.pieces[name] for name, _, _ in fig_specs])
    figs = {}
    for (name, paths, dur), off, tid in zip(fig_specs, offsets,
                                            range(1, len(fig_specs) + 1)):
        trajs = [figures.fit_trajectory(p, dur, n_pieces=cfg.pieces[name])
                 for p in paths]
        figs[name] = Figure(name, tid, off, trajs, paths, dur)
    for ph in phases:
        if ph.kind == 'figure':
            ph.figure = figs[ph.figure]

    plan = Plan(list(names), starts, np.asarray(center, float), ring_ang,
                list(figs.values()), phases, gather_sep)

    # ------------------------------------------------------------ verify
    bounds, t = [], 0.0
    for ph in phases:
        bounds.append((ph.name, t, t + ph.duration))
        t += ph.duration
    plan.report = safety.check_show(
        plan.sample, plan.duration, n,
        radius=cfg.arena_radius, ceiling=cfg.ceiling, center=center,
        floor=cfg.floor, floor_window=(bounds[0][2], bounds[-1][1]),
        phase_bounds=bounds)

    samples = plan.report['samples']
    sep_xy, k, a, b = safety.sample_min_sep(samples[:, :, :2])
    if sep_xy < safety.PLAN_SEPARATION:
        raise ValueError(
            f'show check failed:\n  - plan-view separation {sep_xy:.2f} m < '
            f'{safety.PLAN_SEPARATION:.2f} m - drones {a} and {b} at '
            f'{plan.report["times"][k]:.1f}s: a drone would pass over another')
    dw, dk, da, db = safety.downwash_clearance(samples)

    plan.report.update(
        pieces=total_pieces, config=cfg,
        min_sep_xy=sep_xy, min_sep_xy_pair=(a, b),
        min_sep_xy_time=float(plan.report['times'][k]),
        downwash=dw, downwash_pair=(da, db),
        downwash_time=float(plan.report['times'][dk]),
        wall=sum(ph.slot for ph in phases), beat=60.0 / cfg.bpm)
    return plan


# =========================================================================
# Report extras (the core report is plan_show.report)
# =========================================================================

def report_extras(plan, cfg, names):
    """Plan-view and downwash numbers, and the beat-grid cue sheet."""
    r = plan.report
    out = print
    a, b = r['min_sep_xy_pair']
    da, db = r['downwash_pair']
    out('  CONSTELLATION')
    out(f'    plan-view sep  {r["min_sep_xy"]:5.2f} m  min {safety.PLAN_SEPARATION:.2f} m'
        f'   ({names[a]}/{names[b]} at t={r["min_sep_xy_time"]:.1f}s)')
    out('                   enforced: height is decorative, no drone ever flies over another')
    out(f'    downwash       {r["downwash"]:5.2f} x  the measured ellipsoid '
        f'({safety.DOWNWASH_RXY:.2f} m across, {safety.DOWNWASH_RZ:.2f} m vertical,')
    out(f'                   each + {safety.TRACKING_MARGIN:.2f} m lag)  '
        f'({names[da]}/{names[db]} at t={r["downwash_time"]:.1f}s)')
    out('                   reported, not enforced: the headroom a denser show could')
    out('                   spend once tracking lag has been re-measured on hardware')
    out('')
    beat = r['beat']
    out(f'  CUE SHEET  {cfg.bpm:g} BPM, 4/4 - start the track on "SHOW START";'
        ' bar 1 beat 1 is takeoff')
    out(f'    {"bar.beat":>8}  {"t":>6}  {"phase":<20}  light')
    t = 0.0
    for ph in plan.phases:
        k = int(round(t / beat))
        if ph.lights is None:
            light = '-'
        elif isinstance(ph.lights, (list, tuple)):
            uniq = []
            for v in ph.lights:
                if light_name(v) not in uniq:
                    uniq.append(light_name(v))
            light = ' / '.join(uniq) + ' (per drone)'
        else:
            light = light_name(ph.lights)
        out(f'    {k // 4 + 1:>5}.{k % 4 + 1:<2}  {t:>5.1f}s  {ph.name:<20}  {light}')
        t += ph.slot
    k = int(round(t / beat))
    out(f'    {k // 4 + 1:>5}.{k % 4 + 1:<2}  {t:>5.1f}s  (landed)')
    out('')
