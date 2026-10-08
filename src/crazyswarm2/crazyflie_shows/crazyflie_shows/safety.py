"""Offline collision and envelope checks for choreography.

Nothing here talks to a drone. The point is to prove a figure is safe on the
ground, before anything is armed -- the firmware has no collision avoidance
enabled on this rig, so a crossing pair of goTo paths is a real collision.

The core routine is :func:`transition_min_sep`. During a simultaneous goTo,
every drone moves on a straight line over the same duration, so each pair's
relative position is linear in normalised time and its squared distance is a
quadratic -- the minimum over the leg is closed-form, not sampled.
"""

import os
from itertools import permutations

import numpy as np
import yaml


#: Where the ROOM comes from. Added 2026-10-08.
#:
#: These numbers used to be literals in this file, which made every show
#: carry ONE room's geofence. That is fine for this rig and wrong for anybody
#: who copies the package: an unmeasured arena silently inherits our centre,
#: our radius and our ceiling, and the first thing a reader learns about their
#: own room is where their drone hit the wall. The show code is now
#: setup-agnostic -- the room is config, the choreography stays in the shows.
#:
#: Search order, first hit wins:
#:   1. $CRAZYSWARM_ARENA                   -- explicit, for a second room
#:   2. <crazyflie share>/config/arena.yaml -- the installed rig config
#:   3. ../../crazyflie/config/arena.yaml   -- the source tree, uninstalled
#:
#: A MISSING file is a hard error, deliberately. The tempting alternative --
#: fall back to the values that used to be hard-coded here -- reintroduces
#: exactly the hazard this change removes, and does it silently.
ARENA_FILE_ENV = 'CRAZYSWARM_ARENA'


def _arena_path():
    """The arena.yaml this install should read, or raise saying why not."""
    explicit = os.environ.get(ARENA_FILE_ENV)
    if explicit:
        if not os.path.exists(explicit):
            raise RuntimeError(
                f'{ARENA_FILE_ENV}={explicit!r} does not exist.')
        return explicit
    tried = []
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('crazyflie'),
                         'config', 'arena.yaml')
        tried.append(p)
        if os.path.exists(p):
            return p
    except Exception:                                   # noqa: BLE001
        pass                                            # not built/sourced
    here = os.path.dirname(os.path.abspath(__file__))
    p = os.path.abspath(os.path.join(
        here, '..', '..', 'crazyflie', 'config', 'arena.yaml'))
    tried.append(p)
    if os.path.exists(p):
        return p
    raise RuntimeError(
        'arena.yaml not found -- the show code cannot know how big the room '
        'is.\n  looked in: ' + '\n             '.join(tried)
        + f'\n  Set ${ARENA_FILE_ENV}, or copy '
        'src/crazyswarm2/crazyflie/config/arena.yaml and MEASURE YOUR OWN '
        'ROOM before setting measured: true.')


def _load_arena():
    path = _arena_path()
    with open(path) as fh:
        d = yaml.safe_load(fh) or {}
    try:
        a, sep = d['arena'], d['separation']
        out = {
            'path': path,
            'measured': bool(a['measured']),
            'centre': tuple(float(v) for v in a['centre']),
            'radius_tested': float(a['radius_tested']),
            'radius_lost': float(a['radius_lost']),
            'ceiling_tested': float(a['ceiling_tested']),
            'ceiling_centre_tested': float(a['ceiling_centre_tested']),
            'ceiling_pinch_radius': float(a['ceiling_pinch_radius']),
            'min_separation': float(sep['minimum']),
            'tracking_margin': float(sep['tracking_margin']),
        }
    except (KeyError, TypeError, ValueError) as e:
        raise RuntimeError(f'{path} is not a valid arena file: {e}') from e
    if len(out['centre']) != 2:
        raise RuntimeError(f'{path}: arena.centre must be [x, y]')
    return out


_ARENA = _load_arena()

#: False means THIS ROOM HAS NOT BEEN MEASURED. The planners refuse to clear a
#: flight while it is false -- see :func:`require_measured_arena`. It is not a
#: warning, because a geofence from somebody else's room is not a smaller
#: version of the right answer, it is an unrelated one.
ARENA_MEASURED = _ARENA['measured']
ARENA_FILE = _ARENA['path']


def require_measured_arena(what='this check'):
    """Raise unless the arena numbers were measured in the room being flown."""
    if not ARENA_MEASURED:
        raise SystemExit(
            f'\n  REFUSED: {what} needs a MEASURED arena.\n'
            f'  {ARENA_FILE} has `measured: false`, so the radius, ceiling and\n'
            '  centre in it are placeholders, not this room.\n'
            '  Re-measure (scripts/measure_arena.py for the centroid,\n'
            '  scripts/arena_flight_sweep.py for radius and ceiling), write the\n'
            '  results into that file, then set measured: true.\n')

# Rule of thumb for this rig: 1 m start spacing, and the formation figures in
# multi_trajectory_formation.py were accepted at a 0.84 m worst case.
MIN_SEPARATION = _ARENA['min_separation']   # m, the hard floor - two drones
#                      closer than this is a collision risk, whatever the plan
#                      says. Value lives in arena.yaml (separation.minimum).

#: How much closer the drones actually get than the plan says they will.
#:
#: Measured, `backend:=sim`, 2026-09-07, full show, positions from /tf:
#:
#:     planned minimum separation   0.85 m   (counterflow, by construction)
#:     flown   minimum separation   0.793 m  (counterflow, t+41.4 s)
#:     planned peak speed           1.87 m/s
#:     flown   peak speed           1.95 m/s
#:
#: The gap is controller lag, and it does not cancel out where it matters. Two
#: drones holding a rigid formation lag together, so their separation is
#: unaffected; the counterflow's two groups are moving in *opposite*
#: directions at different radii, so their lags subtract and eat the static
#: radial clearance directly. That is why the one figure designed to have
#: drones fly through each other is also the one where the plan is optimistic.
#:
#: 0.10 m is the measured 0.057 m rounded up, and it is deliberately applied to
#: the whole show rather than to the counterflow alone -- nothing guarantees
#: some future figure will not have the same asymmetry.
#:
#: Confirmed by re-flying the show with the lanes widened to clear the new
#: budget: planned 0.91 m, flown **0.861 m** -- a 0.049 m loss, and now 0.061 m
#: clear of MIN_SEPARATION instead of 0.007 m under it.
TRACKING_MARGIN = _ARENA['tracking_margin']  # m, arena.yaml separation.tracking_margin

# ---------------------------------------------------------------------------
# The volume Motive can actually see -- MEASURED, not inherited
# ---------------------------------------------------------------------------
#: Every show inherited arena_radius 2.5 m and ceiling 2.0 m from the
#: follow-drone prototype. Neither had ever been checked against this room.
#: They are now, by three methods on 2026-10-01, and the room is SMALLER than
#: the inherited numbers in radius and a different shape than assumed.
#:
#: 1. Carried surveys (scripts/measure_arena.py, cf1 walked by hand, 2 runs,
#:    15k samples). Tracking is NOT isotropic -- by 45 deg sector the first
#:    sustained dropout sits at 2.15 m (270-315), 2.25 m (0-45), 2.31 m
#:    (225-270), and never in 180-225, which held past 2.46 m.
#: 2. Flown radius (scripts/arena_flight_sweep.py --mode spiral, cf1 at
#:    1.20 m): lost tracking at 2.24 m.
#: 3. Flown climb (--mode climb, cf5 near centre): clean to the 2.42 m cap,
#:    pose age never above 0.02 s. The ceiling is ABOVE that and unmeasured.
#:
#: The volume is a truncated cone: wide through the flight band, pinching in
#: near the ceiling. A carried body was lost at z 2.33 m but only out at
#: r 1.97 m, while a drone at r 0.55 m flew to 2.42 m untroubled -- so a
#: single "ceiling" number is meaningless without saying at what radius.
#:
#: VERIFIED CLEAN (scripts/arena_flight_sweep.py --mode spiral --levels
#: 1.20,1.95 --r-max 2.00, cf3, 72 waypoints, 3477 samples): zero stale poses
#: and zero follow failures anywhere inside it, worst pose age 0.02 s in every
#: radius band. That run also covers the high-and-far corner (1.95 m altitude
#: at 1.96 m radius) that neither carried survey reached.
#:
#: USE THESE. A show that plans outside ARENA_RADIUS_TESTED is planning
#: somewhere nothing has flown; re-measure before raising it rather than
#: assuming the old 2.5 m.
#: RE-EXPRESSED 2026-10-08 after Motive was recalibrated and the rigid bodies
#: renamed. The room, the cameras and the surveyed volume did not change --
#: only the definition of the world frame did, so the radii and ceilings below
#: are untouched (they are scalar distances about this point, and a rotation
#: does not move them). This centroid is a PHYSICAL place, so it had to be
#: carried into the new frame.
#:
#: The transform was fitted on cf1, cf3 and cf5, whose marks were synced on
#: 2026-10-07 and which had not been touched: their pairwise distances agreed
#: with the yaml to <= 21 mm, which is what proves the drones stayed put and
#: the frame moved rather than the other way round.
#:
#:     old -> new:  rotate +91.59 deg, translate [+0.043, -0.098] m
#:     residual max 0.0159 m, scale 0.9947
#:     (0.033, 0.255) -> (-0.213, -0.072), i.e. 0.410 m away
#:
#: CORROBORATED INDEPENDENTLY, which is the only reason this is a one-line
#: edit and not a re-survey: the escort marks stand the defenders at
#: 1.40 / 0.40 / 1.40 m from this centroid. Measured from the NEW value the
#: parked drones read 1.44 / 0.38 / 1.41 m (max error 36 mm); from the OLD
#: value they read 1.03 / 0.72 / 1.37 m, which matches nothing. The drones
#: were still on their marks; the centre had moved out from under them.
#:
#: A mapped survey is NOT a survey. This is correct while the room and the
#: cameras are unchanged; re-run scripts/arena_flight_sweep.py to re-earn the
#: radii themselves, and do that before trusting the edges again.
#: All five now come from arena.yaml. The comments above are the PROVENANCE
#: of the values this rig measured; the values themselves live in the config so
#: another room can hold different ones without editing code. Names unchanged,
#: so every consumer (escort.py, plan_show.py, plan_escort.py, constellation.py,
#: demo_show.py, choreography.py) is untouched by the move.
ARENA_CENTRE = _ARENA['centre']                  # m, centroid of the TRACKED
#                                                  volume, not the room centre
ARENA_RADIUS_TESTED = _ARENA['radius_tested']    # m, flown clean
ARENA_RADIUS_LOST = _ARENA['radius_lost']        # m, where tracking was lost
CEILING_TESTED = _ARENA['ceiling_tested']        # m, clean at full radius
CEILING_CENTRE_TESTED = _ARENA['ceiling_centre_tested']   # m, near the centre
#: Above this the cone pinches: a carried body held only to ~2.09 m radius in
#: the 2.0-2.5 m band, against 3.1 m lower down.
CEILING_PINCH = _ARENA['ceiling_pinch_radius']   # m, arena.yaml


#: What a *plan* must clear, so that the flown show still clears
#: MIN_SEPARATION. This is the number check_show enforces.
PLAN_SEPARATION = MIN_SEPARATION + TRACKING_MARGIN

# The inherited ARENA_RADIUS = 2.5 / CEILING = 2.0 used to live here, said to
# be "kept only so an explicit caller can still ask for them". They were in
# fact referenced by exactly one thing -- check_envelope's own default
# arguments -- so the comment was true of the intent and false of the code,
# and the one show that took those defaults (demo_show) was checked against a
# radius 0.6 m beyond anywhere a drone has ever been tracked. Deleted
# 2026-10-06; use ARENA_RADIUS_PLAN and ceiling_at().

#: What a *plan* must stay inside, for the same reason PLAN_SEPARATION exists:
#: the drones fly the plan with up to TRACKING_MARGIN of error, and
#: ARENA_RADIUS_TESTED is where tracking was observed to *hold*, not where it
#: is comfortable. Planning to the tested edge means flying past it.
ARENA_RADIUS_PLAN = ARENA_RADIUS_TESTED - TRACKING_MARGIN      # 1.90 m


def ceiling_at(r):
    """The height that has been flown clean at plan-view radius ``r``.

    The volume is a truncated cone, not a box (see ARENA_RADIUS_TESTED): a
    drone near the centre flew to 2.42 m, while out at full radius only 1.95 m
    has been demonstrated. Returning one number for the whole room would
    either forbid the centre climb or bless an untested corner.
    """
    r = np.asarray(r, float)
    high = np.where(r <= 0.60, CEILING_CENTRE_TESTED, CEILING_TESTED)
    return high - TRACKING_MARGIN


def transition_min_sep(src, dst):
    """Exact minimum pairwise separation over a simultaneous straight-line move.

    ``src`` and ``dst`` are equal-length sequences of positions (2D or 3D,
    both the same). Returns the smallest distance any pair reaches at any
    point during the leg -- including at the endpoints.
    """
    worst = np.inf
    for a in range(len(src)):
        for b in range(a + 1, len(src)):
            d0 = np.asarray(src[a], float) - np.asarray(src[b], float)
            d1 = np.asarray(dst[a], float) - np.asarray(dst[b], float)
            dv = d1 - d0
            den = float(np.dot(dv, dv))
            # minimise |d0 + s*dv|^2 over s in [0, 1]
            s = 0.0 if den < 1e-12 else min(1.0, max(
                0.0, -float(np.dot(d0, dv)) / den))
            worst = min(worst, float(np.linalg.norm(d0 + s * dv)))
    return worst


def assign_min_distance(src, dst):
    """Match drones to slots so total travel is smallest.

    Returns a permutation ``p`` with ``dst[p[j]]`` the goal for drone ``j``.
    Brute force over ``n!`` permutations: fine to n=8 (40320), do not use it
    for a large swarm without swapping in scipy's ``linear_sum_assignment``.
    """
    n = len(src)
    if n > 8:
        raise ValueError(
            f'{n} drones: brute-force assignment is too slow, '
            'use scipy.optimize.linear_sum_assignment instead')
    return min(permutations(range(n)),
               key=lambda p: sum(
                   float(np.linalg.norm(np.asarray(src[j], float)[:2]
                                        - np.asarray(dst[p[j]], float)[:2]))
                   for j in range(n)))


def pairs_transit_min(src, dst, dims=3):
    """Vectorised :func:`transition_min_sep`: same closed form, all pairs at once.

    ``dims=2`` measures in plan view only (x, y). Used inside the assignment
    searches below, which evaluate thousands of candidate legs.
    """
    s = np.asarray(src, float)[:, :dims]
    d = np.asarray(dst, float)[:, :dims]
    iu = np.triu_indices(len(s), 1)
    d0 = (s[:, None, :] - s[None, :, :])[iu]
    dv = (d[:, None, :] - d[None, :, :])[iu] - d0
    den = np.einsum('ij,ij->i', dv, dv)
    num = -np.einsum('ij,ij->i', d0, dv)
    ok = den > 1e-12
    u = np.where(ok, np.clip(num / np.where(ok, den, 1.0), 0.0, 1.0), 0.0)
    return float(np.min(np.linalg.norm(d0 + u[:, None] * dv, axis=1)))


def assign_makespan(src, dst, min_sep=None, min_sep_xy=None, accept=None,
                    prefer_sep_xy=None):
    """Match drones to slots so the LONGEST leg is shortest -- and the leg is safe.

    A formation change is simultaneous, so what sets its duration is when the
    *last* drone arrives: the bottleneck (makespan) assignment, not the
    minimum-total-distance one :func:`assign_min_distance` solves. Hoenig et
    al. make exactly this distinction for swarm formation changes.

    Only permutations whose simultaneous straight-line leg keeps every pair
    ``>= min_sep`` (3D) and ``>= min_sep_xy`` (plan view) are considered, plus
    anything ``accept(targets)`` rejects -- use it to check the leg *after*
    this one. Among the survivors: shortest longest-leg, then the widest
    plan-view clearance, then the least total travel.

    ``prefer_sep_xy`` puts safety ahead of speed: any leg that clears it beats
    every leg that does not, however much faster. Pure makespan will happily
    pick a leg 1 cm over budget to save 5 cm of travel; for a first flight on
    hardware that is the wrong trade.

    Returns ``dict(perm, targets, longest, sep, sep_xy)`` or ``None`` if no
    permutation is safe. Brute force over ``n!``: fine to n=8.
    """
    min_sep = PLAN_SEPARATION if min_sep is None else min_sep
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    n = len(src)
    if n > 8:
        raise ValueError(f'{n} drones: brute-force assignment is too slow')
    cost = np.linalg.norm(src[:, None, :] - dst[None, :, :], axis=2)
    rows = np.arange(n)
    best, best_key = None, None
    for p in permutations(range(n)):
        p = list(p)
        legs = cost[rows, p]
        key0 = round(float(legs.max()), 2)
        if best_key is not None and best_key[0] == 0 and key0 > best_key[1]:
            continue
        tgt = dst[p]
        sep = pairs_transit_min(src, tgt, 3)
        if sep < min_sep:
            continue
        sep_xy = pairs_transit_min(src, tgt, 2)
        if min_sep_xy is not None and sep_xy < min_sep_xy:
            continue
        if accept is not None and not accept(tgt):
            continue
        comfy = 0 if prefer_sep_xy is None or sep_xy >= prefer_sep_xy else 1
        key = (comfy, key0, -round(sep_xy, 3), float(legs.sum()))
        if best_key is None or key < best_key:
            best_key = key
            best = dict(perm=p, targets=[t.copy() for t in tgt],
                        longest=float(legs.max()), sep=sep, sep_xy=sep_xy)
    return best


def best_phase_ngon(starts, center, radius, height, ngon_fn, step_deg=1.0):
    """Pick the n-gon rotation that makes the gather leg safest.

    The n-gon's phase is free. With a drone starting near the swarm centre a
    badly chosen phase can squeeze the simultaneous gather below the
    separation budget for *every* assignment, so sweep the phase over one slot
    period and keep the one whose best assignment maximises the worst-case
    pairwise separation along the paths.

    Depends only on initial positions, so it runs before takeoff. Returns
    ``(slots, angles, perm, min_sep)``.
    """
    n = len(starts)
    best = (None, None, None, -1.0)
    for phase in np.arange(0.0, 360.0 / n, step_deg):
        slots, angles = ngon_fn(center, radius, n, height, phase_deg=phase)
        perm = assign_min_distance(starts, slots)
        sep = transition_min_sep(
            [np.asarray(s, float)[:2] for s in starts],
            [np.asarray(slots[perm[j]], float)[:2] for j in range(n)])
        if sep > best[3]:
            best = (slots, angles, perm, sep)
    return best


def check_leg(src, dst, label='leg', min_sep=PLAN_SEPARATION):
    """Assert a transition is safe; returns the separation, raises if not.

    Defaults to PLAN_SEPARATION, not MIN_SEPARATION: the caller is checking a
    *plan*, and the drones fly it with up to TRACKING_MARGIN of error, so a
    leg planned at exactly MIN_SEPARATION is flown under it. That is the
    2026-09-07 incident that created TRACKING_MARGIN in the first place.
    check_show and assign_makespan always used the plan budget; this one gate
    held the lower bar until 2026-10-06, and demo_show -- whose ONLY
    separation gate it is -- took the default.
    """
    sep = transition_min_sep(src, dst)
    if sep < min_sep:
        raise ValueError(
            f'{label}: min separation {sep:.2f} m < {min_sep:.2f} m budget')
    return sep


def check_envelope(positions, radius=ARENA_RADIUS_PLAN, ceiling=None,
                   center=ARENA_CENTRE, label='envelope'):
    """Assert every position is inside the flyable volume.

    Refuses outright on an UNMEASURED arena (``measured: false`` in
    arena.yaml). This is the gate that makes the package safe to copy: every
    plan and every show reaches this function, so a room nobody has surveyed
    cannot be flown by accident on somebody else's numbers. It is a no-op on a
    measured arena, so it changes nothing for this rig.

    Defaults are the MEASURED volume (2026-10-01), centred on ARENA_CENTRE --
    not the inherited 2.5 m from (0, 0), which this function's own defaults
    were the last thing in the package still referencing.

    ``ceiling=None`` applies the truncated-cone limit from :func:`ceiling_at`,
    which is radius-dependent, because one number for the whole room either
    forbids the centre climb or blesses an untested corner. An explicit
    ``ceiling`` can only LOWER it, never raise it above what has been flown --
    the same composition check_show uses.
    """
    require_measured_arena(f'{label} check')
    for i, p in enumerate(positions):
        p = np.asarray(p, float)
        r = float(np.linalg.norm(p[:2] - np.asarray(center, float)))
        if r > radius:
            raise ValueError(
                f'{label}: slot {i} at r={r:.2f} m exceeds arena {radius} m')
        if len(p) > 2:
            limit = ceiling_at(r)
            if ceiling is not None:
                limit = min(float(ceiling), float(limit))
            if p[2] > limit:
                raise ValueError(
                    f'{label}: slot {i} at z={p[2]:.2f} m exceeds the ceiling '
                    f'{float(limit):.2f} m flown clean at r={r:.2f} m')


def scaled_duration(src, dst, avg_speed=0.5, minimum=2.0):
    """Duration for a goTo leg, scaled to the longest path in the group.

    A fixed duration on a long move demands accelerations the drone cannot
    produce -- goTo's planner will not correct an infeasible request, it will
    just destabilise the controller.
    """
    longest = max(float(np.linalg.norm(np.asarray(d, float)[:2]
                                       - np.asarray(s, float)[:2]))
                  for s, d in zip(src, dst))
    return max(minimum, longest / avg_speed)


# ------------------------------------------------------------------------
# Whole-show checking
#
# check_leg() above answers "is this one simultaneous goTo safe?". A show
# built from uploaded polynomial trajectories needs a stronger answer,
# because during a trajectory phase the drones are not on straight lines at
# all -- they are wherever the polynomial says, and two figures that are each
# individually sane can still put two drones in the same cubic metre.
#
# So the show is sampled: every drone's position is evaluated on a common
# time grid across every phase, and the minimum pairwise distance is taken
# over the whole grid. That is an approximation -- a grid can step over a very
# brief close approach -- but at 200 Hz with drones under 2 m/s a drone moves
# 1 cm between samples, far below the margin the budget carries.
#
# 200 Hz rather than 50 is about the *acceleration* check, not separation.
# Acceleration is a second difference, so a coarse grid smooths exactly the
# short jolts worth catching: a boundary discontinuity that reads as 17 m/s^2
# at 400 Hz reads as 4 m/s^2 at 50 Hz and passes a 3 m/s^2 budget. Sampling
# fast enough to see the jolt is what makes the budget mean anything.
# ------------------------------------------------------------------------

#: Sampling rate for the offline show check, Hz. See the note above.
CHECK_RATE = 200.0

#: Envelope the figures are sized against. The Crazyflie 2.1 can exceed both;
#: these are the values at which the PID controller still tracks a polynomial
#: closely rather than lagging it, which matters because a lagging drone is
#: not where the collision check thinks it is.
MAX_SPEED = 2.0    # m/s
MAX_ACCEL = 3.0    # m/s^2


def sample_min_sep(samples):
    """Minimum pairwise distance over a stack of per-drone position samples.

    ``samples`` is ``(n_drones, n_times, 3)``. Returns
    ``(min_sep, time_index, drone_a, drone_b)`` so a failure can be reported
    where it happens rather than as a bare number.
    """
    samples = np.asarray(samples, float)
    n = samples.shape[0]
    worst, where = np.inf, (0, 0, 0)
    for a in range(n):
        for b in range(a + 1, n):
            d = np.linalg.norm(samples[a] - samples[b], axis=1)
            k = int(np.argmin(d))
            if d[k] < worst:
                worst, where = float(d[k]), (k, a, b)
    return (worst,) + where


def path_envelope(samples, dt):
    """Peak speed and acceleration per drone, by central difference.

    ``samples`` is ``(n_drones, n_times, 3)`` on a uniform grid of step
    ``dt``. Returns ``(max_speed, max_accel)`` over all drones.

    This is measured on the *planned* path, so it is what the controller will
    be asked for, not what it will achieve. A figure that exceeds
    :data:`MAX_SPEED` or :data:`MAX_ACCEL` does not fail loudly in flight --
    the drone simply lags the setpoint, which quietly invalidates the
    separation check above. Treat an envelope violation as a collision risk,
    not a performance note.
    """
    samples = np.asarray(samples, float)
    if samples.shape[1] < 3:
        return 0.0, 0.0
    vel = np.gradient(samples, dt, axis=1)
    acc = np.gradient(vel, dt, axis=1)
    return (float(np.max(np.linalg.norm(vel, axis=2))),
            float(np.max(np.linalg.norm(acc, axis=2))))


def rest_to_rest_line(src, dst, t, duration):
    """Where a rest-to-rest ``goTo`` actually is at time ``t``.

    The firmware's ``plan_go_to`` fits a degree-7 polynomial per axis from the
    current state to the goal. When the drone starts and ends at rest with
    zero acceleration -- which every ``goTo`` in this show does, because every
    figure preceding one is rest-to-rest -- that polynomial is the *same*
    scalar time-warp on all three axes. The path is therefore exactly the
    straight segment ``src -> dst``, and the straight-line collision model
    used by :func:`transition_min_sep` is exact rather than approximate.

    That equivalence is the whole reason the figures are built rest-to-rest.
    Issue a ``goTo`` while a drone still has velocity and the per-axis
    polynomials no longer share a time-warp: the path bows away from the
    straight line, by an amount nothing here models.
    """
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    u = float(np.clip(t / duration, 0.0, 1.0))
    return src + (dst - src) * (u ** 3 * (10.0 + u * (-15.0 + 6.0 * u)))


def _locate(ts, mask, phase_bounds):
    """Name the phase (and time) where a per-sample violation mask first fires."""
    idx = np.argmax(mask)
    t = float(ts[idx])
    if phase_bounds:
        for name, t0, t1 in phase_bounds:
            if t0 - 1e-9 <= t <= t1 + 1e-9:
                return f'{t:.1f}s, in "{name}"'
    return f'{t:.1f}s'


def check_show(sample_fn, total_time, n_drones, rate=CHECK_RATE,
               min_sep=PLAN_SEPARATION, radius=ARENA_RADIUS_PLAN, ceiling=None,
               center=(0.0, 0.0), floor=0.15, floor_window=None,
               phase_bounds=None, arena_center=ARENA_CENTRE):
    """Sample an entire choreography and check separation, envelope and volume.

    ``sample_fn(t)`` returns an ``(n_drones, 3)`` array of positions at show
    time ``t``. Returns a report dict; raises ``ValueError`` on any violation
    so a show script cannot arm a plan that fails.

    ``floor_window`` is ``(t0, t1)``: the interval over which ``floor`` is
    enforced. Takeoff necessarily starts on the ground and landing necessarily
    ends there, so the floor is only meaningful between them -- pass the
    cruise window and a figure that dips towards the floor mid-show is still
    caught.

    ``phase_bounds`` is ``[(name, t0, t1), ...]``. It changes nothing about
    the verdict; it just lets a failure say *which figure* is over budget,
    which is the difference between a two-minute fix and an afternoon.

    **The envelope is measured from ``arena_center``, not from ``center``.**
    They are different questions: ``center`` is where the formation is built,
    and a formation can be built anywhere, while the mocap volume is where it
    is and does not move when the drones are parked somewhere else. Measuring
    the radius from the formation's own centroid -- which is what this did
    until 2026-10-02 -- hides exactly the dangerous case: a show whose figures
    all fit a 2 m circle, sitting 0.5 m off-centre, with one arm reaching into
    the untracked corner. ``ceiling`` defaults to the cone of ``ceiling_at``,
    so the limit falls off with radius as the measured volume does.
    """
    n_t = max(3, int(round(total_time * rate)) + 1)
    ts = np.linspace(0.0, total_time, n_t)
    dt = float(ts[1] - ts[0])
    samples = np.transpose(np.array([sample_fn(t) for t in ts], float), (1, 0, 2))

    sep, k, a, b = sample_min_sep(samples)
    v_max, a_max = path_envelope(samples, dt)

    vel = np.gradient(samples, dt, axis=1)
    acc = np.gradient(vel, dt, axis=1)
    speed = np.linalg.norm(vel, axis=2)     # (n_drones, n_times)
    accel = np.linalg.norm(acc, axis=2)

    xy = samples[:, :, :2] - np.asarray(arena_center, float)[:2]
    r = np.linalg.norm(xy, axis=2)
    z = samples[:, :, 2]
    #: Per-sample height limit: the measured cone, and a caller's flat ceiling
    #: too if it named one. Both apply -- a flat number cannot describe a cone,
    #: and a cone should not silently raise a limit a caller asked to lower.
    z_lim = ceiling_at(r)
    if ceiling is not None:
        z_lim = np.minimum(z_lim, float(ceiling))

    report = {
        'min_sep': sep, 'min_sep_time': float(ts[k]), 'min_sep_pair': (a, b),
        'max_speed': v_max, 'max_accel': a_max,
        'max_radius': float(np.max(r)), 'max_z': float(np.max(z)),
        'arena_center': tuple(float(v) for v in np.asarray(arena_center, float)[:2]),
        'min_z': float(np.min(z)),
        'duration': float(total_time), 'samples': samples, 'times': ts,
        'speed': speed, 'accel': accel,
    }

    problems = []
    if sep < min_sep:
        problems.append(
            f'min separation {sep:.2f} m < {min_sep:.2f} m budget - '
            f'drones {a} and {b} at '
            f'{_locate(ts, np.arange(n_t) == k, phase_bounds)}')
    if v_max > MAX_SPEED:
        problems.append(
            f'peak speed {v_max:.2f} m/s > {MAX_SPEED:.2f} m/s at '
            f'{_locate(ts, speed.max(axis=0) > MAX_SPEED, phase_bounds)}')
    if a_max > MAX_ACCEL:
        problems.append(
            f'peak accel {a_max:.2f} m/s^2 > {MAX_ACCEL:.2f} m/s^2 at '
            f'{_locate(ts, accel.max(axis=0) > MAX_ACCEL, phase_bounds)}')
    if report['max_radius'] > radius:
        problems.append(
            f'max radius {report["max_radius"]:.2f} m > arena {radius:.2f} m at '
            f'{_locate(ts, r.max(axis=0) > radius, phase_bounds)}')
    over_z = z > z_lim
    if over_z.any():
        j, i = np.unravel_index(np.argmax(z - z_lim), z.shape)
        problems.append(
            f'height {z[j, i]:.2f} m exceeds the {z_lim[j, i]:.2f} m flown '
            f'clean at r={r[j, i]:.2f} m (drone {j}) at '
            f'{_locate(ts, over_z.any(axis=0), phase_bounds)}')

    lo, hi = floor_window if floor_window else (0.0, total_time)
    win = (ts >= lo) & (ts <= hi)
    if win.any():
        report['min_z_cruise'] = float(z[:, win].min())
        low = z[:, win].min(axis=0) < floor
        if low.any():
            problems.append(
                f'min height {float(z[:, win].min()):.2f} m < floor {floor:.2f} m '
                f'at {_locate(ts[win], low, phase_bounds)}')
    if problems:
        raise ValueError('show check failed:\n  - ' + '\n  - '.join(problems))
    return report


# ------------------------------------------------------------------------
# Plan view and downwash
#
# check_show() enforces one isotropic 3D budget. A show that uses the vertical
# axis needs two more numbers to be honest about what that budget means:
#
# * plan-view separation -- how close two drones get in x/y alone. If it stays
#   over PLAN_SEPARATION, height is purely decorative: no drone is ever above
#   another, and downwash never enters into it.
# * downwash clearance -- the measured anisotropic model (Preiss, Hoenig et
#   al., IROS 2017: two Crazyflies stacked, position error logged at 100 Hz)
#   says the real keep-out region is an ellipsoid, 0.12 m horizontally and
#   0.30 m vertically. A pair is clear when || E^-1 (p - q) || >= 2 with
#   E = diag(r_xy, r_xy, r_z). Reported, not enforced: it is the headroom a
#   denser show could spend once the tracking lag has been re-measured on
#   hardware (see crazyflie_shows/reference/complex-shows-report.html).
# ------------------------------------------------------------------------

#: Measured downwash ellipsoid radii for a Crazyflie, metres.
DOWNWASH_RXY = 0.12
DOWNWASH_RZ = 0.30


def downwash_clearance(samples, r_xy=DOWNWASH_RXY, r_z=DOWNWASH_RZ,
                       margin=TRACKING_MARGIN):
    """How many times over the downwash ellipsoid the closest pair stays.

    ``samples`` is ``(n_drones, n_times, 3)``. Radii are inflated by
    ``margin`` (the measured controller lag). Returns ``(factor, t_index, a, b)``;
    ``factor >= 1`` means clear, and ``2.0`` means twice the required distance.
    """
    samples = np.asarray(samples, float)
    scale = np.array([r_xy + margin, r_xy + margin, r_z + margin])
    n = samples.shape[0]
    worst, where = np.inf, (0, 0, 0)
    for a in range(n):
        for b in range(a + 1, n):
            rho = np.linalg.norm((samples[a] - samples[b]) / scale, axis=1) / 2.0
            k = int(np.argmin(rho))
            if rho[k] < worst:
                worst, where = float(rho[k]), (k, a, b)
    return (worst,) + where
