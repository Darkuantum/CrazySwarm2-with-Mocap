"""Offline collision and envelope checks for choreography.

Nothing here talks to a drone. The point is to prove a figure is safe on the
ground, before anything is armed -- the firmware has no collision avoidance
enabled on this rig, so a crossing pair of goTo paths is a real collision.

The core routine is :func:`transition_min_sep`. During a simultaneous goTo,
every drone moves on a straight line over the same duration, so each pair's
relative position is linear in normalised time and its squared distance is a
quadratic -- the minimum over the leg is closed-form, not sampled.
"""

from itertools import permutations

import numpy as np

# Rule of thumb for this rig: 1 m start spacing, and the formation figures in
# multi_trajectory_formation.py were accepted at a 0.84 m worst case.
MIN_SEPARATION = 0.8   # m, the hard floor - two drones closer than this is a
                       # collision risk, whatever the plan says

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
TRACKING_MARGIN = 0.10  # m

#: What a *plan* must clear, so that the flown show still clears
#: MIN_SEPARATION. This is the number check_show enforces.
PLAN_SEPARATION = MIN_SEPARATION + TRACKING_MARGIN
ARENA_RADIUS = 2.5     # m, horizontal half-extent of the usable mocap volume
CEILING = 2.0          # m


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


def check_leg(src, dst, label='leg', min_sep=MIN_SEPARATION):
    """Assert a transition is safe; returns the separation, raises if not."""
    sep = transition_min_sep(src, dst)
    if sep < min_sep:
        raise ValueError(
            f'{label}: min separation {sep:.2f} m < {min_sep:.2f} m budget')
    return sep


def check_envelope(positions, radius=ARENA_RADIUS, ceiling=CEILING,
                   center=(0.0, 0.0), label='envelope'):
    """Assert every position is inside the flyable volume."""
    for i, p in enumerate(positions):
        p = np.asarray(p, float)
        r = float(np.linalg.norm(p[:2] - np.asarray(center, float)))
        if r > radius:
            raise ValueError(
                f'{label}: slot {i} at r={r:.2f} m exceeds arena {radius} m')
        if len(p) > 2 and p[2] > ceiling:
            raise ValueError(
                f'{label}: slot {i} at z={p[2]:.2f} m exceeds ceiling {ceiling} m')


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
               min_sep=PLAN_SEPARATION, radius=ARENA_RADIUS, ceiling=CEILING,
               center=(0.0, 0.0), floor=0.15, floor_window=None,
               phase_bounds=None):
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

    xy = samples[:, :, :2] - np.asarray(center, float)
    r = np.linalg.norm(xy, axis=2)
    z = samples[:, :, 2]

    report = {
        'min_sep': sep, 'min_sep_time': float(ts[k]), 'min_sep_pair': (a, b),
        'max_speed': v_max, 'max_accel': a_max,
        'max_radius': float(np.max(r)), 'max_z': float(np.max(z)),
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
    if report['max_z'] > ceiling:
        problems.append(
            f'max height {report["max_z"]:.2f} m > ceiling {ceiling:.2f} m at '
            f'{_locate(ts, z.max(axis=0) > ceiling, phase_bounds)}')

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
