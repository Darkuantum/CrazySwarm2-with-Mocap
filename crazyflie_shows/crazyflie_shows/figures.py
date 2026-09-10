"""Formation geometry: slot layouts and uploadable polynomial trajectories.

Pure functions -- no ROS, no drones, no side effects. Everything here can be
imported and unit-tested on a laptop with no radio and no mocap attached.

Conventions
-----------
* positions are numpy arrays, metres, in the ``world`` frame (the same frame
  ``/poses`` publishes in, set by ``motion_capture.yaml: topics.frame_id``)
* angles are degrees at the API surface, radians internally
* a "slot" is a 3D formation position; the assignment of drones to slots is
  handled in :mod:`crazyflie_shows.safety`

The trajectory half of this module
----------------------------------
Everything uploadable is built by :func:`fit_trajectory`, which takes an
arbitrary smooth parametric path ``t -> (x, y, z, yaw)`` and least-squares
fits a piecewise degree-7 polynomial to it -- the only thing the firmware's
high-level commander can execute.

Two rules make the fitted trajectories composable, and both matter:

1. **Every figure is rest-to-rest.** Shape parameters are driven through
   :func:`smoothstep`, a quintic with zero first *and* second derivative at
   both ends. A figure therefore begins and ends with zero velocity and zero
   acceleration, so phases chain without the controller having to absorb a
   discontinuity, and a following ``goTo`` starts from rest (which is what
   makes the straight-line collision model in :mod:`safety` exact).

2. **Every figure starts at the drone's own slot.** ``startTrajectory`` is
   called with ``relative=True``, which shifts the trajectory so ``eval(0)``
   lands on the drone's current setpoint. If the drone is where the figure
   says it should be, the shift is ~0 and the flown path is the planned path.

FIRMWARE PIECE BUDGET -- read before adding a figure
----------------------------------------------------
``TRAJECTORY_MEMORY_SIZE`` is 4096 B and ``sizeof(struct poly4d)`` is
4 axes x 8 coeffs x 4 B + 4 B duration = 132 B, so **31 pieces is the hard
limit for all trajectories resident on one drone at once**
(``crazyflie-firmware/src/modules/src/crtp_commander_high_level.c:84`` and
``src/modules/interface/pptraj.h:84``). Trajectories share that one arena and
are placed by the ``pieceOffset`` argument of ``uploadTrajectory``; see
:func:`pack_offsets`. Exceeding it silently corrupts a neighbouring
trajectory, so :data:`MAX_PIECES` is asserted in the show script.
"""

from math import comb, cos, radians, sin

import numpy as np

try:
    from crazyflie_py.uav_trajectory import Polynomial4D, Trajectory
except ImportError:                                       # pragma: no cover
    # crazyflie_py/__init__.py does `from .crazyswarm_py import Crazyswarm`,
    # which imports rclpy -- so the plain import above drags all of ROS in just
    # to get two numpy-only classes. On the rig that is free (rclpy is there);
    # on a laptop with no ROS it is the difference between being able to plan a
    # show and not. PathFinder locates the file without executing the package
    # __init__, which is the whole trick.
    import importlib.machinery
    import importlib.util
    import os as _os

    _spec = importlib.machinery.PathFinder().find_spec('crazyflie_py')
    if _spec is None or not _spec.submodule_search_locations:
        raise
    _file = _os.path.join(list(_spec.submodule_search_locations)[0],
                          'uav_trajectory.py')
    _s = importlib.util.spec_from_file_location('_cfs_uav_trajectory', _file)
    _m = importlib.util.module_from_spec(_s)
    _s.loader.exec_module(_m)
    Polynomial4D, Trajectory = _m.Polynomial4D, _m.Trajectory

#: Hard firmware limit -- see the module docstring.
MAX_PIECES = 31

#: Degree of every fitted piece. The firmware's PP_DEGREE; do not raise it.
PIECE_DEGREE = 7


# ------------------------------------------------------------------ slots

def slot_position(center, radius, angle_deg, height):
    """One point on a horizontal circle about ``center`` at ``height``."""
    a = radians(angle_deg)
    return np.array([center[0] + radius * cos(a),
                     center[1] + radius * sin(a),
                     height])


def ngon_slots(center, radius, n, height, phase_deg=0.0):
    """``n`` slots evenly spaced on a circle -- a regular n-gon.

    Adjacent separation is ``2 * radius * sin(180/n deg)``: at radius 0.8 m
    that is 1.13 m for n=4, 0.94 m for n=5, 0.80 m for n=6. Check it against
    your minimum-separation budget before flying.
    """
    angles = [phase_deg + 360.0 * k / n for k in range(n)]
    return [slot_position(center, radius, a, height) for a in angles], angles


def line_slots(center, spacing, n, height, heading_deg=0.0):
    """``n`` slots in a straight line through ``center``, evenly spaced."""
    h = radians(heading_deg)
    d = np.array([cos(h), sin(h)])
    offs = (np.arange(n) - (n - 1) / 2.0) * spacing
    return [np.array([center[0] + o * d[0], center[1] + o * d[1], height])
            for o in offs]


def grid_slots(center, spacing, rows, cols, height):
    """``rows * cols`` slots on a rectangular lattice centred on ``center``."""
    out = []
    for r in range(rows):
        for c in range(cols):
            out.append(np.array([
                center[0] + (c - (cols - 1) / 2.0) * spacing,
                center[1] + (r - (rows - 1) / 2.0) * spacing,
                height]))
    return out


def ngon_min_sep(radius, n):
    """Separation between adjacent slots of a regular n-gon."""
    return 2.0 * radius * sin(np.pi / n)


# --------------------------------------------------------- time shaping

def smoothstep(u):
    """Quintic ease, 0 -> 1 on ``u`` in [0, 1]; zero 1st AND 2nd derivative.

    ``s(u) = 10u^3 - 15u^4 + 6u^5``. The quintic (not the cubic
    ``3u^2-2u^3``) is what makes a figure rest-to-rest in *acceleration* as
    well as velocity, so the drone is never asked for a step change in thrust
    at a phase boundary.

    Useful constants, both exact, both used to size figures against the
    speed/accel envelope:

    * ``max s'  = 1.875``  at u = 0.5   -> peak speed is 1.875x the mean
    * ``max s'' = 5.7735``              -> peak tangential accel factor
    """
    u = np.clip(u, 0.0, 1.0)
    return u ** 3 * (10.0 + u * (-15.0 + 6.0 * u))


#: ``max s'`` of :func:`smoothstep` -- peak speed / mean speed.
SMOOTHSTEP_VMAX = 1.875
#: ``max s''`` of :func:`smoothstep`.
SMOOTHSTEP_AMAX = 5.7735


def ease(t, duration):
    """:func:`smoothstep` in seconds rather than normalised time."""
    return smoothstep(t / duration)


def ramped(u, ramp=0.25):
    """0 -> 1 on [0, 1] with a smooth ramp of width ``ramp`` at each end.

    Rate profile: quintic ramp up over ``[0, ramp]``, constant over
    ``[ramp, 1-ramp]``, quintic ramp down over ``[1-ramp, 1]``, scaled so the
    total integral is exactly 1. The plateau rate is ``k = 1 / (1 - ramp)``.

    Why not just use :func:`smoothstep` for everything: it peaks at 1.875x the
    mean rate, and on a circle the centripetal term goes as the *square* of
    the rate -- a 3.5x acceleration penalty paid across the whole figure
    purely to start and stop nicely. Measured, for a 360 deg turn of a 1.1 m
    ring in 6.0 s:

    ==================  ==========  ================
    profile             peak speed  peak accel
    ==================  ==========  ================
    :func:`smoothstep`  2.16 m/s    4.24 m/s^2
    ``ramped(0.25)``    1.54 m/s    2.14 m/s^2
    ==================  ==========  ================

    which on this rig is the difference between a figure inside the envelope
    (:data:`safety.MAX_SPEED`, :data:`safety.MAX_ACCEL`) and one outside it.
    Rate and acceleration are still zero at both ends, so rest-to-rest
    composition is unaffected.

    The trade is that the acceleration is concentrated in the ramps rather
    than spread over the figure. Shrinking ``ramp`` towards 0 approaches a
    constant-rate turn with an impulsive start; 0.2-0.3 is the useful band.
    """
    u = float(np.clip(u, 0.0, 1.0))
    w = float(ramp)
    if w <= 0.0:
        return u
    k = 1.0 / (1.0 - w)             # plateau rate, so the integral is exactly 1
    if u < w:
        return k * w * _smoothstep_integral(u / w)
    if u > 1.0 - w:
        return 1.0 - k * w * _smoothstep_integral((1.0 - u) / w)
    return k * (u - 0.5 * w)


def _smoothstep_integral(w):
    """Integral of :func:`smoothstep` from 0 to ``w``, for ``w`` in [0, 1].

    ``int 10u^3 - 15u^4 + 6u^5 du = 2.5w^4 - 3w^5 + w^6``; note the useful
    identity ``_smoothstep_integral(1) == 0.5``, which is what makes the
    plateau normalisation in :func:`ramped` come out as ``1 / (1 - ramp)``.
    """
    w = float(np.clip(w, 0.0, 1.0))
    return w ** 4 * (2.5 + w * (-3.0 + w))


def ease_ramped(t, duration, ramp=0.25):
    """:func:`ramped` in seconds rather than normalised time."""
    return ramped(t / duration, ramp)


def bump(u, cycles=1):
    """0 at both ends, 1 at each peak, and zero *rate* at both ends.

    ``sin^2(pi * cycles * u)``, equivalently ``(1 - cos(2*pi*cycles*u)) / 2``.

    This is the only correct envelope for a there-and-back excursion in a
    figure that has to compose with its neighbours. The obvious choice,
    ``sin(pi*u)``, has the right *values* at the ends and the wrong
    *derivative*: its rate at u=0 is ``pi/T`` times the amplitude, so a 0.45 m
    rise over 6.2 s starts with a 0.29 m/s velocity step. That is not a
    rounding error -- sampled at 50 Hz it reads as a 7 m/s^2 acceleration
    spike, and on the drone it is a real jolt at the phase boundary, from a
    figure whose own interior is perfectly smooth.

    ``bump`` has zero rate at both ends and so is genuinely rest-to-rest. Its
    second derivative at the ends is ``2*(pi*cycles/T)^2`` times the
    amplitude, not zero -- a step in acceleration, which the controller
    absorbs without complaint; for the show's figures that is under
    0.2 m/s^2.
    """
    return sin(np.pi * cycles * u) ** 2


# ---------------------------------------------------- polynomial fitting

def _derivs(pos_fn, t, lo, hi, h=1e-3):
    """Value, 1st and 2nd time derivative of ``pos_fn`` at ``t``.

    Central differences in the interior; one-sided second-order stencils
    within ``h`` of the ends, because the figure builders clamp their eased
    parameters at [0, T] and a central stencil that steps outside would
    silently read the clamped value and report a derivative of zero.

    ``h`` is **absolute**, not scaled to the piece. That is what makes
    :func:`fit_trajectory` exactly C2: the piece ending at ``t`` and the piece
    starting at ``t`` ask this function the same question and get bit-identical
    answers, so the constraints they are each fitted to agree exactly.
    """
    f = lambda x: np.asarray(pos_fn(float(np.clip(x, lo, hi))), float)
    if t - h < lo:
        f0, f1, f2, f3 = f(t), f(t + h), f(t + 2 * h), f(t + 3 * h)
        return (f0,
                (-3 * f0 + 4 * f1 - f2) / (2 * h),
                (2 * f0 - 5 * f1 + 4 * f2 - f3) / (h * h))
    if t + h > hi:
        f0, f1, f2, f3 = f(t), f(t - h), f(t - 2 * h), f(t - 3 * h)
        return (f0,
                (3 * f0 - 4 * f1 + f2) / (2 * h),
                (2 * f0 - 5 * f1 + 4 * f2 - f3) / (h * h))
    fp, f0, fm = f(t + h), f(t), f(t - h)
    return f0, (fp - fm) / (2 * h), (fp - 2 * f0 + fm) / (h * h)


def _constrained_polyfit(ts, ys, bc0, bc1, degree=PIECE_DEGREE):
    """Least-squares fit of one piece, with position/vel/accel pinned at both ends.

    ``bc0`` and ``bc1`` are ``(value, first, second)`` at local times 0 and
    ``ts[-1]``. Six equality constraints on eight coefficients leave two
    degrees of freedom, which are spent minimising the residual over the
    interior samples.

    Solved by projecting onto the constraint null space: take a particular
    solution ``c0`` of ``A c = b``, take an orthonormal basis ``N`` of
    ``null(A)`` from the SVD, and least-squares ``c = c0 + N z``. That is
    exact in the constraints (to machine precision) rather than
    penalty-weighted, which is the whole point -- a boundary that is *nearly*
    matched is what produces the acceleration spikes this exists to remove.
    """
    T = float(ts[-1])
    npow = degree + 1
    A = np.zeros((6, npow))
    for i in range(npow):                       # p(t) = sum c_i t^i
        A[0, i] = 1.0 if i == 0 else 0.0        # p(0)
        A[1, i] = 1.0 if i == 1 else 0.0        # p'(0)
        A[2, i] = 2.0 if i == 2 else 0.0        # p''(0)
        A[3, i] = T ** i                        # p(T)
        A[4, i] = i * T ** (i - 1) if i >= 1 else 0.0
        A[5, i] = i * (i - 1) * T ** (i - 2) if i >= 2 else 0.0
    b = np.array([bc0[0], bc0[1], bc0[2], bc1[0], bc1[1], bc1[2]], float)

    c0, *_ = np.linalg.lstsq(A, b, rcond=None)
    _, sv, vt = np.linalg.svd(A)
    rank = int(np.sum(sv > max(A.shape) * np.finfo(float).eps * sv[0]))
    N = vt[rank:].T
    if N.shape[1]:
        V = np.vander(ts, npow, increasing=True)
        z, *_ = np.linalg.lstsq(V @ N, ys - V @ c0, rcond=None)
        return c0 + N @ z
    return c0


def fit_trajectory(pos_fn, duration, n_pieces=6, degree=PIECE_DEGREE,
                   n_samples=48, boundaries=None):
    """Fit any smooth ``t -> (x, y, z, yaw)`` path as an uploadable Trajectory.

    Each span gets a degree-7 fit *in piece-local time* (which is what the
    firmware evaluates), per axis, with position, velocity and acceleration
    pinned to the source path at both ends -- so the assembled trajectory is
    exactly C2 across every piece boundary, and rest-to-rest at the outer ends
    whenever the source path is.

    Why constrained and not a plain ``polyfit`` per piece
    ----------------------------------------------------
    Because the drone flies the *second derivative*. An unconstrained fit
    leaves a small position mismatch at each boundary -- typically 1e-4 m,
    which looks like nothing next to a 1 m figure -- but a step of that size
    differentiated twice is an acceleration spike, and the high-level
    commander feeds acceleration forward to the controller. Measured on the
    show's counterflow figure -- same path, same 6 pieces, analytic peak
    acceleration 2.73 m/s^2:

    ==================  ===========  =============  ===========
    fit                 position     peak accel     C2 at joins
    ==================  ===========  =============  ===========
    plain ``polyfit``   2.8e-04 m    17.3 m/s^2     no
    constrained         1.9e-03 m     2.8 m/s^2     yes
    ==================  ===========  =============  ===========

    The trade is explicit and worth taking: six of the eight coefficients now
    go to the boundary conditions instead of to the residual, so position
    error grows about sevenfold -- to 1.9 mm, an order of magnitude below what
    the mocap resolves and two below what the controller tracks. In exchange
    the phantom 17 m/s^2 disappears and the trajectory asks for exactly the
    acceleration the figure was designed for. A plain fit is smooth on a
    position plot and violent in the second derivative.

    ``boundaries`` overrides the uniform split with explicit times
    ``[0, t1, ..., duration]``. Two things make a piece hard to fit, and both
    are fixed by putting a boundary in the right place:

    * a **regime change** in the path (a figure that fans out, then rotates,
      then fans back in) -- a piece straddling the junction has to fit two
      different behaviours at once;
    * **too much arc in one piece** -- degree 7 tracks a 90 deg arc to ~5e-5 m
      and a 60 deg arc to ~1e-7 m, but a 180 deg arc only to ~4e-3 m.

    :func:`counterflow_path` hits both, which is why it ships with a
    ``boundaries`` recipe (see :func:`counterflow_boundaries`).

    ``yaw`` is the 4th channel. Every figure in this package leaves it at 0:
    the drones keep a fixed heading and translate, which keeps the mocap
    rigid-body solve well conditioned and avoids the yaw-offset fly-away
    described in HANDOVER.md section 6.
    """
    if boundaries is None:
        boundaries = np.linspace(0.0, duration, n_pieces + 1)
    boundaries = np.asarray(boundaries, float)
    lo, hi = float(boundaries[0]), float(boundaries[-1])

    # One evaluation per boundary, shared by the pieces on both sides -- which
    # is what makes the joins exactly continuous rather than nearly so.
    bcs = [_derivs(pos_fn, float(t), lo, hi) for t in boundaries]

    traj = Trajectory()
    traj.polynomials = []
    for k, (t0, t1) in enumerate(zip(boundaries[:-1], boundaries[1:])):
        piece_t = float(t1 - t0)
        ts = np.linspace(0.0, piece_t, n_samples)
        vals = np.array([pos_fn(t0 + t) for t in ts], dtype=float)
        coeffs = []
        for axis in range(4):
            coeffs.append(_constrained_polyfit(
                ts, vals[:, axis],
                (bcs[k][0][axis], bcs[k][1][axis], bcs[k][2][axis]),
                (bcs[k + 1][0][axis], bcs[k + 1][1][axis], bcs[k + 1][2][axis]),
                degree))
        traj.polynomials.append(Polynomial4D(piece_t, *coeffs))
    traj.duration = float(boundaries[-1])
    return traj


def eval_traj(traj, t, reverse=False, timescale=1.0, shift=None):
    """Evaluate a Trajectory the way the firmware will, in *wall* seconds.

    ``Trajectory.eval`` in ``crazyflie_py`` walks the pieces accumulating
    durations in float and returns ``None`` when the accumulated total lands a
    few ULPs below ``t`` -- which is exactly what happens at ``t == duration``
    whenever ``duration / n_pieces`` is not exactly representable (6.5 / 6,
    for instance). Clamping into the last piece is the fix; a sampler that
    hits the final instant of every phase would otherwise crash on some
    durations and not others.

    Mirrors ``plan_start_trajectory`` in the firmware
    (``src/modules/src/planner.c``): ``timescale`` stretches wall time,
    ``reverse`` evaluates at ``duration - t``, and ``shift`` is the
    ``relative=True`` translation added to the position.
    """
    span = sum(p.duration for p in traj.polynomials)
    u = float(np.clip(t / timescale, 0.0, span))
    if reverse:
        u = float(np.clip(span - u, 0.0, span))
    out = traj.eval(min(u, span * (1.0 - 1e-12)))
    pos = np.array(out.pos, float)
    return pos if shift is None else pos + np.asarray(shift, float)


def fit_error(traj, pos_fn, n_samples=400):
    """Max position error between a fitted Trajectory and its source path."""
    worst = 0.0
    for t in np.linspace(0.0, traj.duration, n_samples):
        want = np.asarray(pos_fn(t), float)[:3]
        worst = max(worst, float(np.linalg.norm(eval_traj(traj, t) - want)))
    return worst


def pack_offsets(n_pieces_per_traj):
    """Piece offsets that lay trajectories out end to end in the 31-piece arena.

    ``uploadTrajectory(id, pieceOffset, traj)`` writes into one shared block
    of firmware memory. Two trajectories uploaded at the same offset overwrite
    each other with no error reported; that is the failure this exists to
    prevent.

    Returns the offset list, and raises if the total exceeds :data:`MAX_PIECES`.
    """
    offsets, cursor = [], 0
    for n in n_pieces_per_traj:
        offsets.append(cursor)
        cursor += n
    if cursor > MAX_PIECES:
        raise ValueError(
            f'{cursor} pieces exceeds the {MAX_PIECES}-piece firmware '
            f'trajectory memory (4096 B / 132 B per piece). '
            f'Drop a figure or reduce n_pieces.')
    return offsets, cursor


# ----------------------------------------------------------- Bezier tools

def bezier_point(ctrl, u):
    """Evaluate a Bezier curve of any degree at ``u`` in [0, 1] (de Casteljau)."""
    pts = [np.asarray(p, float) for p in ctrl]
    while len(pts) > 1:
        pts = [(1.0 - u) * pts[i] + u * pts[i + 1] for i in range(len(pts) - 1)]
    return pts[0]


def bezier_to_poly(ctrl):
    """Exact power-basis coefficients (ascending) of a Bezier curve on [0, 1].

    Bernstein -> power basis:
    ``c_i = C(n,i) * sum_{j<=i} (-1)^(i-j) * C(i,j) * P_j``.

    Exact, not fitted -- but only valid when the curve is traversed with a
    *uniform* time parameter. The figures here drive ``u`` through
    :func:`smoothstep` so they start and end at rest, and a quintic composed
    with a cubic Bezier is degree 15, past what the firmware can hold. Those
    go through :func:`fit_trajectory` instead. This function is here for the
    uniform-speed case, where it is exact and free.
    """
    pts = [np.asarray(p, float) for p in ctrl]
    n = len(pts) - 1
    if n > PIECE_DEGREE:
        raise ValueError(f'Bezier degree {n} exceeds firmware degree {PIECE_DEGREE}')
    return [comb(n, i) * sum((-1) ** (i - j) * comb(i, j) * pts[j]
                             for j in range(i + 1))
            for i in range(n + 1)]


def bowed_chord(src, dst, bow, u):
    """Point at ``u`` on a cubic Bezier from ``src`` to ``dst``, bowed sideways.

    The two interior control points are pushed ``bow`` metres along the
    left-hand normal of the chord (in the horizontal plane), turning a
    straight transit into an arc. Two drones swapping places on opposite
    bows pass each other with clearance instead of meeting in the middle.
    """
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    d = dst[:2] - src[:2]
    norm = float(np.linalg.norm(d))
    nrm = np.array([-d[1], d[0], 0.0]) / norm if norm > 1e-9 else np.zeros(3)
    return bezier_point([src,
                         src + (dst - src) / 3.0 + bow * nrm,
                         src + 2.0 * (dst - src) / 3.0 + bow * nrm,
                         dst], u)


# -------------------------------------------------------- path builders

def polar_path(center, r_fn, theta_fn, z_fn):
    """Compose radius/angle/height functions of time into a ``pos_fn``.

    ``theta_fn`` returns radians. Almost every ring figure in this package is
    a two-line call to this: the ring's shape is entirely in how ``r``,
    ``theta`` and ``z`` are eased.
    """
    def fn(t):
        r = r_fn(t)
        th = theta_fn(t)
        return (center[0] + r * cos(th), center[1] + r * sin(th), z_fn(t), 0.0)
    return fn


def carousel_path(center, radius, start_angle_deg, sweep_deg, height, duration,
                  ramp=0.25):
    """Rigid rotation about ``center`` by ``sweep_deg``, ramped in and out.

    Every drone runs the same sweep from its own start angle, so the
    formation turns as one rigid body and pairwise separation is *constant*
    for the whole figure -- the safest possible way to move a swarm at speed.
    That constancy is why this is the figure the show reuses at successively
    smaller timescales as it accelerates.

    Use a ``sweep_deg`` that is a whole multiple of 360 (see
    :func:`composes_in_place`). Rotation figures are the ones most likely to
    be reused, and a reused figure that does not land back on its own start
    angle drifts the formation apart -- the module docstring for
    :mod:`crazyflie_shows.choreography` works through why.
    """
    th0 = radians(start_angle_deg)
    dth = radians(sweep_deg)
    return polar_path(
        center,
        lambda t: radius,
        lambda t: th0 + dth * ease_ramped(t, duration, ramp),
        lambda t: height)


def composes_in_place(sweep_deg, tol=1e-6):
    """True if a figure with this sweep returns every drone to its own slot.

    The composition rule for this package, in one predicate. See
    :mod:`crazyflie_shows.choreography` for the derivation.
    """
    return abs((float(sweep_deg) + 180.0) % 360.0 - 180.0) < tol


def breathe_path(center, radius, start_angle_deg, height, duration,
                 shrink=0.35, cycles=2):
    """Ring pulses in and out ``cycles`` times, holding each drone's angle.

    ``r(t) = radius - shrink * bump(t/T, cycles)``: purely radial, identical
    for every drone, so the formation stays a regular n-gon and its
    separation scales exactly with ``r``. Worst case is
    ``ngon_min_sep(radius - shrink, n)`` and nothing else has to be checked.

    :func:`bump` is zero-rate at both ends, so the figure is rest-to-rest with
    no extra easing.
    """
    th = radians(start_angle_deg)
    return polar_path(
        center,
        lambda t: radius - shrink * bump(t / duration, cycles),
        lambda t: th,
        lambda t: height)


def wave_path(center, radius, start_angle_deg, sweep_deg, height, duration,
              amplitude=0.35, phase_frac=0.0, cycles=1):
    """Rotation plus a per-drone vertical sinusoid -- a travelling wave.

    ``phase_frac`` (0..1) offsets this drone in the wave; giving drone ``j``
    of ``n`` a phase of ``j/n`` makes the crest run around the ring once.

    Horizontally this is a rigid rotation, so pairwise horizontal separation
    is unchanged from the n-gon and the z stagger can only *increase* the 3D
    distance. Visually the most complex figure in the show; geometrically one
    of the two safest.
    """
    th0 = radians(start_angle_deg)
    dth = radians(sweep_deg)
    ph = 2.0 * np.pi * phase_frac
    return polar_path(
        center,
        lambda t: radius,
        lambda t: th0 + dth * ease_ramped(t, duration),
        lambda t: height + amplitude * bump(t / duration) * sin(
            2.0 * np.pi * cycles * t / duration + ph))


def counterflow_path(center, ring_radius, lane_radius, start_angle_deg,
                     sweep_deg, height, duration, split=0.25):
    """Fan out to a private radius, counter-rotate, fan back. The crossing figure.

    Three moves welded into one rest-to-rest trajectory:

    * ``[0, split]``      radius eases from ``ring_radius`` to ``lane_radius``,
      angle held
    * ``[split, 1-split]`` angle sweeps by ``sweep_deg`` at ``lane_radius``
    * ``[1-split, 1]``    radius eases back to ``ring_radius``

    Give one subset of drones a large ``lane_radius`` and ``+sweep_deg`` and
    the rest a small ``lane_radius`` and ``-sweep_deg``, and the two groups
    stream through each other head-on, several times. The clearance is not a
    timing coincidence -- it is ``lane_radius_outer - lane_radius_inner``,
    a static radial gap that holds however the angles line up. That is the
    only reason a genuinely crossing figure is safe to fly on a rig with no
    onboard collision avoidance.

    ``sweep_deg`` should be a whole number of turns (or a multiple of the slot
    angle) so each drone lands back on a valid slot.
    """
    th0 = radians(start_angle_deg)
    dth = radians(sweep_deg)
    a, b = split * duration, (1.0 - split) * duration

    def r_fn(t):
        if t <= a:
            return ring_radius + (lane_radius - ring_radius) * ease(t, a)
        if t >= b:
            return lane_radius + (ring_radius - lane_radius) * ease(t - b, duration - b)
        return lane_radius

    def th_fn(t):
        if t <= a:
            return th0
        if t >= b:
            return th0 + dth
        return th0 + dth * ease_ramped(t - a, b - a)

    return polar_path(center, r_fn, th_fn, lambda t: height)


def counterflow_boundaries(duration, split=0.25, turn_pieces=4):
    """Piece boundaries for :func:`counterflow_path`.

    One piece for the fan-out, ``turn_pieces`` for the rotation, one for the
    fan-in -- boundaries landing exactly on the figure's two regime changes.
    Uniform pieces straddle those junctions *and* give each rotation piece too
    much arc; both show up as fit error. Measured on the show's counterflow:

    ======================  ==========
    6 uniform pieces        7.0e-04 m
    6 aligned (turn=4)      4.9e-05 m
    ======================  ==========
    """
    a, b = split * duration, (1.0 - split) * duration
    return [0.0] + list(np.linspace(a, b, turn_pieces + 1)) + [duration]


def bloom_path(center, radius, start_angle_deg, sweep_deg, height, duration,
               expand=0.45, rise=0.5):
    """Finale: the ring flings outward and upward, spins, then snaps back.

    Radius and height both follow :func:`bump` -- out and back in one gesture,
    zero rate at both ends -- while the angle sweeps. Rigid in the
    horizontal plane like :func:`carousel_path`, so separation only ever
    *grows*: at peak expansion the ring is ``(radius + expand) / radius``
    times further apart than the n-gon it started from.
    """
    th0 = radians(start_angle_deg)
    dth = radians(sweep_deg)
    return polar_path(
        center,
        lambda t: radius + expand * bump(t / duration),
        lambda t: th0 + dth * ease_ramped(t, duration),
        lambda t: height + rise * bump(t / duration))


# ---------------------------------------------------- legacy / envelope

def circle_trajectory(center, radius, start_angle_deg, period, height,
                      n_pieces=6):
    """A full 360 deg circle at constant angular rate, as an uploadable traj.

    Kept for ``demo_show``. New choreography should prefer
    :func:`carousel_path` through :func:`fit_trajectory`: this one runs at
    constant omega, so it starts and ends with the full tangential velocity
    and the controller eats that step at both ends.
    """
    omega = 2.0 * np.pi / period
    th0 = radians(start_angle_deg)
    return fit_trajectory(
        polar_path(center,
                   lambda t: radius,
                   lambda t: th0 + omega * t,
                   lambda t: height),
        period, n_pieces=n_pieces)


def tangential_speed(radius, period):
    """Speed a drone flies at on a circle -- sanity-check before uploading."""
    return 2.0 * np.pi * radius / period


def centripetal_accel(radius, period):
    """Lateral acceleration on a circle. Keep well under ~3 m/s^2."""
    return (2.0 * np.pi / period) ** 2 * radius
