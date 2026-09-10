"""The show: one plan, built on the ground, flown by :mod:`swarm_show`.

This module is **pure** -- numpy and :mod:`crazyflie_py.uav_trajectory` only,
no rclpy, no radio, no mocap. That is what lets ``plan_show`` verify the exact
same object that ``swarm_show`` flies, on a laptop, with nothing armed. If you
find yourself importing ROS here, the thing you are adding belongs in
``swarm_show.py`` instead.

Everything an operator has to set for their own rig is in :class:`ShowConfig`
and marked ``>>> FILL IN``. Grep for that tag:

    grep -rn 'FILL IN' crazyflie_shows/


The composition rule -- why every figure ends where it started
--------------------------------------------------------------
``startTrajectory(relative=True)`` shifts an uploaded trajectory so that its
``eval(0)`` lands on the drone's *current* setpoint. The firmware does this
per drone (``plan_start_trajectory`` in ``planner.c``), with each drone's own
shift.

For a rotation figure that matters more than it looks. Say drone ``j``
uploaded a carousel that begins at its own slot angle ``th_j``, and by the
time the phase runs the drone has drifted round to ``th_j + 72``. Its shift
is then ``P(th_j + 72) - P(th_j)``, which *depends on th_j* -- so every drone
gets a different translation, and what should have been one rigid ring turning
about the room centre becomes five drones each orbiting a different centre.
The formation comes apart, and the offline separation check has no idea,
because the check was run on the unshifted geometry.

The rule that removes the whole failure mode:

    **every figure returns each drone to the angle it started at**

i.e. every sweep is a whole multiple of 360 deg (:func:`figures.composes_in_place`).
Then the shift is always ~0, figures can be reused in any order, at any
timescale, forwards or reversed, and the planned geometry is the flown
geometry. :func:`build_plan` asserts it.


The trajectory budget
---------------------
31 pieces per drone, total, for every trajectory resident at once -- see
:data:`figures.MAX_PIECES`. The show fits five figures in 28. The way to add a
sixth is to take pieces from another, not to hope.


The figures, in the order the show flies them
---------------------------------------------
====================  ==============================================  ========
figure                what makes it safe                              sweep
====================  ==============================================  ========
``breathe``           purely radial; ring stays a regular n-gon,       0
                      worst case is ``ngon_min_sep(R - shrink, n)``
``carousel``          rigid rotation; pairwise distance is *constant*  360
``wave``              rigid in xy; the z stagger can only add          360
``counterflow``       two groups on radii ``LANE_OUTER``/``LANE_       +/-360
                      INNER``; clearance is that static radial gap,
                      not a timing coincidence
``bloom``             rigid in xy, and expanding, so separation only   360
                      grows
====================  ==============================================  ========

``counterflow`` is the one where drones genuinely fly through each other. It
is safe for a reason worth stating plainly: nothing about it depends on when
the two groups happen to meet. They are on concentric circles
``LANE_OUTER - LANE_INNER`` apart, so every head-on pass has that clearance
whatever the phase. On a rig with no onboard collision avoidance, that is the
only kind of crossing figure worth flying.
"""

from dataclasses import dataclass, field

import numpy as np

from crazyflie_shows import figures, safety


# =========================================================================
# Configuration -- everything rig-specific is here
# =========================================================================

@dataclass
class ShowConfig:
    """Tunables for the show. Defaults are sized for a ~3 x 3 m usable volume.

    Only the ``>>> FILL IN`` fields are rig-specific. The rest are choreography
    and are safe to leave alone until the show has flown once.
    """

    # ---------------------------------------------------------- >>> FILL IN
    #: Centre the whole show is built around, in the ``world`` frame, metres.
    #: ``None`` uses the centroid of the drones' ``initial_position`` values
    #: from ``crazyflies.yaml``, which is usually right and is self-correcting
    #: if you move the fleet. Set it explicitly to the hand-measured room
    #: centre if the drones do not start symmetrically about it.
    #: HANDOVER.md section 3 records ``(0.0467, -0.1037)`` as the hand-measured
    #: value, next to a computed centroid of ``(0.068, -0.109)``.
    room_center: tuple = None

    #: Usable horizontal half-extent and ceiling, metres, measured from
    #: ``room_center``. The show is checked against these and refuses to run
    #: if any figure leaves the box. MEASURE THESE IN YOUR ROOM -- the
    #: defaults are inherited from ``safety.py`` and have never been verified
    #: against the actual mocap volume.
    arena_radius: float = safety.ARENA_RADIUS
    ceiling: float = safety.CEILING

    #: Uniform scale on every horizontal dimension of the choreography.
    #: Shrinks every radius and every speed; the timing is untouched.
    #:
    #: It does NOT shrink the separation budget, and it cannot:
    #: safety.PLAN_SEPARATION is a physical floor plus a measured tracking
    #: allowance, neither of which is a property of the choreography. So
    #: scaling down eats clearance directly.
    #:
    #: Measured: **the floor is scale 0.99.** At 0.98 the counterflow's inner
    #: pair falls to 0.89 m and plan_show rejects the show. In other words
    #: this choreography is already at its minimum size for five drones --
    #: `scale` is a knob for growing it, not shrinking it.
    #:
    #: What that means in room terms, at PLAN_SEPARATION 0.90 m: the n-gon
    #: needs radius >= 0.90 / (2 sin 36deg) = 0.77 m, and the counterflow
    #: needs lane_inner >= 0.90 / (2 sin 72deg) = 0.47 m with lane_outer >=
    #: lane_inner + 0.90 = 1.37 m. Call it a 1.40 m working radius, so
    #: **2.8 x 2.8 m of clear floor is the minimum this show fits in**, before
    #: takeoff and landing margins. A smaller room needs fewer drones or a
    #: different set of figures, not a smaller scale.
    scale: float = 1.0
    # -------------------------------------------------------- end >>> FILL IN

    #: Formation geometry, metres.
    ring_radius: float = 1.10       # n-gon radius; n=5 -> 1.29 m adjacent sep
    form_height: float = 1.00       # altitude the figures fly at
    breathe_shrink: float = 0.30    # ring contracts to 0.80 m -> 0.94 m sep
    wave_amplitude: float = 0.35    # +/- z of the travelling wave
    #: Counterflow lanes. Three separate constraints pin these down, and all
    #: three are tight -- change one and re-run plan_show:
    #:   1. lane_outer - lane_inner  is the head-on clearance      -> 0.97 m
    #:      This is the show's tightest number and the one that has to carry
    #:      safety.TRACKING_MARGIN: the two groups counter-rotate, so their
    #:      controller lags subtract rather than cancel. Measured in sim, a
    #:      0.85 m design gap flew as 0.79 m. Do not shrink it back.
    #:   2. inner-group members sit 2 slots apart on the n-gon, so
    #:      2 * lane_inner * sin(2*pi/n) is their separation       -> 0.91 m
    #:      This pair is rigid, so it does not strictly need the tracking
    #:      margin -- but check_show applies one budget to the whole show, and
    #:      lane_inner is cheap, so it clears PLAN_SEPARATION too.
    #:   3. a full turn at lane_outer must stay under MAX_ACCEL, and
    #:      centripetal accel goes as radius -- this is what caps lane_outer
    lane_outer: float = 1.45
    lane_inner: float = 0.48
    bloom_expand: float = 0.35      # finale radius overshoot -> peaks at 1.45 m
    bloom_rise: float = 0.45        # finale height overshoot

    #: Takeoff / landing, metres and seconds.
    takeoff_height: float = 1.00
    takeoff_duration: float = 2.5
    home_height: float = 0.75       # above each drone's own initial_position
    land_height: float = 0.04
    land_duration: float = 3.5      # -> ~0.20 m/s descent from home_height
    settle_duration: float = 1.5

    #: Floor enforced between the end of takeoff and the start of landing. A
    #: figure that dips below it is a figure that lands mid-show.
    floor: float = 0.30

    #: Transition legs.
    trans_avg_speed: float = 0.45   # m/s; goTo durations are scaled to this
    trans_min_duration: float = 2.5

    #: Slack added after each phase's nominal duration before the next command
    #: is sent. Every phase is rest-to-rest, so this is only covering command
    #: latency and controller settling, not planner overshoot -- 0.5 s (as in
    #: demo_show) would add 6 s of dead air across twelve phases.
    phase_margin: float = 0.25

    #: Nominal durations of the uploaded figures, seconds. A figure is flown at
    #: ``nominal * timescale``; the timescales in :func:`build_plan` are what
    #: make the show accelerate.
    dur_breathe: float = 5.5
    dur_carousel: float = 6.5
    dur_wave: float = 6.5
    dur_counterflow: float = 10.5
    dur_bloom: float = 6.5

    #: Fraction of the counterflow spent fanning out to the lanes (and again
    #: fanning back); the rest is the counter-rotation. Squeezed from both
    #: sides, which is why it is not simply "as small as possible":
    #:
    #:   * make it SMALLER and the turn gets longer, which is what the outer
    #:     lane wants -- centripetal accel there goes as 1/T^2 at radius 1.30 m
    #:   * make it LARGER and the fan gets longer, which is what the INNER lane
    #:     wants -- it travels 1.10 -> 0.45 m, three times as far as the outer
    #:     lane's 1.10 -> 1.30, and that whole 0.65 m has to fit in the fan
    #:
    #: At 0.12 the inner fan alone hits 3.7 m/s^2 and the plan is rejected; at
    #: 0.30 the outer turn is over budget instead. 0.18 clears both with room.
    #: The inner lane travels 1.10 -> 0.48 m, the outer only 1.10 -> 1.45 m.
    #: Re-run plan_show after touching this, lane_inner or lane_outer.
    counterflow_split: float = 0.18

    #: Pieces per figure. Sum must be <= figures.MAX_PIECES (31). Measured fit
    #: errors at these counts are all <= 6e-5 m; ``plan_show`` reprints them.
    pieces: dict = field(default_factory=lambda: {
        'breathe': 5, 'carousel': 6, 'wave': 5, 'counterflow': 6, 'bloom': 6,
    })

    def scaled(self, v):
        """Apply :attr:`scale` to a horizontal dimension."""
        return v * self.scale


# =========================================================================
# Plan data model
# =========================================================================

@dataclass
class Figure:
    """One uploaded trajectory: a name, a firmware slot, and a path per drone."""

    name: str
    traj_id: int
    piece_offset: int
    trajs: list          # one crazyflie_py Trajectory per drone
    paths: list          # the exact source pos_fn per drone, for fit checking
    duration: float


@dataclass
class Phase:
    """One step of the show, and enough information to sample it offline."""

    name: str
    kind: str            # 'takeoff' | 'goto' | 'figure' | 'land'
    duration: float      # seconds of motion (already includes timescale)
    starts: list         # position of each drone when the phase begins
    ends: list           # position of each drone when it ends
    goals: list = None   # 'goto'/'takeoff'/'land': absolute per-drone target
    figure: Figure = None
    timescale: float = 1.0
    reverse: bool = False
    note: str = ''

    def position(self, j, t):
        """Where drone ``j`` is at ``t`` seconds into this phase."""
        t = float(np.clip(t, 0.0, self.duration))
        if self.kind == 'figure':
            traj = self.figure.trajs[j]
            base = figures.eval_traj(traj, 0.0, self.reverse, self.timescale)
            shift = np.asarray(self.starts[j], float) - base
            return figures.eval_traj(traj, t, self.reverse, self.timescale, shift)
        return safety.rest_to_rest_line(self.starts[j], self.ends[j],
                                        t, self.duration)


@dataclass
class Plan:
    """The whole show: figures to upload, phases to fly, and a sampler."""

    names: list
    starts: list          # each drone's initial_position, from the yaml
    center: np.ndarray
    slot_angles: list     # the n-gon angle each drone holds, degrees
    figs: list            # Figure, in upload order
    phases: list
    gather_sep: float
    report: dict = None   # filled in by build_plan's safety.check_show pass

    @property
    def duration(self):
        """Total seconds of motion, excluding per-phase margins."""
        return sum(p.duration for p in self.phases)

    @property
    def wall_duration(self):
        """What the operator will actually time with a stopwatch."""
        return self.duration + len(self.phases) * PHASE_MARGIN_REF[0]

    def sample(self, t):
        """Positions of all drones at show time ``t``, as ``(n_drones, 3)``."""
        for ph in self.phases:
            if t <= ph.duration:
                return np.array([ph.position(j, t) for j in range(len(self.names))])
            t -= ph.duration
        last = self.phases[-1]
        return np.array(last.ends, float)


#: Filled in by build_plan so Plan.wall_duration can report honestly without
#: every Plan having to carry a config reference.
PHASE_MARGIN_REF = [0.30]


# =========================================================================
# Plan construction
# =========================================================================

def build_plan(names, initial_positions, cfg=None):
    """Build and internally verify the whole show. Pure; safe to call anywhere.

    ``initial_positions`` are the ``initial_position`` values from
    ``crazyflies.yaml`` -- the same source ``cf.initialPosition`` reads on the
    real stack, so the plan the laptop verifies is the plan the swarm flies.

    Raises ``ValueError`` if the plan violates the piece budget, the
    composition rule, the separation budget or the flight envelope. It is
    meant to be impossible to get an unsafe :class:`Plan` back from this
    function.
    """
    cfg = cfg or ShowConfig()
    PHASE_MARGIN_REF[0] = cfg.phase_margin

    starts = [np.asarray(p, float) for p in initial_positions]
    n = len(starts)
    if n < 3:
        raise ValueError(f'the show needs at least 3 drones, got {n}')

    center = (np.asarray(cfg.room_center, float) if cfg.room_center is not None
              else np.mean([p[:2] for p in starts], axis=0))

    R = cfg.scaled(cfg.ring_radius)
    H = cfg.form_height

    # ------------------------------------------------- gather: pick the n-gon
    # The n-gon's phase is free, so spend it on the leg that is actually tight:
    # the simultaneous gather out of wherever the drones happen to be parked.
    slots, angles, perm, gather_sep = safety.best_phase_ngon(
        [p[:2] for p in starts], center, R, H, figures.ngon_slots)
    # perm[j] is the slot index drone j occupies; angles[perm[j]] is its angle.
    slot_idx = [perm[j] for j in range(n)]
    slot_ang = [angles[k] for k in slot_idx]
    ring = [np.asarray(slots[k], float) for k in slot_idx]

    # --------------------------------------------------------- the figures
    # Alternating slots take the outer / inner counterflow lane. On an odd
    # n-gon "alternating" cannot close, which does not matter: the clearance
    # comes from the radial gap between the lanes, not from the pattern.
    outer = [j for j in range(n) if slot_idx[j] % 2 == 0]

    def build(name, path_fn, duration, boundaries_fn=None):
        paths = [path_fn(j) for j in range(n)]
        bnds = boundaries_fn() if boundaries_fn else None
        trajs = [figures.fit_trajectory(
            p, duration, n_pieces=cfg.pieces[name], boundaries=bnds)
            for p in paths]
        return paths, trajs

    specs = []

    specs.append(('breathe', 0.0, *build(
        'breathe',
        lambda j: figures.breathe_path(
            center, R, slot_ang[j], H, cfg.dur_breathe,
            shrink=cfg.scaled(cfg.breathe_shrink), cycles=2),
        cfg.dur_breathe)))

    specs.append(('carousel', 360.0, *build(
        'carousel',
        lambda j: figures.carousel_path(
            center, R, slot_ang[j], 360.0, H, cfg.dur_carousel),
        cfg.dur_carousel)))

    specs.append(('wave', 360.0, *build(
        'wave',
        lambda j: figures.wave_path(
            center, R, slot_ang[j], 360.0, H, cfg.dur_wave,
            amplitude=cfg.wave_amplitude, phase_frac=slot_idx[j] / n, cycles=2),
        cfg.dur_wave)))

    specs.append(('counterflow', 360.0, *build(
        'counterflow',
        lambda j: figures.counterflow_path(
            center, R,
            cfg.scaled(cfg.lane_outer if j in outer else cfg.lane_inner),
            slot_ang[j],
            360.0 if j in outer else -360.0,
            H, cfg.dur_counterflow, split=cfg.counterflow_split),
        cfg.dur_counterflow,
        lambda: figures.counterflow_boundaries(
            cfg.dur_counterflow, cfg.counterflow_split,
            cfg.pieces['counterflow'] - 2))))

    specs.append(('bloom', 360.0, *build(
        'bloom',
        lambda j: figures.bloom_path(
            center, R, slot_ang[j], 360.0, H, cfg.dur_bloom,
            expand=cfg.scaled(cfg.bloom_expand), rise=cfg.bloom_rise),
        cfg.dur_bloom)))

    offsets, total_pieces = figures.pack_offsets(
        [cfg.pieces[name] for name, *_ in specs])

    figs, by_name = [], {}
    for (name, sweep, paths, trajs), off, tid in zip(
            specs, offsets, range(1, len(specs) + 1)):
        if not figures.composes_in_place(sweep):
            raise ValueError(
                f"figure '{name}' sweeps {sweep} deg, which is not a whole "
                'number of turns: it would not return the drones to their own '
                'slots, so relative=True would translate the formation apart. '
                'See the composition rule in this module docstring.')
        f = Figure(name, tid, off, trajs, paths, trajs[0].duration)
        figs.append(f)
        by_name[name] = f

    # ----------------------------------------------------------- the phases
    # Simple and slow at the top, complex and quick at the bottom. The
    # timescale column is the show's tempo: >1 stretches a figure, <1
    # compresses it. carousel is deliberately reused three times -- the same
    # 6 s turn at 1.15, then reversed at 0.90, then 0.80 -- because reusing one
    # figure is the clearest way an audience reads "the same thing, faster",
    # and because it costs 6 pieces instead of 18.
    script = [
        ('breathe',     1.00, False, 'ring breathes twice - the calm opening'),
        ('carousel',    1.12, False, 'slow rigid turn, one full revolution'),
        ('wave',        1.00, False, 'travelling vertical wave while turning'),
        ('carousel',    0.92, True,  'snap reversal - the same turn, backwards and quicker'),
        ('counterflow', 1.00, False, 'lanes split and counter-rotate THROUGH each other'),
        ('carousel',    0.85, False, 'fastest turn of the show'),
        ('bloom',       1.00, False, 'finale: fling out and up, then snap home'),
    ]

    phases = []
    pos = [p.copy() for p in starts]

    def add(name, kind, duration, ends, **kw):
        phases.append(Phase(name, kind, duration,
                            [p.copy() for p in pos],
                            [np.asarray(e, float) for e in ends], **kw))
        for j in range(n):
            pos[j] = np.asarray(ends[j], float)

    hover = [np.array([s[0], s[1], cfg.takeoff_height]) for s in starts]
    add('takeoff', 'takeoff', cfg.takeoff_duration, hover, goals=hover,
        note=f'all drones to {cfg.takeoff_height:.2f} m over their own start')
    add('settle', 'goto', cfg.settle_duration, hover, goals=hover,
        note='hold, let the Kalman filter settle before anything moves sideways')

    gather_dur = safety.scaled_duration(hover, ring, cfg.trans_avg_speed,
                                        cfg.trans_min_duration)
    add('gather', 'goto', gather_dur, ring, goals=ring,
        note=f'form the {n}-gon, min sep {gather_sep:.2f} m')

    for name, ts, rev, note in script:
        f = by_name[name]
        add(f'{name}{" (reversed)" if rev else ""} x{ts:g}', 'figure',
            f.duration * ts, ring, figure=f, timescale=ts, reverse=rev,
            note=note)

    home = [np.array([s[0], s[1], cfg.home_height]) for s in starts]
    home_dur = safety.scaled_duration(ring, home, cfg.trans_avg_speed,
                                      cfg.trans_min_duration)
    add('return home', 'goto', home_dur, home, goals=home,
        note='back over each drone\'s own initial_position')

    ground = [np.array([s[0], s[1], cfg.land_height]) for s in starts]
    add('land', 'land', cfg.land_duration, ground, goals=ground,
        note=f'{(cfg.home_height - cfg.land_height) / cfg.land_duration:.2f} m/s descent')

    plan = Plan(list(names), starts, np.asarray(center, float), slot_ang,
                figs, phases, gather_sep)

    # ------------------------------------------------------------- verify
    # Sampled over every phase, so it covers the polynomial figures and not
    # just the straight-line legs. Raises rather than warns: the flight script
    # calls build_plan() and there is deliberately no way past a failure.
    bounds, t = [], 0.0
    for ph in phases:
        bounds.append((ph.name, t, t + ph.duration))
        t += ph.duration
    plan.report = safety.check_show(
        plan.sample, plan.duration, n,
        radius=cfg.arena_radius, ceiling=cfg.ceiling, center=center,
        floor=cfg.floor, floor_window=(bounds[0][2], bounds[-1][1]),
        phase_bounds=bounds)
    plan.report['pieces'] = total_pieces
    plan.report['config'] = cfg
    return plan
