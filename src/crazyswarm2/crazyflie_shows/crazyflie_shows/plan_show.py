#!/usr/bin/env python3
"""Verify and describe the show offline. No ROS, no radio, no mocap, no drones.

This is the step that makes the rest safe. It builds the *same*
:class:`~crazyflie_shows.choreography.Plan` object that ``swarm_show`` flies,
samples every drone through every phase, and reports separation, speed,
acceleration, volume and the firmware piece budget. If it refuses, the show
would have been unsafe and ``swarm_show`` would have refused too -- at the
point where the drones are already on the floor waiting.

Run it after every choreography edit, and once on the day, after the fleet has
been placed and ``initial_position`` updated::

    ros2 run crazyflie_shows plan_show                 # reads the installed yaml
    ros2 run crazyflie_shows plan_show --plot          # + a PNG of the paths

It also runs with nothing sourced at all, straight from the source tree, which
is the point -- you can plan a show on a laptop that has never seen the rig::

    PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages \\
        python3 -m crazyflie_shows.plan_show --yaml config/crazyflies.yaml
"""

import argparse
import os
import sys

import numpy as np
import yaml

from crazyflie_shows import choreography, constellation, figures, safety

#: Every show the package can plan: name -> (config class, builder, extra report).
SHOWS = {
    'swarm': (choreography.ShowConfig, choreography.build_plan, None),
    'constellation': (constellation.ConstellationConfig, constellation.build_plan,
                      constellation.report_extras),
}


def load_fleet(yaml_path):
    """Enabled drones and their ``initial_position`` from a crazyflies.yaml.

    Reads exactly what the C++ server reads, in the same order (the server
    connects in lexicographic name order), so the drone indices printed here
    match the ones ``swarm_show`` will use.
    """
    with open(yaml_path) as fh:
        doc = yaml.safe_load(fh)
    robots = doc['robots']
    names = sorted(k for k, v in robots.items() if v.get('enabled'))
    if not names:
        raise SystemExit(f'{yaml_path}: no drones have `enabled: true`')
    return names, [np.array(robots[k]['initial_position'], float) for k in names]


def default_yaml():
    """The installed config if the workspace is sourced, else the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('crazyflie_shows'),
                            'config', 'crazyflies.yaml')
    except Exception:
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(here, 'config', 'crazyflies.yaml')


def bar(value, limit, width=28):
    """A single-line usage bar -- how close a number is to its budget."""
    frac = min(1.0, max(0.0, value / limit)) if limit else 0.0
    fill = int(round(frac * width))
    return '[' + '#' * fill + '.' * (width - fill) + f'] {100 * frac:3.0f}%'


def hints(msg, cfg):
    """Levers that actually apply to the budget that failed.

    Generic advice is worse than none: 'scale it down' is the right answer to
    a room violation and precisely the wrong answer to a separation
    violation, which scaling down makes worse.
    """
    out = []
    if 'separation' in msg:
        out += [f'the show is too SMALL for the {safety.PLAN_SEPARATION:.2f} m '
                'budget - scaling down makes',
                'this worse, not better. Instead:',
                '  ShowConfig.lane_outer / lane_inner   widen the gap',
                '  ShowConfig.ring_radius               grow the n-gon',
                '  ShowConfig.breathe_shrink            contract less',
                f'  (current scale {cfg.scale:g}; below 0.99 this always fails -',
                '   the show is already at its minimum size for 5 drones)']
    if 'speed' in msg or 'accel' in msg:
        out += ['a figure is being asked to move faster than the envelope:',
                '  raise its timescale in the script table in build_plan()',
                '  raise the matching ShowConfig.dur_* to stretch it',
                '  ShowConfig.counterflow_split if the counterflow is at fault',
                '  safety.MAX_SPEED / MAX_ACCEL are conservative first-flight',
                '  values - raising them is a decision, not a fix']
    if 'radius' in msg or 'ceiling' in msg or 'height' in msg:
        out += ['the show does not fit the volume:',
                '  --scale 0.9   shrink every radius (watch separation)',
                '  ShowConfig.bloom_expand / bloom_rise are the widest points']
    if 'at least' in msg and 'drones' in msg:
        out += ['too few drones are `enabled: true` in crazyflies.yaml.',
                'The figures need three to make a ring; with two, the',
                'counterflow has no outer group and the n-gon is a line.']
    if 'floor' in msg:
        out += ['a figure dips too low - ShowConfig.wave_amplitude and',
                '  form_height are what set the lowest point mid-show']
    return out or ['  ShowConfig in choreography.py holds every tunable']


def report(plan, cfg, names, verbose=True):
    """Print the full pre-flight description of a plan."""
    r = plan.report
    n = len(names)
    out = print

    out('')
    out('=' * 74)
    out(f'  SHOW PLAN - {n} drones: ' + ', '.join(names))
    out('=' * 74)

    out('')
    out(f'  centre        ({plan.center[0]:+.3f}, {plan.center[1]:+.3f}) m'
        + ('  [ShowConfig.room_center]' if cfg.room_center is not None
           else '  [centroid of initial_position]'))
    out(f'  ring          radius {cfg.scaled(cfg.ring_radius):.2f} m at '
        f'{cfg.form_height:.2f} m, adjacent slot sep '
        f'{figures.ngon_min_sep(cfg.scaled(cfg.ring_radius), n):.2f} m')
    out(f'  scale         {cfg.scale:g}')
    out('  slot angles   ' + ', '.join(
        f'{nm}={a:.0f}deg' for nm, a in zip(names, plan.slot_angles)))

    out('')
    out('  FIGURES (uploaded before takeoff)')
    out(f'    {"id":>3} {"off":>4} {"pieces":>7}  {"nominal":>8}  '
        f'{"fit err":>9}  name')
    for f in plan.figs:
        err = max(figures.fit_error(t, p) for t, p in zip(f.trajs, f.paths))
        out(f'    {f.traj_id:>3} {f.piece_offset:>4} {len(f.trajs[0].polynomials):>7}'
            f'  {f.duration:>7.1f}s  {err:>8.1e}m  {f.name}')
    out(f'    {"":>3} {"":>4} {r["pieces"]:>7}  total, of '
        f'{figures.MAX_PIECES} in firmware trajectory memory')

    out('')
    out('  TIMELINE')
    out(f'    {"t":>6}  {"dur":>5}  {"sep":>5}  phase')
    t = 0.0
    for ph in plan.phases:
        # min separation within just this phase, so a tight phase is visible
        ts = np.linspace(0.0, ph.duration, max(3, int(ph.duration * 30)))
        pts = np.array([[ph.position(j, u) for u in ts] for j in range(n)])
        sep = safety.sample_min_sep(pts)[0]
        flag = '  <-- tightest' if abs(sep - r['min_sep']) < 1e-6 else ''
        out(f'    {t:>5.1f}s {ph.duration:>5.1f}s {sep:>5.2f}m  {ph.name}{flag}')
        if verbose and ph.note:
            out(f'    {"":>6}  {"":>5}  {"":>5}    {ph.note}')
        t += ph.duration
    out(f'    {t:>5.1f}s  total motion; '
        f'{t + len(plan.phases) * cfg.phase_margin:.1f}s wall clock with '
        f'{cfg.phase_margin:g}s margins')

    out('')
    out('  BUDGETS')
    a, b = r['min_sep_pair']
    out(f'    separation  {r["min_sep"]:5.2f} m  min {safety.PLAN_SEPARATION:.2f} m'
        f'   {bar(safety.PLAN_SEPARATION, r["min_sep"])}'
        f'  ({names[a]}/{names[b]} at t={r["min_sep_time"]:.1f}s)')
    out(f'    {"":>12}          = {safety.MIN_SEPARATION:.2f} m floor'
        f' + {safety.TRACKING_MARGIN:.2f} m measured controller lag'
        ' (safety.TRACKING_MARGIN)')
    out(f'    speed       {r["max_speed"]:5.2f} m/s  max {safety.MAX_SPEED:.2f}'
        f'    {bar(r["max_speed"], safety.MAX_SPEED)}')
    out(f'    accel       {r["max_accel"]:5.2f} m/s2 max {safety.MAX_ACCEL:.2f}'
        f'   {bar(r["max_accel"], safety.MAX_ACCEL)}')
    out(f'    radius      {r["max_radius"]:5.2f} m  max {cfg.arena_radius:.2f} m'
        f'   {bar(r["max_radius"], cfg.arena_radius)}')
    out(f'    height      {r["max_z"]:5.2f} m  max {cfg.ceiling:.2f} m'
        f'   {bar(r["max_z"], cfg.ceiling)}')
    out(f'    floor       {r.get("min_z_cruise", r["min_z"]):5.2f} m  min '
        f'{cfg.floor:.2f} m   {bar(cfg.floor, r.get("min_z_cruise", r["min_z"]))}'
        '  (lowest point between takeoff and landing)')
    out(f'    pieces      {r["pieces"]:5d}     max {figures.MAX_PIECES}'
        f'      {bar(r["pieces"], figures.MAX_PIECES)}')

    out('')
    out('  PLAN OK - every budget satisfied.')
    out('  This is geometry, not flight. Nothing here has been in the air.')
    out('=' * 74)
    out('')


def plot(plan, names, path):
    """Top-down path plot plus the separation-over-time trace."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('  (matplotlib not installed - skipping --plot)')
        return

    r = plan.report
    samples, ts = r['samples'], r['times']
    n = len(names)

    fig, (ax, bx) = plt.subplots(1, 2, figsize=(15, 6.5))
    for j, nm in enumerate(names):
        ax.plot(samples[j, :, 0], samples[j, :, 1], lw=1.0, label=nm)
        ax.plot(*plan.starts[j][:2], 'ks', ms=5)
    ax.plot(*plan.center, 'k+', ms=12)
    ax.set_aspect('equal')
    ax.set_title('paths, top down (squares = initial_position)')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    d = np.full(len(ts), np.inf)
    for a in range(n):
        for b in range(a + 1, n):
            d = np.minimum(d, np.linalg.norm(samples[a] - samples[b], axis=1))
    bx.plot(ts, d, lw=1.2)
    bx.axhline(safety.PLAN_SEPARATION, color='r', ls='--',
               label=f'plan budget {safety.PLAN_SEPARATION:.2f} m')
    bx.axhline(safety.MIN_SEPARATION, color='r', ls=':', alpha=0.7,
               label=f'hard floor {safety.MIN_SEPARATION:.2f} m')
    t = 0.0
    for ph in plan.phases:
        bx.axvline(t, color='k', alpha=0.15)
        bx.text(t + 0.1, d.max() * 0.98, ph.name.split(' x')[0],
                rotation=90, va='top', fontsize=7, alpha=0.6)
        t += ph.duration
    bx.set_ylim(0, max(d.max() * 1.05, 1.0))
    bx.set_title('closest pair over the whole show')
    bx.set_xlabel('show time [s]')
    bx.set_ylabel('min separation [m]')
    bx.legend(fontsize=8)
    bx.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f'  wrote {path}')


def main(default_show='swarm'):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--show', choices=sorted(SHOWS), default=default_show,
                    help=f'which show to plan (default {default_show})')
    ap.add_argument('--bpm', type=float, default=None,
                    help='constellation only: tempo of the beat grid')
    ap.add_argument('--yaml', default=None,
                    help='crazyflies.yaml to read initial_position from')
    ap.add_argument('--scale', type=float, default=None,
                    help='shrink the whole show (ShowConfig.scale)')
    ap.add_argument('--arena-radius', type=float, default=None)
    ap.add_argument('--ceiling', type=float, default=None)
    ap.add_argument('--plot', nargs='?', const='show_plan.png', default=None,
                    help='write a paths + separation plot (default show_plan.png)')
    ap.add_argument('--quiet', action='store_true', help='omit phase notes')
    args = ap.parse_args()

    path = args.yaml or default_yaml()
    names, starts = load_fleet(path)
    print(f'  fleet from {path}')

    cfg_cls, build, extras = SHOWS[args.show]
    cfg = cfg_cls()
    if args.bpm is not None:
        if not hasattr(cfg, 'bpm'):
            raise SystemExit(f'--bpm applies to the constellation show, not {args.show}')
        cfg.bpm = args.bpm
    if args.scale is not None:
        cfg.scale = args.scale
    if args.arena_radius is not None:
        cfg.arena_radius = args.arena_radius
    if args.ceiling is not None:
        cfg.ceiling = args.ceiling

    try:
        plan = build(names, starts, cfg)
    except ValueError as e:
        msg = str(e)
        print('\n  PLAN REJECTED\n')
        for line in msg.splitlines():
            print(f'    {line}')
        print('\n  Nothing will fly until this is fixed. For what failed here:')
        for line in hints(msg, cfg):
            print(f'    {line}')
        print('')
        return 1

    report(plan, cfg, names, verbose=not args.quiet)
    if extras:
        extras(plan, cfg, names)
    if args.plot:
        plot(plan, names, args.plot)
    return 0


if __name__ == '__main__':
    sys.exit(main())


def main_constellation():
    """Entry point ``plan_constellation``: plan_show preset to the constellation."""
    return main(default_show='constellation')
