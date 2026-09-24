#!/usr/bin/env python3
"""Verify the escort demo offline. No ROS, no radio, no mocap, no drones.

Same idea as ``plan_show``: build and run the *same* controller
``escort_show`` flies, against a simulated VIP and a scripted adversary, and
refuse if any separation, speed or boundary is violated. Unlike the shows,
the escort is reactive -- there is no fixed trajectory to sample -- so this
integrates the control law at ``rate_hz`` and measures what it produces.

    ros2 run crazyflie_shows plan_escort                 # static VIP + walk
    ros2 run crazyflie_shows plan_escort --walk 0.3      # a specific walk speed
    ros2 run crazyflie_shows plan_escort --sweep         # fastest walk it holds
    ros2 run crazyflie_shows plan_escort --plot /tmp/escort.png

It also runs straight from the source tree with nothing sourced::

    python3 -m crazyflie_shows.plan_escort

What this does NOT prove
------------------------
Controller lag. ``plan_show`` carries a measured TRACKING_MARGIN because
flown separations came out ~0.06 m tighter than planned; nothing equivalent
has been measured for the escort yet, so the margins here are the planned
ones. Fly it in sim and compare before trusting the numbers on hardware.
"""

import argparse
import sys

import numpy as np

from crazyflie_shows import escort


def simulate(cfg, script, walk_speed=0.0, duration=None, walk_bearing=90.0,
             lag_tau=0.25):
    """Run the control law against a scripted encounter. Returns a trace dict.

    The drones are modelled as a first-order lag on the commanded setpoint
    (``lag_tau``), not as perfect followers -- a perfect follower would make
    every separation exactly what the geometry says and prove nothing about
    the clamps. This is a stand-in for real tracking, NOT a measured value.
    """
    duration = script.duration if duration is None else duration
    dt = 1.0 / cfg.rate_hz
    n_steps = int(round(duration / dt))

    p_vip = escort.vip_home(cfg)
    walk = np.array([np.cos(np.radians(walk_bearing)),
                     np.sin(np.radians(walk_bearing)), 0.0]) * walk_speed

    # defenders start on the ring, as they will after the gather leg
    ctrl_start = escort.ring_targets(p_vip, 0.0, cfg)
    ctrl = escort.EscortController(cfg, ctrl_start, p_vip)
    pos = ctrl_start.copy()

    tr = {'t': [], 'vip': [], 'adv': [], 'def': [], 'psi': [], 'engaged': [],
          'clamped': [], 'slot': []}
    for k in range(n_steps):
        t = k * dt
        # the VIP walks, and is turned back at the arena edge rather than
        # marched through the net
        p_next = p_vip + walk * dt
        r = p_next[:2] - np.array(cfg.room_center)
        if np.linalg.norm(r) > cfg.arena_radius - cfg.ring_radius:
            walk = -walk
            p_next = p_vip + walk * dt
        p_vip = p_next

        p_adv = script.target(t, p_vip)
        sp, info = ctrl.step(dt, p_vip, p_adv, pos)
        # first-order lag towards the commanded setpoint
        pos += (sp - pos) * min(dt / max(lag_tau, 1e-3), 1.0)

        tr['t'].append(t)
        tr['vip'].append(p_vip.copy())
        tr['adv'].append(p_adv.copy())
        tr['def'].append(pos.copy())
        # the ideal ring, for the formation-error measure: a run where the
        # guard rescues every separation but the ring has stopped being a ring
        # is a FAILED escort, and nothing else here would notice
        ideal = escort.ring_targets(p_vip, info['psi'], cfg)
        tr['slot'].append(np.array([ideal[ctrl.slot_of(i)] for i in range(len(pos))]))
        tr['psi'].append(info['psi'])
        tr['engaged'].append(info['engaged'])
        tr['clamped'].append(info['clamped'])
    for k in ('t', 'vip', 'adv', 'def', 'psi', 'engaged', 'slot'):
        tr[k] = np.array(tr[k])
    return tr


def measure(tr, cfg):
    """Worst-case numbers over a trace."""
    d = tr['def']                                   # (T, n, 3)
    dt = tr['t'][1] - tr['t'][0] if len(tr['t']) > 1 else 1.0
    pair = np.inf
    for i in range(d.shape[1]):
        for j in range(i + 1, d.shape[1]):
            pair = min(pair, float(np.min(np.linalg.norm(d[:, i] - d[:, j], axis=1))))
    vip = float(np.min(np.linalg.norm(d[:, :, :2] - tr['vip'][:, None, :2], axis=2)))
    adv = float(np.min(np.linalg.norm(d[:, :, :2] - tr['adv'][:, None, :2], axis=2)))
    speed = float(np.max(np.linalg.norm(np.diff(d, axis=0), axis=2) / dt)) if len(d) > 1 else 0.0
    radial = np.linalg.norm(d[:, :, :2] - np.array(cfg.room_center), axis=2)
    # how well did the ring end up facing the adversary while engaged?
    face = np.nan
    if tr['engaged'].any():
        want = np.arctan2(tr['adv'][:, 1] - tr['vip'][:, 1],
                          tr['adv'][:, 0] - tr['vip'][:, 0])
        err = np.abs(escort.wrap_pi(tr['psi'] - want))[tr['engaged']]
        # the tail of each engagement is what matters -- the start is the slew
        face = float(np.degrees(np.median(err)))
    # A speed or accel clamp is routine -- that is the slew doing its job. A
    # SEPARATION clamp is not: it means the geometry put a drone somewhere the
    # guard had to rescue, and the geometry is what plan_escort is meant to
    # prove. Count them apart or the interesting one hides in the noise.
    sep_tags = {'vip', 'adversary', 'defender', 'arena', 'altitude'}
    slew = sum(1 for c in tr['clamped']
               if any(set(r) & {'speed', 'accel'} for r in c.values()))
    sep = sum(1 for c in tr['clamped']
              if any(set(r) & sep_tags for r in c.values()))
    # Formation error: how far each drone is from the slot it is supposed to
    # hold. Measured after the first 2 s, which is the gather transient.
    skip = int(2.0 / dt)
    slot_err = (float(np.max(np.linalg.norm(d[skip:] - tr['slot'][skip:], axis=2)))
                if len(d) > skip else 0.0)
    return {'slot_err': slot_err, 'pair_sep': pair, 'vip_dist': vip,
            'adv_sep': adv, 'peak_speed': speed,
            'max_radius': float(np.max(radial)), 'max_alt': float(np.max(d[:, :, 2])),
            'engaged_frac': float(np.mean(tr['engaged'])), 'face_err_deg': face,
            'slew_steps': slew, 'sep_steps': sep, 'steps': len(tr['t'])}


def verdict(m, cfg):
    """Violations implied by a measurement dict."""
    bad = []
    # 0.5 m is a third of the ring radius: past that the three drones are no
    # longer recognisably a ring around anybody, whatever the separations say.
    if m['slot_err'] > 0.5:
        bad.append(f'formation error {m["slot_err"]:.2f} m -- the drones are '
                   'not holding the ring (the guard may still be keeping every '
                   'separation legal, which is not the same as escorting)')
    if m['pair_sep'] < cfg.min_pair_sep:
        bad.append(f'defender-defender {m["pair_sep"]:.2f} m < '
                   f'{cfg.min_pair_sep:.2f} m')
    if m['vip_dist'] < cfg.min_vip_dist - 0.05:
        bad.append(f'defender-VIP {m["vip_dist"]:.2f} m < '
                   f'{cfg.min_vip_dist:.2f} m')
    if m['adv_sep'] < cfg.min_adv_sep - 0.05:
        bad.append(f'defender-adversary {m["adv_sep"]:.2f} m < '
                   f'{cfg.min_adv_sep:.2f} m')
    if m['peak_speed'] > cfg.v_max * 1.05:
        bad.append(f'peak speed {m["peak_speed"]:.2f} m/s > v_max '
                   f'{cfg.v_max:.2f} m/s')
    if m['max_radius'] > cfg.arena_radius + 0.01:
        bad.append(f'a defender reaches {m["max_radius"]:.2f} m from room '
                   f'centre, outside arena_radius {cfg.arena_radius:.2f} m')
    if m['max_alt'] > cfg.ceiling:
        bad.append(f'altitude {m["max_alt"]:.2f} m over ceiling {cfg.ceiling:.2f} m')
    return bad


def report_case(name, cfg, script, walk, verbose=True):
    """Simulate one case and print its numbers. Returns (measurements, problems)."""
    tr = simulate(cfg, script, walk_speed=walk)
    m = measure(tr, cfg)
    bad = verdict(m, cfg)
    if verbose:
        print(f'  {name}')
        print(f'    separations   defender-defender {m["pair_sep"]:.2f} m   '
              f'to VIP {m["vip_dist"]:.2f} m   to adversary {m["adv_sep"]:.2f} m')
        print(f'    peak speed    {m["peak_speed"]:.2f} m/s (cap {cfg.v_max:.2f})')
        print(f'    formation     worst slot error {m["slot_err"]:.2f} m')
        print(f'    volume        {m["max_radius"]:.2f} m radius, '
              f'{m["max_alt"]:.2f} m up')
        print(f'    blocking      engaged {m["engaged_frac"] * 100:.0f}% of the '
              f'run, ring faced the threat to '
              f'{m["face_err_deg"]:.0f} deg (median)')
        print(f'    guard         slew clamp on {m["slew_steps"]}/{m["steps"]} '
              f'steps (routine), separation clamp on {m["sep_steps"]}'
              f'{"  <- geometry, not the guard, should be doing this" if m["sep_steps"] else ""}')
        for b in bad:
            print(f'    *** {b}')
    return m, bad


def sweep(cfg, script, hi=1.6, step=0.05):
    """Fastest walk speed at which the escort still holds. None if even 0 fails."""
    ok = None
    v = 0.0
    while v <= hi + 1e-9:
        _, bad = report_case('', cfg, script, v, verbose=False)
        if bad:
            break
        ok = v
        v += step
    return ok


def plot(tr, cfg, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.add_patch(plt.Circle(cfg.room_center, cfg.arena_radius, fill=False,
                            ls='--', color='0.7'))
    ax.plot(tr['vip'][:, 0], tr['vip'][:, 1], 'k-', lw=2, label='VIP')
    ax.plot(tr['adv'][:, 0], tr['adv'][:, 1], 'r-', lw=1.5, label='adversary')
    for i in range(tr['def'].shape[1]):
        ax.plot(tr['def'][:, i, 0], tr['def'][:, i, 1], lw=1, label=f'defender {i}')
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title('escort demo, plan view')
    fig.savefig(path, dpi=120, bbox_inches='tight')
    print(f'  wrote {path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--walk', type=float, default=0.3,
                    help='VIP walking speed for the moving case, m/s')
    ap.add_argument('--radius', type=float, help='override ring_radius, m')
    ap.add_argument('--height', type=float, help='override defender height, m')
    ap.add_argument('--vmax', type=float, help='override the speed cap, m/s')
    ap.add_argument('--phase-rate', type=float, help='override ring turn rate, rad/s')
    ap.add_argument('--sweep', action='store_true',
                    help='find the fastest walk the escort holds')
    ap.add_argument('--plot', metavar='PNG', help='write a plan-view plot')
    args = ap.parse_args()

    cfg = escort.EscortConfig()
    for attr, val in (('ring_radius', args.radius), ('height', args.height),
                      ('v_max', args.vmax), ('phase_rate', args.phase_rate)):
        if val is not None:
            setattr(cfg, attr, val)
    script = escort.AdversaryScript(height=cfg.height)

    print('\n  ESCORT DEMO - offline check\n')
    print(f'  ring          {cfg.n_defenders} defenders at {cfg.ring_radius:.2f} m, '
          f'{cfg.height:.2f} m up, slots '
          f'{2 * cfg.ring_radius * np.sin(np.pi / cfg.n_defenders):.2f} m apart')
    print(f'  speeds        cap {cfg.v_max:.2f} m/s, ring turns at '
          f'{cfg.phase_rate:.2f} rad/s = '
          f'{cfg.ring_radius * cfg.phase_rate:.2f} m/s at the slot')
    print(f'  VIP budget    {escort.max_vip_speed(cfg):+.2f} m/s left to follow '
          'a walking person')
    reach = float(np.hypot(*cfg.vip_offset)) + cfg.ring_radius
    print(f'  arena margin  ring reaches {reach:.2f} m of the '
          f'{cfg.arena_radius:.2f} m arena '
          f'({cfg.arena_radius - reach:+.2f} m spare)\n')

    problems = 0
    static_only = escort.check_config(cfg, moving_vip=False)
    moving = escort.check_config(cfg, moving_vip=True)
    for b in static_only:
        print(f'  *** CONFIG: {b}')
        problems += 1
    for b in [x for x in moving if x not in static_only]:
        print(f'  --- CONFIG (moving VIP only): {b}')
    for b in script.check(cfg, escort.vip_home(cfg)):
        print(f'  *** SCRIPT: {b}')
        problems += 1
    if static_only or moving or script.check(cfg, escort.vip_home(cfg)):
        print()

    m_static, bad = report_case('static VIP point', cfg, script, 0.0)
    problems += len(bad)
    print()
    m_walk, bad_walk = report_case(f'VIP walking at {args.walk:.2f} m/s',
                                   cfg, script, args.walk)
    print()

    if args.sweep:
        best = sweep(cfg, script)
        if best is None:
            print('  sweep         even a STATIONARY VIP fails -- fix the config '
                  'first\n')
        else:
            print(f'  sweep         holds up to a {best:.2f} m/s walk with these '
                  f'numbers\n')

    if args.plot:
        plot(simulate(cfg, script, walk_speed=args.walk), cfg, args.plot)

    if problems:
        print(f'  REFUSED: {problems} problem(s) with the STATIC case. Fix the '
              'config before flying anything.\n')
        return 1
    if bad_walk:
        print('  static case OK; the walking case is NOT cleared with these '
              'numbers.\n  That is the open decision (ESCORT.md): raise v_max, '
              'slow the ring,\n  shrink the radius, or walk slower. Stages 1-2 '
              'can fly as-is.\n')
        return 0
    print('  OK - static and walking cases both clear.\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
