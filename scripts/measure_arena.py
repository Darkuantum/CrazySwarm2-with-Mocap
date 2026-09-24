#!/usr/bin/env python3
"""Measure the usable mocap volume by carrying a tracked body around it.

``arena_radius`` (2.5 m) and ``ceiling`` (2.0 m) are inherited numbers that
have never been checked against the volume Motive actually covers. They bound
every show and, for the escort, they set how far a piloted VIP may stray
before the ring stops fitting. This measures them.

    # mocap up, server NOT started (nothing here touches a radio)
    ros2 run crazyflie_shows ...        # no -- this is a plain script:
    python3 scripts/measure_arena.py --body wand --duration 120

Then walk the body slowly through the whole space you intend to fly in:
the corners, the edges, floor height to as high as you can reach, and
deliberately into the places you suspect are bad. Two minutes is plenty.

**What is being measured is tracking coverage, not the room.** The answer is
the region where Motive keeps a lock, which is smaller than the floor plan and
is what the flight code actually needs. Carry the body at the height the
drones fly at, not at your waist, and move slowly -- a body whipped through a
corner drops out from motion blur, not from being outside the volume.

What to carry: the easiest is a powered-off Crazyflie with its own markers, so
what you measure is exactly what Motive will track in flight. A marker wand
works too, as long as it is a defined rigid body in Motive; a single loose
marker is not.

Nothing is written. The script prints the numbers and the exact config lines
to change, because choosing an arena is a judgement about how much margin you
want to the net, not an arithmetic result.
"""

import argparse
import math
import sys
from collections import deque

import numpy as np

# --- pure analysis (no ROS, so it can be tested on the ground) --------------

#: A gap longer than this between samples of the body counts as a dropout --
#: Motive lost the lock. At 50 Hz streaming a healthy stream is 20 ms apart;
#: 0.15 s is ~7 missed frames, well past jitter and well short of a pause for
#: breath.
DROPOUT_S = 0.15

#: How far inside the last-known-good radius to recommend flying. The drones
#: overshoot their plan (crazyflie_shows.safety.TRACKING_MARGIN is 0.10 m for
#: separation) and a body carried by hand samples the boundary coarsely.
EDGE_MARGIN = 0.30


def analyse(samples, centre=None):
    """Summarise (t, x, y, z) samples of one rigid body.

    Returns a dict. ``clean_radius`` is the headline: the largest horizontal
    radius from ``centre`` at which the body was still tracked continuously,
    i.e. the smallest radius at which a dropout began. Everything beyond that
    is somewhere Motive lost it at least once.
    """
    if len(samples) < 2:
        return {'n': len(samples), 'error': 'not enough samples'}
    t = np.array([s[0] for s in samples], float)
    p = np.array([[s[1], s[2], s[3]] for s in samples], float)
    if centre is None:
        centre = np.array([(p[:, 0].max() + p[:, 0].min()) / 2.0,
                           (p[:, 1].max() + p[:, 1].min()) / 2.0])
    centre = np.asarray(centre, float)[:2]
    r = np.linalg.norm(p[:, :2] - centre, axis=1)

    gaps = np.diff(t)
    dropouts = np.flatnonzero(gaps > DROPOUT_S)
    # The radius at which each dropout STARTED -- the last place it was seen.
    drop_r = r[dropouts] if len(dropouts) else np.array([])
    clean = float(drop_r.min()) if len(drop_r) else float(r.max())

    return {
        'n': len(samples),
        'seconds': float(t[-1] - t[0]),
        'centre': centre,
        'x': (float(p[:, 0].min()), float(p[:, 0].max())),
        'y': (float(p[:, 1].min()), float(p[:, 1].max())),
        'z': (float(p[:, 2].min()), float(p[:, 2].max())),
        'r_max': float(r.max()),
        'clean_radius': clean,
        'dropouts': len(dropouts),
        'dropout_radii': np.sort(drop_r),
        'lost_seconds': float(gaps[dropouts].sum()) if len(dropouts) else 0.0,
        'rate_hz': float(len(t) / max(t[-1] - t[0], 1e-6)),
    }


def report(a, cfg_radius=2.5, cfg_ceiling=2.0, cfg_floor=0.3):
    """Print the summary and the config lines it implies. Returns suggestions."""
    if 'error' in a:
        print(f'\n  {a["error"]} ({a["n"]} samples) -- was the body tracked at all?\n')
        return None
    print(f'\n  {a["n"]} samples over {a["seconds"]:.0f} s '
          f'({a["rate_hz"]:.0f} Hz average)')
    print(f'  centre used     [{a["centre"][0]:+.3f}, {a["centre"][1]:+.3f}]')
    print(f'  x range         {a["x"][0]:+.2f} .. {a["x"][1]:+.2f} m')
    print(f'  y range         {a["y"][0]:+.2f} .. {a["y"][1]:+.2f} m')
    print(f'  z range         {a["z"][0]:+.2f} .. {a["z"][1]:+.2f} m')
    print(f'  furthest seen   {a["r_max"]:.2f} m from centre')
    if a['dropouts']:
        radii = ', '.join(f'{v:.2f}' for v in a['dropout_radii'][:8])
        print(f'  dropouts        {a["dropouts"]}, losing {a["lost_seconds"]:.1f} s '
              f'in total, first lost at radii: {radii} m')
    else:
        print('  dropouts        none -- so this is a floor on the volume, not '
              'its edge; you did not reach a boundary')

    found_edge = a['dropouts'] > 0
    suggest_r = max(a['clean_radius'] - EDGE_MARGIN, 0.0)
    suggest_ceil = max(a['z'][1] - EDGE_MARGIN, 0.0)

    if found_edge:
        print(f'\n  tracked cleanly to {a["clean_radius"]:.2f} m, and lost the '
              f'body beyond that; with a {EDGE_MARGIN:.2f} m margin:\n')
        print(f'    arena_radius: {suggest_r:.2f}      # now {cfg_radius:.2f}')
    else:
        # No dropout means no boundary was found: every number here is a LOWER
        # BOUND on the volume, not a measurement of it. Reporting it as an
        # arena would quietly shrink the flyable space to wherever the person
        # happened to walk -- and, worse, could read as "the volume is smaller
        # than configured" when the truth is that nobody went to the edge.
        print(f'\n  no dropouts: the body stayed tracked everywhere it went, '
              f'out to {a["r_max"]:.2f} m.\n  That is a LOWER BOUND on the '
              'volume, not its edge.\n')
        print(f'    arena_radius: at least {min(suggest_r, a["r_max"]):.2f}'
              f'   # now {cfg_radius:.2f}')
        print('\n  To find the real edge, walk it out until Motive drops the '
              'body and re-run.')

    # Altitude: only claim a ceiling or a floor where the body actually went.
    z_lo, z_hi = a['z']
    print(f'\n    z covered:    {z_lo:.2f} .. {z_hi:.2f} m')
    if found_edge:
        print(f'    ceiling:      {suggest_ceil:.2f}      # now {cfg_ceiling:.2f}')
    else:
        print(f'    ceiling:      at least {suggest_ceil:.2f}   '
              f'# now {cfg_ceiling:.2f}')
    if z_lo > cfg_floor + 0.1:
        print(f'    floor:        not measured -- the body never went below '
              f'{z_lo:.2f} m, so the configured {cfg_floor:.2f} m stands')
    else:
        print(f'    floor:        {max(z_lo + 0.1, cfg_floor):.2f}      '
              f'# now {cfg_floor:.2f}')

    print("""
  These are what the mocap can SEE. Check them against what the room will
  let you fly -- the net, the furniture, anyone standing in it -- and take
  whichever is smaller. Nothing was written; edit the yaml yourself.
""")
    if found_edge and suggest_r > cfg_radius:
        print(f'  The volume is bigger than the configured {cfg_radius:.2f} m. '
              'For the escort\n  that buys keep-in radius for a piloted VIP '
              'one-for-one (ESCORT.md).\n')
    elif found_edge and suggest_r < cfg_radius:
        print(f'  *** The volume is SMALLER than the configured '
              f'{cfg_radius:.2f} m. Anything planned\n      against the old '
              'number may already sit outside the tracked space --\n      '
              're-run plan_show, plan_constellation and plan_escort.\n')
    return {'arena_radius': suggest_r, 'ceiling': suggest_ceil,
            'found_edge': found_edge}


# --- the ROS half ----------------------------------------------------------

def collect(body, topic, duration):
    """Samples of one rigid body from /poses. Needs rclpy and a live mocap."""
    import rclpy
    from rclpy.node import Node
    from motion_capture_tracking_interfaces.msg import NamedPoseArray

    samples = deque()
    seen_names = set()

    class Collector(Node):
        def __init__(self):
            super().__init__('measure_arena')
            self.create_subscription(NamedPoseArray, topic, self.cb, 50)
            self.t0 = self.get_clock().now().nanoseconds * 1e-9

        def cb(self, msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            for np_ in msg.poses:
                seen_names.add(np_.name)
                if np_.name == body:
                    p = np_.pose.position
                    samples.append((now, p.x, p.y, p.z))

    rclpy.init()
    node = Collector()
    print(f'  listening on {topic} for rigid body {body!r} -- carry it around '
          f'the volume for {duration:.0f} s')
    print('  (slowly, at flight height, into the corners and the bad spots)')
    end = node.t0 + duration
    last_note = 0.0
    try:
        while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = node.get_clock().now().nanoseconds * 1e-9
            if now - last_note > 5.0:
                last_note = now
                left = end - now
                print(f'\r  {len(samples)} samples, {left:.0f} s left   ',
                      end='', flush=True)
    except KeyboardInterrupt:
        print('\n  stopped early')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print()
    if not samples and seen_names:
        print(f'\n  nothing named {body!r} on {topic}. Bodies seen: '
              f'{", ".join(sorted(seen_names))}\n')
    elif not samples:
        print(f'\n  nothing at all on {topic} -- is the mocap node running?\n')
    return list(samples)


def selftest():
    """Check the analysis on synthetic data, with no mocap anywhere near."""
    # A body carried in a 2.0 m circle at 50 Hz, dropping out past 1.8 m.
    t, out = 0.0, []
    for k in range(2000):
        ang = k * 0.01
        r = 1.0 + 1.2 * math.sin(k * 0.002)          # breathes 1.0 .. 2.2 m
        t += 0.02
        if r > 1.8:                                   # lost out there
            continue
        out.append((t, r * math.cos(ang), r * math.sin(ang), 1.2))
    a = analyse(out, centre=(0.0, 0.0))
    assert a['dropouts'] > 0, a
    assert 1.7 < a['clean_radius'] <= 1.85, a['clean_radius']
    print(f'  selftest OK: clean radius {a["clean_radius"]:.2f} m from '
          f'{a["n"]} synthetic samples, {a["dropouts"]} dropouts')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--body', default='wand',
                    help='rigid-body name in Motive to follow')
    ap.add_argument('--topic', default='/poses')
    ap.add_argument('--duration', type=float, default=120.0)
    ap.add_argument('--centre', nargs=2, type=float, metavar=('X', 'Y'),
                    help='measure radii from here (default: the midpoint of '
                         'what was seen)')
    ap.add_argument('--save', metavar='CSV', help='write the raw samples')
    ap.add_argument('--load', metavar='CSV', help='analyse a saved run instead')
    ap.add_argument('--selftest', action='store_true',
                    help='check the analysis on synthetic data and exit')
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if args.load:
        samples = [tuple(float(v) for v in line.split(','))
                   for line in open(args.load) if not line.startswith('#')]
    else:
        samples = collect(args.body, args.topic, args.duration)
    if args.save and samples:
        with open(args.save, 'w') as fh:
            fh.write('# t,x,y,z\n')
            for s in samples:
                fh.write(','.join(f'{v:.4f}' for v in s) + '\n')
        print(f'  wrote {len(samples)} samples to {args.save}')

    a = analyse(samples, centre=args.centre)
    report(a)
    return 0


if __name__ == '__main__':
    sys.exit(main())
