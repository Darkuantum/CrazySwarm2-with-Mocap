#!/usr/bin/env python3
"""Check the STL pursuit-evasion show offline -- no ROS, no radio.

    ros2 run crazyflie_shows plan_stl                       # 20 seeds, yaml marks
    ros2 run crazyflie_shows plan_stl --seed 1234           # the seed a flight printed
    ros2 run crazyflie_shows plan_stl --against <file.py>   # port still == original?
    python3 -m crazyflie_shows.plan_stl --yaml path/to/crazyflies.yaml

The game never reads mocap, so a seed fixes the whole flight. For each seed
this plays the game (``stl_game.play``) and checks the table that would fly
(``stl_game.verify``): the start move from every drone's ``initial_position``
to its start slot, then every tick of the game and the return leg against a
separation floor. ``stl_show`` runs the same two functions on the seed it is
about to fly, so a seed that passes here is the flight that passes there.

``--against`` is the drift check between this package's copy and the
colleague's script. It loads the original FILE (never imported as a package)
with stub drones, runs it and ``stl_game.fly`` on the same seeds, and diffs
every arm / takeoff / goTo / land / LED call and its time. Run it on every new
version before porting it, and on the old one after any edit to stl_game.
crazyflie_shows/STL.md (package root) has the procedure.

Exit status 0 = every seed passes (and, with ``--against``, both agree).
"""

import argparse
import contextlib
import importlib.util
import io
import sys
import types

import numpy as np
import yaml

from crazyflie_shows import safety, stl_game
from crazyflie_shows.plan_show import default_yaml

# The server's own crazyflies.yaml, found through the package index. A path
# relative to this file only resolved from the source tree, so `ros2 run`
# (which runs the installed copy) failed to find the yaml at all.


def load_pads(path):
    """``{name: (initial_position, enabled)}`` for every robot in the yaml."""
    with open(path) as f:
        robots = yaml.safe_load(f)['robots']
    return {n: (np.asarray(r['initial_position'], float), bool(r.get('enabled')))
            for n, r in robots.items()}


def _names(arg, default):
    return [s.strip() for s in arg.split(',')] if arg else list(default)


# ------------------------------------------------------------------ stubs
# Just enough of crazyflie_py for both the original's main() and
# stl_game.fly(): every call is logged with the stub clock's time.

class _Rec:
    def __init__(self):
        self.t = 0.0
        self.log = []


class _CF:
    def __init__(self, rec, name, pos):
        self.rec, self.prefix = rec, '/' + name
        self.initialPosition = np.asarray(pos, float)

    def _put(self, *entry):
        self.rec.log.append((self.rec.t, self.prefix) + entry)

    def arm(self, arm=True):
        self._put('arm', bool(arm))

    def takeoff(self, targetHeight, duration, groupMask=0):
        self._put('takeoff', float(targetHeight), float(duration))

    def land(self, targetHeight, duration, groupMask=0):
        self._put('land', float(targetHeight), float(duration))

    def goTo(self, goal, yaw, duration, relative=False, groupMask=0):
        self._put('goTo', *(float(v) for v in goal), float(yaw), float(duration))

    def setParam(self, name, value):
        self._put('setParam', name, int(value))


class _Time:
    def __init__(self, rec):
        self.rec = rec

    def sleep(self, d):
        self.rec.t += d

    def time(self):
        return self.rec.t


def _stub_module(rec, pads):
    mod = types.ModuleType('crazyflie_py')

    class Crazyswarm:
        def __init__(self):
            self.timeHelper = _Time(rec)
            self.allcfs = types.SimpleNamespace(
                crazyfliesByName={n: _CF(rec, n, p) for n, (p, _) in pads.items()})
    mod.Crazyswarm = Crazyswarm
    return mod


def run_original(path, seed, pads):
    """Run the original FILE's main() on ``seed`` with stub drones.

    Returns (command log, stdout). The original builds an unseeded generator
    with ``np.random.default_rng()``; it is pointed at ``seed`` for the
    duration of the call and restored after.
    """
    rec = _Rec()
    saved = sys.modules.get('crazyflie_py')
    sys.modules['crazyflie_py'] = _stub_module(rec, pads)
    real_rng = np.random.default_rng
    np.random.default_rng = lambda *a, **k: real_rng(seed)
    out = io.StringIO()
    try:
        spec = importlib.util.spec_from_file_location('_stl_original', path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        with contextlib.redirect_stdout(out):
            mod.main()
    finally:
        np.random.default_rng = real_rng
        if saved is None:
            sys.modules.pop('crazyflie_py', None)
        else:
            sys.modules['crazyflie_py'] = saved
    return rec.log, out.getvalue()


def run_port(game, def_names, intr_names, pads):
    rec = _Rec()
    cf = {n: _CF(rec, n, pads[n][0]) for n in def_names + intr_names}
    stl_game.fly(game, [cf[n] for n in def_names], [cf[n] for n in intr_names],
                 _Time(rec), say=lambda *_: None)
    return rec.log


def compare(a, b, tol=1e-9):
    """None if the two command logs agree, else a description of the first difference."""
    for k, (x, y) in enumerate(zip(a, b)):
        same = (len(x) == len(y) and x[1:3] == y[1:3]
                and all(abs(p - q) <= tol if isinstance(p, float) else p == q
                        for p, q in zip(x, y)))
        if not same:
            return f'call {k}: original {x}\n                    port     {y}'
    if len(a) != len(b):
        return f'the original sent {len(a)} calls, the port {len(b)}'
    return None


# ------------------------------------------------------------------ main

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--yaml', default=default_yaml())
    ap.add_argument('--seeds', type=int, default=20, help='check seeds 0..N-1')
    ap.add_argument('--seed', type=int, help='check this one seed only')
    ap.add_argument('--min-sep', type=float, default=stl_game.MIN_SEP_DEFAULT)
    ap.add_argument('--defenders', help='comma list, default the game\'s own')
    ap.add_argument('--intruders', help='comma list, default the game\'s own')
    ap.add_argument('--against', metavar='ORIGINAL.py',
                    help='also run this original script and diff every command')
    a = ap.parse_args(argv)

    def_names = _names(a.defenders, stl_game.DEFENDER_CF_NAMES)
    intr_names = _names(a.intruders, stl_game.INTRUDER_CF_NAMES)
    pads = load_pads(a.yaml)
    seeds = [a.seed] if a.seed is not None else list(range(a.seeds))

    print(f'\n  STL pursuit-evasion plan  ({a.yaml})')
    print(f'  defenders {",".join(def_names)}   intruders {",".join(intr_names)}')
    missing = [n for n in def_names + intr_names if n not in pads]
    if missing:
        print(f'\n  REFUSE: not in the yaml at all: {", ".join(missing)}\n')
        return 1
    off = [n for n in def_names + intr_names if not pads[n][1]]
    if off:
        print(f'  NOTE: not enabled, so stl_show will refuse until they are: {", ".join(off)}')
    xy = {n: pads[n][0] for n in def_names + intr_names}
    launch = np.array([xy[n][:2] for n in def_names])

    bad = 0
    print(f'\n  floor {a.min_sep:.2f} m   seeds {seeds[0]}..{seeds[-1]}\n')
    print('  seed  outcome                              game    return  closest (pair, when)')
    first = None
    for seed in seeds:
        game = stl_game.play(seed, launch_pads=launch)
        chk = stl_game.verify(game, def_names, intr_names, xy, a.min_sep,
                              max_speed=safety.MAX_SPEED)
        first = first or chk
        flag = 'ok ' if not chk.problems else 'BAD'
        bad += bool(chk.problems)
        print(f'  {seed:>4}  {game.outcome[9:]:<36} {game.game_s:5.1f}s  {game.return_s:5.1f}s  '
              f'{flag} {chk.game_sep:.3f} m {chk.game_pair[0]}-{chk.game_pair[1]}, '
              f'{chk.game_when}')

    print('\n  start move (same every seed, from the yaml marks):')
    print(f'    closest {first.start_sep:.3f} m  {first.start_pair[0]}-{first.start_pair[1]}'
          f'   {"ok" if first.start_sep >= a.min_sep else "BELOW THE FLOOR"}')
    print(f'    longest leg {first.start_leg:.2f} m ({first.start_leg_name}) in '
          f'{stl_game.INIT_MOVE_DURATION:g} s -> peak {first.start_peak:.2f} m/s')
    print(f'  furthest commanded |x| or |y|: {first.max_abs_xy:.2f} m '
          f'(the game clamps to +/-{stl_game.ARENA_X_MAX - stl_game.BOUNDARY_MARGIN:.2f})')
    for line in first.notes:
        print(f'  NOTE {line}')
    start_bad = first.start_sep < a.min_sep
    if start_bad:
        print('\n  The start move is checked from initial_position. If the drones really\n'
              '  sit somewhere else, sync the marks from mocap first (server not running):\n'
              '    ros2 launch crazyflie launch.py server:=False\n'
              '    python3 scripts/sync_initial_positions.py\n'
              '  and re-run this. If they really sit there, re-park them nearer their\n'
              '  start slots (stl_game.DEF_POS_INIT / INTRUDER_POS_INIT).')

    agree = True
    if a.against:
        fp = stl_game.fingerprint(a.against)
        print(f'\n  --against {a.against}')
        if fp == stl_game.SOURCE_SHA256:
            print(f'    sha256 matches {stl_game.SOURCE_NAME}, the version this port '
                  'was made from')
        else:
            print(f'    sha256 {fp[:16]}... is NOT {stl_game.SOURCE_NAME} '
                  f'({stl_game.SOURCE_SHA256[:16]}...): a new version, or an edited one')
        for seed in seeds:
            game = stl_game.play(seed, launch_pads=launch)
            orig, out = run_original(a.against, seed, pads)
            diff = compare(orig, run_port(game, def_names, intr_names, pads))
            orig_outcome = next((s for s in out.splitlines() if s.startswith('Outcome:')), '?')
            if diff or orig_outcome != game.outcome:
                agree = False
                print(f'    seed {seed}: DIFFERENT\n      {diff or "outcome line differs"}')
                print(f'      original: {orig_outcome}\n      port:     {game.outcome}')
                break
        else:
            print(f'    identical on {len(seeds)} seed(s): every command, value and time '
                  f'({len(orig)} calls on the last)')

    ok = not bad and not start_bad and agree
    print(f'\n  {"PLAN OK" if ok else "PLAN REFUSED"}'
          f'{"" if not bad else f"  ({bad}/{len(seeds)} seeds below the floor)"}\n')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
