#!/usr/bin/env python3
"""Fly the constellation show: five drones drawing shapes, on a beat, with lights.

A ~76 s arch through a pentagon, an arrow, a pyramid and a spiral staircase
and back, built from the research in reference/complex-shows-report.html.
Every leg is verified in plan view before anything is armed.

    ros2 launch crazyflie_shows show_launch.py backend:=sim
    ros2 run crazyflie_shows constellation_show --ros-args -p use_sim_time:=true

    # hardware: stack up, scan every address, preflight GUI, plan_constellation, THEN
    ros2 run crazyflie_shows constellation_show

All geometry, timing and verification live in :mod:`constellation`, which is
pure -- ``plan_constellation`` (or ``plan_show --show constellation``) checks
the same object this flies. This file adds three things to ``swarm_show``'s
flight loop, and reuses its preflight checks unchanged:

* **Beat-locked scheduling.** Each phase is commanded at an absolute time,
  ``t0 + sum of the slots before it``, not "after the previous sleep". Command
  latency therefore never accumulates, and the show stays on the beat grid a
  music track can follow. A phase that starts late is reported.
* **Light cues** on the bottom Color LED deck, sent *after* the motion command
  (``setParam`` is fire-and-forget, so a slow cue cannot delay a drone). On by
  default on hardware, off in sim (the sim has no such parameter).
* **A staged abort.** If anything goes wrong after arming -- an exception, or
  Ctrl-C -- the drones are told to land where they are and are then disarmed,
  instead of being left flying their last command. The E-STOP (console,
  preflight GUI ``e``) remains the answer to a drone out of control; this is
  the answer to a script that stopped.

Parameters (``--ros-args -p name:=value``)
------------------------------------------
======================  =======  =====================================
``dry_run``             false    plan + preflight checks, then exit. Never arms,
                                 never uploads. The rehearsal.
``lights``              true*    light cues (*false under use_sim_time)
``bpm``                 120.0    tempo of the beat grid (re-plans)
``check_placement``     true     compare live pose to initial_position
``placement_tol``       0.25     m, how far off a drone may be
``scale``               0.95     grow the whole show (ConstellationConfig.scale)
``arena_radius``        1.90     m, = ARENA_RADIUS_PLAN (tested 2.00 less
                                 the 0.10 tracking margin), NOT the old 2.5
``ceiling``             2.32     m, = CEILING_CENTRE_TESTED 2.42 less the
                                 same margin, NOT the old 2.0
``use_sim_time``        false    REQUIRED under backend:=sim, NEVER on
                                 hardware
======================  =======  =====================================
"""

import sys
import time

import numpy as np
from crazyflie_py import Crazyswarm

from crazyflie_shows import constellation, plan_show
# The abort machinery and the supervisor check used to be defined HERE, which
# is why they were the constellation's alone. They are re-exported below so
# that `from crazyflie_shows.constellation_show import ShowAborted, ...` keeps
# working, but new code should import them from their own modules.
from crazyflie_shows.abort import (ABORT_LAND_DURATION, ShowAborted,  # noqa: F401
                                   abort_land, take_signals)
from crazyflie_shows.preflight import (check_supervisor,  # noqa: F401
                                       report_supervisor)
from crazyflie_shows.swarm_show import _param, check_placement

LED_PARAM = 'colorLedBot.wrgb8888'


def cue(allcfs, cfs, lights):
    """Send one light cue. Never raises: lights must not be able to stop a show."""
    if lights is None:
        return
    try:
        if isinstance(lights, (list, tuple)):
            for cf, v in zip(cfs, lights):
                cf.setParam(LED_PARAM, int(v))
        else:
            allcfs.setParam(LED_PARAM, int(lights))
    except Exception as e:                          # noqa: BLE001
        print(f'    (light cue failed, show continues: {e})', flush=True)


def command(allcfs, cfs, cfg, ph):
    """The motion command for one phase -- identical to swarm_show's loop."""
    if ph.kind == 'takeoff':
        allcfs.takeoff(targetHeight=cfg.takeoff_height, duration=ph.duration)
    elif ph.kind == 'land':
        allcfs.land(targetHeight=cfg.land_height, duration=ph.duration)
    elif ph.kind == 'goto':
        # Unicast, absolute goals: allcfs.goTo is broadcast but relative-only.
        for j, cf in enumerate(cfs):
            cf.goTo(ph.goals[j], 0.0, ph.duration)
    elif ph.kind == 'figure':
        # One broadcast so all drones start the figure on the same radio frame.
        allcfs.startTrajectory(ph.figure.traj_id, timescale=ph.timescale,
                               reverse=ph.reverse, relative=True)
    else:
        raise RuntimeError(f'unknown phase kind {ph.kind!r}')


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    node = allcfs
    cfs = allcfs.crazyflies

    sim = bool(_param(node, 'use_sim_time', False))
    dry_run = bool(_param(node, 'dry_run', False))
    lights_on = bool(_param(node, 'lights', not sim))
    want_placement = bool(_param(node, 'check_placement', True))
    tol = float(_param(node, 'placement_tol', 0.25))

    names = [cf.prefix.lstrip('/') for cf in cfs]
    starts = [np.array(cf.initialPosition, float) for cf in cfs]

    cfg = constellation.ConstellationConfig()
    cfg.bpm = float(_param(node, 'bpm', cfg.bpm))
    cfg.scale = float(_param(node, 'scale', cfg.scale))
    cfg.arena_radius = float(_param(node, 'arena_radius', cfg.arena_radius))
    cfg.ceiling = float(_param(node, 'ceiling', cfg.ceiling))

    # build_plan verifies as it builds and raises on any violation, so there is
    # no path from here to an armed drone flying an unchecked plan.
    plan = constellation.build_plan(names, starts, cfg)
    plan_show.report(plan, cfg, names)
    constellation.report_extras(plan, cfg, names)

    # ------------------------------------------------------------ preflight
    if want_placement and not sim:
        print('  PLACEMENT CHECK (live pose vs initial_position)')
        check_placement(node, cfs, names, plan, tol)
        print('    all drones within tolerance\n')
    elif sim:
        print('  placement check skipped: backend:=sim never publishes '
              '/cfX/pose\n')
    else:
        print('  *** placement check DISABLED by parameter ***\n')

    # ------------------------------------------------- supervisor go/no-go
    if not sim:
        if not report_supervisor(node, names):
            return 1

    print(f'  lights: {"ON" if lights_on else "off"}'
          f'{" (sim has no LED deck)" if sim and not lights_on else ""}')

    # The rehearsal stops HERE, not before the checks above: a rehearsal whose
    # answer is only "the geometry is fine" tells you nothing about the rig, and
    # the checks are all read-only. Everything past this point touches a drone.
    if dry_run:
        print('\n  dry_run - plan and preflight checks done; nothing uploaded, '
              'nothing armed.\n')
        return 0

    # -------------------------------------------------------------- upload
    #
    # This is the slow part and it LOOKS like a hang: every piece of every
    # figure goes to every drone as unicast packets over one radio, with no
    # progress from the library. MEASURED on this rig 2026-10-02: 55 s for
    # 4 figures x 5 drones, and one weak link (cf1) took 5 s of that on a
    # single figure. An operator who reads silence as a crash kills the show
    # here -- which happened -- so print a drone at a time and say how long it
    # should take, and say plainly that nothing is armed yet.
    n_t = len(plan.figs) * len(cfs)
    print(f'  uploading {len(plan.figs)} figures to {len(cfs)} drones '
          f'({plan.report["pieces"]} pieces each) - {n_t} transfers over ONE radio')
    print(f'  this takes ~{max(10, int(round(n_t * 2.7))):d} s and prints as it goes. '
          'NOTHING IS ARMED until "SHOW START";')
    print('  do not interrupt - a part-uploaded fleet has to be re-uploaded from '
          'the start.', flush=True)
    t_up = time.time()
    for i, f in enumerate(plan.figs, 1):
        t_f = time.time()
        print(f'    [{i}/{len(plan.figs)}] id {f.traj_id} @ offset '
              f'{f.piece_offset:>2}  {f.name:12} ', end='', flush=True)
        for j, cf in enumerate(cfs):
            cf.uploadTrajectory(f.traj_id, f.piece_offset, f.trajs[j])
            print(f'{names[j]} ', end='', flush=True)
        print(f' {time.time() - t_f:4.1f}s', flush=True)
    print(f'  uploads done in {time.time() - t_up:.0f}s', flush=True)

    # ----------------------------------------------------------------- fly
    armed = False
    try:
        take_signals()
        # BEFORE the loop, not after: take_signals() is already installed, so a
        # signal arriving between the first and last arm() would otherwise
        # raise with armed still False, skip the abort entirely, and leave part
        # of the fleet armed. Disarming an already-disarmed drone is harmless.
        armed = True
        for cf in cfs:
            cf.arm(True)
        timeHelper.sleep(1.0)

        t0 = timeHelper.time()
        beat = plan.report['beat']
        print(f'\n  SHOW START - {plan.report["wall"]:.1f}s, {cfg.bpm:g} BPM '
              '(start the track now)\n', flush=True)

        t_next = t0
        for ph in plan.phases:
            wait = t_next - timeHelper.time()
            if wait > 0:
                timeHelper.sleep(wait)
            late = timeHelper.time() - t_next
            k = int(round((t_next - t0) / beat))
            print(f'  [{k // 4 + 1:>3}.{k % 4 + 1}  t+{t_next - t0:5.1f}s] {ph.name}'
                  f'{"  - " + ph.note if ph.note else ""}'
                  f'{f"   (late {late * 1000:.0f} ms)" if late > 0.05 else ""}',
                  flush=True)
            command(allcfs, cfs, cfg, ph)
            if lights_on:
                cue(allcfs, cfs, ph.lights)
            t_next += ph.slot

        wait = t_next - timeHelper.time()
        if wait > 0:
            timeHelper.sleep(wait)
        for cf in cfs:
            cf.arm(False)
        armed = False
        print(f'\n  [t+{timeHelper.time() - t0:5.1f}s] landed, disarmed - done\n')
        return 0
    except ShowAborted as e:
        # Deliberate stop: land, then exit quietly. A traceback here would only
        # bury the one line the operator needs to read ("landed and disarmed").
        if armed:
            abort_land(allcfs, cfs, timeHelper, cfg.land_height, str(e))
        return 130
    except BaseException as e:                     # noqa: BLE001
        if armed:
            abort_land(allcfs, cfs, timeHelper, cfg.land_height, repr(e))
        raise                                      # a real fault: keep the trace


if __name__ == '__main__':
    sys.exit(main())
