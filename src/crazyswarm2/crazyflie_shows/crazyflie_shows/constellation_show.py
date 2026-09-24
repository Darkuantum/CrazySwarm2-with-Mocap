#!/usr/bin/env python3
"""Fly the constellation show: five drones drawing shapes, on a beat, with lights.

A ~78 s arch through a pentagon, an arrow, a pyramid and a spiral staircase
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
``dry_run``             false    plan and report, then exit. Never arms.
``lights``              true*    light cues (*false under use_sim_time)
``bpm``                 120.0    tempo of the beat grid (re-plans)
``check_placement``     true     compare live pose to initial_position
``placement_tol``       0.25     m, how far off a drone may be
``scale``               1.0      grow the whole show (ConstellationConfig.scale)
``arena_radius``        2.5      m
``ceiling``             2.0      m
======================  =======  =====================================
"""

import signal
import sys

import numpy as np
from crazyflie_py import Crazyswarm

from crazyflie_shows import constellation, plan_show
from crazyflie_shows.swarm_show import _param, check_placement

LED_PARAM = 'colorLedBot.wrgb8888'
ABORT_LAND_DURATION = 4.0      # s -- a gentle descent from wherever they are


class ShowAborted(Exception):
    """Raised by the signal handler installed in :func:`take_signals`."""


def take_signals():
    """Handle SIGINT/SIGTERM ourselves so an abort can still command a landing.

    MEASURED, sim, 2026-09-20: without this, Ctrl-C mid-show left the drones
    flying. rclpy installs its own handler in ``rclpy.init`` (inside
    ``Crazyswarm()``) which shuts the ROS context down *before* the exception
    reaches this script, so the landing call then dies with "the given context
    is not valid" -- the one moment it is needed. Replacing the disposition
    after init keeps the context alive long enough to land.

    A second Ctrl-C during the abort restores the default handler and kills
    the process outright: the operator must always be able to give up on the
    script and reach for the E-STOP.
    """
    def handler(signum, _frame):
        raise ShowAborted('Ctrl-C' if signum == signal.SIGINT else f'signal {signum}')
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)


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


def abort_land(allcfs, cfs, timeHelper, cfg, why):
    """Land where they are and disarm. Best effort: report what worked."""
    print(f'\n  *** ABORT ({why}) - landing all drones where they are ***', flush=True)
    signal.signal(signal.SIGINT, signal.SIG_DFL)     # a second Ctrl-C kills us
    try:
        allcfs.land(targetHeight=cfg.land_height, duration=ABORT_LAND_DURATION)
        timeHelper.sleep(ABORT_LAND_DURATION + 0.5)
        for cf in cfs:
            cf.arm(False)
        print('  landed and disarmed.\n', flush=True)
    except BaseException as e:                     # noqa: BLE001
        print(f'  could NOT complete the abort landing ({e!r}).\n'
              '  Use the E-STOP (console or preflight GUI "e") if a drone is '
              'still airborne.\n', flush=True)


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

    if dry_run:
        print('  dry_run - nothing armed, nothing uploaded.\n')
        return 0

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

    print(f'  lights: {"ON" if lights_on else "off"}'
          f'{" (sim has no LED deck)" if sim and not lights_on else ""}')

    # -------------------------------------------------------------- upload
    print(f'  uploading {len(plan.figs)} figures to {len(cfs)} drones '
          f'({plan.report["pieces"]} pieces each)')
    for f in plan.figs:
        for j, cf in enumerate(cfs):
            cf.uploadTrajectory(f.traj_id, f.piece_offset, f.trajs[j])
        print(f'    id {f.traj_id} @ offset {f.piece_offset:>2}  {f.name}')

    # ----------------------------------------------------------------- fly
    armed = False
    try:
        take_signals()
        for cf in cfs:
            cf.arm(True)
        armed = True
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
            abort_land(allcfs, cfs, timeHelper, cfg, str(e))
        return 130
    except BaseException as e:                     # noqa: BLE001
        if armed:
            abort_land(allcfs, cfs, timeHelper, cfg, repr(e))
        raise                                      # a real fault: keep the trace


if __name__ == '__main__':
    sys.exit(main())
