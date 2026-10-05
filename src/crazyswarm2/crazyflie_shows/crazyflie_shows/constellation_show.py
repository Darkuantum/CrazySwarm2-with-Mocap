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
``dry_run``             false    plan + preflight checks, then exit. Never arms,
                                 never uploads. The rehearsal.
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
import time

import numpy as np
import rclpy
from crazyflie_interfaces.msg import Status
from crazyflie_py import Crazyswarm
from rclpy.qos import qos_profile_sensor_data

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


def check_supervisor(node, names, timeout=6.0):
    """Refuse to fly drones the firmware will not arm. Runs BEFORE the upload.

    MEASURED 2026-10-02: five drones sat in the supervisor's LOCKED state, which
    an emergency stop latches and which **only a power cycle clears** -- the
    firmware's state machine has exactly one transition out of locked, back to
    locked, blocked always. Nothing in the flight path noticed: the show checked
    placement, spent 55 s uploading trajectories, and only then would it have
    failed at arm(). The operator read the silent upload as a hang and killed it,
    which looked like a bug in the show.

    So: ask every drone what the supervisor thinks, before anything is uploaded.
    Battery thresholds match the console's health page.
    """
    latest = {}

    def mk(nm):
        def cb(msg):
            latest[nm] = msg
        return cb

    subs = [node.create_subscription(Status, f'/{nm}/status', mk(nm),
                                     qos_profile_sensor_data) for nm in names]
    deadline = time.time() + timeout            # /status is published at 1 Hz
    while time.time() < deadline and len(latest) < len(names):
        rclpy.spin_once(node, timeout_sec=0.1)
    for sub in subs:
        node.destroy_subscription(sub)

    problems, notes = [], []
    for nm in names:
        st = latest.get(nm)
        if st is None:
            notes.append(f'{nm}: no /{nm}/status in {timeout:g} s - cannot check '
                         'the supervisor (old firmware, or a dead link)')
            continue
        info, v = st.supervisor_info, st.battery_voltage
        if info & Status.SUPERVISOR_INFO_IS_LOCKED:
            problems.append(f'{nm}: supervisor LOCKED (0x{info:04x}) - where an '
                            'E-STOP leaves a drone. Battery out and in; nothing '
                            'else clears it')
        elif not info & Status.SUPERVISOR_INFO_CAN_BE_ARMED:
            problems.append(f'{nm}: the firmware says it cannot be armed '
                            f'(0x{info:04x}) - preflight checks have not passed')
        if info & Status.SUPERVISOR_INFO_IS_TUMBLED:
            problems.append(f'{nm}: tumbled - stand it back on its feet')
        if v and v < 3.7:
            problems.append(f'{nm}: battery {v:.2f} V, below 3.7 V - swap it')
        elif v and v < 3.8:
            notes.append(f'{nm}: battery {v:.2f} V - low for a 76 s show')
        elif st is not None:
            notes.append(f'{nm}: ready, battery {v:.2f} V, rssi {st.rssi}')
    return problems, notes


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
        print('  SUPERVISOR CHECK (what the firmware will let us do)')
        problems, notes = check_supervisor(node, names)
        for line in notes:
            print(f'    {line}')
        if problems:
            print('\n  REFUSING TO FLY - the drones cannot be armed:', flush=True)
            for p in problems:
                print(f'    - {p}')
            print('\n  Nothing was uploaded. Fix the above and re-run; '
                  '`dry_run:=true` re-checks without uploading.\n')
            return 1
        print('    all drones armable\n')

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
