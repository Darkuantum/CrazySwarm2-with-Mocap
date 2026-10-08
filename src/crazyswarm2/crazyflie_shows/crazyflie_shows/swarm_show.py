#!/usr/bin/env python3
"""Fly the show. Sim first, always.

    ros2 launch crazyflie_shows show_launch.py backend:=sim
    ros2 run crazyflie_shows swarm_show --ros-args -p use_sim_time:=true

    # hardware: stack up, scan every address, check the preflight GUI, THEN
    ros2 launch crazyflie_shows show_launch.py
    ros2 run crazyflie_shows swarm_show

``use_sim_time`` is not optional in sim: the sim clock runs ~4x slower than
wall time, so without it the script races ahead of the physics and every
phase is cut short.

This file is deliberately thin. All the geometry, all the timing and all the
verification live in :mod:`crazyflie_shows.choreography`, which is pure and
runs on a laptop -- so ``plan_show`` checks the *same object* this flies.
What is left here is the part that needs a radio:

  1. read the fleet off the live stack
  2. build the plan (which refuses to return an unsafe one)
  3. check every drone is physically where the plan thinks it is
  4. upload, arm, walk the phases, land, disarm

Parameters (``--ros-args -p name:=value``)
------------------------------------------
======================  =======  =====================================
``dry_run``             false    plan and report, then exit. Never arms.
``check_placement``     true     compare live pose to initial_position
``placement_tol``       0.25     m, how far off a drone may be
``scale``               1.0      shrink the whole show (ShowConfig.scale)
``arena_radius``        1.90     m, = ARENA_RADIUS_PLAN (tested 2.00 less
                                 the 0.10 tracking margin), NOT the old 2.5
``ceiling``             2.32     m, = CEILING_CENTRE_TESTED 2.42 less the
                                 same margin, NOT the old 2.0
======================  =======  =====================================
"""

import sys

import numpy as np
import rclpy
from crazyflie_py import Crazyswarm

from crazyflie_shows import choreography, plan_show, safety
from crazyflie_shows.abort import ShowAborted, abort_land, take_signals
from crazyflie_shows.preflight import report_supervisor


def _param(node, name, default):
    """Read a ROS parameter, declaring it on first use."""
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value


def wait_for_poses(node, cfs, timeout=25.0, report_every=5.0):
    """Block until every drone has published at least one ``/cfX/pose``.

    Returns the list of drones still silent when the timeout expires.

    **The timeout is 25 s, not 6.** MEASURED 2026-10-02 from every
    crazyflie_server log since June: unicast telemetry on this rig drops out
    routinely -- the 09-17 show that flew cleanly had cf3 reporting a low
    receive rate in 98 of its 435 seconds, and sustained silences of 10 s or
    more are normal with five drones on one radio. A 6 s window therefore
    failed drones that were fine, which reads as "the show is broken" and sends
    you looking for a fault that is not there. Flight itself does not depend on
    this: the server sends mocap poses to the drones as BROADCASTS, which carry
    no ACK and are never retried, so a drone can fly well while its telemetry
    is lossy. This gate is about proving the chain once, so give it time to be
    proved, and say which drones are still missing while waiting.

    ``/cfX/pose`` is the *onboard* estimate, forwarded by the server from the
    firmware's default pose log topic. With ``stabilizer.estimator: 2`` that
    estimate is the Kalman filter with the mocap position fused in, so a drone
    publishing a sane pose has proved the whole chain -- Motive, the multicast
    join, the server, the radio link -- end to end. A drone that publishes
    nothing has proved none of it, and per HANDOVER.md section 6 flying it is
    the fly-away case.
    """
    t0 = node.get_clock().now().nanoseconds
    end = t0 + int(timeout * 1e9)
    nxt = t0 + int(report_every * 1e9)
    missing = lambda: [cf.prefix.lstrip('/') for cf in cfs if not cf.poseStamped]
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if all(cf.poseStamped for cf in cfs):
            return []
        now = node.get_clock().now().nanoseconds
        if now >= nxt:
            nxt = now + int(report_every * 1e9)
            print(f'    waiting for a pose from: {", ".join(missing())} '
                  f'({(now - t0) / 1e9:.0f}/{timeout:.0f} s - telemetry on this '
                  'radio is lossy, this is normal)', flush=True)
    return missing()


#: How long a drone gets to produce its first /cfX/pose. See wait_for_poses.
POSE_WAIT_S = 25.0


def check_placement(node, cfs, names, plan, tol):
    """Refuse to fly if any drone is not where its ``initial_position`` says.

    This is the check that the 2026-08-04 collision needed (HANDOVER.md
    section 6: "Wrong-corner placement produced the real collision"). The whole
    plan -- slot assignment, gather leg, every separation number -- is derived
    from the yaml positions. A drone in the wrong corner does not make the
    plan wrong on paper; it makes the paper irrelevant.

    ``tol`` is not arbitrary. ``takeoff`` climbs from wherever the drone
    actually is, and the ``settle`` phase then pulls it laterally onto the
    yaml position -- a leg the plan models as a no-op. Two drones displaced
    ``tol`` towards each other shrink the 1.36 m hover separation by ``2*tol``,
    so the default 0.25 m leaves 0.86 m, still over the 0.80 m budget. Raising
    it eats that margin directly.
    """
    silent = wait_for_poses(node, cfs, timeout=POSE_WAIT_S)
    if silent:
        raise SystemExit(
            f'\n  NO POSE from: {", ".join(silent)}\n'
            '  These drones have never published /cfX/pose, so nothing has\n'
            '  confirmed they are being tracked. Flying an untracked drone is\n'
            '  the fly-away case (HANDOVER.md 6). Check, in this order:\n'
            f'  Nothing arrived in {POSE_WAIT_S:.0f} s, which is longer than this '
            'radio\'s usual dropouts. Check, in this order:\n'
            '    ros2 topic hz /poses                  # is the mocap stream alive?\n'
            '    ros2 topic hz /cfX/status             # does ANY telemetry return?\n'
            '    grep "Low unicast receive rate" ~/.ros/log/<newest>/launch.log\n'
            '  A receive rate near 0.1 with thousands of packets sent is a radio\n'
            '  problem (antenna placement, 2.4 GHz interference), not a mocap one.\n')

    bad = []
    for cf, nm, want in zip(cfs, names, plan.starts):
        got = np.array(cf.get_position(), float)
        d = float(np.linalg.norm(got[:2] - np.asarray(want, float)[:2]))
        flag = 'OK ' if d <= tol else 'BAD'
        print(f'    {flag} {nm:>6}  yaml ({want[0]:+.2f},{want[1]:+.2f})  '
              f'live ({got[0]:+.2f},{got[1]:+.2f})  off by {d:.2f} m')
        if d > tol:
            bad.append((nm, d))
    if bad:
        raise SystemExit(
            '\n  PLACEMENT CHECK FAILED: '
            + ', '.join(f'{nm} off by {d:.2f} m' for nm, d in bad)
            + f'\n  (tolerance {tol:.2f} m)\n'
            '  Either move the drone to its initial_position, or update\n'
            '  initial_position in crazyflie/config/crazyflies.yaml from /poses (never\n'
            '  from /cfX/pose - HANDOVER.md 6), rebuild, and re-run plan_show.\n')


def main():
    # The room must have been MEASURED before anything plans or flies.
    # No-op on this rig; the gate exists so a copy of this package in an
    # unsurveyed room refuses instead of inheriting our geofence.
    safety.require_measured_arena('swarm_show')
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    node = allcfs
    cfs = allcfs.crazyflies

    sim = bool(_param(node, 'use_sim_time', False))
    dry_run = bool(_param(node, 'dry_run', False))
    want_placement = bool(_param(node, 'check_placement', True))
    tol = float(_param(node, 'placement_tol', 0.25))

    names = [cf.prefix.lstrip('/') for cf in cfs]
    starts = [np.array(cf.initialPosition, float) for cf in cfs]
    if len(cfs) < 3:
        raise SystemExit(
            f'the show needs at least 3 drones, the server offers {len(cfs)}. '
            'Check `enabled:` in crazyflie/config/crazyflies.yaml.')

    cfg = choreography.ShowConfig()
    cfg.scale = float(_param(node, 'scale', cfg.scale))
    cfg.arena_radius = float(_param(node, 'arena_radius', cfg.arena_radius))
    cfg.ceiling = float(_param(node, 'ceiling', cfg.ceiling))

    # build_plan verifies as it builds and raises on any budget violation, so
    # there is no path from here to an armed drone flying an unchecked plan.
    plan = choreography.build_plan(names, starts, cfg)
    plan_show.report(plan, cfg, names)

    # ------------------------------------------------------------ preflight
    if want_placement and not sim:
        print('  PLACEMENT CHECK (live pose vs initial_position)')
        check_placement(node, cfs, names, plan, tol)
        print('    all drones within tolerance\n')
    elif sim:
        # The topic is advertised on backend:=sim but nothing is ever
        # published on it (measured: no message in 12 s with the stack up and
        # the drones idle), so there is nothing to compare against. On
        # hardware it is the firmware's default pose log, forwarded by the
        # server, and the check is real.
        print('  placement check skipped: backend:=sim never publishes '
              '/cfX/pose\n')
    else:
        print('  *** placement check DISABLED by parameter ***\n')

    # ------------------------------------------------- supervisor go/no-go
    # An E-STOP latches the firmware's supervisor into LOCKED, which only a
    # power cycle clears, and arm() is fire-and-forget -- so without this the
    # failure arrives as "nothing took off" after the whole upload.
    if not sim and not report_supervisor(node, names):
        return 1

    # The rehearsal stops HERE, not before the checks above: a rehearsal whose
    # answer is only "the geometry is fine" tells you nothing about the rig,
    # and every check above is read-only. Everything past this point touches a
    # drone.
    if dry_run:
        print('  dry_run - plan and preflight checks done; nothing uploaded, '
              'nothing armed.\n')
        return 0

    # -------------------------------------------------------------- upload
    # Sequential and blocking: uploadTrajectory spins until the service
    # returns, so this is 5 drones x 5 figures = 25 round trips over one
    # radio. Expect a few seconds, and expect it before anything is armed.
    print(f'  uploading {len(plan.figs)} figures to {len(cfs)} drones '
          f'({plan.report["pieces"]} pieces each)')
    for f in plan.figs:
        for j, cf in enumerate(cfs):
            cf.uploadTrajectory(f.traj_id, f.piece_offset, f.trajs[j])
        print(f'    id {f.traj_id} @ offset {f.piece_offset:>2}  {f.name}')

    # ----------------------------------------------------------------- fly
    #
    # Everything from here to the disarm runs under the abort machinery in
    # :mod:`crazyflie_shows.abort`. Until 2026-10-06 it did not, and a Ctrl-C
    # mid-figure left five armed drones airborne with no script -- on the one
    # show that has actually flown on hardware.
    armed = False
    try:
        take_signals()
        # BEFORE the arm loop: a signal arriving between the first and last
        # arm() would otherwise raise with armed still False and skip the
        # abort, leaving part of the fleet armed.
        armed = True
        for cf in cfs:
            cf.arm(True)
        timeHelper.sleep(1.0)

        t0 = timeHelper.time()
        print(f'\n  SHOW START - {plan.duration:.1f}s of motion, '
              f'~{plan.duration + len(plan.phases) * cfg.phase_margin:.1f}s total\n')

        for ph in plan.phases:
            print(f'  [t+{timeHelper.time() - t0:5.1f}s] {ph.name}'
                  f'{"  - " + ph.note if ph.note else ""}', flush=True)

            if ph.kind == 'takeoff':
                allcfs.takeoff(targetHeight=cfg.takeoff_height, duration=ph.duration)
            elif ph.kind == 'land':
                allcfs.land(targetHeight=cfg.land_height, duration=ph.duration)
            elif ph.kind == 'goto':
                # Per-drone and unicast: allcfs.goTo is broadcast but hardcodes
                # relative=True, and every goal here is absolute. Five service
                # calls spread over some tens of milliseconds, which is nothing
                # against a 2.5 s leg.
                for j, cf in enumerate(cfs):
                    cf.goTo(ph.goals[j], 0.0, ph.duration)
            elif ph.kind == 'figure':
                # One broadcast packet, so all five start the figure on the same
                # radio frame. This is the only reason a rigid formation stays
                # rigid at speed -- five unicast starts would smear the ring by
                # however long the calls took.
                allcfs.startTrajectory(ph.figure.traj_id,
                                       timescale=ph.timescale,
                                       reverse=ph.reverse,
                                       relative=True)
            else:
                raise RuntimeError(f'unknown phase kind {ph.kind!r}')

            timeHelper.sleep(ph.duration + cfg.phase_margin)

        for cf in cfs:
            cf.arm(False)
        armed = False
        print(f'\n  [t+{timeHelper.time() - t0:5.1f}s] landed, disarmed - done\n')
        return 0
    except ShowAborted as e:
        # Deliberate stop: land, then exit quietly. A traceback here would only
        # bury the one line the operator needs ("landed and disarmed").
        if armed:
            abort_land(allcfs, cfs, timeHelper, cfg.land_height, str(e))
        return 130
    except BaseException as e:                     # noqa: BLE001
        if armed:
            abort_land(allcfs, cfs, timeHelper, cfg.land_height, repr(e))
        raise                                      # a real fault: keep the trace


if __name__ == '__main__':
    sys.exit(main())
