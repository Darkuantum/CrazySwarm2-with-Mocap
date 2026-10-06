#!/usr/bin/env python3
"""Starter show: takeoff -> n-gon gather -> rigid 360 spin -> home -> land.

Template for new choreography. The structure to copy:

  1. CONSTANTS at the top, with units and the reason for the value
  2. PLAN ON THE GROUND -- all geometry and assignment resolved before arming,
     from cf.initialPosition (the yaml value), never from live pose
  3. VERIFY the plan offline -- every simultaneous leg checked against the
     separation budget; the script refuses to arm if a leg is unsafe
  4. ARM -> phases -> LAND, each phase being `command; sleep(duration + margin)`,
     all of it inside the abort machinery of :mod:`crazyflie_shows.abort` so a
     Ctrl-C lands the drones instead of abandoning them in the air

Run (hardware)::

    ros2 run crazyflie_shows demo_show

Run (sim -- the sim clock is ~4x slower than wall time, so the flag is not
optional)::

    ros2 run crazyflie_shows demo_show --ros-args -p use_sim_time:=true
"""

import sys

import numpy as np
from crazyflie_py import Crazyswarm

from crazyflie_shows import figures, safety
from crazyflie_shows.abort import ShowAborted, abort_land, take_signals
from crazyflie_shows.preflight import report_supervisor
from crazyflie_shows.swarm_show import _param

# ---------------------------------------------------------------- constants
FORM_HEIGHT = 1.0        # m, formation altitude
#: m, n-gon radius; adjacent sep = 2R*sin(180/n). Raised from 0.8 to 0.9 on
#: 2026-10-06 together with check_leg's default budget: on the five enabled
#: marks, R=0.80 plans a gather transit of 0.844 m, which clears the flown
#: floor (MIN_SEPARATION 0.80) but NOT the plan budget (PLAN_SEPARATION 0.90)
#: -- and at this rig's measured 0.05-0.06 m of tracking loss it would be
#: flown at ~0.79 m, under the floor. R=0.90 plans 0.917 m.
GATHER_RADIUS = 0.9
SPIN_PERIOD = 10.0       # s for a full 360; ~0.57 m/s tangential at R=0.9
TAKEOFF_HEIGHT = 1.0     # m
TAKEOFF_DURATION = 2.0   # s
HOME_HEIGHT = 0.75       # m above each drone's own initial_position
LAND_HEIGHT = 0.04       # m
LAND_DURATION = 4.5      # s -> ~0.16 m/s descent from HOME_HEIGHT
TRANS_AVG_SPEED = 0.5    # m/s average for distance-scaled legs
MARGIN = 0.5             # s of slack added to every sleep


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    cfs = allcfs.crazyflies
    n = len(cfs)
    if n < 2:
        raise SystemExit(
            f'demo_show needs at least 2 drones, found {n}. '
            'Check `enabled:` in crazyflies.yaml.')

    # ------------------------------------------------------ plan (on ground)
    starts = [np.array(cf.initialPosition)[:2] for cf in cfs]
    center = np.mean(starts, axis=0)
    hovers = [np.array(cf.initialPosition) + np.array([0., 0., TAKEOFF_HEIGHT])
              for cf in cfs]

    slots, angles, perm, gather_sep = safety.best_phase_ngon(
        starts, center, GATHER_RADIUS, FORM_HEIGHT, figures.ngon_slots)
    goals = [slots[perm[j]] for j in range(n)]

    # one circle per drone, each starting at its own slot angle, so that
    # starting them together rotates the whole n-gon rigidly
    spin = [figures.circle_trajectory(center, GATHER_RADIUS,
                                      angles[perm[j]], SPIN_PERIOD,
                                      FORM_HEIGHT)
            for j in range(n)]

    # ----------------------------------------------------- verify (on ground)
    #
    # The envelope is set by where the drones are PARKED, not by the figure:
    # on the 2026-10-05 marks the gather slots reach 1.21 m while the hover
    # columns over the marks reach 1.76 m of the 1.90 m budget. Checking only
    # the figure therefore checks the wrong thing -- the same lesson the
    # constellation learned. check_envelope defaults to the MEASURED volume
    # (ARENA_RADIUS_PLAN from ARENA_CENTRE, with the truncated-cone ceiling).
    homes = [np.array(cf.initialPosition) + np.array([0., 0., HOME_HEIGHT])
             for cf in cfs]
    safety.check_envelope(goals, label='gather slots')
    safety.check_envelope(hovers, label='hover columns over the marks')
    safety.check_envelope(homes, label='home columns')
    # The spin is a rigid rotation of the n-gon about `center`, so its worst
    # plan-view radius is |center - arena centre| + GATHER_RADIUS, reached at
    # some point by some drone. Check that point rather than the slots alone.
    spin_worst = float(np.linalg.norm(center - np.asarray(safety.ARENA_CENTRE,
                                                          float))) + GATHER_RADIUS
    if spin_worst > safety.ARENA_RADIUS_PLAN:
        raise ValueError(
            f'spin circle reaches r={spin_worst:.2f} m from the arena centre, '
            f'outside the {safety.ARENA_RADIUS_PLAN:.2f} m budget')
    safety.check_leg([h[:2] for h in hovers], [g[:2] for g in goals], 'gather')
    safety.check_leg([g[:2] for g in goals], [h[:2] for h in hovers], 'return')
    print(f'plan ok: {n} drones, gather min sep {gather_sep:.2f} m '
          f'(budget {safety.PLAN_SEPARATION:.2f}), spin '
          f'{figures.tangential_speed(GATHER_RADIUS, SPIN_PERIOD):.2f} m/s '
          f'/ {figures.centripetal_accel(GATHER_RADIUS, SPIN_PERIOD):.2f} m/s^2, '
          f'worst radius {max(spin_worst, max(float(np.linalg.norm(np.asarray(h)[:2] - np.asarray(safety.ARENA_CENTRE, float))) for h in hovers)):.2f} m '
          f'of {safety.ARENA_RADIUS_PLAN:.2f} m', flush=True)

    # ------------------------------------------------- supervisor go/no-go
    # Before the upload, not after: an E-STOP latches the firmware's
    # supervisor into LOCKED (only a power cycle clears it) and arm() is
    # fire-and-forget, so the failure would otherwise show up as "nothing
    # took off" with no reason given.
    if not bool(_param(allcfs, 'use_sim_time', False)):
        if not report_supervisor(allcfs, [cf.prefix.lstrip('/') for cf in cfs]):
            return 1

    # -------------------------------------------------------------- upload
    for j, cf in enumerate(cfs):
        cf.uploadTrajectory(1, 0, spin[j])

    # ----------------------------------------------------------------- fly
    #
    # Step 4 of the template: everything that can leave a drone in the air
    # runs under the abort machinery. Without it a Ctrl-C mid-spin leaves the
    # fleet flying its last setpoint -- rclpy tears the ROS context down
    # before a handler here could land them, which is what take_signals()
    # exists to prevent.
    armed = False
    try:
        take_signals()
        # BEFORE the arm loop, so a signal mid-loop still disarms the fleet.
        armed = True
        for cf in cfs:
            cf.arm(True)
        timeHelper.sleep(1.0)

        t0 = timeHelper.time()

        def phase(msg):
            print(f'[t+{timeHelper.time() - t0:5.1f}s] {msg}', flush=True)

        phase('takeoff')
        allcfs.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=TAKEOFF_DURATION)
        timeHelper.sleep(TAKEOFF_DURATION + MARGIN)

        phase('settle over start positions')
        for j, cf in enumerate(cfs):
            cf.goTo(hovers[j], 0, 2.0)
        timeHelper.sleep(2.0 + MARGIN)

        phase(f'gather -> {n}-gon (min sep {gather_sep:.2f} m)')
        dur = safety.scaled_duration(hovers, goals, TRANS_AVG_SPEED)
        for j, cf in enumerate(cfs):
            cf.goTo(goals[j], 0, dur)
        timeHelper.sleep(dur + MARGIN)

        # broadcast: one radio packet, so the formation turns as a rigid body.
        # relative=True pins eval(0) to each drone's current setpoint.
        phase(f'rigid 360 spin ({SPIN_PERIOD:.0f} s)')
        allcfs.startTrajectory(1, timescale=1.0, relative=True)
        timeHelper.sleep(SPIN_PERIOD + 1.0)

        phase('return home')
        dur = safety.scaled_duration(goals, hovers, TRANS_AVG_SPEED)
        for j, cf in enumerate(cfs):
            home = np.array(cfs[j].initialPosition) + np.array([0., 0., HOME_HEIGHT])
            cf.goTo(home, 0, dur)
        timeHelper.sleep(dur + MARGIN)

        phase('landing')
        allcfs.land(targetHeight=LAND_HEIGHT, duration=LAND_DURATION)
        timeHelper.sleep(LAND_DURATION + MARGIN)

        for cf in cfs:
            cf.arm(False)
        armed = False
        phase('landed - done')
        return 0
    except ShowAborted as e:
        if armed:
            abort_land(allcfs, cfs, timeHelper, LAND_HEIGHT, str(e))
        return 130
    except BaseException as e:                     # noqa: BLE001
        if armed:
            abort_land(allcfs, cfs, timeHelper, LAND_HEIGHT, repr(e))
        raise


if __name__ == '__main__':
    sys.exit(main())
