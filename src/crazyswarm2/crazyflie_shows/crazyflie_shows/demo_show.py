#!/usr/bin/env python3
"""Starter show: takeoff -> n-gon gather -> rigid 360 spin -> home -> land.

Template for new choreography. The structure to copy:

  1. CONSTANTS at the top, with units and the reason for the value
  2. PLAN ON THE GROUND -- all geometry and assignment resolved before arming,
     from cf.initialPosition (the yaml value), never from live pose
  3. VERIFY the plan offline -- every simultaneous leg checked against the
     separation budget; the script refuses to arm if a leg is unsafe
  4. ARM -> phases -> LAND, each phase being `command; sleep(duration + margin)`

Run (hardware)::

    ros2 run crazyflie_shows demo_show

Run (sim -- the sim clock is ~4x slower than wall time, so the flag is not
optional)::

    ros2 run crazyflie_shows demo_show --ros-args -p use_sim_time:=true
"""

import numpy as np
from crazyflie_py import Crazyswarm

from crazyflie_shows import figures, safety

# ---------------------------------------------------------------- constants
FORM_HEIGHT = 1.0        # m, formation altitude
GATHER_RADIUS = 0.8      # m, n-gon radius; adjacent sep = 2R*sin(180/n)
SPIN_PERIOD = 10.0       # s for a full 360; ~0.50 m/s tangential at R=0.8
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
    safety.check_envelope(goals, label='gather slots')
    safety.check_leg([h[:2] for h in hovers], [g[:2] for g in goals], 'gather')
    safety.check_leg([g[:2] for g in goals], [h[:2] for h in hovers], 'return')
    print(f'plan ok: {n} drones, gather min sep {gather_sep:.2f} m, '
          f'spin {figures.tangential_speed(GATHER_RADIUS, SPIN_PERIOD):.2f} m/s '
          f'/ {figures.centripetal_accel(GATHER_RADIUS, SPIN_PERIOD):.2f} m/s^2',
          flush=True)

    # -------------------------------------------------------------- upload
    for j, cf in enumerate(cfs):
        cf.uploadTrajectory(1, 0, spin[j])

    # ----------------------------------------------------------------- fly
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
    phase('landed - done')


if __name__ == '__main__':
    main()
