#!/usr/bin/env python

"""Formation waypoint tour followed by an n-gon/triangle formation dance."""

from itertools import permutations
from math import atan2, cos, degrees, radians, sin

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Polynomial4D, Trajectory
import numpy as np

FORM_HEIGHT = 1.0
# gather-circle radius (historic name: half-diagonal of the 4-drone square).
# n drones sit on a regular n-gon at this radius; adjacent separation is
# 2*R*sin(180/n deg) = 0.94 m for n=5 (pentagon), 1.13 m for n=4.
SQUARE_HALF_DIAG = 0.8
ROTATE_PERIOD = 10.0  # full 360 deg; tangential speed at R=0.8 is ~0.50 m/s
TRANS_DURATION = 2.0  # short formation morphs (gather, triangle)
# long rigid moves (shift to orbit ring, return home) scale with the longest
# xy path among the drones: duration = max(TRANS_DURATION, longest / 0.5)
# -> average <= 0.5 m/s, rest-to-rest peak ~0.9 m/s
TRANS_AVG_SPEED = 0.5
# triangle + tail-pair formation (5 slots), pointing along +x; offsets from
# the formation center. min pairwise slot distance is 0.84 m (tail-to-tail;
# rear-corner-to-tail is 0.87 m).
TRI_TAIL_OFFSETS = [(0.8, 0.0),      # apex
                    (-0.1, 0.6),     # rear-left
                    (-0.1, -0.6),    # rear-right
                    (-0.95, 0.42),   # tail-left
                    (-0.95, -0.42)]  # tail-right
# opening waypoint tour: each xy offset is applied to EVERY drone's own
# start hover position (rigid group translation at FORM_HEIGHT), so the
# separation stays exactly the start-position spacing throughout — no
# assignment needed. A final (0, 0) leg returns everyone to their start.
WAYPOINT_OFFSETS = [(0.6, 0.0), (-0.6, 0.5), (0.0, -0.6)]
# swarm orbit: the whole formation translates rigidly around the room center
ROOM_CENTER = np.array([0.0467, -0.1037])  # user-measured mocap-area center
ORBIT_R = 1.2
# tangential speed at R=1.2 is 2*pi*1.2/12 ~= 0.63 m/s (ORBIT_PERIOD kept at
# 12), centripetal accel ~0.33 m/s^2 — well inside the 1.3 m/s envelope
ORBIT_PERIOD = 12.0


def slot_position(center, radius, angle_deg):
    a = radians(angle_deg)
    return np.array([center[0] + radius * cos(a),
                     center[1] + radius * sin(a),
                     FORM_HEIGHT])


def circle_trajectory(center, radius, start_angle_deg, period, height,
                      n_pieces=6):
    """Full 360 deg circle about center, starting at start_angle_deg.

    Built as n_pieces equal arcs; per piece and axis a degree-7
    polynomial is fitted (in piece-local time) to the exact circle.
    z is constant at height, yaw is constant 0. eval(0) == eval(period)
    == the slot position at start_angle_deg. 6 pieces keeps the total
    trajectory-memory footprint under the firmware's ~4 KB limit while
    the deg-7 fit over 60 deg is still exact to ~1e-7 m.
    """
    piece_t = period / n_pieces
    theta0 = radians(start_angle_deg)
    omega = 2.0 * np.pi / period
    ts = np.linspace(0.0, piece_t, 64)
    traj = Trajectory()
    traj.polynomials = []
    for k in range(n_pieces):
        ang = theta0 + omega * (k * piece_t + ts)
        # numpy.polyfit returns highest degree first; Polynomial stores
        # ascending (p[0] = constant term), so reverse.
        px = np.polyfit(ts, center[0] + radius * np.cos(ang), 7)[::-1]
        py = np.polyfit(ts, center[1] + radius * np.sin(ang), 7)[::-1]
        pz = np.zeros(8)
        pz[0] = height
        pyaw = np.zeros(8)
        traj.polynomials.append(Polynomial4D(piece_t, px, py, pz, pyaw))
    traj.duration = period
    return traj


def triangle_slots(center, n_cf):
    """World-frame triangle+tail slots around the formation center."""
    return [np.array([center[0] + dx, center[1] + dy, FORM_HEIGHT])
            for dx, dy in TRI_TAIL_OFFSETS[:n_cf]]


def transition_min_sep(src, dst):
    """Exact min pairwise separation over simultaneous straight goTo paths.

    All drones move linearly over the same duration, so each pair's
    relative position is linear in normalized time s and its squared
    distance is a quadratic — the minimum on [0, 1] is closed-form.
    """
    worst = np.inf
    for a in range(len(src)):
        for b in range(a + 1, len(src)):
            d0 = np.asarray(src[a] - src[b], dtype=float)
            dv = np.asarray(dst[a] - dst[b], dtype=float) - d0
            den = float(np.dot(dv, dv))
            s = 0.0 if den < 1e-12 else min(
                1.0, max(0.0, -float(np.dot(d0, dv)) / den))
            worst = min(worst, float(np.linalg.norm(d0 + s * dv)))
    return worst


def plan_gather(starts):
    """Regular n-gon gather slots + assignment from the initial positions.

    The n-gon's phase offset is a free parameter: with a drone starting
    near the swarm center a fixed 45 deg offset can squeeze the
    simultaneous straight-line gather below 0.8 m separation for EVERY
    assignment. Sweep the offset in 1 deg steps over one slot period
    (360/n) and keep the offset whose min-total-distance assignment
    maximizes the exact worst-case pairwise separation along the gather
    paths. Depends only on initial positions, so it runs before takeoff.

    Returns (center, base_angles, slots, best) where slots[best[j]] is
    drone j's gather goal.
    """
    n_cf = len(starts)
    center = np.mean(starts, axis=0)
    best_sep = -1.0
    for off in np.arange(0.0, 360.0 / n_cf, 1.0):
        cand_angles = [45.0 + off + 360.0 * k / n_cf for k in range(n_cf)]
        cand_slots = [slot_position(center, SQUARE_HALF_DIAG, a)
                      for a in cand_angles]
        perm = min(permutations(range(n_cf)),
                   key=lambda p: sum(
                       np.linalg.norm(starts[j] - cand_slots[p[j]][:2])
                       for j in range(n_cf)))
        sep = transition_min_sep(
            starts, [cand_slots[perm[j]][:2] for j in range(n_cf)])
        if sep > best_sep:
            best_sep = sep
            base_angles, slots, best = cand_angles, cand_slots, perm
    return center, base_angles, slots, best


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # enable logging
    allcfs.setParam('usd.logging', 1)

    # regular n-gon gather slots around the swarm center (offset sweep +
    # min-distance assignment, see plan_gather); depends only on initial
    # positions, known before takeoff
    n_cf = len(allcfs.crazyflies)
    starts = [np.array(cf.initialPosition)[:2] for cf in allcfs.crazyflies]
    center, base_angles, slots, best = plan_gather(starts)
    angles = [base_angles[best[j]] for j in range(n_cf)]

    # one continuous 360 deg circle per drone, starting at its own slot angle
    circle_trajs = [circle_trajectory(center, SQUARE_HALF_DIAG, angles[j],
                                      ROTATE_PERIOD, FORM_HEIGHT)
                    for j in range(n_cf)]

    # swarm orbit: nearest point P on the room-center orbit circle to the
    # formation center C; the formation is rigidly shifted by (P - C), then
    # every drone flies the SAME room-center circle (relative=True shifts it
    # onto each drone's own position, i.e. a rigid translation of the swarm)
    theta_start = atan2(center[1] - ROOM_CENTER[1], center[0] - ROOM_CENTER[0])
    orbit_entry = ROOM_CENTER + ORBIT_R * np.array([cos(theta_start),
                                                    sin(theta_start)])
    orbit_shift = np.array([orbit_entry[0] - center[0],
                            orbit_entry[1] - center[1], 0.0])
    orbit_traj = circle_trajectory(ROOM_CENTER, ORBIT_R, degrees(theta_start),
                                   ORBIT_PERIOD, FORM_HEIGHT)

    TRIALS = 1
    for i in range(TRIALS):
        for idx, cf in enumerate(allcfs.crazyflies):
            # rotation circle at the start of trajectory memory, the shared
            # orbit circle right after it (trajectory ids 1/2 kept so the
            # startTrajectory calls below are unchanged)
            cf.uploadTrajectory(1, 0, circle_trajs[idx])
            cf.uploadTrajectory(2, circle_trajs[idx].n_pieces(), orbit_traj)

        # Arm
        for cf in allcfs.crazyflies:
            cf.arm(True)
        timeHelper.sleep(1.0)

        t0 = timeHelper.time()

        def phase(msg):
            print(f'[t+{timeHelper.time()-t0:5.1f}s] {msg}', flush=True)

        phase('takeoff')
        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)
        phase('moving to start positions')
        hovers = [np.array(cf.initialPosition) + np.array([0.0, 0.0, 1.0])
                  for cf in allcfs.crazyflies]
        for j, cf in enumerate(allcfs.crazyflies):
            cf.goTo(hovers[j], 0, 2.0)
        timeHelper.sleep(2.0)

        # formation waypoint tour: the same xy offset goes to EVERY drone's
        # own start hover position, so the group translates rigidly and the
        # separation stays exactly the start spacing. Each leg's duration is
        # distance-scaled like the other long moves (rigid translation: the
        # longest — i.e. every — move equals the leg vector length).
        prev = np.zeros(2)
        for k, (dx, dy) in enumerate(WAYPOINT_OFFSETS + [(0.0, 0.0)]):
            off = np.array([dx, dy])
            leg_dur = max(TRANS_DURATION,
                          float(np.linalg.norm(off - prev)) / TRANS_AVG_SPEED)
            phase(f'waypoint {k + 1}' if k < len(WAYPOINT_OFFSETS)
                  else 'back to start positions')
            for j, cf in enumerate(allcfs.crazyflies):
                cf.goTo(hovers[j] + np.array([dx, dy, 0.0]), 0, leg_dur)
            timeHelper.sleep(leg_dur + 0.5)
            prev = off

        # gather onto the precomputed n-gon slots
        phase('gather -> pentagon')
        for j, cf in enumerate(allcfs.crazyflies):
            cf.goTo(slots[best[j]], 0, TRANS_DURATION)
        timeHelper.sleep(TRANS_DURATION + 0.5)

        # rotate the n-gon a full +360 degrees about the center in one
        # continuous motion: each drone flies its uploaded circle, which
        # starts (and ends) at its own slot; relative=True pins eval(0)
        # to the current setpoint, so the formation rotates rigidly
        phase('pentagon 360 spin (10 s)')
        allcfs.startTrajectory(1, timescale=1.0, relative=True)
        timeHelper.sleep(ROTATE_PERIOD + 1.0)

        # morph onto the triangle+tail formation (min-distance assignment
        # avoids crossings)
        phase('morph -> triangle + tail')
        tri = triangle_slots(center, n_cf)
        cur = [slot_position(center, SQUARE_HALF_DIAG, angles[j])[:2]
               for j in range(n_cf)]
        tbest = min(permutations(range(n_cf)),
                    key=lambda p: sum(np.linalg.norm(cur[j] - tri[p[j]][:2])
                                      for j in range(n_cf)))
        tri_assigned = [tri[tbest[j]] for j in range(n_cf)]
        for j, cf in enumerate(allcfs.crazyflies):
            cf.goTo(tri_assigned[j], 0, TRANS_DURATION)
        timeHelper.sleep(TRANS_DURATION + 0.5)

        # rigidly translate the formation onto the orbit entry point; the
        # ring can make this a long move, so scale the duration to the
        # longest xy path (rigid shift: identical for every drone)
        phase('shift to orbit ring')
        shift_dur = max(TRANS_DURATION,
                        float(np.linalg.norm(orbit_shift[:2]))
                        / TRANS_AVG_SPEED)
        for j, cf in enumerate(allcfs.crazyflies):
            cf.goTo(tri_assigned[j] + orbit_shift, 0, shift_dur)
        timeHelper.sleep(shift_dur + 0.5)

        # one smooth revolution of the whole formation around the room
        # center: every drone flies the same uploaded circle; relative=True
        # pins eval(0) to its current setpoint, so the swarm translates
        # rigidly and returns to the entry point
        phase('orbit around room center (12 s)')
        allcfs.startTrajectory(2, timescale=1.0, relative=True)
        timeHelper.sleep(ORBIT_PERIOD + 1.0)

        # a DIRECT return home from the ring crossed paths at R=2.0
        # (verified offline: 1 crossing); at R=1.2 it happens to be
        # crossing-free, but the pentagon-expand leg is KEPT so the return
        # stays safe for any radius/fleet: first expand back onto each
        # drone's own gather-pentagon slot (verified 0 crossings, min sep
        # 0.84 m), then fly home from the pentagon — the exact reverse of
        # the gather, which was chosen for max separation. Both legs are
        # distance-scaled like the ring shift.
        phase('expand -> pentagon (return leg)')
        pent_dur = max(TRANS_DURATION, max(
            float(np.linalg.norm((tri_assigned[j] + orbit_shift)[:2]
                                 - slots[best[j]][:2]))
            for j in range(n_cf)) / TRANS_AVG_SPEED)
        for j, cf in enumerate(allcfs.crazyflies):
            cf.goTo(slots[best[j]], 0, pent_dur)
        timeHelper.sleep(pent_dur + 0.5)

        # then home from the pentagon slots, and slow descent
        phase('return home')
        home_dur = max(TRANS_DURATION, max(
            float(np.linalg.norm(slots[best[j]][:2]
                                 - np.array(cf.initialPosition)[:2]))
            for j, cf in enumerate(allcfs.crazyflies)) / TRANS_AVG_SPEED)
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0.0, 0.0, 0.75])
            cf.goTo(pos, 0, home_dur)
        timeHelper.sleep(home_dur + 0.5)

        phase('landing')
        # gentle descent: ~0.71 m from the 0.75 m return-home hover over
        # 4.5 s is ~0.16 m/s
        allcfs.land(targetHeight=0.04, duration=4.5)
        timeHelper.sleep(5.0)
        phase('landed - done')

    # disable logging
    allcfs.setParam('usd.logging', 0)


if __name__ == '__main__':
    main()
