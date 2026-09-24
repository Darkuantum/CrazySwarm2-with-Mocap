#!/usr/bin/env python3
"""Fly the escort demo: three defenders ring a VIP, a fourth probes, they block.

    # ALWAYS sim first
    ros2 launch crazyflie_shows show_launch.py backend:=sim
    ros2 run crazyflie_shows escort_show --ros-args -p use_sim_time:=true

    # hardware: scan every enabled address, preflight GUI, plan_escort, THEN
    ros2 run crazyflie_shows escort_show --ros-args -p vip_mode:=point

All geometry and every clamp live in :mod:`crazyflie_shows.escort`, which is
pure -- ``plan_escort`` verifies the same objects this flies. What is here is
the part that needs a radio and a mocap stream.

This is NOT a choreographed show. ``swarm_show`` and ``constellation_show``
command a plan that was fully known before takeoff; this one streams
setpoints that depend on where a person and an adversary actually are. Three
consequences the other two scripts do not have to handle, and this one does:

* **Staleness is fatal, not cosmetic.** A show keeps flying a correct plan if
  mocap hiccups. Here a stale VIP pose means the ring is centred on where
  somebody *was*. So: no VIP pose for ``vip_stale_hold_s`` and the defenders
  stop where they are; for ``vip_stale_land_s`` and they land, uninvited.
  Landing next to a confused operator beats orbiting a guess.
* **The setpoint stream owns the drones.** Streamed setpoints bypass the
  high-level commander, so ``notifySetpointsStop`` must be called before any
  goTo/land, or the stale stream fights the landing.
  The stream is ``cmdFullState``, not ``cmdPosition``, for one blunt reason:
  ``crazyflie_sil.cmdPosition`` is commented out and the sim server subscribes
  only to ``cmd_full_state``, so a cmdPosition demo cannot be flown in sim at
  all -- it would take off, hover, and ignore every setpoint. Hardware takes
  both. Sending the guard's own velocity as the feedforward term is also
  better than sending position alone.
* **The adversary is scripted** (``adversary:=scripted``) until somebody has
  watched the block work. ``adversary:=external`` instead tracks a rigid body
  flown by a human -- a teleoperated Crazyflie, or later something bigger --
  and commands nothing.

Parameters (``--ros-args -p name:=value``)
------------------------------------------
======================  ==========  =================================
``dry_run``             false       verify and report, then exit
``vip_mode``            point       ``point`` | ``mocap``
``vip_point``           room centre m, the static VIP mark; defaults to
                        + offset   room centre + EscortConfig.vip_offset
``vip_name``            operator    rigid body to escort (vip_mode:=mocap)
``adversary``           scripted    ``scripted`` | ``external`` | ``none``
``adversary_name``      adv         its rigid body (adversary:=external)
``defenders``           ''          e.g. ``cf1,cf2,cf3``; default = first 3
                                    enabled drones (comma-separated STRING,
                                    not a list -- see the code comment)
``adversary_drone``     ''          which drone flies the script; default =
                                    the 4th enabled drone
``lights``              true*       LED cues (*off under use_sim_time)
``ring_radius``         1.5         m  } overrides of EscortConfig; every
``height``              1.2         m  } one is re-verified before takeoff,
``v_max``               0.6         m/s} and refused if it does not check out
``phase_rate``          0.3         rad/s
``check_placement``     true        compare live pose to initial_position
======================  ==========  =================================
"""

import sys

import numpy as np
import rclpy
from crazyflie_py import Crazyswarm
from motion_capture_tracking_interfaces.msg import NamedPoseArray

from crazyflie_shows import escort, safety
from crazyflie_shows.constellation_show import (ShowAborted, abort_land, cue,
                                                take_signals)
from crazyflie_shows.swarm_show import _param, check_placement

#: wrgb8888 values for the Color LED deck. The adversary must be obviously
#: different from every defender at a glance, from across the room, on video.
LED_DEFENDER = 0xFF0000FF      # blue
LED_ADVERSARY = 0xFFFF0000     # red
LED_BLOCKING = 0xFFFFFF00      # amber, the defender currently on the threat line

TAKEOFF_HEIGHT = 0.6
TAKEOFF_DURATION = 3.0
LAND_HEIGHT = 0.04
LAND_DURATION = 4.0


class PoseCache:
    """Latest ``/poses`` entry per rigid body, with its arrival time.

    Reads ``/poses``, never ``/cfX/pose``: the onboard estimate is seeded from
    the yaml, so using it as ground truth is circular (CLAUDE.md). The VIP and
    the adversary are not drones anyway -- they only exist on ``/poses``.
    """

    def __init__(self, node, topic='/poses'):
        self.node = node
        self.pos = {}
        self.stamp = {}
        node.create_subscription(NamedPoseArray, topic, self._cb, 10)

    def _cb(self, msg):
        t = self.node.get_clock().now().nanoseconds * 1e-9
        for np_ in msg.poses:
            p = np_.pose.position
            self.pos[np_.name] = np.array([p.x, p.y, p.z])
            self.stamp[np_.name] = t

    def get(self, name, now, max_age):
        """Position if it is fresher than ``max_age``, else None."""
        if name not in self.pos or now - self.stamp[name] > max_age:
            return None
        return self.pos[name].copy()

    def age(self, name, now):
        return np.inf if name not in self.stamp else now - self.stamp[name]


def _cfg_from_params(node):
    cfg = escort.EscortConfig()
    for name in ('ring_radius', 'height', 'v_max', 'phase_rate', 'alert_radius',
                 'release_radius', 'min_vip_dist', 'rate_hz'):
        setattr(cfg, name, float(_param(node, name, getattr(cfg, name))))
    return cfg


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    node = allcfs
    cfs = allcfs.crazyflies
    names = [cf.prefix.lstrip('/') for cf in cfs]

    sim = bool(_param(node, 'use_sim_time', False))
    dry_run = bool(_param(node, 'dry_run', False))
    lights_on = bool(_param(node, 'lights', not sim))
    vip_mode = str(_param(node, 'vip_mode', 'point'))
    vip_name = str(_param(node, 'vip_name', 'operator'))
    cfg_default = escort.EscortConfig()
    # The static VIP mark is room centre plus vip_offset, NOT room centre: a
    # centred VIP leaves the adversary no room to approach through (see
    # EscortConfig.vip_offset and ESCORT.md).
    vip_point = list(_param(node, 'vip_point',
                            escort.vip_home(cfg_default)[:2].tolist()))
    adv_mode = str(_param(node, 'adversary', 'scripted'))
    adv_name = str(_param(node, 'adversary_name', 'adv'))
    # A comma-separated STRING, not a string array: rclpy infers the type of
    # a `[]` default as BYTE_ARRAY, so `-p defenders:=[cf1,cf2,cf3]` dies with
    # InvalidParameterTypeException (verified 2026-09-23). Same family as the
    # `declare_parameter(name, {})` trap in CLAUDE.md -- an empty container
    # default carries no type.
    want_defenders = [n for n in
                      str(_param(node, 'defenders', '')).replace(' ', '').split(',') if n]
    adv_drone_name = str(_param(node, 'adversary_drone', ''))
    want_placement = bool(_param(node, 'check_placement', True))

    cfg = _cfg_from_params(node)
    script = escort.AdversaryScript(height=cfg.height)

    # ------------------------------------------------------------ who is who
    if want_defenders:
        missing = [n for n in want_defenders if n not in names]
        if missing:
            raise SystemExit(f'defenders {missing} are not enabled in the yaml '
                             f'(enabled: {names})')
        defenders = list(want_defenders)
        auto_adv = None
    else:
        # By geometry, not by name order -- see escort.pick_roles.
        starts = {n: np.array(cf.initialPosition, float)
                  for n, cf in zip(names, cfs)}
        defenders, auto_adv = escort.pick_roles(names, starts, cfg)
    if len(defenders) < cfg.n_defenders:
        raise SystemExit(f'need {cfg.n_defenders} defenders, the stack has '
                         f'{len(names)} drones ({names})')

    adv_drone = None
    if adv_mode == 'scripted':
        adv_drone = (adv_drone_name or auto_adv
                     or next((n for n in names if n not in defenders), ''))
        if not adv_drone:
            raise SystemExit('adversary:=scripted needs a drone that is not a '
                             f'defender; enabled = {names}, defenders = {defenders}')
    idx = {n: i for i, n in enumerate(names)}
    dcfs = [cfs[idx[n]] for n in defenders]
    acf = cfs[idx[adv_drone]] if adv_drone else None

    # ------------------------------------------------------------- verify it
    print('\n  ESCORT DEMO\n')
    print(f'  defenders     {", ".join(defenders)}')
    print(f'  adversary     {adv_mode}'
          f'{" as " + adv_drone if adv_drone else ""}'
          f'{" (rigid body " + adv_name + ")" if adv_mode == "external" else ""}')
    vip_desc = (str(np.round(vip_point, 3).tolist()) if vip_mode == 'point'
                else vip_name)
    print(f'  VIP           {vip_mode} {vip_desc}')
    print(f'  ring          {cfg.ring_radius:.2f} m at {cfg.height:.2f} m, '
          f'turning at {cfg.phase_rate:.2f} rad/s, cap {cfg.v_max:.2f} m/s')
    print(f'  walk budget   {escort.max_vip_speed(cfg):+.2f} m/s '
          '(how fast the VIP may move)\n')

    problems = escort.check_config(cfg, moving_vip=(vip_mode == 'mocap'))
    if adv_mode == 'scripted':
        problems += script.check(cfg)
    for b in problems:
        print(f'  *** {b}')
    if problems:
        raise SystemExit('\n  REFUSED: fix the configuration (plan_escort '
                         'reproduces this offline, with no drones).\n')
    # --- the gather, checked on the ground, from the yaml marks -----------
    # Predictive: the real check runs again after takeoff on live poses. This
    # one exists so a bad placement is caught BEFORE anything arms, which is
    # the only moment it can still be fixed by moving a drone.
    marks = {n: np.array(cf.initialPosition, float) for n, cf in zip(names, cfs)}
    p_vip0 = np.array([vip_point[0], vip_point[1], 0.0]) if vip_mode == 'point' else None
    if p_vip0 is not None:
        src = [np.array([*marks[n][:2], TAKEOFF_HEIGHT]) for n in defenders]
        ctrl0 = escort.EscortController(cfg, src, p_vip0)
        slots0 = escort.ring_targets(p_vip0, ctrl0.phase.psi, cfg)
        dst = [slots0[ctrl0.slot_of(i)] for i in range(len(defenders))]
        labels = list(defenders)
        if adv_drone:
            src.append(np.array([*marks[adv_drone][:2], TAKEOFF_HEIGHT]))
            dst.append(script.target(0.0, p_vip0))
            labels.append(adv_drone)
            why = escort.adversary_start_problem(marks[adv_drone], p_vip0, cfg)
            if why:
                problems.append(why)
        problems += escort.check_gather(src, dst, labels, cfg)
        for n, a, b in zip(labels, src, dst):
            print(f'    {n:>4}  {np.round(a[:2], 2).tolist()} -> '
                  f'{np.round(b[:2], 2).tolist()}   '
                  f'{float(np.linalg.norm(b - a)):.2f} m')
        print(f'    gather legs clear {safety.transition_min_sep(src, dst):.2f} m '
              f'(needs {safety.PLAN_SEPARATION:.2f})\n')
    for b in problems:
        print(f'  *** {b}')
    if problems:
        raise SystemExit('\n  REFUSED: fix the above before arming.\n')

    print('  configuration checks out. Run plan_escort for the full '
          'encounter simulation.\n')
    if dry_run:
        print('  dry_run - nothing armed.\n')
        return 0

    # ------------------------------------------------------------- preflight
    poses = PoseCache(node)
    if vip_mode == 'mocap' or adv_mode == 'external':
        need = ([vip_name] if vip_mode == 'mocap' else []) + \
               ([adv_name] if adv_mode == 'external' else [])
        print(f'  waiting for rigid bodies on /poses: {", ".join(need)}')
        deadline = timeHelper.time() + 10.0
        while timeHelper.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(n in poses.pos for n in need):
                break
        missing = [n for n in need if n not in poses.pos]
        if missing:
            raise SystemExit(
                f'  no /poses entry for {missing}. Create the rigid body in '
                'Motive with exactly that name (asymmetric markers, so the '
                'body cannot flip), and check the mocap node is publishing.\n')
        print('    all present\n')

    if want_placement and not sim:
        print('  PLACEMENT CHECK (live pose vs initial_position)')
        check_placement(node, cfs, names, _YamlStarts(cfs),
                        float(_param(node, 'placement_tol', 0.25)))
        print('    all drones within tolerance\n')

    def vip_now(now):
        if vip_mode == 'point':
            return np.array([vip_point[0], vip_point[1], 0.0])
        return poses.get(vip_name, now, cfg.vip_stale_hold_s)

    # ------------------------------------------------------------------ fly
    armed = False
    streaming = False
    try:
        take_signals()
        for cf in cfs:
            cf.arm(True)
        armed = True
        timeHelper.sleep(1.0)

        if lights_on:
            for cf in dcfs:
                cue(allcfs, [cf], [LED_DEFENDER])
            if acf is not None:
                cue(allcfs, [acf], [LED_ADVERSARY])

        allcfs.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=TAKEOFF_DURATION)
        timeHelper.sleep(TAKEOFF_DURATION + 0.5)

        # gather onto the ring with the high-level commander, BEFORE streaming:
        # the goTos are the one leg where drones cross the volume, and goTo is
        # smoother than yanking a fresh setpoint stream across the room.
        now = timeHelper.time()
        rclpy.spin_once(node, timeout_sec=0.05)
        p_vip = vip_now(now)
        if p_vip is None:
            raise RuntimeError(f'no fresh pose for the VIP ({vip_name}) at takeoff')
        here = [_live(poses, n, cf, now) for n, cf in zip(defenders, dcfs)]
        ctrl = escort.EscortController(cfg, here, p_vip)
        slots = escort.ring_targets(p_vip, ctrl.phase.psi, cfg)
        gather = max(3.0, float(np.max([np.linalg.norm(slots[ctrl.slot_of(i)] - here[i])
                                        for i in range(len(dcfs))])) / 0.4)
        # Same check again, now on where the drones REALLY are. Refusing here
        # means landing, not exiting -- they are already in the air.
        live_src = list(here)
        live_dst = [slots[ctrl.slot_of(i)] for i in range(len(dcfs))]
        live_labels = list(defenders)
        if acf is not None:
            live_src.append(_live(poses, adv_drone, acf, now))
            live_dst.append(script.target(0.0, p_vip))
            live_labels.append(adv_drone)
        why = escort.check_gather(live_src, live_dst, live_labels, cfg)
        if acf is not None:
            bad_adv = escort.adversary_start_problem(live_src[-1], p_vip, cfg)
            if bad_adv:
                why.append(bad_adv)
        if why:
            raise RuntimeError('gather refused on live poses: ' + '; '.join(why))

        print(f'  gathering onto the ring ({gather:.1f} s)')
        for i, cf in enumerate(dcfs):
            cf.goTo(slots[ctrl.slot_of(i)], 0.0, gather)
        if acf is not None:
            acf.goTo(script.target(0.0, p_vip), 0.0, gather)
        timeHelper.sleep(gather + 0.5)

        # The gather was flown by the high-level commander, so the guards'
        # internal setpoints are still at the pre-gather positions. Re-seed
        # them from the live poses or the first streamed setpoint yanks every
        # defender back across the ring (see EscortController.resync).
        rclpy.spin_once(node, timeout_sec=0.05)
        now = timeHelper.time()
        seeds = []
        for i, (n, cf) in enumerate(zip(defenders, dcfs)):
            want = slots[ctrl.slot_of(i)]
            got = _live(poses, n, cf, now)
            # Trust the live pose only if it is plausible. In sim the fallback
            # estimate can still read near the floor right after a goTo, and
            # seeding a setpoint there commands the drone DOWN, which then
            # trips the altitude and defender clamps (observed 2026-09-23).
            # The same guard matters on hardware: a stale or wrong pose here
            # is the one input nothing else checks.
            if got[2] < cfg.floor or float(np.linalg.norm(got - want)) > 1.0:
                print(f'    {n}: live pose {np.round(got, 2).tolist()} is not '
                      f'near its slot {np.round(want, 2).tolist()} -- seeding '
                      'from the slot instead', flush=True)
                got = want
            seeds.append(got)
        ctrl.resync(seeds)

        # ---------------------------------------------------------- stream
        t0 = timeHelper.time()
        duration = script.duration if adv_mode != 'none' else 30.0
        adv_guard = escort.SetpointGuard(cfg, script.target(0.0, p_vip)) \
            if acf is not None else None
        print(f'  ESCORT LIVE - {duration:.0f} s, streaming at '
              f'{cfg.rate_hz:g} Hz\n')
        streaming = True
        was_engaged = None
        last_clamp = ''
        last_untrusted = False
        last = t0
        while True:
            rclpy.spin_once(node, timeout_sec=0.0)
            now = timeHelper.time()
            t = now - t0
            if t >= duration:
                break
            step = max(now - last, 1e-3)
            last = now

            p_vip = vip_now(now)
            if p_vip is None:
                age = poses.age(vip_name, now)
                if age > cfg.vip_stale_land_s:
                    raise RuntimeError(
                        f'VIP ({vip_name}) has been lost for {age:.1f} s -- '
                        'landing rather than escorting a guess')
                # hold: keep the last setpoints, command nothing new
                for cf, g in zip(dcfs, ctrl.guards):
                    _stream(cf, g.sp, np.zeros(3))
                timeHelper.sleepForRate(cfg.rate_hz)
                continue

            if adv_mode == 'scripted':
                p_adv = adv_guard.step(script.target(t, p_vip), step)
                _stream(acf, p_adv, adv_guard.v)
            elif adv_mode == 'external':
                p_adv = poses.get(adv_name, now, 0.5)     # gone = no threat
            else:
                p_adv = None

            live = [_live(poses, n, cf, now) for n, cf in zip(defenders, dcfs)]
            sp, info = ctrl.step(step, p_vip, p_adv, live)
            for i, cf in enumerate(dcfs):
                _stream(cf, sp[i], ctrl.guards[i].v)

            if info['engaged'] != was_engaged:
                was_engaged = info['engaged']
                what = ('BLOCKING - ring facing the adversary' if was_engaged
                        else 'clear - ring at rest')
                print(f'  [t+{t:5.1f}s] {what}', flush=True)
                if lights_on:
                    blocker = next((i for i in range(len(dcfs))
                                    if ctrl.slot_of(i) == 0), 0)
                    cue(allcfs, [dcfs[blocker]],
                        [LED_BLOCKING if was_engaged else LED_DEFENDER])
            # Only SEPARATION clamps are worth a line, and only when the set
            # of them changes: a slew clamp is the guard doing its job, and
            # printing either one per step buries the run in its own log.
            who = ', '.join(f'{defenders[i]}:{"+".join(sorted(set(r) - {"speed", "accel"}))}'
                            for i, r in sorted(info['clamped'].items())
                            if set(r) - {'speed', 'accel'})
            if info['untrusted'] and not last_untrusted:
                print(f'  [t+{t:5.1f}s] ignoring the live pose of '
                      f'{", ".join(defenders[i] for i in info["untrusted"])} '
                      '-- too far from what was commanded to be believable',
                      flush=True)
            last_untrusted = bool(info['untrusted'])
            if who != last_clamp:
                last_clamp = who
                if who:
                    print(f'  [t+{t:5.1f}s] guard clamped {who}', flush=True)
            timeHelper.sleepForRate(cfg.rate_hz)

        # ------------------------------------------------------------ land
        for cf in dcfs + ([acf] if acf else []):
            cf.notifySetpointsStop()
        streaming = False
        allcfs.land(targetHeight=LAND_HEIGHT, duration=LAND_DURATION)
        timeHelper.sleep(LAND_DURATION + 0.5)
        for cf in cfs:
            cf.arm(False)
        armed = False
        print('\n  landed, disarmed - done\n')
        return 0

    except ShowAborted as e:
        if armed:
            _stop_stream(dcfs, acf, streaming)
            abort_land(allcfs, cfs, timeHelper, _LandCfg(), str(e))
        return 130
    except BaseException as e:                        # noqa: BLE001
        if armed:
            _stop_stream(dcfs, acf, streaming)
            abort_land(allcfs, cfs, timeHelper, _LandCfg(), repr(e))
        raise


#: Streamed setpoints carry no acceleration or body rate: the ring is a slow
#: position task and a fabricated acceleration would only fight the onboard
#: controller.
_ZERO = np.zeros(3)


def _stream(cf, pos, vel):
    """One streamed setpoint. See the module docstring for why not cmdPosition."""
    cf.cmdFullState(np.asarray(pos, float), np.asarray(vel, float), _ZERO,
                    0.0, _ZERO)


def _live(poses, name, cf, now):
    """Where a defender actually is: /poses first, onboard estimate second.

    /poses is the mocap truth the whole demo is built on. ``get_position()``
    is the drone's own estimate, which is seeded from the yaml and so agrees
    with the plan even when the drone is in the wrong place -- fine as a
    fallback for one stale frame, wrong as the primary source (CLAUDE.md,
    "initial_position comes from /poses, never /cfX/pose").
    """
    p = poses.get(name, now, 0.3)
    return p if p is not None else np.array(cf.get_position(), float)


class _YamlStarts:
    """``check_placement`` wants a plan; all it reads is ``.starts``."""

    def __init__(self, cfs):
        self.starts = [np.array(cf.initialPosition, float) for cf in cfs]


class _LandCfg:
    """What ``abort_land`` reads off a show config (it only wants the height)."""

    land_height = LAND_HEIGHT


def _stop_stream(dcfs, acf, streaming):
    """Hand the drones back to the high-level commander before landing them.

    Without this the setpoint stream is still the most recent thing the
    firmware heard, and the landing is fought by a stream that stopped being
    updated the moment the exception was raised.
    """
    if not streaming:
        return
    for cf in list(dcfs) + ([acf] if acf else []):
        try:
            cf.notifySetpointsStop()
        except Exception:                              # noqa: BLE001
            pass


if __name__ == '__main__':
    sys.exit(main())
