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

What bounds a manual adversary
------------------------------
With ``adversary:=scripted`` every leg is proven on the ground:
``AdversaryScript.check`` refuses a script whose closest approach is inside
``ring_radius + min_adv_sep``, so the attacker cannot be *told* to fly into
the ring. ``adversary:=manual`` has no script to check, so the proof has to be
replaced by a clamp at flight time.

MEASURED in sim, 2026-10-02, with no clamp: holding one direction key flew the
commanded adversary from -1.47 m straight THROUGH the VIP point, passing
**0.036 m** from it, and it stopped only at the arena wall. The defenders were
fine throughout (0.999 m against their 1.00 m VIP floor) -- they yield to the
attacker by design, so nothing in the ring slows it down.

It now gets exactly one extra floor: ``min_adv_sep`` from the VIP, applied
after the slew like every other separation. That is 0.80 m -- deliberately far
less than the ``ring_radius + min_adv_sep`` the scripted legs respect, because
an attacker halted at 1.80 m would look the same whether or not any defender
was flying. It must be able to penetrate the ring for the block to read; it
must not be able to reach the DJI.

It is still NOT clamped against the defenders, and that is deliberate: the
probe radius IS ``ring_radius + min_adv_sep``, so at the probe the lead
defender stands exactly ``min_adv_sep`` away -- on the boundary, with a 1 mm
deadband. Pushing both apart there would have the guards firing every step of
every encounter, which is the condition they exist to REPORT. The defenders
already give way, but with lag, and the VIP floor makes that lag WORSE: the
attacker now parks 0.80 m off the VIP, which is where the blocker wants to
stand, so measured adversary-to-defender fell from 0.758 m (no VIP floor) to
0.643 m against a 0.800 m figure. That is slew, not policy, and 0.64 m between
two 10 cm airframes is not a collision -- but it is the number to watch, and
clamping the pair apart as well is an open decision, not a settled one.

0.80 m is a reused constant, not a measurement of what a DJI's prop wash does
to a 30 g Crazyflie (ESCORT.md, "A DJI standoff is still unset"). Treat it as
a floor that stops a collision, not as a safe hovering distance.

Parameters (``--ros-args -p name:=value``)
------------------------------------------
======================  ==========  =================================
``dry_run``             false       verify and report, then exit
``vip_mode``            point       ``point`` | ``mocap`` | ``manual``
                                    ``manual`` = a PointStamped you drive on
                                    /escort/vip (escort_teleop), for testing
                                    with nobody and no DJI in the volume
``vip_point``           room centre m, the static VIP mark; defaults to
                        + offset   room centre + EscortConfig.vip_offset
``vip_name``            operator    rigid body to escort (vip_mode:=mocap)
``adversary``           scripted    ``scripted`` | ``external`` | ``manual``
                                    | ``none``. ``manual`` = /escort/adversary,
                                    driven by escort_teleop. READ THE WARNING
                                    under "A manual adversary is unverified"
                                    below before flying it near anything.
``adversary_name``      adv         its rigid body (adversary:=external)
``defenders``           ''          e.g. ``cf1,cf2,cf3``; default = first 3
                                    enabled drones (comma-separated STRING,
                                    not a list -- see the code comment)
``adversary_drone``     ''          which drone flies the script; default =
                                    the 4th enabled drone
``lights``              true*       LED cues (*off under use_sim_time)
``paced``               false       OPERATOR-PACED. The adversary holds its
                                    current leg until you press Enter, and
                                    arming waits for you too. Show time still
                                    runs -- the controller, the guards and
                                    every staleness watchdog are real-time --
                                    only the attacker's place in its script is
                                    yours. Reads a LINE, so it works from a
                                    terminal or the mission console's stdin
                                    box. 'q' lands.
``check_placement``     true        compare live pose to initial_position
``placement_tol``       0.25        m, how far off a drone may be
``vip_airborne``        (auto)      ring rides the VIP's own altitude;
                                    defaults true only for vip_mode:=mocap
``duration``            (script)    s. The script's own length for
                                    adversary:=scripted, else 120 s
``use_sim_time``        false       REQUIRED under backend:=sim, NEVER on
                                    hardware
----------------------  ----------  ---------------------------------
Overrides of EscortConfig (``_cfg_from_params``). Every one is re-verified
before takeoff and refused if it does not check out:
``ring_radius``         1.0         m. Also the chord the closing wall uses
``height``              1.2         m
``v_max``               0.6         m/s
``phase_rate``          0.3         rad/s
``alert_radius``        1.88        m, adversary-to-VIP distance that ENGAGES
``release_radius``      2.0         m, the larger one that disengages
``min_vip_dist``        1.0         m, hard floor defender-to-VIP
                                    (= ring_radius by design)
``rate_hz``             20.0        setpoint stream rate per drone
``vip_height_offset``   0.0         m, ring altitude relative to an airborne
                                    VIP
======================  ==========  =================================

``probe < alert < release < retreat`` has to hold; ``plan_escort`` refuses if
it does not, so re-run it after changing either radius.
"""

import select
import sys

import numpy as np
import rclpy
from crazyflie_py import Crazyswarm
from geometry_msgs.msg import PointStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray

from crazyflie_shows import escort, safety
from crazyflie_shows.escort_teleop import ADVERSARY_TOPIC, VIP_TOPIC
from crazyflie_shows.constellation import LIGHT
from crazyflie_shows.escort_viz import EscortViz
from crazyflie_shows.constellation_show import (ShowAborted, abort_land, cue,
                                                take_signals)
from crazyflie_shows.swarm_show import _param, check_placement

#: wrgb8888 values for the Color LED deck. The adversary must be obviously
#: different from every defender at a glance, from across the room, on video.
# Built through constellation.wrgb(), which enforces the two rules these
# constants used to break. They were 0xFF0000FF / 0xFFFF0000 / 0xFFFFFF00:
#
#  * the white channel was 0xFF on all three. wrgb() caps it below 0x80 because
#    the value travels as a ROS integer parameter on its way to the firmware's
#    uint32, and 0xFF in the top byte puts every one of them past int32 max
#    (0xFF0000FF = 4_278_190_335 > 2_147_483_647).
#  * with the white LED at FULL on every cue, "blue", "red" and "amber" all
#    washed toward white -- three colours that had to be told apart across a
#    room, rendered as three shades of white.
#
# Never spotted because escort has never flown and backend:=sim has no LED
# deck at all (the cue raises keyError there and is swallowed by cue()).
LED_DEFENDER = LIGHT['deep_blue']    # on the ring, nothing to do
LED_ADVERSARY = LIGHT['red']         # the threat
LED_BLOCKING = LIGHT['amber']        # the LEAD, standing on the threat bearing
LED_WING = LIGHT['violet']           # the other two, closed up beside the lead

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


class PointCache:
    """Latest keyboard-driven target, with its arrival time.

    Deliberately the same shape as :class:`PoseCache`: a manual target that
    stops arriving is treated exactly like a mocap body that dropped out --
    hold, then land. Closing the teleop is therefore a legitimate way to end a
    test, and a teleop that crashes is not a way to strand the drones.
    """

    def __init__(self, node, topic):
        self.node = node
        self.pos = None
        self.stamp = -1e9
        node.create_subscription(PointStamped, topic, self._cb, 10)

    def _cb(self, msg):
        self.stamp = self.node.get_clock().now().nanoseconds * 1e-9
        self.pos = np.array([msg.point.x, msg.point.y, msg.point.z])

    def get(self, now, max_age):
        if self.pos is None or now - self.stamp > max_age:
            return None
        return self.pos.copy()

    def age(self, now):
        return np.inf if self.pos is None else now - self.stamp


def _cfg_from_params(node):
    cfg = escort.EscortConfig()
    for name in ('ring_radius', 'height', 'v_max', 'phase_rate', 'alert_radius',
                 'release_radius', 'min_vip_dist', 'rate_hz',
                 'vip_height_offset'):
        setattr(cfg, name, float(_param(node, name, getattr(cfg, name))))
    return cfg


#: Operator pacing. The show is an encounter between three drones, a scripted
#: attacker and a human flying a DJI; coordinating that on a wall clock means
#: every party has to be ready at a time nobody chose. Paced mode replaces the
#: adversary's timed legs with "the operator presses Enter", so the attacker
#: only ever moves when the person watching the room decides it should.
#:
#: Reads a LINE, not a keypress: no termios, no cbreak, so it works from a
#: plain terminal AND from the mission console's stdin box. select() keeps it
#: non-blocking, because the defenders are streaming setpoints at 20 Hz and a
#: blocking read would stall them into their own staleness watchdog.
#: m the VIP must come back INSIDE the keep-in radius before the ring
#: resumes following. Without hysteresis a DJI hovering on the line would make
#: the formation start and stop following several times a second.
FENCE_HYST = 0.25


def operator_said(block=False, timeout=0.0):
    """'' if nothing typed; else the stripped line ('' counts as Enter -> ' ')."""
    try:
        r, _, _ = select.select([sys.stdin], [], [], None if block else timeout)
    except Exception:                                   # noqa: BLE001
        return ''
    if not r:
        return ''
    line = sys.stdin.readline()
    if line == '':                                      # EOF: stdin closed
        return 'EOF'
    return line.strip() or ' '                          # bare Enter


def operator_gate(prompt, pump=None):
    """Hold until the operator says go. ``pump`` keeps the drones fed."""
    print(f'\n  >>> {prompt}\n      [Enter] continue   [q] land now', flush=True)
    while True:
        said = operator_said(block=(pump is None), timeout=0.05)
        if said in ('q', 'Q', 'EOF'):
            raise ShowAborted('operator asked to land')
        if said:
            return
        if pump is not None:
            pump()


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
    paced = bool(_param(node, 'paced', False))
    _armed_yet = {'v': False}

    if vip_mode not in ('point', 'mocap', 'manual'):
        raise SystemExit(f"vip_mode must be point | mocap | manual, not {vip_mode!r}")
    if adv_mode not in ('scripted', 'external', 'manual', 'none'):
        raise SystemExit("adversary must be scripted | external | manual | none, "
                         f'not {adv_mode!r}')

    cfg = _cfg_from_params(node)
    # A mocap VIP is airborne by default, because that is what the live demo
    # is: a hand-flown DJI being escorted. A person wearing a hat is the
    # exception and says so with vip_airborne:=false.
    cfg.vip_airborne = bool(_param(node, 'vip_airborne', vip_mode == 'mocap'))
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
    if adv_mode in ('scripted', 'manual'):
        adv_drone = (adv_drone_name or auto_adv
                     or next((n for n in names if n not in defenders), ''))
        if not adv_drone:
            raise SystemExit(f'adversary:={adv_mode} needs a drone that is not '
                             f'a defender; enabled = {names}, '
                             f'defenders = {defenders}')
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
                else VIP_TOPIC if vip_mode == 'manual' else vip_name)
    print(f'  VIP           {vip_mode} {vip_desc}')
    where = (f'level with the VIP {cfg.vip_height_offset:+.2f} m'
             if cfg.vip_airborne else f'at {cfg.height:.2f} m')
    print(f'  ring          {cfg.ring_radius:.2f} m, {where}, '
          f'turning at {cfg.phase_rate:.2f} rad/s, cap {cfg.v_max:.2f} m/s')
    if cfg.vip_airborne:
        print(f'  VIP keep-in   stay within {escort.vip_keep_in(cfg):.2f} m of '
              f'[{cfg.room_center[0]:+.2f}, {cfg.room_center[1]:+.2f}] and under '
              f'{escort.max_vip_speed(cfg):.2f} m/s, or the ring cannot hold')
    print(f'  walk budget   {escort.max_vip_speed(cfg):+.2f} m/s '
          '(how fast the VIP may move)\n')

    problems = escort.check_config(cfg, moving_vip=(vip_mode != 'point'))
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
    # A manual VIP starts wherever the teleop starts, which is the same mark
    # by default -- close enough to check the gather against before arming.
    p_vip0 = (np.array([vip_point[0], vip_point[1], 0.0])
              if vip_mode in ('point', 'manual') else None)
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

    manual_vip = PointCache(node, VIP_TOPIC) if vip_mode == 'manual' else None
    manual_adv = PointCache(node, ADVERSARY_TOPIC) if adv_mode == 'manual' else None
    for cache, what, topic in ((manual_vip, 'VIP', VIP_TOPIC),
                               (manual_adv, 'adversary', ADVERSARY_TOPIC)):
        if cache is None:
            continue
        # Refusing to arm until the operator's keyboard is actually publishing
        # is the point: a manual target that never arrives is a demo that
        # takes off and immediately holds, then lands.
        print(f'  waiting for the {what} teleop on {topic}')
        print(f'    ros2 run crazyflie_shows escort_teleop --ros-args '
              f'-p target:={"vip" if cache is manual_vip else "adversary"}')
        deadline = timeHelper.time() + 20.0
        while timeHelper.time() < deadline and cache.pos is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if cache.pos is None:
            raise SystemExit(f'\n  nothing is publishing {topic}. Start the '
                             'teleop in another terminal first.\n')
        print(f'    {what} at {np.round(cache.pos, 2).tolist()}\n')

    if want_placement and not sim:
        print('  PLACEMENT CHECK (live pose vs initial_position)')
        check_placement(node, cfs, names, _YamlStarts(cfs),
                        float(_param(node, 'placement_tol', 0.25)))
        print('    all drones within tolerance\n')

    def vip_now(now):
        if vip_mode == 'point':
            return np.array([vip_point[0], vip_point[1], 0.0])
        if vip_mode == 'manual':
            return manual_vip.get(now, cfg.vip_stale_hold_s)
        return poses.get(vip_name, now, cfg.vip_stale_hold_s)

    def vip_age(now):
        if vip_mode == 'manual':
            return manual_vip.age(now)
        return poses.age(vip_name, now)

    # ------------------------------------------------------------------ fly
    armed = False
    streaming = False
    try:
        take_signals()
        # Only the drones with a role are armed and flown. A spare hovering
        # through the demo is a drone in the separation budget, in the shot,
        # and on the radio, doing nothing to earn any of it. The ABORT path
        # still broadcasts to everything -- landing a drone that should not be
        # flying is never the wrong move.
        flying = list(dcfs) + ([acf] if acf is not None else [])
        idle = [n for n in names if n not in defenders and n != adv_drone]
        if idle:
            print(f'  grounded (no role): {", ".join(idle)}')
        for cf in flying:
            if paced and not _armed_yet['v']:
                _armed_yet['v'] = True
                operator_gate('ARM and take off the defenders? Everyone clear '
                              'of the volume, DJI still on the ground, E-STOP in '
                              'hand.')
            cf.arm(True)
        armed = True
        timeHelper.sleep(1.0)

        if lights_on:
            for cf in dcfs:
                cue(allcfs, [cf], [LED_DEFENDER])
            if acf is not None:
                cue(allcfs, [acf], [LED_ADVERSARY])

        for cf in flying:
            cf.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=TAKEOFF_DURATION)
        timeHelper.sleep(TAKEOFF_DURATION + 0.5)

        # gather onto the ring with the high-level commander, BEFORE streaming:
        # the goTos are the one leg where drones cross the volume, and goTo is
        # smoother than yanking a fresh setpoint stream across the room.
        now = timeHelper.time()
        rclpy.spin_once(node, timeout_sec=0.05)
        p_vip = vip_now(now)
        if p_vip is None:
            raise RuntimeError(f'no fresh pose for the VIP ({vip_name}) at takeoff')
        adv_first = (manual_adv.get(now, 2.0) if adv_mode == 'manual'
                     else script.target(0.0, vip_now(now)))
        if adv_mode == 'manual' and adv_first is None:
            raise RuntimeError('the adversary teleop stopped publishing before '
                               'the gather')
        # Expected: straight up off its own mark, which is what takeoff does.
        blind = []
        here = []
        for n, cf in zip(defenders, dcfs):
            want = np.array([*marks[n][:2], TAKEOFF_HEIGHT])
            p, ok = _live(poses, n, cf, now, expect=want)
            here.append(p)
            if not ok:
                blind.append(n)
        ctrl = escort.EscortController(cfg, here, p_vip)
        # RViz markers: the VIP, the commanded ring, the threat bearing and a
        # trail of what each drone was told. In sim the only thing on screen is
        # the drones' own /tf, so there is nothing to tell an intercept from a
        # follow -- see escort_viz. Costs nothing when RViz is not looking.
        viz = EscortViz(node, len(dcfs), names=defenders)
        slots = escort.ring_targets(p_vip, ctrl.phase.psi, cfg)
        gather = max(3.0, float(np.max([np.linalg.norm(slots[ctrl.slot_of(i)] - here[i])
                                        for i in range(len(dcfs))])) / 0.4)
        # Same check again, now on where the drones REALLY are. Refusing here
        # means landing, not exiting -- they are already in the air.
        live_src = list(here)
        live_dst = [slots[ctrl.slot_of(i)] for i in range(len(dcfs))]
        live_labels = list(defenders)
        if acf is not None:
            p_a, ok_a = _live(poses, adv_drone, acf, now,
                              expect=np.array([*marks[adv_drone][:2], TAKEOFF_HEIGHT]))
            if not ok_a:
                blind.append(adv_drone)
            live_src.append(p_a)
            live_dst.append(adv_first)
            live_labels.append(adv_drone)
        if blind:
            why_blind = ('(expected under backend:=sim)' if sim
                         else '*** ON HARDWARE THIS IS A MOCAP PROBLEM ***')
            print(f'    no believable live pose for {", ".join(blind)} -- '
                  f'checking the gather against their marks instead {why_blind}',
                  flush=True)
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
            acf.goTo(adv_first, 0.0, gather)
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
        # An explicit `duration` always wins; the default is the script's own
        # length when there is a script, and a long leash when a human is
        # driving and decides when the test is over.
        duration = float(_param(node, 'duration',
                                script.duration if adv_mode == 'scripted' else 120.0))
        adv_guard = escort.SetpointGuard(cfg, script.target(0.0, p_vip)) \
            if acf is not None else None
        print(f'  ESCORT LIVE - {duration:.0f} s, streaming at '
              f'{cfg.rate_hz:g} Hz\n')
        streaming = True
        was_engaged = None
        last_clamp = ''
        last_untrusted = False
        last_gripe = -1e9
        last = t0
        _hold_t = {'last': timeHelper.time()}

        def _hold():
            """One control iteration while a gate is waiting.

            A gate that merely sleeps is not safe: the defenders would sit on
            their last setpoint with nothing re-asserting the separation guards
            and nothing checking the mocap staleness watchdogs. This keeps the
            real loop turning -- with NO adversary, so the ring simply holds --
            so a VIP dropout during a gate still lands the show, and the guards
            still clamp.
            """
            rclpy.spin_once(node, timeout_sec=0.0)
            now_h = timeHelper.time()
            dt_h = max(now_h - _hold_t['last'], 1e-3)
            _hold_t['last'] = now_h
            p_v = vip_now(now_h)
            if p_v is None:                      # VIP lost mid-gate
                if vip_age(now_h) > cfg.vip_stale_land_s:
                    raise ShowAborted('VIP pose lost while holding')
                return
            live_h = [_live(poses, n, cf, now_h) for n, cf in zip(defenders, dcfs)]
            sp_h, _ = ctrl.step(dt_h, p_v, None, live_h)
            for i_h, cf_h in enumerate(dcfs):
                _stream(cf_h, sp_h[i_h], ctrl.guards[i_h].v)
            timeHelper.sleepForRate(cfg.rate_hz)

        # last_good must START valid: if the VIP is already outside on the very
        # first iteration the fence trips immediately and reads it, and None
        # there is a crash at the worst possible moment. The VIP home is the
        # safe fallback -- it is where the ring was planned to sit.
        _home = np.asarray([*escort.vip_home(cfg)[:2], cfg.height], float)
        fence = {'on': True, 'centre': _home.copy(), 'last_good': _home.copy(),
                 'was_airborne': cfg.vip_airborne}
        leg_i = 0
        # FIRST IN, LAST OUT. The Crazyflies are already up and holding a ring;
        # the DJI goes up last and comes down first, so the big aircraft is
        # never manoeuvred past three hovering drones and the ring is never
        # broken while it is in the air.
        #
        # During those two legs the ring is PINNED to a constant height
        # (vip_airborne off -> escort.ring_targets holds cfg.height) instead of
        # riding vip_height_offset above the VIP. Without that the ring chases
        # the DJI: on the floor it would drag the defenders down to floor level
        # before take-off, and on the way down it would follow it into the
        # ground. Pinned, the pilot can climb and descend through the ring's
        # plane without moving it at all.
        phase = 'dji_up' if (paced and vip_mode == 'mocap') else 'encounter'
        track_vip = cfg.vip_airborne
        if phase == 'dji_up':
            cfg.vip_airborne = False          # pin the ring, ignore VIP altitude
            print(f'\n  ring holding a constant {cfg.height:.2f} m while the DJI '
                  'climbs -- it will not follow the VIP until you say so')
            operator_gate('PILOT: take off the DJI, climb into the ring and hold '
                          'steady. Press Enter once it is up and settled.',
                          pump=_hold)
            cfg.vip_airborne = track_vip      # now the ring rides with the VIP
            phase = 'encounter'
            print(f'  ring now tracking the VIP {cfg.vip_height_offset:+.2f} m')
        if paced:
            print(f'\n  PACED: the adversary waits for you at every leg '
                  f'({script.n_legs} of them). The defenders react on their own.')
            operator_gate(f'begin? next: {script.label(0)}')
        while True:
            rclpy.spin_once(node, timeout_sec=0.0)
            now = timeHelper.time()
            t = now - t0
            if paced:
                # Show time keeps running (the controller, the guards and every
                # staleness watchdog are all real-time); only the adversary's
                # place in its script is frozen until Enter.
                said = operator_said(timeout=0.0)
                if said in ('q', 'Q', 'EOF'):
                    print('\n  operator ended the encounter', flush=True)
                    break
                if said:
                    leg_i += 1
                    if leg_i >= script.n_legs:
                        print('\n  last leg done', flush=True)
                        break
                    print(f'\n  >>> leg {leg_i + 1}/{script.n_legs}: '
                          f'{script.label(leg_i)}\n      [Enter] next   [q] land',
                          flush=True)
            elif t >= duration:
                break
            step = max(now - last, 1e-3)
            last = now

            p_vip = vip_now(now)
            if p_vip is None:
                age = vip_age(now)
                if age > cfg.vip_stale_land_s:
                    raise RuntimeError(
                        f'VIP ({vip_name if vip_mode == "mocap" else VIP_TOPIC}) '
                        f'has been lost for {age:.1f} s -- '
                        'landing rather than escorting a guess')
                # hold: keep the last setpoints, command nothing new
                for cf, g in zip(dcfs, ctrl.guards):
                    _stream(cf, g.sp, np.zeros(3))
                timeHelper.sleepForRate(cfg.rate_hz)
                continue

            if adv_mode in ('scripted', 'manual'):
                if adv_mode == 'manual':
                    # A stale teleop freezes the adversary where it is; it does
                    # NOT land the demo, because the defenders and the person
                    # are still fine -- there is simply no threat moving.
                    want = manual_adv.get(now, 1.0)
                    want = adv_guard.sp if want is None else want
                else:
                    want = script.target(
                        script.leg_mid(leg_i) if paced else t, p_vip)
                # A manual adversary gets ONE floor: it may not touch the VIP.
                #
                # Only in manual mode, and only against the VIP. A SCRIPTED
                # adversary needs neither -- AdversaryScript.check already
                # refuses a script whose closest leg is inside
                # ring_radius + min_adv_sep, so the clamp could never fire.
                # And neither mode clamps against the DEFENDERS, deliberately:
                # the probe radius IS ring_radius + min_adv_sep, so at the
                # probe the lead defender stands exactly min_adv_sep away, on
                # the boundary with a 1 mm deadband. Pushing both apart there
                # would have the guards firing every step of every encounter,
                # which is the signal they exist to raise. The defenders
                # already yield (they push away from p_adv); measured lag puts
                # them 0.758 m from a 0.800 m floor, which is slew, not policy.
                #
                # min_adv_sep, NOT ring_radius + min_adv_sep: the attacker has
                # to be able to visibly penetrate the ring or the block means
                # nothing -- stopping it at 1.80 m would look identical with no
                # defenders flying at all. 0.80 m lets it through the ring and
                # still cannot reach the DJI. It is a reused constant, not a
                # measurement against a DJI's prop wash (ESCORT.md).
                guard_vip = p_vip if adv_mode == 'manual' else None
                p_adv = adv_guard.step(want, step, p_vip=guard_vip,
                                       vip_dist=cfg.min_adv_sep)
                _stream(acf, p_adv, adv_guard.v)
            elif adv_mode == 'external':
                p_adv = poses.get(adv_name, now, 0.5)     # gone = no threat
            else:
                p_adv = None

            # ---- geofence -------------------------------------------------
            # The VIP is hand-flown and can leave. Without this the ring simply
            # keeps chasing: the far slots clamp against the arena and the
            # formation deforms into a crescent with the guard firing every
            # step (vip_problems says so -- "the ring is no longer a ring").
            #
            # Instead the ring STOPS following and holds where it was. Coming
            # back needs TWO things, not one: the VIP inside the arena again
            # AND back inside the ring the defenders are still holding. Arena
            # alone would let a DJI re-acquire from outside the formation and
            # drag the whole ring across the room to meet it.
            stray = float(np.linalg.norm(np.asarray(p_vip, float)[:2]
                                         - np.array(cfg.room_center)))
            keep_in = escort.vip_keep_in(cfg)
            if fence['on']:
                if stray > keep_in:
                    fence['on'] = False
                    fence['centre'] = np.asarray(fence['last_good'], float).copy()
                    fence['was_airborne'] = cfg.vip_airborne
                    cfg.vip_airborne = False          # hold altitude too
                    print(f'\n  [t+{t:5.1f}s] GEOFENCE: the VIP is {stray:.2f} m '
                          f'out, past {keep_in:.2f} m -- the ring STOPS following '
                          'and holds. Fly it back INTO the ring to resume.',
                          flush=True)
                else:
                    fence['last_good'] = np.asarray(p_vip, float).copy()
            else:
                in_arena = stray <= keep_in - FENCE_HYST
                in_ring = float(np.linalg.norm(
                    np.asarray(p_vip, float)[:2] - fence['centre'][:2])) <= cfg.ring_radius
                if in_arena and in_ring:
                    fence['on'] = True
                    cfg.vip_airborne = fence['was_airborne']
                    fence['last_good'] = np.asarray(p_vip, float).copy()
                    print(f'\n  [t+{t:5.1f}s] GEOFENCE cleared: the VIP is back '
                          'inside the arena and inside the ring -- following '
                          'again', flush=True)
            # what the ring is actually centred on
            p_track = p_vip if fence['on'] else fence['centre']

            live = [_live(poses, n, cf, now) for n, cf in zip(defenders, dcfs)]
            sp, info = ctrl.step(step, p_track, p_adv, live)
            for i, cf in enumerate(dcfs):
                _stream(cf, sp[i], ctrl.guards[i].v)
            # Draw AFTER the setpoints are away: the picture must never delay a
            # drone, exactly as the light cues must not (see cue()).
            # A scripted adversary has a knowable future, so draw it: sample the
            # script ahead of now. A teleoperated or mocap-tracked one does not
            # -- pass nothing rather than draw a line the demo cannot know.
            adv_plan = None
            if adv_mode == 'scripted' and p_adv is not None:
                base = script.leg_mid(leg_i) if paced else t
                adv_plan = [script.target(base + k * 0.5, p_vip) for k in range(1, 25)]
            viz.publish(p_vip, p_adv, sp,
                        dict(info, slot_of=ctrl.slot_of), cfg.ring_radius,
                        adv_plan=adv_plan)

            if info['engaged'] != was_engaged:
                was_engaged = info['engaged']
                what = ('BLOCKING - wall closing on the adversary' if was_engaged
                        else 'clear - wall opening back to the ring')
                print(f'  [t+{t:5.1f}s] {what}', flush=True)
                if lights_on:
                    # The blocker is the drone holding the LEAD slot, not slot
                    # 0. `lead` exists precisely so the ring does not have to
                    # spin slot 0 all the way onto the threat (up to 180 deg,
                    # measured at 155) -- it is chosen at first contact as
                    # whichever slot is already closest in bearing. Cueing slot
                    # 0 therefore lit the right drone only when lead happened
                    # to be 0, i.e. about one engagement in three, which is why
                    # the lights told an incoherent story.
                    #
                    # All three are cued, not just the blocker: the wall
                    # closing is the biggest thing an audience can see, and
                    # leaving the two wings unchanged threw that away.
                    lead_slot = info['lead']
                    cols = [LED_BLOCKING if ctrl.slot_of(i) == lead_slot else LED_WING
                            for i in range(len(dcfs))] if was_engaged \
                        else [LED_DEFENDER] * len(dcfs)
                    cue(allcfs, dcfs, cols)
            # Only SEPARATION clamps are worth a line, and only when the set
            # of them changes: a slew clamp is the guard doing its job, and
            # printing either one per step buries the run in its own log.
            parts = [f'{defenders[i]}:{"+".join(sorted(set(r) - {"speed", "accel"}))}'
                     for i, r in sorted(info['clamped'].items())
                     if set(r) - {'speed', 'accel'}]
            # The ADVERSARY's guard too, or the one clamp a manual operator
            # most needs to see fires silently: holding a key drives the
            # attacker onto the VIP, and only `vip` in this line says the
            # floor caught it. Measured before it was reported: the commanded
            # adversary passed 0.036 m from the VIP with nothing in the log.
            if acf is not None:
                extra = sorted(set(adv_guard.reasons) - {'speed', 'accel'})
                if extra:
                    parts.append(f'{adv_drone}:{"+".join(extra)}')
            who = ', '.join(parts)
            # Only while the ring is actually FOLLOWING. Once the geofence
            # holds it, vip_problems' complaints are about a VIP the ring has
            # stopped tracking, and the keep-in one becomes false in the most
            # misleading way -- it says "the ring is no longer a ring" when
            # holding is precisely what keeps it one. Report the hold instead,
            # with the distance, so a fenced excursion is not silent either.
            if fence['on']:
                gripes = escort.vip_problems(cfg, p_vip, info['v_vip'], live)
            else:
                back = fence['centre']
                gripes = [f'GEOFENCE holding: VIP {stray:.2f} m from room '
                          f'centre (limit {keep_in:.2f}), ring parked on '
                          f'[{back[0]:+.2f}, {back[1]:+.2f}] -- bring the VIP '
                          f'back inside {cfg.ring_radius:.2f} m of that point '
                          'to resume']
            if gripes and now - last_gripe > 3.0:
                last_gripe = now
                for g in gripes:
                    print(f'  [t+{t:5.1f}s] VIP: {g}', flush=True)
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
        if paced and vip_mode == 'mocap':
            # Pin the ring again before the DJI descends, or it follows the VIP
            # down and puts the defenders on the floor with it.
            cfg.vip_airborne = False
            print(f'\n  ring pinned at {cfg.height:.2f} m -- it will NOT follow '
                  'the DJI down')
            operator_gate('PILOT: bring the DJI DOWN now and land it. The '
                          'defenders are holding their ring and will stay up '
                          'until you confirm it is on the ground.', pump=_hold)
            print('  DJI down -- landing the defenders')
        for cf in dcfs + ([acf] if acf else []):
            cf.notifySetpointsStop()
        streaming = False
        for cf in flying:
            cf.land(targetHeight=LAND_HEIGHT, duration=LAND_DURATION)
        timeHelper.sleep(LAND_DURATION + 0.5)
        for cf in flying:
            cf.arm(False)
        armed = False
        print('\n  defenders landed and disarmed -- done\n')
        return 0

    except ShowAborted as e:
        if armed:
            _stop_stream(dcfs, acf, streaming)
            abort_land(allcfs, cfs, timeHelper, _LandCfg(), str(e))
        else:
            # Nothing was flying, so there is nothing to land -- but say so.
            # Silence here reads as a crash, and the likeliest way to reach it
            # is paced:=true with stdin not wired: operator_said() sees EOF
            # immediately and aborts at the ARM gate. From the mission console
            # that looked like an unexplained exit 130.
            print(f'\n  ended before arming: {e}\n'
                  '  (paced mode reads a LINE from stdin -- run it from a '
                  'terminal, or use the console\'s stdin box; stdin closed or '
                  '/dev/null counts as "land now".)\n', flush=True)
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


def _live(poses, name, cf, now, expect=None, tol=1.0):
    """Where a defender actually is: /poses first, onboard estimate second.

    /poses is the mocap truth the whole demo is built on. ``get_position()``
    is the drone's own estimate, which is seeded from the yaml and so agrees
    with the plan even when the drone is in the wrong place -- fine as a
    fallback for one stale frame, wrong as the primary source (CLAUDE.md,
    "initial_position comes from /poses, never /cfX/pose").

    With ``expect`` given, a pose that disagrees with it by more than ``tol``
    (or sits below the floor) is rejected in favour of ``expect``, and the
    caller is told via the second return value. That is not a nicety: under
    ``backend:=sim`` there is no /poses at all and ``get_position()`` returns
    [0, 0, 0], so every drone reads as stacked on the origin -- which made the
    gather check refuse a perfectly good sim run. A check that cannot tell
    "the drones are too close" from "I cannot see the drones" is worse than no
    check, because it teaches you to ignore it.
    """
    p = poses.get(name, now, 0.3)
    if p is None:
        p = np.array(cf.get_position(), float)
    if expect is None:
        return p
    expect = np.asarray(expect, float)
    if p[2] < 0.05 or float(np.linalg.norm(p - expect)) > tol:
        return expect, False
    return p, True


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
