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
``alert_radius``        2.10        m, adversary-to-VIP distance that ENGAGES
``release_radius``      2.22        m, the larger one that disengages
``min_vip_dist``        1.0         m, hard floor defender-to-VIP
                                    (= ring_radius by design)
``rate_hz``             20.0        setpoint stream rate per drone
``vip_height_offset``   0.30        m, ring altitude relative to an airborne
                                    VIP
``dji_clear_height``    1.70        m, where the ring waits while the DJI
                                    takes off / lands. Must clear the DJI's
                                    hover altitude; bounded above by
                                    safety.CEILING_TESTED (1.95 m)
======================  ==========  =================================

``probe < alert < release < retreat`` has to hold; ``plan_escort`` refuses if
it does not, so re-run it after changing either radius.
"""

import select
import sys
import time

import numpy as np
import rclpy
from crazyflie_py import Crazyswarm
from geometry_msgs.msg import PointStamped
from rclpy.qos import qos_profile_sensor_data

from motion_capture_tracking_interfaces.msg import NamedPoseArray

from crazyflie_shows import escort, safety
from crazyflie_shows.escort_teleop import ADVERSARY_TOPIC, VIP_TOPIC
from crazyflie_shows.constellation import LIGHT
from crazyflie_shows.escort_viz import EscortViz
from crazyflie_shows.abort import ShowAborted, abort_land, take_signals
from crazyflie_shows.constellation_show import cue
from crazyflie_shows.preflight import report_supervisor
from crazyflie_shows.swarm_show import (_param, check_placement,
                                        wait_for_mocap)

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
# CYAN, not amber. amber is wrgb(r=0xFF, g=0x50) and the adversary is
# wrgb(r=0xFF) -- amber IS red with a little green, and across a room the
# drone doing the blocking and the drone being blocked read as the same
# colour. Reported from the first hardware flight, 2026-10-06. Cyan is the
# only cue in the palette with no red channel at all.
LED_BLOCKING = LIGHT['blocker']      # the LEAD, standing on the threat bearing
LED_WING = LIGHT['violet']           # the other two, closed up beside the lead

#: STAGE cues, as opposed to the ROLE cues above.
#:
#: The role colours say what each drone is doing; these say where the SCRIPT
#: is, so the operator can see at a glance that the drones and the script
#: agree. On an operator-paced show that cross-check is the thing you cannot
#: get any other way: the console says which stage it thinks it is on, and
#: until now the drones only ever said which role they held.
#:
#: The adversary carries it, because it is the one drone whose role does not
#: change through the run and the one the audience is already watching:
#:   amber   = taking station, not pressing
#:   red     = ATTACKING
#:   magenta = stood down, withdrawing
LED_ADV_STAGE = (LIGHT['amber'], LIGHT['red'], LIGHT['magenta'])

#: Everything goes amber while the show is WAITING ON A HUMAN -- the two DJI
#: legs, where the ring is pinned high and nothing moves until the pilot says
#: the aircraft is up or down. A fleet that has gone uniformly amber means
#: "your move", which is exactly when an operator is least sure.
LED_STAGE_WAIT = LIGHT['amber']

#: The geofence hold has its OWN colour, and must keep one.
#:
#: It was amber, on the reasoning that a fence hold is also "parked, waiting
#: on a human". That reasoning is wrong in practice and the flight proved it
#: (2026-10-07): the operator saw the defenders go amber several times and
#: could not tell which of those were fence trips, because the operator gates
#: turn them amber too. A cue that cannot be told from another cue is not a
#: cross-check, which is the entire reason the stage cues exist.
#:
#: Worse, a fence trip happens mid-ATTACK, when the adversary is red -- and
#: amber is wrgb(r=0xFF, g=0x50), i.e. red with a little green, which this
#: file already records as unreadable against red across a room. So the old
#: cue turned all three defenders the colour of the attacker at the one
#: moment the operator most needs to read roles.
#:
#: Cyan, despite cyan having failed as the BLOCKER cue against the resting
#: ring's deep_blue. That failure was two colours on different drones at the
#: same time; this cue goes on all three defenders at once and so never
#: appears beside deep_blue -- and the adversary stays red throughout, as a
#: fixed reference. Cyan is also the only entry in the palette with no red
#: channel, so it cannot be confused with the attacker.
LED_FENCE = LIGHT['cyan']

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
        # SENSOR QoS, not the default. motion_capture_tracking publishes
        # /poses BEST_EFFORT; a RELIABLE subscription is INCOMPATIBLE with it
        # and rclpy delivers NOTHING -- "No messages will be received from
        # it", as a one-line WARN at startup and silence thereafter.
        #
        # This shipped broken and nothing caught it: under backend:=sim there
        # is no /poses at all, so the miss looks exactly like the expected
        # "no believable live pose ... (expected under backend:=sim)", and the
        # escort has never flown on hardware. It disables everything that
        # needs mocap -- vip_mode:=mocap (the DJI), adversary:=external, the
        # geofence, the staleness watchdogs and the live placement check.
        # Every other tool in this repo already uses sensor QoS here.
        node.create_subscription(NamedPoseArray, topic, self._cb,
                                 qos_profile_sensor_data)

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
                 'vip_height_offset', 'dji_clear_height'):
        setattr(cfg, name, float(_param(node, name, getattr(cfg, name))))
    return cfg


#: What to type to advance a paced show. A bare Enter is the natural thing in
#: a terminal, but the mission console's stdin box sends a LINE and used to
#: refuse to send an empty one, so operators typed a stray letter at every
#: gate to get past it. ``operator_said`` has always treated ANY non-'q' line
#: as "continue", so the hint now says so instead of leaving people guessing.
CONTINUE_HINT = '[Enter] (or any text) continue   [q] land'


def _clock(when=None):
    """Local wall-clock HH:MM:SS, for matching a log line against a video.

    Show-relative ``t+`` is the right axis for reasoning about the run and the
    wrong one for reviewing it: after the 2026-10-07 flight the defenders'
    colour changes could not be matched to geofence events, because the only
    timestamps printed were relative to a t0 nobody had recorded. Anything an
    operator may need to find again in a recording carries both.
    """
    return time.strftime('%H:%M:%S', time.localtime(when))


def _fence_report(fence):
    """Every geofence episode, with wall clock, as a table to lay on a video.

    Printed on every exit path that had a fence dict, including aborts -- a
    run that ended badly is the one whose timeline matters most.
    """
    holds = list(fence.get('holds', ()))
    if fence.get('since') and not fence.get('on'):
        # Still held when the show ended: report it as an open episode rather
        # than dropping it, which is what the first version of this did.
        holds.append((fence.get('since_t', 0.0), fence['since'],
                      time.time() - fence['since']))
        open_last = True
    else:
        open_last = False
    if not holds:
        print('  geofence: never tripped -- the VIP stayed inside keep-in\n',
              flush=True)
        return
    print(f'\n  GEOFENCE EPISODES ({len(holds)})')
    print('    #   show time   wall clock   held')
    for i, (t_show, wall, held) in enumerate(holds, 1):
        tail = '  (still held at exit)' if open_last and i == len(holds) else ''
        print(f'    {i:<3} t+{t_show:6.1f}s   {_clock(wall)}     '
              f'{held:5.1f} s{tail}')
    print('    the defenders were CYAN for exactly these windows\n',
          flush=True)

#: When a paced leg counts as FINISHED, so the operator is told rather than
#: having to guess from a picture that stops moving. Both must hold: the
#: adversary's commanded setpoint has converged on its leg target, and the
#: ring has stopped moving. The ring one matters -- the attacker arrives
#: first and the defenders are still swinging onto the new threat bearing,
#: which is exactly the part worth watching.
LEG_DONE_TOL = 0.08    # m, commanded adversary to its leg target
LEG_DONE_V = 0.05      # m/s, fastest commanded defender setpoint

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
    # Only what select() itself raises on a closed or odd stdin means "nothing
    # typed". This was `except Exception`, and the abort signal almost always
    # lands while we sit in this select(): it swallowed the ShowAborted that
    # abort.take_signals() raises, so a Ctrl-C at a paced gate did nothing,
    # the console's Stop escalated to SIGKILL, and nothing landed (measured
    # in sim 2026-10-09). ShowAborted is now a BaseException as well, so
    # either change alone closes it; both stay.
    try:
        r, _, _ = select.select([sys.stdin], [], [], None if block else timeout)
    except (OSError, ValueError):
        return ''
    if not r:
        return ''
    line = sys.stdin.readline()
    if line == '':                                      # EOF: stdin closed
        return 'EOF'
    return line.strip() or ' '                          # bare Enter


def drain_stdin():
    """Throw away anything already typed. Returns True if 'q' was among it.

    A gate must be answered by a decision made AFTER reading it. Keystrokes
    queued while the previous leg ran would otherwise satisfy the next gate
    the instant it opens -- and on 2026-10-06 that collapsed the whole landing
    sequence: extra presses skipped the "bring the DJI down" confirmation, so
    the defenders landed alongside the DJI instead of after it. A 'q' in the
    buffer is still honoured; it is the one input that is never premature.
    """
    sawq = False
    while select.select([sys.stdin], [], [], 0.0)[0]:
        line = sys.stdin.readline()
        if line == '':
            break
        if line.strip() in ('q', 'Q'):
            sawq = True
    return sawq


def operator_gate(prompt, pump=None, spin=None):
    """Hold until the operator says go.

    ``pump`` runs one control iteration per wait tick -- it keeps the drones
    fed AND the ROS caches fresh. ``spin`` is the weaker version for gates
    reached before anything is flying: it only services callbacks.

    Passing NEITHER makes the gate block in select() without ever calling
    spin_once, which freezes every pose timestamp for as long as the operator
    thinks. That is not a cosmetic bug -- the next staleness check reports the
    operator's own reaction time as a mocap dropout and lands the show.
    """
    if drain_stdin():                       # a queued 'q' still lands
        raise ShowAborted('operator asked to land')
    print(f'\n  >>> {prompt}\n      {CONTINUE_HINT}', flush=True)
    while True:
        said = operator_said(block=(pump is None and spin is None), timeout=0.05)
        if said in ('q', 'Q', 'EOF'):
            raise ShowAborted('operator asked to land')
        if said:
            return
        if pump is not None:
            pump()
        elif spin is not None:
            spin()


def main():
    # The room must have been MEASURED before anything plans or flies.
    # No-op on this rig; the gate exists so a copy of this package in an
    # unsurveyed room refuses instead of inheriting our geofence.
    safety.require_measured_arena('escort_show')
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
    adv_mode = str(_param(node, 'adversary', 'reactive'))
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
    if adv_mode not in ('scripted', 'reactive', 'external', 'manual', 'none'):
        raise SystemExit("adversary must be reactive | scripted | external | "
                         "manual | none, "
                         f'not {adv_mode!r}')

    cfg = _cfg_from_params(node)
    # A mocap VIP is airborne by default, because that is what the live demo
    # is: a hand-flown DJI being escorted. A person wearing a hat is the
    # exception and says so with vip_airborne:=false.
    cfg.vip_airborne = bool(_param(node, 'vip_airborne', vip_mode == 'mocap'))
    script = (escort.ReactiveAdversary(height=cfg.height)
              if adv_mode == 'reactive'
              else escort.AdversaryScript(height=cfg.height))

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
    if adv_mode in ('scripted', 'reactive', 'manual'):
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
        # C2: the drones themselves must be on /poses, not just the VIP. The
        # `need` gate above only ever waited for the VIP body (and the
        # adversary when `external`), so a defender whose Motive body was
        # missing or misnamed sailed through preflight -- /cfX/pose is the
        # yaml-seeded onboard estimate and looks healthy regardless.
        print('  MOCAP CHECK (every flying drone must be on /poses)')
        live = wait_for_mocap(node, list(names))
        print('    all tracked\n')
        print('  PLACEMENT CHECK (mocap vs initial_position)')
        check_placement(node, cfs, names, _YamlStarts(cfs),
                        float(_param(node, 'placement_tol', 0.25)), live=live)
        print('    all drones within tolerance\n')

    # ------------------------------------------------- supervisor go/no-go
    # Only the drones that will actually fly. An E-STOP latches the firmware
    # into LOCKED and arm() is fire-and-forget, so a partial arm would give a
    # two-drone "ring" around a person that the controller, the viz and the
    # operator all believe is three.
    if not sim:
        will_fly = list(defenders) + ([adv_drone] if adv_drone else [])
        if not report_supervisor(node, will_fly):
            return 1

    # The rehearsal stops HERE, after the read-only rig checks, so that a
    # dry run exercises mocap, placement and the supervisor -- the things that
    # actually stop a demo -- and not just the geometry.
    if dry_run:
        print('  dry_run - plan and preflight checks done; nothing armed.\n')
        return 0

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
    # Declared out here so the abort handlers can still print the fence
    # timeline: the real initialisation is inside the try, and an abort before
    # it would otherwise raise NameError inside an except clause.
    fence = {'on': True, 'holds': []}
    try:
        take_signals()
        # BEFORE the arm loop below: in paced mode operator_gate() sits inside
        # that loop, so the window between the first and last arm() is seconds
        # long, and a signal in it would otherwise skip the abort entirely and
        # leave part of the fleet armed.
        armed = True
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
                              'hand.',
                              spin=lambda: rclpy.spin_once(node, timeout_sec=0.02))
            cf.arm(True)
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
        # The wall clock goes in the banner so every later "t+" in this run
        # can be converted to a video timecode without guessing t0.
        print(f'  ESCORT LIVE - {duration:.0f} s, streaming at '
              f'{cfg.rate_hz:g} Hz   t0 = {_clock()}\n')
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
            # The ADVERSARY too, or it falls out of the sky.
            #
            # Once a drone has been fed low-level setpoints the firmware
            # commander cuts thrust ~2 s after the stream stops. This pump fed
            # only the defenders, so every gate opened AFTER the encounter
            # starved the attacker: on 2026-10-06 the dji_down gate lasted as
            # long as it took to land the DJI and the attacker simply dropped.
            # It holds its last commanded point -- it is not part of the
            # encounter while a gate is open, it just has to stay airborne.
            if acf is not None and streaming:
                _stream(acf, adv_guard.sp, np.zeros(3))
            # A gate can be open for as long as a human takes, which is
            # exactly when nobody is watching the drones.
            if streaming:
                act_h, msg_h = containment(now_h, dt_h)
                if act_h == 'land':
                    raise ShowAborted(f'CONTAINMENT: {msg_h}')
            timeHelper.sleepForRate(cfg.rate_hz)

        # last_good must START valid: if the VIP is already outside on the very
        # first iteration the fence trips immediately and reads it, and None
        # there is a crash at the worst possible moment. The VIP home is the
        # safe fallback -- it is where the ring was planned to sit.
        _home = np.asarray([*escort.vip_home(cfg)[:2], cfg.height], float)
        # 'holds' is the episode log: one (t_show, wallclock, duration) per
        # trip, printed as a table when the show ends. The LED cue tells an
        # operator the fence is ON right now; it cannot tell them WHEN, and
        # after the 2026-10-07 run the colour changes could not be matched to
        # fence events at all -- there was no timestamp to match them to.
        # Every fence line therefore carries wall-clock as well as show time,
        # because the thing being matched against is a video recording.
        fence = {'on': True, 'centre': _home.copy(), 'last_good': _home.copy(),
                 'was_airborne': cfg.vip_airborne, 'show_height': cfg.height,
                 'since': None, 'since_t': 0.0, 'holds': []}
        leg_i = 0
        leg_done = {'v': False}      # announced completion of the current leg?
        stood_said = {'v': False}    # announced the attacker giving up?
        stage_cued = {'v': None}     # which stage the lights currently show

        # ---- containment, every drone --------------------------------------
        # The VIP had a geofence and a staleness watchdog; the Crazyflies had
        # neither, and self_stale_s was dead config. A drone that lost tracking
        # was commanded from a model that could not see it while its own
        # estimator drifted -- the 2026-10-07 fly-away. This watches each
        # flying drone's ACTUAL pose, because clamping a setpoint proves
        # nothing about the aircraft.
        _watch = {n: {} for n in defenders + ([adv_drone] if acf else [])}
        last_contain = [0.0]         # throttles the 'holding' line

        def containment(now_c, dt_c):
            """Worst verdict over all flying drones: (action, message)."""
            worst, msg = 'ok', None
            pairs = [(defenders[i], ctrl.guards[i].sp) for i in range(len(dcfs))]
            if acf is not None:
                pairs.append((adv_drone, adv_guard.sp))
            for name, sp_c in pairs:
                if name not in poses.stamp:
                    # C1, 2026-10-09. This used to `continue` unconditionally,
                    # which meant a drone NEVER seen on /poses in a session was
                    # skipped by the watchdog FOREVER -- no hold, no land, for
                    # the whole flight. The machinery below already handles it
                    # (poses.age() returns inf for an unseen name and
                    # contain_check lands at self_stale_land_s), so the guard
                    # was the only thing standing between a misnamed or
                    # missing Motive body and an uncontained drone. That is
                    # the configuration that produced the 2026-10-07 fly-away.
                    # Skipping is correct ONLY in sim, where there is no
                    # /poses at all; on hardware an untracked drone is the
                    # hazard, not an absence of information.
                    if sim:
                        continue
                    return 'land', (f'{name}: never seen on /poses -- no mocap '
                                    'body with that name, so nothing can '
                                    'contain it')
                act, why = escort.contain_check(
                    cfg, poses.pos.get(name), poses.age(name, now_c),
                    sp_c, dt_c, _watch[name])
                if act == 'land':
                    return 'land', f'{name}: {why}'
                if act == 'hold' and worst == 'ok':
                    worst, msg = 'hold', f'{name}: {why}'
            return worst, msg
        # Where the adversary's CURRENT leg is measured from.
        #
        # AdversaryScript resolves a leg against the VIP's position RIGHT NOW,
        # so a moving VIP drags the attacker with it at a fixed radius -- on
        # the first hardware flight it read as the attacker flying formation
        # with the DJI, which is the single most scripted-looking thing in the
        # demo. Freezing the anchor at the start of each leg makes the attacker
        # fly its own path to a fixed point in the room; it still ATTACKS where
        # the VIP was when it committed, so a pilot who moves genuinely evades
        # it instead of towing it. (The real answer is a reactive policy that
        # re-plans -- ESCORT.md; this is the part that fits before Thursday.)
        leg_anchor = {'p': None}
        p_track = None               # fenced VIP; set at the end of each loop
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
        show_height = cfg.height
        if phase == 'dji_up':
            # Pin the ring HIGH, not at the show altitude. See
            # EscortConfig.dji_clear_height: parking it at `height` puts the
            # defenders exactly in the climbing DJI's path, and a DJI that
            # settles above them leaves three Crazyflies in its downwash.
            cfg.vip_airborne = False          # pin the ring, ignore VIP altitude
            cfg.height = cfg.dji_clear_height
            print(f'\n  ring CLIMBING to {cfg.height:.2f} m to clear the DJI '
                  f'(show altitude is {show_height:.2f} m) -- it will not '
                  'follow the VIP until you say so')
            # Whole fleet amber: the show is waiting on the pilot, and a
            # uniformly amber fleet is the one state an operator can read
            # from across the room without looking at the console.
            if lights_on:
                cue(allcfs, flying, [LED_STAGE_WAIT] * len(flying))
            operator_gate('PILOT: wait for the ring to settle high, THEN take '
                          'off the DJI, climb to your hover height under the '
                          'ring and hold steady. Enter once it is up and settled.',
                          pump=_hold)
            if lights_on:
                # back to roles; the loop re-cues the adversary's stage because
                # stage_cued is cleared.
                cue(allcfs, dcfs, [LED_DEFENDER] * len(dcfs))
                stage_cued['v'] = None
            cfg.height = show_height
            cfg.vip_airborne = track_vip      # now the ring rides with the VIP
            phase = 'encounter'
            print(f'  ring now tracking the VIP {cfg.vip_height_offset:+.2f} m')
        if paced:
            print(f'\n  PACED: the adversary waits for you at every leg '
                  f'({script.n_legs} of them). The defenders react on their own.')
            # pump=_hold is NOT optional. An unpumped gate blocks in
            # select() and never calls spin_once, so NO /poses callbacks run
            # and every pose timestamp freezes at the moment the gate opened.
            # The first staleness check after the gate then measures how long
            # the OPERATOR took to press Enter and reports it as a lost VIP.
            # Measured on hardware 2026-10-06: 7.9 s at this prompt ->
            # "VIP (operator) has been lost for 8.1 s", landing a show whose
            # mocap never missed a frame. It also left the defenders coasting
            # on their last setpoint with no guard running.
            operator_gate(f'begin? next: leg 1/{script.n_legs} '
                          f'-- {script.label(0)}', pump=_hold)
        # leg 1 is measured from wherever the VIP is when the encounter opens
        _seed = vip_now(timeHelper.time())
        # p_vip0 is None in mocap mode, and np.asarray(None) would mask the
        # real failure (a VIP the loop is about to abort on anyway).
        leg_anchor['p'] = np.asarray(
            _seed if _seed is not None else escort.vip_home(cfg), float).copy()
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
                    leg_done['v'] = False
                    # p_vip still holds last iteration's value (50 ms old)
                    leg_anchor['p'] = np.asarray(p_vip, float).copy()
                    stood_said['v'] = False
                    if leg_i >= script.n_legs:
                        print('\n  last leg done', flush=True)
                        break
                    print(f'\n  >>> leg {leg_i + 1}/{script.n_legs} STARTED: '
                          f'{script.label(leg_i)}\n      {CONTINUE_HINT}',
                          flush=True)
            elif t >= duration:
                break
            step = max(now - last, 1e-3)
            last = now

            # Containment BEFORE anything is computed from these poses.
            act_c, msg_c = containment(now, step)
            if act_c == 'land':
                raise RuntimeError(f'CONTAINMENT: {msg_c}')
            if act_c == 'hold':
                if now - last_contain[0] > 2.0:
                    last_contain[0] = now
                    print(f'  [t+{t:5.1f}s] holding -- {msg_c}', flush=True)
                for cf, g in zip(dcfs, ctrl.guards):
                    _stream(cf, g.sp, np.zeros(3))
                if acf is not None:
                    _stream(acf, adv_guard.sp, np.zeros(3))
                timeHelper.sleepForRate(cfg.rate_hz)
                continue

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

            if adv_mode in ('scripted', 'reactive', 'manual'):
                if adv_mode == 'manual':
                    # A stale teleop freezes the adversary where it is; it does
                    # NOT land the demo, because the defenders and the person
                    # are still fine -- there is simply no threat moving.
                    want = manual_adv.get(now, 1.0)
                    want = adv_guard.sp if want is None else want
                else:
                    if adv_mode == 'reactive':
                        # It flies at the RING's altitude, not the VIP's: the
                        # ring now rides above an airborne DJI, and an
                        # attacker left at the VIP's height would come in
                        # underneath the drones meant to be facing it.
                        script.height = escort.ring_height(cfg, p_vip)
                        # Paced: the operator advances it. UNPACED: advance
                        # on time, or the phase stays 0 and it never attacks
                        # at all -- it just holds its station for the whole
                        # run, which is what a free-running sim showed.
                        script.set_phase(
                            leg_i if paced
                            else int(t / (script.duration / script.n_legs)),
                            p_vip)
                        # The FENCED VIP, not the raw one. p_track holds last
                        # iteration's value here (50 ms old) because the fence
                        # is evaluated further down the loop. Using p_vip let
                        # the attacker chase a strayed DJI out of the arena
                        # while the ring correctly stayed behind -- the one
                        # drone still commanded to follow it anywhere.
                        want = script.target(now - t0,
                                             p_vip if p_track is None else p_track,
                                             [g.sp for g in ctrl.guards], cfg)
                    else:
                        anchor = (leg_anchor['p']
                                  if paced and leg_anchor['p'] is not None
                                  else p_vip)
                        want = script.target(
                            script.leg_mid(leg_i) if paced else t, anchor)
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
                # Backstop for both UNVERIFIED modes. A scripted adversary
                # is proven offline and needs none; a manual or reactive one
                # has no proof, and the sweep that tuned the reactive gains
                # found a setting that walked it to 0.01 m of the VIP. The
                # gradient is what SHOULD turn it away; this is what happens
                # if the gradient is wrong on the day.
                guard_vip = (p_vip if adv_mode in ('manual', 'reactive')
                             else None)
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
                    # Hold the altitude the ring is AT, not cfg.height.
                    #
                    # vip_airborne=False makes ring_height() return cfg.height,
                    # which is the FLOOR-VIP show altitude (1.20 m) and has
                    # nothing to do with where the ring currently is. With the
                    # ring riding vip_height_offset above a DJI at 1.3-1.5 m it
                    # sits at 1.6-1.8 m, so tripping the fence DROPPED it to
                    # 1.20 m -- level with or BELOW the DJI, which is the one
                    # geometry this file calls unsurvivable, and it happened
                    # exactly when the DJI was already misbehaving. Reported
                    # from a live run, 2026-10-07: "the drones just drop to the
                    # DJI level during the attack phase".
                    fence['show_height'] = cfg.height
                    cfg.height = escort.ring_height(cfg, p_vip)
                    cfg.vip_airborne = False          # hold altitude too
                    fence['since'] = time.time()
                    fence['since_t'] = t
                    print(f'\n  [t+{t:5.1f}s {_clock()}] GEOFENCE HOLD #'
                          f'{len(fence["holds"]) + 1}: the VIP is {stray:.2f} m '
                          f'out, past {keep_in:.2f} m -- the ring STOPS following '
                          'and holds, defenders go CYAN. Fly it back INTO the '
                          'ring to resume.', flush=True)
                    # Cue the defenders to LED_FENCE: without it the ring
                    # looks like it is escorting normally while it is in fact
                    # holding a point the DJI has left, which is the one state
                    # an operator most needs to see from the floor. Cued on
                    # the TRANSITION only. See LED_FENCE for why this is not
                    # the amber the operator gates use -- sharing that colour
                    # made the two indistinguishable in flight.
                    if lights_on:
                        cue(allcfs, dcfs, [LED_FENCE] * len(dcfs))
                else:
                    fence['last_good'] = np.asarray(p_vip, float).copy()
            else:
                in_arena = stray <= keep_in - FENCE_HYST
                in_ring = float(np.linalg.norm(
                    np.asarray(p_vip, float)[:2] - fence['centre'][:2])) <= cfg.ring_radius
                if in_arena and in_ring:
                    fence['on'] = True
                    cfg.vip_airborne = fence['was_airborne']
                    cfg.height = fence['show_height']
                    fence['last_good'] = np.asarray(p_vip, float).copy()
                    held = (time.time() - fence['since']
                            if fence['since'] else 0.0)
                    fence['holds'].append((fence['since_t'], fence['since'],
                                           held))
                    print(f'\n  [t+{t:5.1f}s {_clock()}] GEOFENCE CLEARED after '
                          f'{held:.1f} s: the VIP is back inside the arena and '
                          'inside the ring -- following again', flush=True)
                    # Back to role colours. The engaged-transition cue only
                    # fires when engagement CHANGES, so without this the ring
                    # would stay amber until the next block or release.
                    # ctrl.engaged / ctrl.lead, NOT info[...]: the geofence
                    # block runs BEFORE ctrl.step in this loop, so `info` here
                    # is the previous iteration's and does not exist on the
                    # first pass. The controller's own attributes always do.
                    if lights_on:
                        cue(allcfs, dcfs,
                            [LED_BLOCKING if ctrl.slot_of(i) == ctrl.lead
                             else LED_WING for i in range(len(dcfs))]
                            if ctrl.engaged else [LED_DEFENDER] * len(dcfs))
            # what the ring is actually centred on
            #
            # NOT slewed, and that is a measured decision. Switching this point
            # on a fence transition looked like the cause of a separation dip
            # to 0.67 m; gliding it at 0.60 m/s changed the measurement to
            # 0.68 m, i.e. not at all. The dip tracks VIP MOTION, not fence
            # transitions -- it is the same walking-case compression
            # plan_escort already refuses to clear (0.81 m defender-defender
            # at a 0.30 m/s walk). Do not re-add a slew here expecting it to
            # help separations; fix the walking case instead.
            p_track = p_vip if fence['on'] else fence['centre']

            live = [_live(poses, n, cf, now) for n, cf in zip(defenders, dcfs)]
            # A reactive attacker aims at the ring's WIDEST GAP, so turning
            # to where it is now is always one move behind. Hand the ring the
            # gap it is heading for instead: measured in sim 2026-10-06, the
            # ring turns 720 deg instead of 216 and holds the attacker at
            # 0.46 m instead of 0.01 m. Engagement and every separation still
            # use the real p_adv -- only the rotation target is anticipated.
            # NO gap-bearing override. Feeding the ring the gap the attacker
            # is AIMING at sounded like anticipation and was the opposite: with
            # flank_bias the attacker deliberately picks gaps far from itself,
            # so the ring left the attacker uncovered to go and stand on an
            # opening 120 deg away. Measured 2026-10-06: the nearest defender
            # sat 58 deg off the attacker (worst 105), against 24 deg when the
            # ring simply tracks where the attacker IS.
            #
            # The controller does its own anticipation from the threat's
            # measured bearing RATE, bounded by lead_max_rad -- forward bias
            # that cannot run off to the far side of the circle.
            p_threat = None
            sp, info = ctrl.step(step, p_track, p_adv, live, p_threat=p_threat)
            for i, cf in enumerate(dcfs):
                _stream(cf, sp[i], ctrl.guards[i].v)
            # Draw AFTER the setpoints are away: the picture must never delay a
            # drone, exactly as the light cues must not (see cue()).
            # A scripted adversary has a knowable future, so draw it: sample the
            # script ahead of now. A teleoperated or mocap-tracked one does not
            # -- pass nothing rather than draw a line the demo cannot know.
            adv_plan = None
            if adv_mode == 'scripted' and p_adv is not None:   # reactive: no plan to draw
                base = script.leg_mid(leg_i) if paced else t
                a_plan = (leg_anchor['p'] if paced and leg_anchor['p'] is not None
                          else p_vip)
                adv_plan = [script.target(base + k * 0.5, a_plan) for k in range(1, 25)]
            viz.publish(p_vip, p_adv, sp,
                        dict(info, slot_of=ctrl.slot_of), cfg.ring_radius,
                        adv_plan=adv_plan)

            # ---- paced: say when the leg has actually FINISHED ----------
            # The adversary is frozen on leg_mid(leg_i), so it flies to that
            # point and then just sits there. Nothing on screen distinguished
            # "still flying this leg" from "arrived, waiting for you", so the
            # operator was pressing Enter on a guess -- and pressing early
            # cuts off the very reaction the leg exists to show.
            if (paced and adv_mode in ('scripted', 'reactive')
                    and not leg_done['v'] and p_adv is not None):
                ring_v = max((float(np.linalg.norm(g.v)) for g in ctrl.guards),
                             default=0.0)
                if adv_mode == 'reactive':
                    # The attack phase is over when the attacker GIVES UP, not
                    # when it arrives somewhere -- that is the whole point of
                    # it. The other two phases end on reaching its station.
                    if leg_i == 1:
                        reach = 0.0 if script.stood_down else 1e9
                    else:
                        st = script.station(p_vip)
                        reach = float(np.linalg.norm(
                            np.asarray(p_adv, float)[:2] - st[:2]))
                else:
                    goal = script.target(script.leg_mid(leg_i),
                                         leg_anchor['p'] if leg_anchor['p'] is not None
                                         else p_vip)
                    reach = float(np.linalg.norm(np.asarray(p_adv, float)[:2]
                                                 - np.asarray(goal, float)[:2]))
                if reach <= LEG_DONE_TOL and ring_v <= LEG_DONE_V:
                    leg_done['v'] = True
                    d_vip = float(np.linalg.norm(np.asarray(p_adv, float)[:2]
                                                 - np.asarray(p_vip, float)[:2]))
                    nxt = (f'leg {leg_i + 2}/{script.n_legs} -- '
                           f'{script.label(leg_i + 1)}'
                           if leg_i + 1 < script.n_legs
                           else 'END -- land the defenders')
                    print(f'\n  >>> leg {leg_i + 1}/{script.n_legs} COMPLETE: '
                          f'{script.label(leg_i)}\n'
                          f'      adversary settled {d_vip:.2f} m from the VIP, '
                          f'ring {"BLOCKING" if info["engaged"] else "clear"}\n'
                          f'      next: {nxt}\n'
                          f'      {CONTINUE_HINT}', flush=True)

            # The attacker giving up is the demo's payoff. Say so -- it left
            # silently on the first sim run and read as the show breaking.
            if (adv_mode == 'reactive' and script.stood_down
                    and not stood_said['v']):
                stood_said['v'] = True
                d_sd = float(np.linalg.norm(np.asarray(p_adv, float)[:2]
                                            - np.asarray(p_vip, float)[:2]))
                print(f'\n  [t+{t:5.1f}s] ATTACKER STOOD DOWN -- blocked for '
                      f'{script.give_up_s:.0f} s without getting nearer than '
                      f'{d_sd:.2f} m. It is withdrawing to its station.\n',
                      flush=True)

            # STAGE cue: the adversary's colour tracks which leg the script is
            # on, so the drones and the console can be compared at a glance.
            # Only on CHANGE -- a setParam per drone per step would be radio
            # traffic the rig cannot spare (see the link-stall gotcha).
            stage_now = (leg_i if paced
                         else int(t / max(script.duration / script.n_legs, 1e-6)))
            stage_now = max(0, min(stage_now, len(LED_ADV_STAGE) - 1))
            if lights_on and acf is not None and stage_cued['v'] != stage_now:
                stage_cued['v'] = stage_now
                cue(allcfs, [acf], [LED_ADV_STAGE[stage_now]])

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
            cfg.height = cfg.dji_clear_height
            print(f'\n  ring CLIMBING to {cfg.height:.2f} m to clear the DJI '
                  '-- it will NOT follow the DJI down')
            if lights_on:
                cue(allcfs, flying, [LED_STAGE_WAIT] * len(flying))
            operator_gate('PILOT: wait for the ring to lift clear, THEN bring '
                          'the DJI DOWN and land it. The defenders hold high '
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
        print('\n  defenders landed and disarmed -- done')
        _fence_report(fence)
        return 0

    except ShowAborted as e:
        if armed:
            _stop_stream(dcfs, acf, streaming)
            abort_land(allcfs, cfs, timeHelper, LAND_HEIGHT, str(e))
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
        _fence_report(fence)
        return 130
    except BaseException as e:                        # noqa: BLE001
        if armed:
            _stop_stream(dcfs, acf, streaming)
            abort_land(allcfs, cfs, timeHelper, LAND_HEIGHT, repr(e))
        _fence_report(fence)
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
