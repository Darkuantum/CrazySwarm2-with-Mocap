#!/usr/bin/env python3
"""Wire /poses -> follow_core.Follower -> crazyflie_py, for one Crazyflie.

This is the Crazyflie-phase adapter the rest of the package exists to
support: the whole point of flying this before the Starling has real
perception is that the *same* Follower object -- same arbiter, same speed
governor, same track supervisor -- runs here fed by a synthetic, perfect
mocap target, and later on the Starling fed by real UWB range + vision
bearing. See follow_core/follower.py's own docstring, and HANDOVER.md, for
why that specific de-risking is the reason this package exists at all.

Nothing about the control law lives here. What lives here is entirely rig
plumbing and the safety envelope on top of it:

  - subscribe /poses once (via MocapTargetTracker), split out the drone body
    and the operator body by configured name
  - feed both into follow_core.Follower at a fixed rate (default 20 Hz)
  - convert the resulting Setpoint into a streaming `cmdPosition` at that
    same rate
  - enforce, independently of follow_core, an 8x8 m soft boundary around
    ROOM_CENTER, a hard altitude ceiling, capped velocity/acceleration/turn
    rate, and an immediate hold-then-land if the operator body goes stale --
    see `SetpointGuard` below

Streaming vs. high-level commands (crazyflie_py's own documented behaviour,
crazyflie.py Crazyflie.notifySetpointsStop docstring): the instant this node
sends its first `cmdPosition`, the firmware leaves high-level command mode
for good, until `notifySetpointsStop()` is called. So the sequence here is
strictly: arm -> high-level `takeoff()` -> stream `cmdPosition` continuously
(this is also how HOLD/ORBIT/LAND are realised -- as different Setpoints from
the *same* streaming channel, never a mode switch) -> on shutdown,
`notifySetpointsStop()` -> high-level `land()` -> disarm. LAND mode itself is
just standoff.py's own descending-in-place Setpoint, ridden all the way to
the floor via the same streaming calls; the high-level `land()` at shutdown
only exists to hand back to onboard control for the final disarm.

Run (after copying into a workspace that already has crazyflie_py,
crazyflie_interfaces and motion_capture_tracking sourced -- see HANDOVER.md):

    ros2 launch crazyflie_follow follow_launch.py backend:=sim
    ros2 launch crazyflie_follow follow_launch.py
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from crazyflie_py import Crazyswarm
from std_msgs.msg import String

from crazyflie_follow.mocap_target import MocapTargetTracker
from crazyflie_follow import safety
from follow_core import (
    Command,
    CommandSource,
    Follower,
    FollowMode,
    FollowParams,
    VehicleState,
    wrap_pi,
)

_MODE_TOKENS = {
    'follow': FollowMode.FOLLOW,
    'hold': FollowMode.HOLD,
    'orbit': FollowMode.ORBIT,
    'land': FollowMode.LAND,
    'estop': FollowMode.ESTOP,
    'idle': FollowMode.IDLE,
}


@dataclass
class NodeConfig:
    drone_name: str
    operator_name: str
    poses_topic: str
    command_topic: str
    rate_hz: float

    room_center: Tuple[float, float]
    boundary_half_width_m: float
    altitude_floor_m: float
    altitude_ceiling_m: float

    hard_speed_cap_m_s: float
    hard_accel_cap_m_s2: float
    max_yaw_rate_rad_s: float

    operator_stale_hold_s: float
    operator_stale_land_s: float
    drone_stale_s: float

    takeoff_height_m: float
    takeoff_duration_s: float
    land_height_m: float
    land_duration_s: float

    mocap_lowpass_hz: float

    follow: FollowParams


def load_config(node) -> NodeConfig:
    """Declare and read every ROS parameter this node uses.

    All defaults here match config/follow.yaml exactly, so the node is safe
    to run with no params file at all (e.g. `ros2 run` during bare-metal
    bring-up) -- the yaml exists to make the numbers visible and editable,
    not because the node depends on it being present.
    """
    defaults = {
        'drone_name': 'cf1',
        'operator_name': 'operator',
        'poses_topic': '/poses',
        'command_topic': '/follow/cmd',
        'rate_hz': 20.0,
        'room_center_x': 0.0467,
        'room_center_y': -0.1037,
        'boundary_half_width_m': 4.0,
        'altitude_floor_m': 0.05,
        'altitude_ceiling_m': 1.5,
        'hard_speed_cap_m_s': 0.6,
        'hard_accel_cap_m_s2': 1.0,
        'max_yaw_rate_rad_s': 1.0,
        'operator_stale_hold_s': 0.4,
        'operator_stale_land_s': 2.0,
        'drone_stale_s': 0.3,
        'takeoff_height_m': 1.0,
        'takeoff_duration_s': 2.5,
        'land_height_m': 0.04,
        'land_duration_s': 3.5,
        'mocap_lowpass_hz': 4.0,
        # follow_core.FollowParams
        'standoff_m': 2.0,
        'height_m': 1.2,
        'follow_angle_rad': float(np.pi),
        'orbit_rate_rad_s': 0.35,
        'min_standoff_m': 1.5,
        'speed_cap_near_m_s': 0.6,
        'speed_cap_far_m_s': 1.0,
        'governor_far_m': 3.0,
        'max_accel_m_s2': 1.0,
        'lost_timeout_s': 2.0,
        'command_timeout_s': 2.0,
        'estop_timeout_s': 1.5,
    }
    for name, value in defaults.items():
        node.declare_parameter(name, value)

    g = lambda name: node.get_parameter(name).value  # noqa: E731

    follow = FollowParams(
        standoff_m=float(g('standoff_m')),
        height_m=float(g('height_m')),
        follow_angle_rad=float(g('follow_angle_rad')),
        orbit_rate_rad_s=float(g('orbit_rate_rad_s')),
        min_standoff_m=float(g('min_standoff_m')),
        speed_cap_near_m_s=float(g('speed_cap_near_m_s')),
        speed_cap_far_m_s=float(g('speed_cap_far_m_s')),
        governor_far_m=float(g('governor_far_m')),
        max_accel_m_s2=float(g('max_accel_m_s2')),
        lost_timeout_s=float(g('lost_timeout_s')),
        command_timeout_s=float(g('command_timeout_s')),
        estop_timeout_s=float(g('estop_timeout_s')),
    )

    return NodeConfig(
        drone_name=str(g('drone_name')),
        operator_name=str(g('operator_name')),
        poses_topic=str(g('poses_topic')),
        command_topic=str(g('command_topic')),
        rate_hz=float(g('rate_hz')),
        room_center=(float(g('room_center_x')), float(g('room_center_y'))),
        boundary_half_width_m=float(g('boundary_half_width_m')),
        altitude_floor_m=float(g('altitude_floor_m')),
        altitude_ceiling_m=float(g('altitude_ceiling_m')),
        hard_speed_cap_m_s=float(g('hard_speed_cap_m_s')),
        hard_accel_cap_m_s2=float(g('hard_accel_cap_m_s2')),
        max_yaw_rate_rad_s=float(g('max_yaw_rate_rad_s')),
        operator_stale_hold_s=float(g('operator_stale_hold_s')),
        operator_stale_land_s=float(g('operator_stale_land_s')),
        drone_stale_s=float(g('drone_stale_s')),
        takeoff_height_m=float(g('takeoff_height_m')),
        takeoff_duration_s=float(g('takeoff_duration_s')),
        land_height_m=float(g('land_height_m')),
        land_duration_s=float(g('land_duration_s')),
        mocap_lowpass_hz=float(g('mocap_lowpass_hz')),
        follow=follow,
    )


class SetpointGuard:
    """The last thing between follow_core's output and the radio.

    follow_core.governor.SpeedGovernor already clamps standoff and slews its
    own speed cap (governor.py) -- this is deliberately redundant with that,
    not a replacement for it:

      - it re-derives the min-standoff clamp from THIS node's own reading of
        the operator, not from follow_core's internal target estimate, so a
        bug in the EKF/estimate plumbing cannot silently defeat it;
      - it adds hard, config-level caps on velocity, acceleration and yaw
        rate that do not vary with a FollowParams mistake upstream -- "so a
        backup pilot can take over" per the project brief, i.e. sized for
        human reaction time rather than for follow performance;
      - it clamps to the 8x8 m boundary box and the altitude ceiling, neither
        of which follow_core knows about at all (they are properties of this
        rig, not of the follow algorithm).

    safety.py proves, offline, that the *unclamped* geometry should rarely
    reach these clamps in normal operation. This class is what actually
    enforces them at runtime regardless of whether that proof held.
    """

    def __init__(self, cfg: NodeConfig) -> None:
        self.cfg = cfg
        self._last_pos: Optional[np.ndarray] = None
        self._last_yaw: float = 0.0
        self._last_vel = np.zeros(3)
        self._last_t: Optional[float] = None

    @property
    def last_position(self) -> np.ndarray:
        assert self._last_pos is not None, 'reset_to() was never called'
        return self._last_pos

    @property
    def last_yaw(self) -> float:
        return self._last_yaw

    def reset_to(self, position, yaw: float, now: float) -> None:
        """Seed the guard's notion of "current setpoint" (e.g. after takeoff,
        before the first streamed command, so the first tick is not a jump).
        """
        self._last_pos = np.asarray(position, dtype=float).reshape(3)
        self._last_yaw = float(yaw)
        self._last_vel = np.zeros(3)
        self._last_t = now

    def apply(self, goal_position, goal_yaw: float,
             operator_xy: Optional[Tuple[float, float]], now: float
             ) -> Tuple[np.ndarray, float, List[str]]:
        """Clamp+slew one commanded setpoint. Returns (position, yaw, reasons)."""
        cfg = self.cfg
        pos = np.asarray(goal_position, dtype=float).reshape(3).copy()
        reasons: List[str] = []

        # -- altitude ---------------------------------------------------
        if pos[2] > cfg.altitude_ceiling_m:
            pos[2] = cfg.altitude_ceiling_m
            reasons.append(f'altitude ceiling {cfg.altitude_ceiling_m:.2f} m')
        if pos[2] < cfg.altitude_floor_m:
            pos[2] = cfg.altitude_floor_m
            reasons.append(f'altitude floor {cfg.altitude_floor_m:.2f} m')

        # -- 8x8 m boundary box around ROOM_CENTER -----------------------
        cx, cy = cfg.room_center
        hw = cfg.boundary_half_width_m
        clamped_x = min(max(pos[0], cx - hw), cx + hw)
        clamped_y = min(max(pos[1], cy - hw), cy + hw)
        if clamped_x != pos[0]:
            pos[0] = clamped_x
            reasons.append('x boundary')
        if clamped_y != pos[1]:
            pos[1] = clamped_y
            reasons.append('y boundary')

        # -- min standoff, re-derived from this node's own operator read --
        if operator_xy is not None:
            op = np.asarray(operator_xy, dtype=float)
            delta = pos[:2] - op
            d = float(np.linalg.norm(delta))
            if d < cfg.follow.min_standoff_m:
                direction = delta / d if d > 1e-6 else np.array([1.0, 0.0])
                pos[:2] = op + direction * cfg.follow.min_standoff_m
                reasons.append('min standoff (node re-check)')

        if self._last_pos is None or self._last_t is None:
            self.reset_to(pos, goal_yaw, now)
            return self._last_pos.copy(), self._last_yaw, reasons

        dt = max(1e-3, now - self._last_t)

        # -- velocity cap: limit how far the position may step this tick --
        step = pos - self._last_pos
        dist = float(np.linalg.norm(step))
        max_step = cfg.hard_speed_cap_m_s * dt
        if dist > max_step and dist > 1e-9:
            pos = self._last_pos + step * (max_step / dist)
            reasons.append(f'speed cap {cfg.hard_speed_cap_m_s:.2f} m/s')

        # -- acceleration cap: limit the change in the realised velocity ---
        realized_vel = (pos - self._last_pos) / dt
        dv = realized_vel - self._last_vel
        dv_norm = float(np.linalg.norm(dv))
        max_dv = cfg.hard_accel_cap_m_s2 * dt
        if dv_norm > max_dv and dv_norm > 1e-9:
            realized_vel = self._last_vel + dv * (max_dv / dv_norm)
            pos = self._last_pos + realized_vel * dt
            reasons.append(f'accel cap {cfg.hard_accel_cap_m_s2:.2f} m/s^2')
        else:
            realized_vel = (pos - self._last_pos) / dt

        # -- yaw turn-rate cap --------------------------------------------
        dyaw = float(wrap_pi(goal_yaw - self._last_yaw))
        max_dyaw = cfg.max_yaw_rate_rad_s * dt
        if abs(dyaw) > max_dyaw:
            dyaw = max_dyaw if dyaw > 0.0 else -max_dyaw
            reasons.append(f'yaw rate cap {cfg.max_yaw_rate_rad_s:.2f} rad/s')
        yaw = float(wrap_pi(self._last_yaw + dyaw))

        self._last_vel = realized_vel
        self._last_pos = pos.copy()
        self._last_yaw = yaw
        self._last_t = now
        return pos, yaw, reasons


def main(args=None):
    swarm = Crazyswarm()          # blocks until crazyflie_server's services exist
    node = swarm.allcfs           # an rclpy.node.Node subclass -- reuse it directly
    timeHelper = swarm.timeHelper
    log = node.get_logger()

    cfg = load_config(node)

    if cfg.drone_name not in node.crazyfliesByName:
        raise SystemExit(
            f'drone {cfg.drone_name!r} is not enabled in this workspace\'s '
            f'crazyflies.yaml (enabled: {sorted(node.crazyfliesByName)})')
    cf = node.crazyfliesByName[cfg.drone_name]

    # -------- offline safety proof, BEFORE anything arms --------------
    report = safety.preflight(
        cfg.follow, boundary_half_width_m=cfg.boundary_half_width_m,
        hard_speed_cap_m_s=cfg.hard_speed_cap_m_s,
        hard_accel_cap_m_s2=cfg.hard_accel_cap_m_s2)
    log.info(f'safety preflight OK: {report}')

    follower = Follower(cfg.follow)
    mocap = MocapTargetTracker(
        node, cfg.drone_name, cfg.operator_name,
        topic=cfg.poses_topic, lowpass_hz=cfg.mocap_lowpass_hz)
    guard = SetpointGuard(cfg)

    def on_cmd(msg: String) -> None:
        token = msg.data.strip().lower()
        now = timeHelper.time()
        if token == 'clear_estop':
            follower.arbiter.clear_estop()
            log.info('operator command: clear_estop')
            return
        mode = _MODE_TOKENS.get(token)
        if mode is None:
            log.warn(f'ignoring unrecognised command {msg.data!r} on '
                    f'{cfg.command_topic}')
            return
        source = (CommandSource.ESTOP_BUTTON if mode is FollowMode.ESTOP
                 else CommandSource.TABLET)
        follower.on_command(Command(mode=mode, source=source, stamp=now))

    node.create_subscription(String, cfg.command_topic, on_cmd, 10)

    # -------- wait for both rigid bodies before arming anything ---------
    log.info(f'waiting for {cfg.drone_name!r} and {cfg.operator_name!r} on '
             f'{cfg.poses_topic}...')
    while rclpy.ok() and not (mocap.drone.initialized and mocap.operator.initialized):
        rclpy.spin_once(node, timeout_sec=0.2)
    if not rclpy.ok():
        return
    log.info('both rigid bodies visible -- arming')

    # -------- arm + takeoff (high-level; no streaming setpoint sent yet) -
    cf.arm(True)
    timeHelper.sleep(1.0)
    cf.takeoff(targetHeight=cfg.takeoff_height_m, duration=cfg.takeoff_duration_s)
    timeHelper.sleep(cfg.takeoff_duration_s + 0.5)

    start_xy = mocap.drone.position[:2]
    guard.reset_to(
        np.array([start_xy[0], start_xy[1], cfg.takeoff_height_m]),
        mocap.drone.yaw, timeHelper.time())

    log.info('follow_node ready -- default mode HOLD; publish "follow" to '
             f'{cfg.command_topic} (or run keyboard_cmd) to engage')

    last_logged_mode: Optional[FollowMode] = None
    drone_stale_logged = False

    try:
        while rclpy.ok():
            now = timeHelper.time()

            if mocap.drone.is_stale(now, cfg.drone_stale_s):
                # We no longer know where the vehicle is. Nothing computed
                # this tick can be trusted, so hold the last good streamed
                # setpoint (already what re-sending it does) rather than
                # attempt to compute a new one from a stale position -- and
                # say so loudly, since this means the mocap link itself is
                # the problem, not the follow logic.
                if not drone_stale_logged:
                    log.error(
                        f'{cfg.drone_name} missing from {cfg.poses_topic} for '
                        f'{mocap.drone.age(now):.2f} s -- holding last '
                        'setpoint. Own position is unknown; take over '
                        'manually if this persists (see HANDOVER.md abort '
                        'procedure).')
                    drone_stale_logged = True
                cf.cmdPosition(guard.last_position, guard.last_yaw)
                timeHelper.sleepForRate(cfg.rate_hz)
                continue
            drone_stale_logged = False

            vehicle = VehicleState(
                stamp=mocap.drone.stamp, position=mocap.drone.position,
                velocity=mocap.drone.velocity, yaw=mocap.drone.yaw)
            follower.on_vehicle(vehicle)

            operator_stale_hold = mocap.operator.is_stale(now, cfg.operator_stale_hold_s)
            operator_stale_land = mocap.operator.is_stale(now, cfg.operator_stale_land_s)

            if not operator_stale_hold:
                follower.on_mocap_target(
                    mocap.operator.position, mocap.operator.stamp,
                    velocity=mocap.operator.velocity)
            # else: deliberately do not feed a stale sample -- letting the
            # EKF's timestamps age is what lets follow_core's own supervisor
            # (supervisor.py) see the target as DEGRADED/LOST too. The two
            # thresholds below act faster and are this node's own,
            # independent backstop -- see module docstring.

            out = follower.step(now)
            mode = out.mode
            reason = out.reason

            if operator_stale_land:
                mode = FollowMode.LAND
                reason = (f'operator stale {mocap.operator.age(now):.2f} s > '
                         f'operator_stale_land_s={cfg.operator_stale_land_s:.2f}')
            elif operator_stale_hold and mode in (FollowMode.FOLLOW, FollowMode.ORBIT):
                mode = FollowMode.HOLD
                reason = (f'operator stale {mocap.operator.age(now):.2f} s > '
                         f'operator_stale_hold_s={cfg.operator_stale_hold_s:.2f}')

            if mode is not last_logged_mode:
                log.info(f'mode -> {mode.value}  ({reason})')
                last_logged_mode = mode

            operator_xy = (
                (float(mocap.operator.position[0]), float(mocap.operator.position[1]))
                if mocap.operator.initialized else None)

            # If the envelope forced LAND/HOLD above, do NOT send the
            # FOLLOW/ORBIT goal follow_core computed before the override --
            # derive the forced mode's setpoint from THIS node's own last
            # commanded position instead. This deliberately does not touch
            # follow_core.planner/.governor a second time this tick (both
            # are stateful, and already advanced once inside follower.step()
            # above): a hold-in-place or a fixed-rate descent from "wherever
            # we currently are" is simple enough not to need them, and
            # staying out of follow_core's internals here keeps this
            # override path legible as *this node's* safety behaviour, not
            # a second, subtly different invocation of the control law.
            if mode is FollowMode.HOLD and out.mode is not FollowMode.HOLD:
                goal_position = guard.last_position.copy()
                goal_yaw = guard.last_yaw
            elif mode is FollowMode.LAND and out.mode is not FollowMode.LAND:
                dt_guard = (1.0 / cfg.rate_hz if guard._last_t is None
                           else max(1e-3, now - guard._last_t))
                descend_rate_m_s = 0.35   # matches standoff.py's own LAND ramp
                descended_z = max(cfg.altitude_floor_m,
                                  guard.last_position[2] - descend_rate_m_s * dt_guard)
                goal_position = np.array(
                    [guard.last_position[0], guard.last_position[1], descended_z])
                goal_yaw = guard.last_yaw
            else:
                goal_position = out.setpoint.position
                goal_yaw = out.setpoint.yaw

            pos, yaw, clamp_reasons = guard.apply(
                goal_position, goal_yaw, operator_xy, now)
            if clamp_reasons:
                log.warn('setpoint guard clamped: ' + ', '.join(clamp_reasons),
                         throttle_duration_sec=2.0)

            cf.cmdPosition(pos, yaw)

            timeHelper.sleepForRate(cfg.rate_hz)
    except KeyboardInterrupt:
        pass
    finally:
        log.info('shutting down -- notify_setpoints_stop, land, disarm')
        cf.notifySetpointsStop()
        timeHelper.sleep(0.2)
        cf.land(targetHeight=cfg.land_height_m, duration=cfg.land_duration_s)
        timeHelper.sleep(cfg.land_duration_s + 0.5)
        cf.arm(False)


if __name__ == '__main__':
    main()
