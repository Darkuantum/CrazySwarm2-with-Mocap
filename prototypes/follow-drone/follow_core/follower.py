"""Top-level composition: sensors in, one setpoint out.

This is the object both platforms drive. The Crazyflie testbed feeds it mocap
in place of UWB and vision; the Starling feeds it real sensors. Because the
logic lives here and not in either ROS node, what gets validated in the mocap
volume in week 6 is literally the same code that flies on the Starling in
week 12 -- which is the entire justification for the Crazyflie phase.

Nothing in this module imports ROS, cflib, or numpy-adjacent hardware helpers.
Keep it that way.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .arbiter import CommandArbiter
from .ekf import TargetEKF
from .governor import SpeedGovernor
from .standoff import StandoffPlanner
from .supervisor import TrackSupervisor
from .types import (
    BearingMeasurement,
    Command,
    CommandSource,
    FollowMode,
    FollowParams,
    RangeMeasurement,
    Setpoint,
    TargetEstimate,
    TrackHealth,
    VehicleState,
)


class HapticCue:
    """Codes written to the band's haptic characteristic. Match firmware."""

    NONE = 0
    ACK = 1          # command accepted
    TARGET_LOST = 2  # double pulse
    FAILSAFE = 3     # long buzz


@dataclass
class FollowerOutput:
    stamp: float
    setpoint: Setpoint
    mode: FollowMode
    health: TrackHealth
    estimate: Optional[TargetEstimate]
    haptic: int = HapticCue.NONE
    reason: str = ""


class Follower:
    def __init__(self, params: Optional[FollowParams] = None) -> None:
        self.p = params or FollowParams()
        self.p.validate()
        self.ekf = TargetEKF(self.p)
        self.planner = StandoffPlanner(self.p)
        self.governor = SpeedGovernor(self.p)
        self.arbiter = CommandArbiter(self.p)
        self.supervisor = TrackSupervisor(self.p)

        self.vehicle: Optional[VehicleState] = None
        self._pending_range: Optional[RangeMeasurement] = None
        self._pending_bearing: Optional[BearingMeasurement] = None
        self._last_step: Optional[float] = None
        self._last_health = TrackHealth.LOST
        self._last_mode = FollowMode.IDLE

    # -- inputs --------------------------------------------------------------

    def on_vehicle(self, state: VehicleState) -> None:
        self.vehicle = state

    def on_command(self, cmd: Command) -> None:
        self.arbiter.submit(cmd)

    def on_range(self, meas: RangeMeasurement) -> None:
        if self.vehicle is None:
            return
        if not self.ekf.initialized:
            # Cannot triangulate from range alone; hold it for a bearing to
            # arrive, or seed a coarse guess along the camera boresight if the
            # bearing never comes.
            self._pending_range = meas
            if self._pending_bearing is not None:
                self.ekf.initialize_from_range_bearing(
                    meas, self._pending_bearing, self.vehicle
                )
                self._pending_range = None
                self._pending_bearing = None
            return
        self.ekf.update_range(meas, self.vehicle)

    def on_bearing(self, meas: BearingMeasurement) -> None:
        if self.vehicle is None:
            return
        if not self.ekf.initialized:
            self._pending_bearing = meas
            if self._pending_range is not None:
                self.ekf.initialize_from_range_bearing(
                    self._pending_range, meas, self.vehicle
                )
                self._pending_range = None
                self._pending_bearing = None
            return
        self.ekf.update_bearing(meas, self.vehicle)

    def on_mocap_target(self, position, stamp: float, velocity=None) -> None:
        """Crazyflie-phase shortcut: a perfect target from OptiTrack.

        Lets the control law, arbiter, governor and supervisor be validated in
        week 4 with no perception at all -- which is the documented failure mode
        this project is built to avoid (a comparable thesis reached its final
        flight test with a non-functional tracker and had to simulate it).
        """
        pos = np.asarray(position, dtype=float).reshape(3)
        if not self.ekf.initialized:
            self.ekf.initialize(pos, stamp, velocity=velocity, pos_std=0.02, vel_std=0.2)
        else:
            self.ekf.predict(stamp)
            H = np.zeros((3, TargetEKF.DIM))
            H[:, :3] = np.eye(3)
            R = np.eye(3) * (0.01**2)
            nu = pos - self.ekf.x[:3]
            S = H @ self.ekf.P @ H.T + R
            self.ekf._apply(H, nu, S)
        self.ekf.last_range_stamp = stamp
        self.ekf.last_bearing_stamp = stamp

    # -- output --------------------------------------------------------------

    def step(self, now: float) -> FollowerOutput:
        dt = 0.0 if self._last_step is None else max(0.0, now - self._last_step)
        self._last_step = now

        if self.vehicle is None:
            return FollowerOutput(
                stamp=now,
                setpoint=Setpoint(
                    stamp=now,
                    position=np.zeros(3),
                    velocity_ff=np.zeros(3),
                    yaw=0.0,
                    max_speed=0.0,
                    mode=FollowMode.IDLE,
                ),
                mode=FollowMode.IDLE,
                health=TrackHealth.LOST,
                estimate=None,
                reason="no vehicle state",
            )

        if self.ekf.initialized:
            self.ekf.predict(now)

        health = self.supervisor.update(
            now,
            self.ekf.last_range_stamp,
            self.ekf.last_bearing_stamp,
            self.ekf.initialized,
        )
        estimate = self.ekf.estimate(health) if self.ekf.initialized else None

        # Trust the covariance over the clock. Range keeps arriving and keeps
        # being accepted while vision is gone, so the freshness stamps look
        # healthy right up until the vehicle flies to a position the filter was
        # never confident about.
        if estimate is not None and health is not TrackHealth.LOST:
            if float(np.max(estimate.position_std[:2])) > self.p.max_position_std_m:
                health = TrackHealth.LOST
                estimate.health = health

        cmd = self.arbiter.resolve(now)
        mode = cmd.mode
        reason = f"cmd:{cmd.source.value}"

        # FR-8: a lost target downgrades FOLLOW/ORBIT to HOLD. It does not touch
        # LAND or ESTOP -- those must still be honoured with no target at all.
        if health is TrackHealth.LOST and mode in (FollowMode.FOLLOW, FollowMode.ORBIT):
            mode = FollowMode.HOLD
            reason = "target lost"

        setpoint = self.planner.compute(
            mode,
            estimate,
            self.vehicle,
            now,
            standoff_m=cmd.standoff_m,
            height_m=cmd.height_m,
            follow_angle_rad=cmd.follow_angle_rad,
        )
        setpoint = self.governor.apply(setpoint, self.vehicle, estimate, dt)

        haptic = HapticCue.NONE
        if health is TrackHealth.LOST and self._last_health is not TrackHealth.LOST:
            haptic = HapticCue.TARGET_LOST
        elif mode is FollowMode.ESTOP and self._last_mode is not FollowMode.ESTOP:
            haptic = HapticCue.FAILSAFE
        elif mode is not self._last_mode:
            haptic = HapticCue.ACK

        self._last_health = health
        self._last_mode = mode

        return FollowerOutput(
            stamp=now,
            setpoint=setpoint,
            mode=setpoint.mode,
            health=health,
            estimate=estimate,
            haptic=haptic,
            reason=reason,
        )
