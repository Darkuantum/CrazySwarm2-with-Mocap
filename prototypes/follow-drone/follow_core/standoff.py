"""Standoff geometry: where the vehicle should be, given where the operator is.

The output is a GOAL, not a path. On the Starling this goal is handed to
voxl-mapper, which owns the obstacle-free route to it. The follower never
commands a straight line it has not checked -- that separation is the whole
reason obstacle avoidance can live in the planner (see docs/ARCHITECTURE.md).
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from .types import (
    FollowMode,
    FollowParams,
    Setpoint,
    TargetEstimate,
    VehicleState,
    wrap_pi,
)

# Below this speed the operator's course is meaningless noise, so we freeze the
# follow bearing instead of letting the vehicle chase a spinning estimate.
_COURSE_MIN_SPEED = 0.35


class StandoffPlanner:
    """Turns a target estimate into a vehicle goal pose.

    Holds a little state: the last committed course (so the vehicle does not
    orbit a stationary operator) and the orbit phase.
    """

    def __init__(self, params: Optional[FollowParams] = None) -> None:
        self.p = params or FollowParams()
        self._course: Optional[float] = None
        self._orbit_phase: float = 0.0
        self._last_stamp: Optional[float] = None
        self._land_z: Optional[float] = None
        # Commanded follow bearing, slew-limited. See _slew_bearing.
        self._bearing: Optional[float] = None

    def reset(self) -> None:
        self._course = None
        self._orbit_phase = 0.0
        self._last_stamp = None
        self._land_z = None
        self._bearing = None

    def _slew_bearing(self, desired: float, standoff: float, dt: float) -> float:
        """Rate-limit the follow bearing to what the vehicle can actually fly.

        The naive law puts the goal at (operator course + follow angle). But
        course is an instantaneous quantity: when the operator turns a corner
        or reverses, it steps discontinuously and the goal teleports to the far
        side of them. The vehicle then flies a chord straight past the operator
        -- which is both a tracking failure and a safety one.

        The vehicle orbiting at `standoff` at speed v sustains an angular rate
        of v / standoff, so that is the honest limit. Exceeding it does not make
        the drone move faster; it just commands a goal it cannot reach.
        """
        if self._bearing is None or dt <= 0.0:
            self._bearing = float(desired)
            return self._bearing
        max_rate = self.p.speed_cap_far_m_s / max(standoff, 1e-3)
        step = float(wrap_pi(desired - self._bearing))
        limit = max_rate * dt
        if step > limit:
            step = limit
        elif step < -limit:
            step = -limit
        self._bearing = float(wrap_pi(self._bearing + step))
        return self._bearing

    def _update_course(self, target: TargetEstimate) -> float:
        """Latch the operator's direction of travel, ignoring standstill jitter."""
        if target.speed >= _COURSE_MIN_SPEED:
            self._course = target.course_rad
        elif self._course is None:
            self._course = target.course_rad
        return float(self._course)

    def compute(
        self,
        mode: FollowMode,
        target: Optional[TargetEstimate],
        vehicle: VehicleState,
        now: float,
        standoff_m: Optional[float] = None,
        height_m: Optional[float] = None,
        follow_angle_rad: Optional[float] = None,
    ) -> Setpoint:
        dt = 0.0 if self._last_stamp is None else max(0.0, now - self._last_stamp)
        self._last_stamp = now

        standoff = self.p.standoff_m if standoff_m is None else float(standoff_m)
        standoff = max(standoff, self.p.min_standoff_m)
        height = self.p.height_m if height_m is None else float(height_m)
        angle = (
            self.p.follow_angle_rad
            if follow_angle_rad is None
            else float(follow_angle_rad)
        )

        # Modes that never reference the target.
        if mode in (FollowMode.HOLD, FollowMode.IDLE, FollowMode.ESTOP):
            return Setpoint(
                stamp=now,
                position=vehicle.position.copy(),
                velocity_ff=np.zeros(3),
                yaw=vehicle.yaw,
                max_speed=0.0,
                mode=mode,
            )

        if mode is FollowMode.LAND:
            if self._land_z is None:
                self._land_z = float(vehicle.position[2])
            self._land_z = max(0.0, self._land_z - 0.35 * dt)
            pos = vehicle.position.copy()
            pos[2] = self._land_z
            return Setpoint(
                stamp=now,
                position=pos,
                velocity_ff=np.zeros(3),
                yaw=vehicle.yaw,
                max_speed=0.5,
                mode=mode,
            )

        # FOLLOW / ORBIT both need a target.
        if target is None:
            return Setpoint(
                stamp=now,
                position=vehicle.position.copy(),
                velocity_ff=np.zeros(3),
                yaw=vehicle.yaw,
                max_speed=0.0,
                mode=FollowMode.HOLD,
            )

        course = self._update_course(target)

        if mode is FollowMode.ORBIT:
            self._orbit_phase = float(wrap_pi(self._orbit_phase + self.p.orbit_rate_rad_s * dt))
            bearing = self._orbit_phase
            # Orbiting adds a tangential component on top of the operator's motion.
            tangential = self.p.orbit_rate_rad_s * standoff
            vel_ff = target.velocity + np.array(
                [-np.sin(bearing) * tangential, np.cos(bearing) * tangential, 0.0]
            )
        else:
            bearing = self._slew_bearing(float(wrap_pi(course + angle)), standoff, dt)
            vel_ff = target.velocity.copy()

        offset = np.array([np.cos(bearing) * standoff, np.sin(bearing) * standoff, 0.0])
        goal = target.position + offset
        goal[2] = target.position[2] + height

        # Yaw so the camera keeps the operator in frame -- which is also what
        # keeps the vision bearing measurement alive.
        to_target = target.position[:2] - goal[:2]
        yaw = float(np.arctan2(to_target[1], to_target[0]))

        return Setpoint(
            stamp=now,
            position=goal,
            velocity_ff=vel_ff,
            yaw=yaw,
            max_speed=self.p.speed_cap_far_m_s,
            mode=mode,
        )
