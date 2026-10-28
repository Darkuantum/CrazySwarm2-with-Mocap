"""Speed governor -- the requirement that the vehicle is gentle near a person.

NFR-4 says the speed cap near a person must be justified by measurement rather
than picked arbitrarily. The published impact-test toolchain maps pre-impact
speed to impulse and contact duration, which inverts a force constraint into a
velocity bound. This module is where that bound is enforced; substitute your own
measured numbers into FollowParams once the bench tests are done.

Two independent protections, because they fail differently:
  1. Speed is capped as a function of range to the operator.
  2. The commanded position is clamped so it never sits closer than
     min_standoff_m, regardless of what the planner asked for.
A bug in the control law can defeat (1). It cannot defeat (2).
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from .types import FollowParams, Setpoint, TargetEstimate, VehicleState


class SpeedGovernor:
    def __init__(self, params: Optional[FollowParams] = None) -> None:
        self.p = params or FollowParams()
        self._last_cap: Optional[float] = None

    def reset(self) -> None:
        self._last_cap = None

    def speed_cap_for_range(self, range_m: float) -> float:
        """Piecewise-linear cap: full speed far away, crawling speed up close."""
        p = self.p
        r = float(range_m)
        if r <= p.min_standoff_m:
            return p.speed_cap_near_m_s
        if r >= p.governor_far_m:
            return p.speed_cap_far_m_s
        frac = (r - p.min_standoff_m) / (p.governor_far_m - p.min_standoff_m)
        return p.speed_cap_near_m_s + frac * (p.speed_cap_far_m_s - p.speed_cap_near_m_s)

    def apply(
        self,
        setpoint: Setpoint,
        vehicle: VehicleState,
        target: Optional[TargetEstimate],
        dt: float,
    ) -> Setpoint:
        p = self.p
        pos = setpoint.position.copy()
        vel_ff = setpoint.velocity_ff.copy()

        if target is not None:
            # (2) Hard clamp: push the goal out to the minimum standoff sphere.
            delta = pos - target.position
            horiz = delta[:2]
            d_h = float(np.linalg.norm(horiz))
            if d_h < p.min_standoff_m:
                if d_h < 1e-6:
                    # Degenerate: goal is on top of the operator. Retreat along
                    # the vehicle's current bearing from them.
                    away = vehicle.position[:2] - target.position[:2]
                    n = float(np.linalg.norm(away))
                    horiz = (away / n) if n > 1e-6 else np.array([1.0, 0.0])
                else:
                    horiz = horiz / d_h
                pos[:2] = target.position[:2] + horiz * p.min_standoff_m

            range_now = float(np.linalg.norm(vehicle.position - target.position))
            cap = self.speed_cap_for_range(range_now)
        else:
            cap = setpoint.max_speed

        # The planner's max_speed is a HARD ceiling and is always applied. An
        # earlier version skipped it when it was zero, on the theory that zero
        # meant "unset" -- which silently discarded exactly the case that
        # matters, a HOLD or ESTOP setpoint asking for no motion at all.
        hard_ceiling = float(setpoint.max_speed)
        cap = min(cap, hard_ceiling)

        if hard_ceiling <= 0.0:
            # A commanded stop takes effect now. Slewing down to zero would put
            # a ramp between the operator pressing stop and the vehicle
            # stopping, which is the one place a smooth profile is wrong.
            cap = 0.0
            self._last_cap = 0.0
        else:
            # Otherwise slew-limit so the vehicle does not step-change speed.
            if self._last_cap is not None and dt > 0.0:
                max_delta = p.max_accel_m_s2 * dt
                cap = float(np.clip(cap, self._last_cap - max_delta, self._last_cap + max_delta))
            self._last_cap = cap

        # Feedforward may not exceed the cap either.
        v = float(np.linalg.norm(vel_ff))
        if v > cap > 0.0:
            vel_ff = vel_ff * (cap / v)
        elif cap <= 0.0:
            vel_ff = np.zeros(3)

        return Setpoint(
            stamp=setpoint.stamp,
            position=pos,
            velocity_ff=vel_ff,
            yaw=setpoint.yaw,
            max_speed=max(cap, 0.0),
            mode=setpoint.mode,
        )
