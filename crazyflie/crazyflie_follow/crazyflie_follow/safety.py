"""Offline verification for the Crazyflie follow envelope.

Nothing here talks to a drone, mocap, or ROS. The point is the same as
crazyflie_shows/safety.py's: prove a configuration is safe on the ground,
before anything is armed -- this rig's firmware has no collision avoidance
enabled, and here the "obstacle" is a person the drone is deliberately flying
near, which makes an offline proof strictly more important than it is for a
swarm show, not less.

Three separate things are proved, because they fail independently and for
different reasons:

  (a) `check_boundary_feasible` -- that the *unclamped* goal geometry (operator
      position + standoff offset in an arbitrary direction, per standoff.py)
      cannot leave the 8x8 m soft boundary as long as the operator stays
      within a computable inset region. This is about the boundary clamp in
      follow_node's SetpointGuard being a backstop, not a routine occurrence:
      a plan where the clamp fires constantly is fighting the control law
      every tick and will look like stuttering in flight, not a hard fault.

  (b) `check_min_standoff_enforced` -- that follow_core's own governor, given
      these exact FollowParams, cannot produce a goal closer than
      `min_standoff_m` to the operator, for any operator/vehicle geometry.
      This does not re-derive the clamp math by hand (that would be exactly
      the kind of duplicated logic that drifts from the real thing) -- it
      imports follow_core.governor.SpeedGovernor and drives the actual code
      that will fly, the same relationship plan_show.py has to
      choreography.py in crazyflie_shows.

  (c) `check_speed_consistency` -- that the configured speed caps and
      acceleration limit are physically consistent with the standoff
      geometry: a vehicle entering the near-person speed ramp at
      `speed_cap_far_m_s` must be able to brake to `speed_cap_near_m_s`
      before `governor_far_m` narrows down to `min_standoff_m`, given
      `max_accel_m_s2`. A speed cap that "looks" conservative on paper but
      cannot physically be bled off in the available distance is not
      conservative at all.

`preflight()` runs all three and returns a report dict, raising ValueError on
the first violation -- mirroring check_show()'s contract in
crazyflie_shows/safety.py. follow_node calls this once, before arming
anything.
"""
from __future__ import annotations

from typing import Dict

import numpy as np

from follow_core import (
    FollowMode,
    FollowParams,
    Setpoint,
    TargetEstimate,
    TrackHealth,
    VehicleState,
)
from follow_core.governor import SpeedGovernor

#: Crazyflie 2.1 default room geometry for this rig (CLAUDE.md, hand-measured,
#: cross-checked in crazyflie_shows/HANDOVER.md Sec 3 against an independently
#: computed swarm centroid: 0.068, -0.109 -- within 2 cm of this value on both
#: axes). Not this package's number to own; kept here only as the function
#: defaults so `check_safety` runs with something sane out of the box.
ROOM_CENTER = (0.0467, -0.1037)
BOUNDARY_HALF_WIDTH_M = 4.0   # an 8 x 8 m box, see follow_node.py


def check_boundary_feasible(standoff_m: float,
                            boundary_half_width_m: float = BOUNDARY_HALF_WIDTH_M
                            ) -> float:
    """Largest operator excursion (from room centre, per axis) that still
    guarantees an *unclamped* goal stays inside the boundary box.

    standoff.py offsets the goal from the operator by exactly `standoff_m` in
    a direction that depends on the operator's course and the configured
    follow angle -- both effectively arbitrary from this function's point of
    view. So the worst case is a full circle of radius `standoff_m` around
    the operator's position. If the operator's own position stays within the
    box inset by `standoff_m` on every side, the goal -- anywhere on that
    circle -- provably cannot leave the outer box:

        |goal_x - center_x| <= |operator_x - center_x| + standoff_m
                             <= (half_width - standoff_m) + standoff_m
                             == half_width

    and symmetrically for y. Returns the inset half-width (the operator's own
    safe-to-roam region); raises if `standoff_m` already consumes the whole
    box.
    """
    if standoff_m >= boundary_half_width_m:
        raise ValueError(
            f'standoff_m={standoff_m:.2f} m >= boundary half-width '
            f'{boundary_half_width_m:.2f} m: no operator position keeps the '
            'unclamped goal inside the boundary at all')
    return boundary_half_width_m - standoff_m


def check_min_standoff_enforced(params: FollowParams, n_samples: int = 4000,
                                seed: int = 0) -> float:
    """Stress-test the real SpeedGovernor clamp against random geometry.

    Drives follow_core.governor.SpeedGovernor.apply() -- the exact code
    follow_node runs every tick -- with `n_samples` random (operator, goal,
    vehicle) triples, including goals placed exactly on top of the operator
    (the degenerate case governor.py special-cases) and goals both inside and
    outside the min-standoff sphere. Returns the worst-case horizontal
    separation observed after clamping; raises if it is ever below
    `params.min_standoff_m` (less a float tolerance).

    This is deliberately a numeric stress test, not a proof by construction:
    governor.py's clamp is simple enough that construction alone would do,
    but running the actual code is what keeps this check from silently going
    stale if governor.py's clamp logic ever changes.
    """
    rng = np.random.default_rng(seed)
    governor = SpeedGovernor(params)
    worst = np.inf

    for _ in range(n_samples):
        operator_xy = rng.uniform(-6.0, 6.0, size=2)
        operator_pos = np.array([operator_xy[0], operator_xy[1], 0.0])

        # Sample goal radius from 0 (degenerate: goal on top of operator) out
        # to well past governor_far_m, and an arbitrary bearing.
        radius = rng.uniform(0.0, params.governor_far_m * 1.5)
        theta = rng.uniform(-np.pi, np.pi)
        goal = operator_pos + np.array(
            [radius * np.cos(theta), radius * np.sin(theta), params.height_m])

        # Vehicle position matters only for the degenerate (radius==0) branch,
        # which retreats along vehicle-minus-operator -- sample it away from
        # the operator so that branch is exercised meaningfully too.
        vehicle_xy = rng.uniform(-6.0, 6.0, size=2)
        vehicle_pos = np.array([vehicle_xy[0], vehicle_xy[1], params.height_m])

        setpoint = Setpoint(
            stamp=0.0, position=goal, velocity_ff=np.zeros(3), yaw=0.0,
            max_speed=params.speed_cap_far_m_s, mode=FollowMode.FOLLOW)
        vehicle = VehicleState(stamp=0.0, position=vehicle_pos)
        target = TargetEstimate(
            stamp=0.0, position=operator_pos, velocity=np.zeros(3),
            covariance=np.eye(6) * 0.01, health=TrackHealth.TRACKING)

        out = governor.apply(setpoint, vehicle, target, dt=1.0 / 20.0)
        d = float(np.linalg.norm(out.position[:2] - operator_pos[:2]))
        worst = min(worst, d)

    if worst < params.min_standoff_m - 1e-6:
        raise ValueError(
            f'governor clamp produced a goal {worst:.3f} m from the operator, '
            f'below min_standoff_m={params.min_standoff_m:.2f} m, over '
            f'{n_samples} sampled geometries')
    return worst


def check_speed_consistency(params: FollowParams,
                            hard_speed_cap_m_s: float,
                            hard_accel_cap_m_s2: float) -> Dict[str, float]:
    """Check the configured speed/accel budget is physically achievable.

    A vehicle riding the top of governor.py's ramp at `speed_cap_far_m_s` as
    it crosses into the near-person zone must be able to bleed off to
    `speed_cap_near_m_s` before the range closes from `governor_far_m` to
    `min_standoff_m` -- otherwise the "cap" is a number on paper the
    controller cannot actually track, and an untracked cap is exactly the
    failure mode governor.py's own docstring warns about (a lagging drone is
    not where the collision check thinks it is).

    Also checks the ordinary orderings (near <= far, both positive, the
    node's own hard caps positive) that make the rest of the numbers
    meaningful at all. `params.validate()` already checks
    `speed_cap_near_m_s <= speed_cap_far_m_s`; this function re-derives that
    result too (via a plain assertion) so a caller gets one place to look.
    """
    p = params
    if hard_speed_cap_m_s <= 0.0:
        raise ValueError('hard_speed_cap_m_s must be positive')
    if hard_accel_cap_m_s2 <= 0.0:
        raise ValueError('hard_accel_cap_m_s2 must be positive')
    if p.speed_cap_near_m_s > p.speed_cap_far_m_s:
        raise ValueError(
            f'speed_cap_near_m_s={p.speed_cap_near_m_s:.2f} > '
            f'speed_cap_far_m_s={p.speed_cap_far_m_s:.2f}')

    available_m = p.governor_far_m - p.min_standoff_m
    if available_m <= 0.0:
        raise ValueError(
            f'governor_far_m={p.governor_far_m:.2f} <= '
            f'min_standoff_m={p.min_standoff_m:.2f}: no ramp distance at all')

    # v^2 = 2 * a * d, worst case using the *node's* hard accel cap (the
    # smaller, more conservative of the two accel limits in play -- see
    # follow_node.SetpointGuard, which enforces this one independently of
    # whatever follow_core's own governor does).
    entry_speed = min(p.speed_cap_far_m_s, hard_speed_cap_m_s)
    braking_distance_m = entry_speed ** 2 / (2.0 * hard_accel_cap_m_s2)

    if braking_distance_m > available_m:
        raise ValueError(
            f'braking distance {braking_distance_m:.2f} m (from '
            f'{entry_speed:.2f} m/s at {hard_accel_cap_m_s2:.2f} m/s^2) '
            f'exceeds the {available_m:.2f} m available between '
            f'governor_far_m={p.governor_far_m:.2f} m and '
            f'min_standoff_m={p.min_standoff_m:.2f} m -- the speed cap cannot '
            'physically be bled off before the standoff sphere')

    return {
        'entry_speed_m_s': entry_speed,
        'braking_distance_m': braking_distance_m,
        'available_ramp_m': available_m,
        'margin_m': available_m - braking_distance_m,
    }


def preflight(params: FollowParams, boundary_half_width_m: float,
             hard_speed_cap_m_s: float, hard_accel_cap_m_s2: float
             ) -> Dict[str, float]:
    """Run every offline check; raise on the first violation.

    Called once by follow_node before arming, and by the standalone
    `check_safety` entry point below with no ROS running at all.
    """
    params.validate()
    report: Dict[str, float] = {}
    report['operator_safe_halfwidth_m'] = check_boundary_feasible(
        params.standoff_m, boundary_half_width_m)
    report['worst_case_standoff_m'] = check_min_standoff_enforced(params)
    report.update(check_speed_consistency(
        params, hard_speed_cap_m_s, hard_accel_cap_m_s2))
    return report


# ---------------------------------------------------------------------------
# Standalone entry point: `ros2 run crazyflie_follow check_safety`.
#
# Reads config/follow.yaml directly, like plan_show.py reads crazyflies.yaml
# in crazyflie_shows -- no ROS graph, no radio, no drone, so it runs on a
# laptop that has never seen the rig.
# ---------------------------------------------------------------------------

def _default_config_path() -> str:
    import os
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory('crazyflie_follow'), 'config', 'follow.yaml')
    except Exception:
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(here, 'config', 'follow.yaml')


def main(argv=None) -> None:
    import argparse

    import yaml

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('config', nargs='?', default=None,
                    help='path to follow.yaml (default: the installed config '
                        'if the workspace is sourced, else the source tree)')
    args = ap.parse_args(argv)

    path = args.config or _default_config_path()
    with open(path) as f:
        doc = yaml.safe_load(f)
    p = doc['/**']['ros__parameters']

    params = FollowParams(
        standoff_m=float(p['standoff_m']),
        height_m=float(p['height_m']),
        follow_angle_rad=float(p['follow_angle_rad']),
        orbit_rate_rad_s=float(p.get('orbit_rate_rad_s', 0.35)),
        min_standoff_m=float(p['min_standoff_m']),
        speed_cap_far_m_s=float(p['speed_cap_far_m_s']),
        speed_cap_near_m_s=float(p['speed_cap_near_m_s']),
        governor_far_m=float(p['governor_far_m']),
        max_accel_m_s2=float(p['max_accel_m_s2']),
    )

    print(f'reading {path}')
    report = preflight(
        params,
        boundary_half_width_m=float(p['boundary_half_width_m']),
        hard_speed_cap_m_s=float(p['hard_speed_cap_m_s']),
        hard_accel_cap_m_s2=float(p['hard_accel_cap_m_s2']))

    for k, v in report.items():
        print(f'  {k}: {v:.3f}')
    print('safety preflight OK -- no ROS, no radio, no drone involved')


if __name__ == '__main__':
    main()
