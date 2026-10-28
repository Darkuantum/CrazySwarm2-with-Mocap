import numpy as np
import pytest

from follow_core.governor import SpeedGovernor
from follow_core.types import (
    FollowMode, FollowParams, Setpoint, TargetEstimate, VehicleState,
)


def mk_sp(pos, vff=(0, 0, 0), cap=3.0):
    return Setpoint(stamp=0.0, position=np.array(pos, float),
                    velocity_ff=np.array(vff, float), yaw=0.0,
                    max_speed=cap, mode=FollowMode.FOLLOW)


def tgt(pos=(0, 0, 0)):
    return TargetEstimate(stamp=0.0, position=np.array(pos, float),
                          velocity=np.zeros(3), covariance=np.eye(6) * 0.01)


def test_speed_cap_is_monotonic_in_range():
    g = SpeedGovernor(FollowParams())
    caps = [g.speed_cap_for_range(r) for r in np.linspace(0.0, 8.0, 40)]
    assert all(b >= a - 1e-12 for a, b in zip(caps, caps[1:]))


def test_speed_cap_endpoints():
    p = FollowParams(min_standoff_m=1.5, governor_far_m=4.0,
                     speed_cap_near_m_s=0.6, speed_cap_far_m_s=3.0)
    g = SpeedGovernor(p)
    assert g.speed_cap_for_range(0.5) == pytest.approx(0.6)
    assert g.speed_cap_for_range(1.5) == pytest.approx(0.6)
    assert g.speed_cap_for_range(4.0) == pytest.approx(3.0)
    assert g.speed_cap_for_range(50.0) == pytest.approx(3.0)
    assert g.speed_cap_for_range(2.75) == pytest.approx(1.8)


def test_hard_clamp_pushes_goal_out_to_min_standoff():
    """Second, independent protection: even a broken control law cannot command
    the vehicle inside the minimum standoff sphere."""
    p = FollowParams(min_standoff_m=1.5)
    g = SpeedGovernor(p)
    out = g.apply(mk_sp((0.3, 0.0, 1.5)), VehicleState(stamp=0, position=[3, 0, 2]), tgt(), 0.05)
    assert np.linalg.norm(out.position[:2]) == pytest.approx(1.5)


def test_degenerate_goal_on_top_of_operator_retreats_sensibly():
    g = SpeedGovernor(FollowParams(min_standoff_m=1.5))
    v = VehicleState(stamp=0, position=[2.0, 0.0, 2.0])
    out = g.apply(mk_sp((0.0, 0.0, 1.5)), v, tgt(), 0.05)
    d = np.linalg.norm(out.position[:2])
    assert d == pytest.approx(1.5)
    # It should retreat along the vehicle's existing bearing, not an arbitrary one.
    assert out.position[0] > 0.0


def test_feedforward_is_clipped_to_the_cap():
    g = SpeedGovernor(FollowParams())
    v = VehicleState(stamp=0, position=[2.0, 0, 2.0])   # 2 m from operator
    out = g.apply(mk_sp((-3, 0, 2), vff=(10.0, 0, 0)), v, tgt(), 0.05)
    assert np.linalg.norm(out.velocity_ff) <= out.max_speed + 1e-9
    assert out.max_speed < 3.0     # close to a person -> reduced


def test_cap_is_slew_limited():
    p = FollowParams(max_accel_m_s2=2.0)
    g = SpeedGovernor(p)
    far = VehicleState(stamp=0, position=[20.0, 0, 2.0])
    g.apply(mk_sp((-3, 0, 2)), far, tgt(), 0.05)
    near = VehicleState(stamp=0, position=[1.6, 0, 2.0])
    out = g.apply(mk_sp((-3, 0, 2)), near, tgt(), 0.05)
    # Cannot drop by more than max_accel*dt = 0.1 in one 50 ms tick.
    assert out.max_speed >= 3.0 - 0.1 - 1e-9


def test_no_target_leaves_setpoint_speed_alone():
    g = SpeedGovernor(FollowParams())
    v = VehicleState(stamp=0, position=[0, 0, 2.0])
    out = g.apply(mk_sp((1, 1, 2), cap=1.0), v, None, 0.05)
    assert out.max_speed == pytest.approx(1.0)


def test_a_zero_speed_ceiling_is_honoured_not_discarded():
    """Regression: a HOLD/ESTOP setpoint asks for max_speed 0. An earlier
    version treated 0 as 'unset' and handed back the range-based cap instead,
    so a vehicle told to hold still carried a multi-m/s velocity budget."""
    g = SpeedGovernor(FollowParams())
    v = VehicleState(stamp=0, position=[-2.5, 0, 2.0])
    g.apply(mk_sp((-2.5, 0, 2), cap=3.0), v, tgt(), 0.05)     # build up _last_cap
    hold = mk_sp((-2.5, 0, 2), cap=0.0)
    hold.mode = FollowMode.HOLD
    out = g.apply(hold, v, tgt(), 0.05)
    assert out.max_speed == 0.0
    assert np.allclose(out.velocity_ff, 0.0)


def test_stop_is_immediate_and_not_slew_limited():
    g = SpeedGovernor(FollowParams(max_accel_m_s2=0.5))
    v = VehicleState(stamp=0, position=[-8.0, 0, 2.0])
    for _ in range(20):
        g.apply(mk_sp((-8, 0, 2), cap=3.0), v, tgt(), 0.05)
    out = g.apply(mk_sp((-8, 0, 2), cap=0.0), v, tgt(), 0.05)
    assert out.max_speed == 0.0, "a commanded stop must not ramp down"
