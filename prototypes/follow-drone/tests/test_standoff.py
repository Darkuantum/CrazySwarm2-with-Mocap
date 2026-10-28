import numpy as np
import pytest

from follow_core.standoff import StandoffPlanner
from follow_core.types import FollowMode, FollowParams, TargetEstimate, VehicleState


def tgt(pos, vel=(0, 0, 0), stamp=0.0):
    return TargetEstimate(
        stamp=stamp, position=np.array(pos, float), velocity=np.array(vel, float),
        covariance=np.eye(6) * 0.05,
    )


def veh(pos=(0, 0, 2.0), stamp=0.0):
    return VehicleState(stamp=stamp, position=np.array(pos, float))


def test_goal_sits_behind_a_walking_operator():
    p = FollowParams(standoff_m=3.0, height_m=1.5, follow_angle_rad=np.pi)
    sp = StandoffPlanner(p)
    t = tgt((0, 0, 0), (1.0, 0, 0))          # walking +x
    s = sp.compute(FollowMode.FOLLOW, t, veh(), 0.0)
    assert s.position[0] == pytest.approx(-3.0)   # 3 m behind
    assert s.position[1] == pytest.approx(0.0, abs=1e-9)
    assert s.position[2] == pytest.approx(1.5)


def test_follow_angle_zero_puts_the_drone_in_front():
    p = FollowParams(standoff_m=2.0, follow_angle_rad=0.0)
    sp = StandoffPlanner(p)
    s = sp.compute(FollowMode.FOLLOW, tgt((0, 0, 0), (1.0, 0, 0)), veh(), 0.0)
    assert s.position[0] == pytest.approx(2.0)


def test_yaw_always_points_at_the_operator():
    sp = StandoffPlanner(FollowParams(standoff_m=2.0, follow_angle_rad=np.pi))
    for course in (0.0, 1.1, -2.4, 3.0):
        v = np.array([np.cos(course), np.sin(course), 0.0])
        s = sp.compute(FollowMode.FOLLOW, tgt((0, 0, 0), v), veh(), 0.0)
        to_t = np.array([0.0, 0.0]) - s.position[:2]
        assert s.yaw == pytest.approx(float(np.arctan2(to_t[1], to_t[0])), abs=1e-9)
        sp.reset()


def test_stationary_operator_does_not_spin_the_vehicle():
    """Below the course threshold the bearing must latch, not chase noise."""
    sp = StandoffPlanner(FollowParams(standoff_m=2.0))
    first = sp.compute(FollowMode.FOLLOW, tgt((0, 0, 0), (0.9, 0.0, 0)), veh(), 0.0)
    goals = []
    rng = np.random.default_rng(3)
    for k in range(30):
        jitter = rng.normal(0, 0.02, 3)
        jitter[2] = 0.0
        g = sp.compute(FollowMode.FOLLOW, tgt((0, 0, 0), jitter), veh(), 0.05 * (k + 1))
        goals.append(g.position[:2])
    goals = np.array(goals)
    assert np.max(np.linalg.norm(goals - first.position[:2], axis=1)) < 1e-6


def test_orbit_advances_the_phase():
    p = FollowParams(standoff_m=2.0, orbit_rate_rad_s=0.5)
    sp = StandoffPlanner(p)
    a = sp.compute(FollowMode.ORBIT, tgt((0, 0, 0)), veh(), 0.0)
    b = sp.compute(FollowMode.ORBIT, tgt((0, 0, 0)), veh(), 1.0)
    ang_a = np.arctan2(a.position[1], a.position[0])
    ang_b = np.arctan2(b.position[1], b.position[0])
    assert abs((ang_b - ang_a) - 0.5) < 1e-6


def test_hold_and_estop_freeze_at_current_position():
    sp = StandoffPlanner()
    v = veh((1.2, -3.4, 2.2))
    for m in (FollowMode.HOLD, FollowMode.IDLE, FollowMode.ESTOP):
        s = sp.compute(m, tgt((0, 0, 0), (2, 0, 0)), v, 0.0)
        assert np.allclose(s.position, v.position)
        assert s.max_speed == 0.0
        assert np.allclose(s.velocity_ff, 0.0)


def test_land_descends_monotonically():
    sp = StandoffPlanner()
    v = veh((0, 0, 2.0))
    zs = [sp.compute(FollowMode.LAND, None, v, 0.2 * k).position[2] for k in range(1, 12)]
    assert all(b <= a + 1e-12 for a, b in zip(zs, zs[1:]))
    assert zs[-1] < 2.0


def test_follow_without_a_target_degrades_to_hold():
    sp = StandoffPlanner()
    v = veh((1.0, 1.0, 2.0))
    s = sp.compute(FollowMode.FOLLOW, None, v, 0.0)
    assert s.mode is FollowMode.HOLD
    assert np.allclose(s.position, v.position)


def test_standoff_request_cannot_go_below_the_minimum():
    p = FollowParams(standoff_m=3.0, min_standoff_m=1.5)
    sp = StandoffPlanner(p)
    s = sp.compute(FollowMode.FOLLOW, tgt((0, 0, 0), (1, 0, 0)), veh(), 0.0, standoff_m=0.2)
    assert np.linalg.norm(s.position[:2]) == pytest.approx(1.5)
