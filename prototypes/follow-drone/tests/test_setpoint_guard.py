"""The Crazyflie node's last line of defence.

SetpointGuard is what actually keeps the aircraft inside the room and away from
the operator on a rig with no onboard collision avoidance. follow_core's
governor is the first clamp; this is the independent second one, re-derived from
the node's own reading of the operator position. It has to hold even when the
layer above it is wrong.
"""
import importlib.util
import os
import sys

import numpy as np
import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SAFETY = os.path.join(
    REPO, "crazyflie", "crazyflie_follow", "crazyflie_follow", "safety.py"
)


def _load_safety():
    if not os.path.exists(SAFETY):
        pytest.skip("crazyflie safety.py not present")
    sys.path.insert(0, REPO)
    spec = importlib.util.spec_from_file_location("_cf_safety", SAFETY)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


class Cfg:
    """Duck-typed stand-in for the node's NodeConfig."""

    def __init__(self, follow, half_width=4.0, center=(0.0, 0.0)):
        self.follow = follow
        self.room_center = center
        self.boundary_half_width_m = half_width
        self.altitude_ceiling_m = 2.5
        self.altitude_floor_m = 0.3
        self.hard_speed_cap_m_s = 0.6
        self.hard_accel_cap_m_s2 = 1.0
        self.max_yaw_rate_rad_s = 1.0


@pytest.fixture
def mod():
    return _load_safety()


@pytest.fixture
def cfg():
    from follow_core import FollowParams

    return Cfg(FollowParams(min_standoff_m=1.5))


def test_altitude_and_boundary_are_clamped(mod, cfg):
    g = mod.SetpointGuard(cfg)
    g.reset_to([0.0, 0.0, 1.0], 0.0, 0.0)
    pos, _, reasons = g.apply([99.0, -99.0, 99.0], 0.0, None, 10.0)
    assert pos[2] <= cfg.altitude_ceiling_m + 1e-9
    assert abs(pos[0]) <= cfg.boundary_half_width_m + 1e-9
    assert abs(pos[1]) <= cfg.boundary_half_width_m + 1e-9


def test_speed_cap_limits_the_step(mod, cfg):
    g = mod.SetpointGuard(cfg)
    g.reset_to([0.0, 0.0, 1.0], 0.0, 0.0)
    pos, _, reasons = g.apply([3.0, 0.0, 1.0], 0.0, None, 0.05)
    step = float(np.linalg.norm(pos - np.array([0.0, 0.0, 1.0])))
    assert step <= cfg.hard_speed_cap_m_s * 0.05 + 1e-6
    assert any("speed cap" in r for r in reasons)


def test_min_standoff_survives_the_slew(mod, cfg):
    """Regression: the pre-slew clamp is undone by the speed/accel caps.

    The guard clamps the requested goal out to min_standoff, then the speed and
    accel caps interpolate back toward the PREVIOUS setpoint. If that previous
    setpoint was already inside the standoff sphere -- exactly what happens when
    the operator walks toward a drone holding station -- the result is inside
    the sphere again and the clamp achieved nothing.
    """
    g = mod.SetpointGuard(cfg)
    # Previous setpoint is 0.4 m from where the operator now is.
    g.reset_to([0.4, 0.0, 1.0], 0.0, 0.0)
    operator = (0.0, 0.0)
    # Ask for something far away; the speed cap will refuse to let us get there.
    pos, _, reasons = g.apply([5.0, 0.0, 1.0], 0.0, operator, 0.05)
    d = float(np.linalg.norm(pos[:2] - np.asarray(operator)))
    assert d >= cfg.follow.min_standoff_m - 1e-6, (
        f"commanded {d:.3f} m from the operator, floor is "
        f"{cfg.follow.min_standoff_m} m; reasons={reasons}"
    )


def test_standoff_holds_over_an_operator_walking_into_the_drone(mod, cfg):
    """The realistic version: operator advances 1.2 m/s straight at a hovering
    drone for three seconds. The guard must retreat, every single tick."""
    g = mod.SetpointGuard(cfg)
    g.reset_to([2.0, 0.0, 1.0], 0.0, 0.0)
    dt, t = 0.05, 0.0
    worst = 1e9
    for k in range(60):
        t += dt
        op_x = -1.0 + 1.2 * t          # walks from -1.0 toward +x
        operator = (op_x, 0.0)
        # The layer above keeps naively asking to hold station.
        pos, _, _ = g.apply([2.0, 0.0, 1.0], 0.0, operator, t)
        worst = min(worst, float(np.linalg.norm(pos[:2] - np.asarray(operator))))
    assert worst >= cfg.follow.min_standoff_m - 1e-6, (
        f"closest commanded approach was {worst:.3f} m"
    )


def test_yaw_rate_is_capped(mod, cfg):
    g = mod.SetpointGuard(cfg)
    g.reset_to([0.0, 0.0, 1.0], 0.0, 0.0)
    _, yaw, reasons = g.apply([0.0, 0.0, 1.0], 3.0, None, 0.05)
    assert abs(yaw) <= cfg.max_yaw_rate_rad_s * 0.05 + 1e-9
    assert any("yaw rate" in r for r in reasons)


def test_offline_preflight_passes_for_the_shipped_defaults(mod):
    """The offline proofs must actually pass for the config we intend to fly."""
    from follow_core import FollowParams

    p = FollowParams()
    worst = mod.check_min_standoff_enforced(p, n_samples=2000)
    assert worst >= p.min_standoff_m - 1e-6, f"governor let a goal to {worst:.3f} m"

    report = mod.preflight(
        p,
        boundary_half_width_m=4.0,
        hard_speed_cap_m_s=0.6,
        hard_accel_cap_m_s2=1.0,
    )
    assert isinstance(report, dict) and report
