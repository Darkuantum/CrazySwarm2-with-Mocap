"""End-to-end behaviour of the composed follower, including the PRD's own
acceptance criteria run through the closed-loop simulation."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from follow_core import (  # noqa: E402
    Command, CommandSource, FollowMode, FollowParams, Follower, HapticCue,
    TrackHealth, VehicleState,
)
from tools.sim import run_sim  # noqa: E402


def veh(t=0.0, pos=(-2.5, 0.0, 2.0)):
    return VehicleState(stamp=t, position=np.array(pos, float))


def test_no_vehicle_state_is_idle_not_a_crash():
    f = Follower()
    out = f.step(0.0)
    assert out.mode is FollowMode.IDLE
    assert out.reason == "no vehicle state"


def test_follow_downgrades_to_hold_when_target_is_lost():
    """FR-8: no target means hold. It must not translate to search."""
    p = FollowParams(lost_timeout_s=1.0)
    f = Follower(p)
    f.on_vehicle(veh(0.0))
    f.on_mocap_target([0, 0, 0], 0.0, [1, 0, 0])
    f.on_command(Command(mode=FollowMode.FOLLOW, source=CommandSource.TABLET, stamp=0.0))
    assert f.step(0.1).mode is FollowMode.FOLLOW

    # Sensors go quiet. Vehicle state keeps arriving, as it would in flight.
    for t in np.arange(0.2, 6.0, 0.1):
        f.on_vehicle(veh(float(t)))
        f.on_command(Command(mode=FollowMode.FOLLOW, source=CommandSource.TABLET, stamp=float(t)))
        out = f.step(float(t))
    assert out.health is TrackHealth.LOST
    assert out.mode is FollowMode.HOLD
    assert out.reason == "target lost"
    assert np.allclose(out.setpoint.position, out.setpoint.position)
    assert out.setpoint.max_speed == 0.0


def test_estop_is_honoured_with_no_target_at_all():
    f = Follower()
    f.on_vehicle(veh(0.0))
    f.on_command(Command(mode=FollowMode.ESTOP, source=CommandSource.ESTOP_BUTTON, stamp=0.0))
    out = f.step(0.1)
    assert out.mode is FollowMode.ESTOP
    assert out.setpoint.max_speed == 0.0


def test_land_still_works_with_no_target():
    """A lost target must not prevent the operator landing the aircraft."""
    f = Follower()
    f.on_vehicle(veh(0.0))
    z = []
    for t in np.arange(0.1, 3.0, 0.1):
        # Every command source must heartbeat -- the arbiter expires a stale
        # command by design. See README, "Commands must be re-asserted".
        f.on_vehicle(veh(float(t)))
        f.on_command(Command(mode=FollowMode.LAND, source=CommandSource.TABLET, stamp=float(t)))
        z.append(f.step(float(t)).setpoint.position[2])
    assert z[-1] < z[0]


def test_haptic_fires_once_on_target_loss_not_every_tick():
    p = FollowParams(lost_timeout_s=0.5)
    f = Follower(p)
    f.on_vehicle(veh(0.0))
    f.on_mocap_target([0, 0, 0], 0.0)
    f.on_command(Command(mode=FollowMode.FOLLOW, source=CommandSource.TABLET, stamp=0.0))
    f.step(0.1)
    cues = []
    for t in np.arange(0.2, 5.0, 0.1):
        f.on_vehicle(veh(float(t)))
        f.on_command(Command(mode=FollowMode.FOLLOW, source=CommandSource.TABLET, stamp=float(t)))
        cues.append(f.step(float(t)).haptic)
    assert cues.count(HapticCue.TARGET_LOST) == 1


def test_governor_caps_speed_as_the_operator_is_approached():
    p = FollowParams(standoff_m=2.0)
    f = Follower(p)
    caps = []
    for i, d in enumerate([8.0, 4.0, 3.0, 2.0, 1.6]):
        t = i * 1.0
        f.on_vehicle(veh(t, (-d, 0.0, 2.0)))
        f.on_mocap_target([0, 0, 0], t)
        f.on_command(Command(mode=FollowMode.FOLLOW, source=CommandSource.TABLET, stamp=t))
        caps.append(f.step(t).setpoint.max_speed)
    assert caps == sorted(caps, reverse=True), caps


def test_goal_never_sits_inside_the_minimum_standoff():
    p = FollowParams(standoff_m=2.5, min_standoff_m=1.5)
    f = Follower(p)
    rng = np.random.default_rng(11)
    for i in range(200):
        t = i * 0.05
        f.on_vehicle(veh(t, rng.uniform(-4, 4, 3) * [1, 1, 0] + [0, 0, 2.0]))
        f.on_mocap_target(rng.uniform(-1, 1, 3) * [1, 1, 0], t)
        f.on_command(Command(mode=FollowMode.FOLLOW, source=CommandSource.TABLET, stamp=t))
        out = f.step(t)
        if out.estimate is not None and out.mode is FollowMode.FOLLOW:
            d = np.linalg.norm(out.setpoint.position[:2] - out.estimate.position[:2])
            assert d >= p.min_standoff_m - 1e-6, f"goal {d:.3f} m from operator"


# -- PRD acceptance, run through the closed-loop simulation ------------------

@pytest.mark.parametrize("scenario", ["straight", "eight", "fast", "occlude"])
def test_fr1_acceptance_in_simulation(scenario):
    """FR-1: mean standoff error <= 0.35 m, p95 <= 0.75 m."""
    r = run_sim(scenario=scenario, duration=20.0, seed=42)
    assert r.mean_err <= 0.35, f"{scenario}: mean {r.mean_err:.3f} m"
    assert r.p95_err <= 0.75, f"{scenario}: p95 {r.p95_err:.3f} m"


@pytest.mark.parametrize("seed", [1, 7, 13, 99])
def test_fr1_is_not_a_lucky_seed(seed):
    r = run_sim(scenario="straight", duration=20.0, seed=seed)
    assert r.mean_err <= 0.35, f"seed {seed}: mean {r.mean_err:.3f} m"
    assert r.p95_err <= 0.75, f"seed {seed}: p95 {r.p95_err:.3f} m"


def test_uwb_carries_the_track_through_occlusion():
    """The design claim: vision drops out, UWB keeps the lock. If this fails,
    the hybrid architecture is not earning its extra hardware."""
    r = run_sim(scenario="occlude", duration=20.0, seed=42)
    assert r.vis_fraction < 0.95, "occlusion scenario did not actually occlude"
    assert r.lost_fraction < 0.05, f"lost {r.lost_fraction * 100:.1f}% of the time"
    assert r.mean_err <= 0.35


def test_vehicle_never_gets_closer_than_the_safety_floor():
    for scenario in ("straight", "eight", "fast", "occlude"):
        r = run_sim(scenario=scenario, duration=20.0, seed=42)
        assert r.min_separation >= 1.35, f"{scenario}: {r.min_separation:.3f} m"
