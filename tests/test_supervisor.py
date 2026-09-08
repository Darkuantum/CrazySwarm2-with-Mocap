from follow_core.supervisor import TrackSupervisor
from follow_core.types import FollowParams, TrackHealth


def test_both_sensors_fresh_is_tracking():
    s = TrackSupervisor()
    assert s.update(1.0, 0.95, 0.95, True) is TrackHealth.TRACKING


def test_one_sensor_stale_is_degraded():
    p = FollowParams(range_timeout_s=1.0, bearing_timeout_s=0.5)
    s = TrackSupervisor(p)
    # Vision gone (occluded), UWB still ranging -- still flyable.
    assert s.update(2.0, 1.8, 1.0, True) is TrackHealth.DEGRADED
    # UWB gone, vision alive.
    s.reset()
    assert s.update(2.0, 0.5, 1.9, True) is TrackHealth.DEGRADED


def test_both_stale_becomes_lost_only_after_the_timeout():
    p = FollowParams(range_timeout_s=1.0, bearing_timeout_s=0.5, lost_timeout_s=2.0)
    s = TrackSupervisor(p)
    assert s.update(10.0, 5.0, 5.0, True) is TrackHealth.DEGRADED   # grace period
    assert s.update(11.0, 5.0, 5.0, True) is TrackHealth.DEGRADED
    assert s.update(12.1, 5.0, 5.0, True) is TrackHealth.LOST


def test_recovery_resets_the_lost_timer():
    p = FollowParams(lost_timeout_s=2.0)
    s = TrackSupervisor(p)
    s.update(10.0, 5.0, 5.0, True)
    assert s.update(10.5, 10.4, 10.4, True) is TrackHealth.TRACKING
    assert s.update(11.0, 10.4, 10.4, True) is TrackHealth.DEGRADED
    # Timer restarted at 11.0, so 12.0 is still inside the grace period.
    assert s.update(12.0, 10.4, 10.4, True) is TrackHealth.DEGRADED


def test_uninitialized_filter_is_always_lost():
    s = TrackSupervisor()
    assert s.update(1.0, 1.0, 1.0, False) is TrackHealth.LOST
