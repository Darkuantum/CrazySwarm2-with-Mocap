import pytest

from follow_core.arbiter import CommandArbiter
from follow_core.types import Command, CommandSource, FollowMode, FollowParams


def cmd(mode, source, t, **kw):
    return Command(mode=mode, source=source, stamp=t, **kw)


def test_tablet_outranks_gesture():
    a = CommandArbiter()
    a.submit(cmd(FollowMode.ORBIT, CommandSource.GESTURE, 1.0))
    a.submit(cmd(FollowMode.FOLLOW, CommandSource.TABLET, 0.9))
    assert a.resolve(1.1).mode is FollowMode.FOLLOW    # older, but higher priority


def test_estop_latches_and_beats_everything():
    a = CommandArbiter()
    a.submit(cmd(FollowMode.ESTOP, CommandSource.ESTOP_BUTTON, 1.0))
    a.submit(cmd(FollowMode.FOLLOW, CommandSource.TABLET, 5.0))
    out = a.resolve(5.1)
    assert out.mode is FollowMode.ESTOP
    assert a.estop_latched


def test_estop_survives_its_own_staleness():
    """A latched E-stop must not expire just because the band went quiet --
    a dead band is the worst possible moment to resume following."""
    a = CommandArbiter()
    a.submit(cmd(FollowMode.ESTOP, CommandSource.ESTOP_BUTTON, 1.0))
    assert a.resolve(600.0).mode is FollowMode.ESTOP


def test_estop_clears_only_on_explicit_action():
    a = CommandArbiter()
    a.submit(cmd(FollowMode.ESTOP, CommandSource.ESTOP_BUTTON, 1.0))
    a.submit(cmd(FollowMode.FOLLOW, CommandSource.TABLET, 2.0))
    assert a.resolve(2.1).mode is FollowMode.ESTOP
    a.clear_estop()
    a.submit(cmd(FollowMode.FOLLOW, CommandSource.TABLET, 2.2))
    assert a.resolve(2.3).mode is FollowMode.FOLLOW


def test_stop_latches_until_an_explicit_resume():
    a = CommandArbiter()
    a.submit(cmd(FollowMode.FOLLOW, CommandSource.TABLET, 1.0))
    a.submit(cmd(FollowMode.HOLD, CommandSource.GESTURE, 2.0))
    assert a.resolve(2.1).mode is FollowMode.HOLD
    assert a.stop_latched
    # A stale re-assertion of FOLLOW must not sneak past the latch...
    assert a.resolve(2.5).mode is FollowMode.HOLD
    # ...but a fresh, explicit FOLLOW does.
    a.submit(cmd(FollowMode.FOLLOW, CommandSource.TABLET, 3.0))
    assert a.resolve(3.1).mode is FollowMode.FOLLOW
    assert not a.stop_latched


def test_stale_commands_are_ignored():
    a = CommandArbiter(FollowParams(command_timeout_s=2.0))
    a.submit(cmd(FollowMode.FOLLOW, CommandSource.TABLET, 1.0))
    assert a.resolve(2.5).mode is FollowMode.FOLLOW
    out = a.resolve(4.0)
    assert out.mode is FollowMode.HOLD
    assert out.source is CommandSource.INTERNAL


def test_out_of_order_commands_are_dropped():
    a = CommandArbiter()
    a.submit(cmd(FollowMode.FOLLOW, CommandSource.TABLET, 5.0))
    a.submit(cmd(FollowMode.ORBIT, CommandSource.TABLET, 4.0))   # arrives late
    assert a.resolve(5.1).mode is FollowMode.FOLLOW


def test_no_commands_at_all_holds():
    a = CommandArbiter()
    out = a.resolve(0.0)
    assert out.mode is FollowMode.HOLD
    assert out.source is CommandSource.INTERNAL


def test_command_parameters_are_carried_through():
    a = CommandArbiter()
    a.submit(cmd(FollowMode.FOLLOW, CommandSource.TABLET, 1.0, standoff_m=4.0, height_m=2.5))
    out = a.resolve(1.1)
    assert out.standoff_m == 4.0 and out.height_m == 2.5


def test_a_late_arriving_old_resume_cannot_clear_a_stop():
    """Regression: a FOLLOW that pre-dates the STOP must not resurrect following,
    even if it arrives afterwards and is still inside the freshness window."""
    a = CommandArbiter()
    a.submit(cmd(FollowMode.HOLD, CommandSource.GESTURE, 2.0))
    a.submit(cmd(FollowMode.FOLLOW, CommandSource.TABLET, 1.5))   # stale, arrives late
    assert a.resolve(2.1).mode is FollowMode.HOLD
    assert a.stop_latched
