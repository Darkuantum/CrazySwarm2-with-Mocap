"""Command arbitration -- one place where precedence is decided.

Two command channels (tablet and gesture band) plus internal supervisor
overrides converge here. Precedence exists in exactly one file so that "what
wins?" is answerable by reading, not by tracing message ordering.

Precedence, highest first:
  1. ESTOP_BUTTON   hardware button on the band. Latches. Nothing overrides it.
  2. TABLET         the operator with a screen and full situational awareness.
  3. GESTURE        convenient, but a classifier can be wrong.
  4. INTERNAL       supervisor-generated (target lost -> hold).

Two latches, because "stop" that un-stops itself is not a stop:
  * ESTOP latches until an explicit clear_estop() (long-press on the band).
  * STOP  latches HOLD until an explicit FOLLOW/ORBIT/LAND arrives.

Staleness matters as much as precedence: a command from a channel that has gone
quiet is not a command. Anything older than command_timeout_s is ignored, which
is what stops a dropped BLE link from leaving the vehicle in FOLLOW forever.
"""
from __future__ import annotations

from typing import Dict, Optional

from .types import Command, CommandSource, FollowMode, FollowParams

_PRIORITY: Dict[CommandSource, int] = {
    CommandSource.ESTOP_BUTTON: 100,
    CommandSource.TABLET: 60,
    CommandSource.GESTURE: 40,
    CommandSource.INTERNAL: 10,
}

# Commands that clear a latched STOP. ESTOP is deliberately not clearable here.
_RESUME_MODES = (FollowMode.FOLLOW, FollowMode.ORBIT, FollowMode.LAND)


class CommandArbiter:
    def __init__(self, params: Optional[FollowParams] = None) -> None:
        self.p = params or FollowParams()
        self._latest: Dict[CommandSource, Command] = {}
        self._estop_latched = False
        self._stop_latched = False
        # When the stop was asserted. Only a command that POST-DATES this may
        # clear it -- otherwise a stale FOLLOW still inside its freshness
        # window can resurrect following after an explicit stop.
        self._stop_stamp: float = float("-inf")

    # -- input ---------------------------------------------------------------

    def submit(self, cmd: Command) -> None:
        prev = self._latest.get(cmd.source)
        # Reject out-of-order duplicates from the same source.
        if prev is not None and cmd.stamp < prev.stamp:
            return
        self._latest[cmd.source] = cmd

        if cmd.source is CommandSource.ESTOP_BUTTON and cmd.mode is FollowMode.ESTOP:
            self._estop_latched = True
        elif cmd.mode is FollowMode.HOLD:
            self._stop_latched = True
            self._stop_stamp = cmd.stamp
        elif cmd.mode in _RESUME_MODES and cmd.stamp >= self._stop_stamp:
            self._stop_latched = False

    def clear_estop(self) -> None:
        """Explicit operator action (band long-press). Never automatic."""
        self._estop_latched = False
        self._latest.pop(CommandSource.ESTOP_BUTTON, None)

    @property
    def estop_latched(self) -> bool:
        return self._estop_latched

    @property
    def stop_latched(self) -> bool:
        return self._stop_latched

    # -- output --------------------------------------------------------------

    def resolve(self, now: float) -> Command:
        if self._estop_latched:
            return Command(
                mode=FollowMode.ESTOP, source=CommandSource.ESTOP_BUTTON, stamp=now
            )

        # The E-stop channel has its own, tighter liveness requirement: if the
        # band stops reporting we must not silently lose our only hardware stop.
        estop_cmd = self._latest.get(CommandSource.ESTOP_BUTTON)
        if estop_cmd is not None and (now - estop_cmd.stamp) > self.p.estop_timeout_s:
            self._latest.pop(CommandSource.ESTOP_BUTTON, None)

        fresh = [
            c
            for s, c in self._latest.items()
            if s is not CommandSource.ESTOP_BUTTON
            and (now - c.stamp) <= self.p.command_timeout_s
        ]
        if not fresh:
            return Command(mode=FollowMode.HOLD, source=CommandSource.INTERNAL, stamp=now)

        # A latched stop wins over any surviving command. submit() is the only
        # place the latch is cleared, and only by a command that post-dates the
        # stop -- so if we are still latched here, nothing has authorised a
        # resume and HOLD is the answer regardless of who is shouting loudest.
        if self._stop_latched:
            return Command(mode=FollowMode.HOLD, source=CommandSource.INTERNAL, stamp=now)

        return max(fresh, key=lambda c: (_PRIORITY[c.source], c.stamp))
