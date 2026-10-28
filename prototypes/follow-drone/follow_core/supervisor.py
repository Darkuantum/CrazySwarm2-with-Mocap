"""Track health supervision -- FR-8, degrade safely when the operator is lost.

The important behaviour here is the one that is easy to get wrong: when the
target is lost, the vehicle HOLDS. It does not translate to search. A drone
that wanders looking for someone is a drone flying on a stale belief about
where the obstacles are, near a person it can no longer see.
"""
from __future__ import annotations

from typing import Optional

from .types import FollowParams, TrackHealth


class TrackSupervisor:
    def __init__(self, params: Optional[FollowParams] = None) -> None:
        self.p = params or FollowParams()
        self._lost_since: Optional[float] = None
        self.health = TrackHealth.LOST

    def reset(self) -> None:
        self._lost_since = None
        self.health = TrackHealth.LOST

    def update(
        self,
        now: float,
        last_range_stamp: Optional[float],
        last_bearing_stamp: Optional[float],
        initialized: bool,
    ) -> TrackHealth:
        p = self.p
        range_ok = (
            last_range_stamp is not None and (now - last_range_stamp) <= p.range_timeout_s
        )
        bearing_ok = (
            last_bearing_stamp is not None
            and (now - last_bearing_stamp) <= p.bearing_timeout_s
        )

        if not initialized:
            self.health = TrackHealth.LOST
            self._lost_since = self._lost_since if self._lost_since is not None else now
            return self.health

        if range_ok and bearing_ok:
            self._lost_since = None
            self.health = TrackHealth.TRACKING
        elif range_ok or bearing_ok:
            # One sensor carries the estimate. UWB alone still gives range and
            # identity; vision alone still gives bearing. Either is flyable at
            # reduced confidence -- but not indefinitely.
            self._lost_since = None
            self.health = TrackHealth.DEGRADED
        else:
            if self._lost_since is None:
                self._lost_since = now
            if (now - self._lost_since) >= p.lost_timeout_s:
                self.health = TrackHealth.LOST
            else:
                self.health = TrackHealth.DEGRADED
        return self.health
