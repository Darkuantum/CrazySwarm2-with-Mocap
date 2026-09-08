"""Core data types, frames and units for the follower stack.

Frame conventions (fixed for the whole project -- convert at the boundaries):

  world   Local, right-handed, Z up (ENU-like). This is the frame qVIO /
          OptiTrack publish the vehicle pose in. PX4's NED conversion happens
          in the ROS adapter, never in here.
  body    Vehicle frame, X forward, Y left, Z up.
  camera  Optical frame per REP-103: X right, Y down, Z forward.

Units are SI throughout: metres, seconds, radians. Angles are wrapped to
(-pi, pi]. Timestamps are float seconds on a single monotonic clock.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


def wrap_pi(a: float | np.ndarray) -> float | np.ndarray:
    """Wrap angle(s) to (-pi, pi]."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


class FollowMode(Enum):
    """Vehicle behaviour mode. Ordering is not significance -- see Arbiter."""

    IDLE = "idle"
    FOLLOW = "follow"
    ORBIT = "orbit"
    HOLD = "hold"
    LAND = "land"
    ESTOP = "estop"


class CommandSource(Enum):
    """Where a command came from. Precedence is defined in arbiter.py."""

    ESTOP_BUTTON = "estop_button"   # hardware button on the band
    TABLET = "tablet"
    GESTURE = "gesture"
    INTERNAL = "internal"           # supervisor-generated (e.g. target lost)


class TrackHealth(Enum):
    """Quality of the target estimate. Drives the supervisor state machine."""

    TRACKING = "tracking"     # both sensors contributing
    DEGRADED = "degraded"     # one sensor stale, estimate still usable
    LOST = "lost"             # no usable estimate


@dataclass(frozen=True)
class Command:
    """A single operator command with provenance and a timestamp."""

    mode: FollowMode
    source: CommandSource
    stamp: float
    # Optional parameter overrides carried with the command.
    standoff_m: Optional[float] = None
    height_m: Optional[float] = None
    follow_angle_rad: Optional[float] = None


@dataclass
class VehicleState:
    """Vehicle pose and velocity in the world frame."""

    stamp: float
    position: np.ndarray                      # (3,) world
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    yaw: float = 0.0                          # world heading, rad
    # Rotation world <- camera. Identity means camera optical axis is world +Z,
    # which is never true in practice; the adapter supplies the real value.
    R_world_cam: np.ndarray = field(default_factory=lambda: np.eye(3))

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float).reshape(3)
        self.velocity = np.asarray(self.velocity, dtype=float).reshape(3)
        self.R_world_cam = np.asarray(self.R_world_cam, dtype=float).reshape(3, 3)


@dataclass
class RangeMeasurement:
    """UWB two-way-ranging return from the operator's tag.

    The tag is a specific radio, so `tag_id` is an unambiguous identity claim --
    this is what anchors the follow lock against a visually similar person.
    """

    stamp: float
    range_m: float
    tag_id: int
    std_m: float = 0.10


@dataclass
class BearingMeasurement:
    """Direction to a detected person, in the camera optical frame.

    Produced from a detector bounding-box centroid plus camera intrinsics.
    Carries no identity of its own -- identity comes from gating against the
    UWB-anchored estimate.
    """

    stamp: float
    azimuth_rad: float     # atan2(x, z) in camera frame; +ve to the right
    elevation_rad: float   # atan2(-y, hypot(x, z)); +ve upward
    std_rad: float = 0.02
    score: float = 1.0     # detector confidence, 0..1


@dataclass
class TargetEstimate:
    """Fused estimate of the operator, in the world frame."""

    stamp: float
    position: np.ndarray
    velocity: np.ndarray
    covariance: np.ndarray          # (6,6)
    health: TrackHealth = TrackHealth.LOST

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float).reshape(3)
        self.velocity = np.asarray(self.velocity, dtype=float).reshape(3)
        self.covariance = np.asarray(self.covariance, dtype=float).reshape(6, 6)

    @property
    def position_std(self) -> np.ndarray:
        return np.sqrt(np.clip(np.diag(self.covariance)[:3], 0.0, None))

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.velocity))

    @property
    def course_rad(self) -> float:
        """Direction of travel in the world XY plane."""
        return float(np.arctan2(self.velocity[1], self.velocity[0]))


@dataclass
class Setpoint:
    """What the follower asks the vehicle to do.

    `position` is a goal in the world frame. On the Starling this is handed to
    voxl-mapper, which owns the obstacle-free path to it -- the follower never
    commands a straight line it has not checked.
    """

    stamp: float
    position: np.ndarray
    velocity_ff: np.ndarray            # feedforward, m/s
    yaw: float                         # world heading, rad
    max_speed: float                   # governor output, m/s
    mode: FollowMode

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float).reshape(3)
        self.velocity_ff = np.asarray(self.velocity_ff, dtype=float).reshape(3)


@dataclass
class FollowParams:
    """Tunables. Defaults are the values the PRD's acceptance criteria assume."""

    # -- standoff geometry (mirrors PX4 FLW_TGT_* semantics) ------------------
    standoff_m: float = 2.5            # horizontal separation from the operator
    height_m: float = 2.0              # altitude above the operator's feet
    follow_angle_rad: float = np.pi    # pi = directly behind the direction of travel
    orbit_rate_rad_s: float = 0.35     # ORBIT mode angular rate

    # -- safety governor (NFR-4) ---------------------------------------------
    min_standoff_m: float = 1.5        # never commanded closer than this
    speed_cap_far_m_s: float = 3.0     # full speed beyond governor_far_m
    speed_cap_near_m_s: float = 0.6    # speed allowed at min_standoff_m
    governor_far_m: float = 4.0
    max_accel_m_s2: float = 2.0

    # -- estimator ------------------------------------------------------------
    process_accel_std: float = 1.2     # person acceleration noise, m/s^2
    vertical_accel_scale: float = 0.1  # people mostly stay on the floor
    range_gate_chi2: float = 9.0       # 1-DOF chi-square gate (~3 sigma)
    bearing_gate_chi2: float = 11.8    # 2-DOF chi-square gate (~99.7%)
    range_huber_k: float = 2.0         # robustify against documented UWB outliers
    # Divergence recovery: after this many consecutive rejections from either
    # sensor, inflate the covariance rather than gate the filter to death.
    divergence_reject_limit: int = 6
    divergence_inflation: float = 4.0

    # -- staleness / supervision (FR-8) --------------------------------------
    range_timeout_s: float = 1.0
    bearing_timeout_s: float = 0.5
    lost_timeout_s: float = 2.0
    # A range-only update is consistent with any point on the range sphere, so
    # with vision gone the tangential error is unobservable and the covariance
    # grows. Past this horizontal 1-sigma the estimate is not good enough to
    # fly to, whatever the freshness stamps say.
    max_position_std_m: float = 1.2
    command_timeout_s: float = 2.0
    estop_timeout_s: float = 1.5

    def validate(self) -> None:
        if self.min_standoff_m <= 0.0:
            raise ValueError("min_standoff_m must be positive")
        if self.standoff_m < self.min_standoff_m:
            raise ValueError("standoff_m must be >= min_standoff_m")
        if self.governor_far_m <= self.min_standoff_m:
            raise ValueError("governor_far_m must exceed min_standoff_m")
        if self.speed_cap_near_m_s > self.speed_cap_far_m_s:
            raise ValueError("near speed cap must not exceed far speed cap")
        if self.lost_timeout_s <= 0.0:
            raise ValueError("lost_timeout_s must be positive")
