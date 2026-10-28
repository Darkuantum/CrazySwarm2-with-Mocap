"""Platform-independent follower logic.

Public API -- this is the contract the ROS 2 nodes and the Crazyflie testbed
adapt to. Nothing below imports ROS, cflib or any hardware library.
"""
from .arbiter import CommandArbiter
from .ekf import TargetEKF
from .follower import Follower, FollowerOutput, HapticCue
from .governor import SpeedGovernor
from .standoff import StandoffPlanner
from .supervisor import TrackSupervisor
from .types import (
    BearingMeasurement,
    Command,
    CommandSource,
    FollowMode,
    FollowParams,
    RangeMeasurement,
    Setpoint,
    TargetEstimate,
    TrackHealth,
    VehicleState,
    wrap_pi,
)

__version__ = "0.1.0"

__all__ = [
    "BearingMeasurement", "Command", "CommandArbiter", "CommandSource",
    "FollowMode", "FollowParams", "Follower", "FollowerOutput", "HapticCue",
    "RangeMeasurement", "Setpoint", "SpeedGovernor", "StandoffPlanner",
    "TargetEKF", "TargetEstimate", "TrackHealth", "TrackSupervisor",
    "VehicleState", "wrap_pi", "__version__",
]
