"""ROS <-> follow_core enum mappings, in one place so nothing drifts.

follow_msgs/msg (rosidl-generated) has no notion of Python Enum, so every
message boundary needs an explicit int<->Enum table. Keeping exactly one copy
of each table means "what does mode=2 mean on the wire" has one answer.
"""
from follow_ros2._bootstrap import ensure_follow_core

ensure_follow_core()

from follow_core import CommandSource, FollowMode, TrackHealth  # noqa: E402

from follow_msgs.msg import FollowCommand, TargetEstimate  # noqa: E402

MODE_TO_ROS = {
    FollowMode.IDLE: FollowCommand.MODE_IDLE,
    FollowMode.FOLLOW: FollowCommand.MODE_FOLLOW,
    FollowMode.ORBIT: FollowCommand.MODE_ORBIT,
    FollowMode.HOLD: FollowCommand.MODE_HOLD,
    FollowMode.LAND: FollowCommand.MODE_LAND,
    FollowMode.ESTOP: FollowCommand.MODE_ESTOP,
}
MODE_FROM_ROS = {v: k for k, v in MODE_TO_ROS.items()}

SOURCE_TO_ROS = {
    CommandSource.TABLET: FollowCommand.SOURCE_TABLET,
    CommandSource.GESTURE: FollowCommand.SOURCE_GESTURE,
    CommandSource.ESTOP_BUTTON: FollowCommand.SOURCE_ESTOP_BUTTON,
    CommandSource.INTERNAL: FollowCommand.SOURCE_INTERNAL,
}
SOURCE_FROM_ROS = {v: k for k, v in SOURCE_TO_ROS.items()}

HEALTH_TO_ROS = {
    TrackHealth.TRACKING: TargetEstimate.HEALTH_TRACKING,
    TrackHealth.DEGRADED: TargetEstimate.HEALTH_DEGRADED,
    TrackHealth.LOST: TargetEstimate.HEALTH_LOST,
}


def stamp_to_seconds(stamp) -> float:
    """builtin_interfaces/Time -> float seconds.

    Uses the raw .sec/.nanosec fields rather than constructing an
    rclpy.time.Time -- identical fields exist on Foxy and Humble, whereas
    rclpy.time.Time's constructor/arithmetic has picked up convenience
    overloads post-Foxy that we do not want to depend on.
    """
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def seconds_to_stamp(seconds: float, stamp_msg) -> None:
    """Fill a builtin_interfaces/Time-like message in place from float seconds."""
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    stamp_msg.sec = sec
    stamp_msg.nanosec = nanosec
