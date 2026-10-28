"""mapper_bridge -- hands the follow goal to voxl-mapper and enforces the
"no plan, no motion" rule.

============================================================================
THE SINGLE MOST IMPORTANT ARCHITECTURAL CONSTRAINT IN THIS PROJECT
============================================================================
PX4's Collision Prevention feature (the thing that normally keeps a PX4
vehicle from flying into obstacles) is POSITION-MODE ONLY. It is NOT active
in Offboard mode -- and this whole stack flies in Offboard mode, because
that is how follow_core's setpoints reach the flight controller. That means:
if this project relied on PX4 Collision Prevention for obstacle avoidance,
it would have NO obstacle avoidance at all, silently, the entire time it is
following someone.

Obstacle avoidance on this vehicle therefore has to come from somewhere
else: voxl-mapper's ESDF-based trajectory planner. For that to actually be
in the loop, voxl-vision-hub's `offboard_mode` setting must be set to
"trajectory" (in its config, commonly under
/etc/modalai/voxl-vision-px4.conf on the vehicle) so that voxl-vision-hub
flies the setpoints voxl-mapper publishes to its `plan_msgs` pipe, rather
than flying a raw straight-line goal. THIS IS A ONE-TIME SYSTEM
CONFIGURATION STEP ON THE VEHICLE, NOT SOMETHING THIS ROS NODE CAN SET AT
RUNTIME -- see README.md's deployment checklist. If you ever find this
vehicle flying straight lines to the follow goal, check that setting FIRST.

Given that, this node's job is narrow but critical: it forwards the follow
goal to voxl-mapper, watches for evidence of a currently-valid plan, and if
that evidence goes stale or was never established, it publishes a HOLD
command -- it does NOT let the vehicle fall back to flying the goal
directly. A goal handed to a mapper that silently stopped planning (crashed,
lost the map, whatever) must not turn into "fly there anyway."

-- How the HOLD is enforced, given follow_core's arbiter -------------------
follow_core.arbiter.CommandArbiter latches HOLD on any HOLD-mode command
regardless of source priority (see arbiter.py: `_stop_latched`), and only
lifts it on a FOLLOW/ORBIT/LAND command whose stamp is >= the stamp of the
HOLD that latched it. A single HOLD from this node would therefore only
"win" until the next periodic FOLLOW heartbeat from the tablet arrives with
a newer timestamp -- even though the plan might still be invalid. So this
node RE-ASSERTS its HOLD command on every monitoring tick (`check_rate_hz`,
comfortably faster than follow_core's `command_timeout_s`) for as long as
the plan is invalid, keeping its stamp ahead of any resume attempt. It never
publishes a resume itself -- resuming FOLLOW/ORBIT/LAND, once a plan is
valid again, is left to the operator (tablet/gesture), consistent with
CommandArbiter's own precedence rules.

-- Exact voxl-mapper ROS 2 topic/message names: UNCONFIRMED -----------------
This was written without access to VOXL 2 hardware. `mapper_goal_topic` and
`mapper_plan_topic` below are configurable and default to plausible names,
but MUST be confirmed with `ros2 topic list` / `ros2 interface show ...` on
the actual vehicle image before flying -- see README.md.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

from follow_ros2._bootstrap import ensure_follow_core

ensure_follow_core()

from follow_msgs.msg import FollowCommand  # noqa: E402


class MapperBridge(Node):
    def __init__(self) -> None:
        super().__init__("mapper_bridge")

        self.declare_parameter("goal_pose_topic", "/follow/goal_pose")
        # UNCONFIRMED names -- see module docstring.
        self.declare_parameter("mapper_goal_topic", "/mapper/goal_pose")
        self.declare_parameter("mapper_plan_topic", "/mapper/path")
        self.declare_parameter("command_topic", "/follow/command")

        self.declare_parameter("plan_timeout_s", 1.0)
        self.declare_parameter("min_plan_poses", 2)
        self.declare_parameter("check_rate_hz", 10.0)

        self._plan_timeout_s = float(self.get_parameter("plan_timeout_s").value)
        self._min_plan_poses = int(self.get_parameter("min_plan_poses").value)

        self._have_goal = False
        self._last_plan_stamp = None
        self._last_plan_len = 0
        self._holding = False

        self.create_subscription(
            PoseStamped, self.get_parameter("goal_pose_topic").value, self._on_goal, 10
        )
        self.create_subscription(
            Path, self.get_parameter("mapper_plan_topic").value, self._on_plan, 10
        )
        self._pub_mapper_goal = self.create_publisher(
            PoseStamped, self.get_parameter("mapper_goal_topic").value, 10
        )
        self._pub_command = self.create_publisher(
            FollowCommand, self.get_parameter("command_topic").value, 10
        )

        rate_hz = float(self.get_parameter("check_rate_hz").value)
        self.create_timer(1.0 / rate_hz, self._on_check)

        self.get_logger().info(
            "mapper_bridge up. REMINDER: voxl-vision-hub offboard_mode must be "
            "'trajectory' on the vehicle for voxl-mapper's ESDF plan to actually "
            "be flown -- see this module's docstring."
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_goal(self, msg: PoseStamped) -> None:
        self._have_goal = True
        # Forward as-is. If voxl-mapper expects a different frame_id, set it
        # here once confirmed against the real interface.
        self._pub_mapper_goal.publish(msg)

    def _on_plan(self, msg: Path) -> None:
        self._last_plan_stamp = self._now()
        self._last_plan_len = len(msg.poses)

    def _plan_is_valid(self, now: float) -> bool:
        if self._last_plan_stamp is None:
            return False
        if (now - self._last_plan_stamp) > self._plan_timeout_s:
            return False
        return self._last_plan_len >= self._min_plan_poses

    def _on_check(self) -> None:
        if not self._have_goal:
            return
        now = self._now()
        valid = self._plan_is_valid(now)
        if not valid:
            # Re-asserted every tick -- see module docstring on why a single
            # HOLD is not enough against the arbiter's stamp-based latch.
            cmd = FollowCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.mode = FollowCommand.MODE_HOLD
            cmd.source = FollowCommand.SOURCE_INTERNAL
            cmd.standoff_m = float("nan")
            cmd.height_m = float("nan")
            cmd.follow_angle_rad = float("nan")
            self._pub_command.publish(cmd)
            if not self._holding:
                self.get_logger().warn(
                    "mapper_bridge: no valid voxl-mapper plan -- holding position "
                    "rather than flying an unchecked straight line"
                )
            self._holding = True
        elif self._holding:
            self.get_logger().info(
                "mapper_bridge: valid plan observed again -- HOLD no longer "
                "being re-asserted (resume is an operator decision, not this node's)"
            )
            self._holding = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapperBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
