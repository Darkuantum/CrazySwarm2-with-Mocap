"""follow_node -- the ROS 2 adapter around follow_core.Follower.

Subscribes vehicle odometry + the two sensor message types + operator
commands, runs Follower.step() on a timer, and publishes the fused estimate,
a status summary, a goal pose (for mapper_bridge / RViz) and RViz markers.

No follow logic lives here. Every decision about modes, gating, standoff
geometry or speed limiting is made inside follow_core; this file only
translates ROS message <-> follow_core dataclass and drives the clock.

-- Timer rate: 20 Hz default -----------------------------------------------
PX4's own collision-prevention reference implementation validated obstacle
data at only 10 Hz, so 20 Hz leaves comfortable headroom for this loop
without pretending we need vision-frame-rate control; see `rate_hz` param.

-- ENU/NED, explicitly -------------------------------------------------------
follow_core's world frame is fixed: right-handed, Z-up, ENU-like (see
follow_core/types.py). Odometry sources on this platform are NOT
consistently in that frame:
  * A ROS-native VIO/SLAM stack (and most `voxl-mpa-to-ros2` bridges, which
    are expected to already publish REP-105 ENU / REP-103-adjacent FLU
    odometry) needs no conversion.
  * A raw PX4-native feed (NED world / FRD body, un-converted) does.
The `vio_frame_convention` parameter selects which is true for your topic --
default is 'enu_flu' (no conversion) because that is what a ROS-facing bridge
is expected to publish; set it to 'ned_frd' if you point this node at a raw
PX4/NED feed. VERIFY THIS against the actual VOXL 2 image and topic you use;
getting it wrong silently mirrors the world frame and every follow behaviour
degrades in a confusing, hard-to-diagnose way. See README.md.
"""
from __future__ import annotations

import dataclasses

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from follow_ros2._bootstrap import ensure_follow_core

ensure_follow_core()

from follow_core import (  # noqa: E402
    BearingMeasurement,
    Command,
    FollowParams,
    Follower,
    RangeMeasurement,
    TrackHealth,
    VehicleState,
)

from follow_msgs.msg import (  # noqa: E402
    FollowCommand,
    FollowStatus,
    PersonBearing,
    TargetEstimate,
    UwbRange,
)

from follow_ros2.frames import (  # noqa: E402
    ned_frd_to_enu_flu,
    quat_to_matrix,
    rpy_to_matrix,
    yaw_from_matrix,
)
from follow_ros2.ros_enums import (  # noqa: E402
    HEALTH_TO_ROS,
    MODE_FROM_ROS,
    MODE_TO_ROS,
    SOURCE_FROM_ROS,
    stamp_to_seconds,
)

# Body(FLU) <- camera(REP-103 optical) rotation for a camera mounted level and
# facing straight forward (no tilt): camera +Z (forward) aligns with body +X,
# camera +X (right) aligns with body -Y, camera +Y (down) aligns with body -Z.
# The cam_roll/pitch/yaw_deg parameters below apply an *additional* tilt on
# top of this baseline -- they are "how far off level-forward is the camera
# actually mounted", not the whole extrinsic from scratch.
_BODY_CAM_LEVEL_FORWARD = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ]
)


class FollowNode(Node):
    def __init__(self) -> None:
        super().__init__("follow_node")

        # -- topics ------------------------------------------------------
        # NOTE: the exact topic voxl-mpa-to-ros2 publishes VIO odometry on
        # varies by image/version (forum reports both bare "/qvio" and
        # namespaced variants). CONFIRM with `ros2 topic list` /
        # `ros2 topic info` on the actual VOXL 2 before flying, and override
        # via config/follow_params.yaml if it differs.
        self.declare_parameter("vehicle_odom_topic", "/qvio")
        self.declare_parameter("range_topic", "/uwb/range")
        self.declare_parameter("bearing_topic", "/person/bearing")
        self.declare_parameter("command_topic", "/follow/command")
        self.declare_parameter("estimate_topic", "/follow/target_estimate")
        self.declare_parameter("status_topic", "/follow/status")
        self.declare_parameter("goal_pose_topic", "/follow/goal_pose")
        self.declare_parameter("markers_topic", "/follow/markers")
        self.declare_parameter("world_frame_id", "world")

        # -- timing / frame handling --------------------------------------
        self.declare_parameter("rate_hz", 20.0)
        # 'enu_flu' (no conversion, default) or 'ned_frd' -- see module
        # docstring.
        self.declare_parameter("vio_frame_convention", "enu_flu")

        # -- default measurement noise, used when a UwbRange/PersonBearing
        # message reports std <= 0 (i.e. "unspecified"). Mirrors
        # RangeMeasurement/BearingMeasurement dataclass defaults.
        self.declare_parameter("default_range_std_m", 0.10)
        self.declare_parameter("default_bearing_std_rad", 0.02)

        # -- static camera extrinsic (mounting rotation on top of the
        # level-forward baseline, plus a translation reserved for
        # visualisation / future use -- follow_core's EKF is bearing-only
        # from the vehicle origin and does not consume the translation).
        self.declare_parameter("cam_roll_deg", 0.0)
        self.declare_parameter("cam_pitch_deg", 0.0)
        self.declare_parameter("cam_yaw_deg", 0.0)
        self.declare_parameter("cam_offset_x_m", 0.0)
        self.declare_parameter("cam_offset_y_m", 0.0)
        self.declare_parameter("cam_offset_z_m", 0.0)

        # -- FollowParams, exposed field-by-field so config/follow_params.yaml
        # can tune the exact same tunables follow_core.FollowParams defines,
        # with follow_core's own defaults as the ROS parameter defaults (one
        # source of truth for the numbers).
        default_params = FollowParams()
        for f in dataclasses.fields(FollowParams):
            self.declare_parameter(f.name, getattr(default_params, f.name))
        overrides = {
            f.name: self.get_parameter(f.name).value
            for f in dataclasses.fields(FollowParams)
        }
        self.p = FollowParams(**overrides)
        self.follower = Follower(self.p)

        cam_r = np.radians(self.get_parameter("cam_roll_deg").value)
        cam_p = np.radians(self.get_parameter("cam_pitch_deg").value)
        cam_y = np.radians(self.get_parameter("cam_yaw_deg").value)
        self._r_body_cam = _BODY_CAM_LEVEL_FORWARD @ rpy_to_matrix(cam_r, cam_p, cam_y)
        self._cam_offset = np.array(
            [
                self.get_parameter("cam_offset_x_m").value,
                self.get_parameter("cam_offset_y_m").value,
                self.get_parameter("cam_offset_z_m").value,
            ]
        )

        self._vio_convention = self.get_parameter("vio_frame_convention").value
        if self._vio_convention not in ("enu_flu", "ned_frd"):
            raise ValueError(
                "vio_frame_convention must be 'enu_flu' or 'ned_frd', got "
                f"{self._vio_convention!r}"
            )
        self._world_frame_id = self.get_parameter("world_frame_id").value
        self._default_range_std_m = float(self.get_parameter("default_range_std_m").value)
        self._default_bearing_std_rad = float(
            self.get_parameter("default_bearing_std_rad").value
        )

        # -- subscriptions --------------------------------------------------
        self.create_subscription(
            Odometry, self.get_parameter("vehicle_odom_topic").value, self._on_odom, 10
        )
        self.create_subscription(
            UwbRange, self.get_parameter("range_topic").value, self._on_range, 10
        )
        self.create_subscription(
            PersonBearing, self.get_parameter("bearing_topic").value, self._on_bearing, 10
        )
        self.create_subscription(
            FollowCommand, self.get_parameter("command_topic").value, self._on_command, 20
        )

        # -- publishers -------------------------------------------------
        self._pub_estimate = self.create_publisher(
            TargetEstimate, self.get_parameter("estimate_topic").value, 10
        )
        self._pub_status = self.create_publisher(
            FollowStatus, self.get_parameter("status_topic").value, 10
        )
        self._pub_goal = self.create_publisher(
            PoseStamped, self.get_parameter("goal_pose_topic").value, 10
        )
        self._pub_markers = self.create_publisher(
            MarkerArray, self.get_parameter("markers_topic").value, 10
        )

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f"follow_node up: rate={rate_hz} Hz, vio_frame_convention="
            f"{self._vio_convention}, world_frame_id={self._world_frame_id}"
        )

    # -- clock -------------------------------------------------------------

    def _now(self) -> float:
        # Foxy-safe: rclpy.time.Time.nanoseconds exists on both distros; we
        # deliberately do not use Duration/Time arithmetic beyond this.
        return self.get_clock().now().nanoseconds * 1e-9

    # -- subscription callbacks ---------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        position = np.array([p.x, p.y, p.z], dtype=float)
        velocity = np.array([v.x, v.y, v.z], dtype=float)
        r_world_body = quat_to_matrix(q.x, q.y, q.z, q.w)

        if self._vio_convention == "ned_frd":
            position, velocity, r_world_body = ned_frd_to_enu_flu(
                position, velocity, r_world_body
            )
        # else 'enu_flu': already in the frame follow_core expects, no-op.

        yaw = yaw_from_matrix(r_world_body)
        r_world_cam = r_world_body @ self._r_body_cam

        stamp = stamp_to_seconds(msg.header.stamp)
        self.follower.on_vehicle(
            VehicleState(
                stamp=stamp,
                position=position,
                velocity=velocity,
                yaw=yaw,
                R_world_cam=r_world_cam,
            )
        )

    def _on_range(self, msg: UwbRange) -> None:
        std_m = float(msg.std_m) if msg.std_m > 0.0 else self._default_range_std_m
        self.follower.on_range(
            RangeMeasurement(
                stamp=stamp_to_seconds(msg.header.stamp),
                range_m=float(msg.range_m),
                tag_id=int(msg.tag_id),
                std_m=std_m,
            )
        )

    def _on_bearing(self, msg: PersonBearing) -> None:
        std_rad = float(msg.std_rad) if msg.std_rad > 0.0 else self._default_bearing_std_rad
        self.follower.on_bearing(
            BearingMeasurement(
                stamp=stamp_to_seconds(msg.header.stamp),
                azimuth_rad=float(msg.azimuth_rad),
                elevation_rad=float(msg.elevation_rad),
                std_rad=std_rad,
                score=float(msg.score),
            )
        )

    def _on_command(self, msg: FollowCommand) -> None:
        mode = MODE_FROM_ROS.get(msg.mode)
        source = SOURCE_FROM_ROS.get(msg.source)
        if mode is None or source is None:
            self.get_logger().warn(
                f"FollowCommand with unknown mode={msg.mode} source={msg.source} ignored"
            )
            return

        def _override(value: float):
            # NaN is the documented sentinel for "no override" -- see
            # follow_msgs/msg/FollowCommand.msg.
            return None if value != value else float(value)

        self.follower.on_command(
            Command(
                mode=mode,
                source=source,
                stamp=stamp_to_seconds(msg.header.stamp),
                standoff_m=_override(msg.standoff_m),
                height_m=_override(msg.height_m),
                follow_angle_rad=_override(msg.follow_angle_rad),
            )
        )

    # -- main loop -----------------------------------------------------------

    def _on_timer(self) -> None:
        now = self._now()
        out = self.follower.step(now)
        self._publish_estimate(out)
        self._publish_status(out)
        self._publish_goal(out)
        self._publish_markers(out)

    def _publish_estimate(self, out) -> None:
        msg = TargetEstimate()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame_id
        est = out.estimate
        if est is not None:
            msg.position = Point(
                x=float(est.position[0]), y=float(est.position[1]), z=float(est.position[2])
            )
            msg.velocity = Vector3(
                x=float(est.velocity[0]), y=float(est.velocity[1]), z=float(est.velocity[2])
            )
            msg.covariance = [float(x) for x in est.covariance.reshape(-1)]
        msg.health = HEALTH_TO_ROS[out.health]
        self._pub_estimate.publish(msg)

    def _publish_status(self, out) -> None:
        msg = FollowStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame_id
        msg.mode = MODE_TO_ROS[out.mode]
        msg.health = HEALTH_TO_ROS[out.health]
        if out.estimate is not None and self.follower.vehicle is not None:
            msg.range_to_target_m = float(
                np.linalg.norm(self.follower.vehicle.position - out.estimate.position)
            )
        else:
            msg.range_to_target_m = float("nan")
        msg.speed_cap_m_s = float(out.setpoint.max_speed)
        # Cumulative gate diagnostics -- these are public counters on
        # follow_core's EKF (follow_core/ekf.py), not reimplemented here.
        ekf = self.follower.ekf
        msg.range_accepted = int(ekf.range_accepted)
        msg.range_rejected = int(ekf.range_rejected)
        msg.bearing_accepted = int(ekf.bearing_accepted)
        msg.bearing_rejected = int(ekf.bearing_rejected)
        msg.reason = out.reason
        self._pub_status.publish(msg)

    def _publish_goal(self, out) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame_id
        sp = out.setpoint
        msg.pose.position = Point(
            x=float(sp.position[0]), y=float(sp.position[1]), z=float(sp.position[2])
        )
        half_yaw = sp.yaw * 0.5
        msg.pose.orientation.z = float(np.sin(half_yaw))
        msg.pose.orientation.w = float(np.cos(half_yaw))
        self._pub_goal.publish(msg)

    def _publish_markers(self, out) -> None:
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()

        est_marker = Marker()
        est_marker.header.frame_id = self._world_frame_id
        est_marker.header.stamp = stamp
        est_marker.ns = "follow"
        est_marker.id = 0
        est_marker.type = Marker.SPHERE
        est_marker.action = Marker.ADD
        est_marker.scale.x = est_marker.scale.y = est_marker.scale.z = 0.35
        if out.estimate is not None:
            p = out.estimate.position
            est_marker.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            est_marker.pose.orientation.w = 1.0
            if out.health is TrackHealth.TRACKING:
                est_marker.color.r, est_marker.color.g, est_marker.color.b = 0.0, 1.0, 0.0
            elif out.health is TrackHealth.DEGRADED:
                est_marker.color.r, est_marker.color.g, est_marker.color.b = 1.0, 0.8, 0.0
            else:
                est_marker.color.r, est_marker.color.g, est_marker.color.b = 1.0, 0.0, 0.0
            est_marker.color.a = 0.9
        else:
            est_marker.action = Marker.DELETE
        arr.markers.append(est_marker)

        goal_marker = Marker()
        goal_marker.header.frame_id = self._world_frame_id
        goal_marker.header.stamp = stamp
        goal_marker.ns = "follow"
        goal_marker.id = 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.25
        gp = out.setpoint.position
        goal_marker.pose.position = Point(x=float(gp[0]), y=float(gp[1]), z=float(gp[2]))
        goal_marker.pose.orientation.w = 1.0
        goal_marker.color.r, goal_marker.color.g, goal_marker.color.b = 0.2, 0.4, 1.0
        goal_marker.color.a = 0.9
        arr.markers.append(goal_marker)

        self._pub_markers.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
