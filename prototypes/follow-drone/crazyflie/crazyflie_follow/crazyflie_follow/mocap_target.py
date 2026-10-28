"""Split /poses into the drone body and the operator body, with velocity.

`motion_capture_tracking` publishes every rigid body Motive streams on one
topic, `/poses` (`motion_capture_tracking_interfaces/msg/NamedPoseArray`,
nominally 50 Hz per pose_bridge.py / CLAUDE.md) -- there is no separate
"target" sensor in this phase. That single fact is *why* the Crazyflie phase
works at all: the operator wears a second mocap-tracked rigid body (named
"operator" by default -- see config/follow.yaml and README.md for how to
build it in Motive), and this module's only job is to pick that one entry and
the drone's own entry out of the array by name, and turn each into the two
things follow_core actually needs: a position and a velocity.

Position needs no filtering -- OptiTrack in this volume is sub-millimetre.
Velocity is a different story: it is not measured at all, only differenced,
and a raw two-frame finite difference on an already-clean signal still turns
any residual marker jitter or frame-time jitter into a velocity spike, which
governor.py's speed cap and standoff.py's course-latch both consume directly.
`BodyTracker` runs that difference through a one-pole low-pass instead of
using it raw.

Measured frame-rate caveat (crazyflie_shows/HANDOVER.md Sec 4a, item 13,
2026-09-07): this rig's `/poses` has been observed at ~27 Hz with gaps up to
0.215 s against a ~42 Hz socket rate, because Motive's datagrams exceed the
1500 B MTU and Wi-Fi multicast is unacknowledged -- one dropped fragment
drops the whole frame. `BodyTracker`'s low-pass time constant and
`MAX_TRUSTED_DT` below are sized against *that* number, not the nominal 50 Hz,
so a follow session degrades gracefully on this rig as it currently stands
rather than assuming a diagnosis that has not been re-run since.
"""
from __future__ import annotations

from typing import Dict, Optional

import numpy as np
from motion_capture_tracking_interfaces.msg import NamedPoseArray

#: Low-pass corner for the finite-difference velocity. Low enough to reject
#: marker jitter, high enough not to lag a person walking at ~1.3 m/s (a
#: brisk walk) by more than a few centimetres -- see governor.py, which
#: slew-limits its own output on top of this, so a slightly laggy velocity
#: here is not the last line of defense.
DEFAULT_LOWPASS_HZ = 4.0

#: A gap this long (see module docstring) is no longer "this frame was a bit
#: late" -- it is "the body dropped out". Differencing across it would invent
#: a velocity from two positions that likely are not causally close in time,
#: so the tracker instead resets its velocity to zero and waits for two
#: consecutive good frames before trusting a differenced velocity again.
MAX_TRUSTED_DT = 0.30


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """World-frame yaw (rotation about +Z) from a geometry_msgs orientation.

    Standard ZYX-intrinsic yaw extraction; exact regardless of roll/pitch.
    """
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


class BodyTracker:
    """Tracks one named rigid body across successive /poses frames.

    Every consumer is expected to call :meth:`stale` rather than assume the
    body is present -- a body missing from the array for a few frames (a
    marker occlusion, a person turning side-on) must never raise, it must
    just read as stale for exactly as long as it actually is.
    """

    def __init__(self, name: str, lowpass_hz: float = DEFAULT_LOWPASS_HZ,
                max_trusted_dt: float = MAX_TRUSTED_DT) -> None:
        self.name = name
        self._tau = 1.0 / (2.0 * np.pi * lowpass_hz)
        self._max_trusted_dt = float(max_trusted_dt)
        self.stamp: Optional[float] = None
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.yaw = 0.0
        self.seen_count = 0
        self.missing_count = 0

    @property
    def initialized(self) -> bool:
        return self.stamp is not None

    def age(self, now: float) -> float:
        if self.stamp is None:
            return float('inf')
        return now - self.stamp

    def is_stale(self, now: float, timeout_s: float) -> bool:
        return self.age(now) > timeout_s

    def update(self, stamp: float, position, yaw: Optional[float] = None) -> None:
        """Record a new sighting. `stamp` is the pose's own capture time."""
        position = np.asarray(position, dtype=float).reshape(3)
        if self.stamp is not None:
            dt = stamp - self.stamp
            if 1e-4 < dt <= self._max_trusted_dt:
                raw_vel = (position - self.position) / dt
                # Exact discretisation of a continuous one-pole low-pass:
                # alpha = dt / (tau + dt), so the corner frequency does not
                # depend on the (jittery) frame rate.
                alpha = dt / (self._tau + dt)
                self.velocity = self.velocity + alpha * (raw_vel - self.velocity)
            elif dt > self._max_trusted_dt:
                # Gap too long to trust a difference across it -- see
                # MAX_TRUSTED_DT above.
                self.velocity = np.zeros(3)
            # dt <= 0 (out-of-order/duplicate frame): keep the existing
            # velocity estimate rather than dividing by a tiny or negative dt.
        self.position = position
        if yaw is not None:
            self.yaw = float(yaw)
        self.stamp = float(stamp)
        self.seen_count += 1
        self.missing_count = 0

    def mark_absent(self) -> None:
        """Call once per /poses frame in which this body did not appear."""
        self.missing_count += 1


class MocapTargetTracker:
    """Owns the /poses subscription and exposes the drone + operator bodies.

    Attach directly to whatever rclpy Node is already running the control
    loop (follow_node embeds this on `swarm.allcfs`) rather than running as
    its own node with its own publishers: the control loop needs the lowest
    possible latency between a mocap frame arriving and a setpoint going out,
    and every extra publish/subscribe hop costs a DDS round trip for no
    benefit here.
    """

    def __init__(self, node, drone_name: str, operator_name: str,
                topic: str = '/poses', lowpass_hz: float = DEFAULT_LOWPASS_HZ,
                qos: int = 10) -> None:
        self.node = node
        self.drone = BodyTracker(drone_name, lowpass_hz=lowpass_hz)
        self.operator = BodyTracker(operator_name, lowpass_hz=lowpass_hz)
        self._by_name: Dict[str, BodyTracker] = {
            drone_name: self.drone,
            operator_name: self.operator,
        }
        self._sub = node.create_subscription(
            NamedPoseArray, topic, self._on_poses, qos)

    def _on_poses(self, msg: NamedPoseArray) -> None:
        from rclpy.time import Time
        now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        seen = set()
        for entry in msg.poses:
            tracker = self._by_name.get(entry.name)
            if tracker is None:
                continue
            seen.add(entry.name)
            p = entry.pose.position
            q = entry.pose.orientation
            tracker.update(now, (p.x, p.y, p.z),
                            yaw=yaw_from_quaternion(q.x, q.y, q.z, q.w))
        for name, tracker in self._by_name.items():
            if name not in seen:
                tracker.mark_absent()


def main(args=None):
    """Standalone bench diagnostic: report drone/operator tracking status.

    Not part of the flight path -- follow_node embeds MocapTargetTracker
    directly for latency reasons (see the class docstring). This entry point
    exists so the two rigid bodies can be checked (are both present? what
    rate? sane position?) before follow_node, the crazyflie_server, or the
    radio are ever involved. Mirrors crazyflie_shows/scripts/mocap_verify.sh
    in spirit, one level up the stack: at the parsed-message level rather
    than raw NatNet.

        ros2 run crazyflie_follow mocap_target \\
            --ros-args -p drone_name:=cf1 -p operator_name:=operator
    """
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node('mocap_target_diag')
    node.declare_parameter('drone_name', 'cf1')
    node.declare_parameter('operator_name', 'operator')
    node.declare_parameter('poses_topic', '/poses')
    node.declare_parameter('report_period_s', 1.0)
    drone_name = node.get_parameter('drone_name').value
    operator_name = node.get_parameter('operator_name').value
    topic = node.get_parameter('poses_topic').value
    period = float(node.get_parameter('report_period_s').value)

    tracker = MocapTargetTracker(node, drone_name, operator_name, topic=topic)

    def report():
        now = node.get_clock().now().nanoseconds * 1e-9
        for tr in (tracker.drone, tracker.operator):
            if not tr.initialized:
                node.get_logger().warn(f'{tr.name}: never seen on {topic}')
                continue
            node.get_logger().info(
                f'{tr.name}: pos={np.round(tr.position, 3).tolist()} '
                f'vel={np.round(tr.velocity, 3).tolist()} yaw={tr.yaw:+.2f} rad  '
                f'age={tr.age(now) * 1e3:.0f} ms  missing_frames={tr.missing_count}')

    node.create_timer(period, report)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
