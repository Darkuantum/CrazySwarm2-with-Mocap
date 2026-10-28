"""uwb_node -- serial bridge to the operator's UWB/gesture wristband.

Reads the band's wire protocol (see uwb_protocol.py) off a serial port
(pyserial) and:
  * republishes ranging results as follow_msgs/UwbRange,
  * translates GESTURE and ESTOP packets into follow_msgs/FollowCommand,
  * publishes an ESTOP command THE INSTANT an ESTOP-pressed packet is parsed
    -- synchronously, in the same read callback that parsed it, never queued
    behind a timer tick or batched with other traffic.

All serial I/O runs on a single ROS timer (non-blocking reads, polled at
`poll_rate_hz`) rather than a background thread, so there is exactly one
thread touching the serial port and the executor -- simpler and more
portable across Foxy/Humble than adding thread-safety of our own.

-- ESTOP latch / clear, and the gap between FollowCommand and CommandArbiter --
follow_core.arbiter.CommandArbiter latches ESTOP on any ESTOP-mode command
from ESTOP_BUTTON, and that latch can ONLY be lifted by calling its
`clear_estop()` method directly -- there is no FollowMode value that means
"clear the latch", so no ordinary Command can undo it. To bridge that
honestly instead of inventing a new wire message, this node treats an ESTOP
packet's press->release edge (pressed: 1 -> 0) as the operator's "long-press
release" action and publishes FollowCommand(mode=MODE_IDLE,
source=SOURCE_ESTOP_BUTTON). follow_node.py's command handler special-cases
exactly that (source, mode) pair and calls
`self.follower.arbiter.clear_estop()` (a public method on a public attribute)
before doing anything else with it. See follow_node.py for the receiving
side; keep both halves of this convention in sync if it ever changes.

Also note: HEARTBEAT carries an `estop_latched` flag from the band's own
firmware. If it ever reports latched=True without us having just seen an
ESTOP-pressed packet (e.g. that one packet was lost on a noisy link), this
node asserts ESTOP anyway -- an E-stop channel must fail loud, not quiet.
"""
from __future__ import annotations

import time

import rclpy
import serial
from rclpy.node import Node

from follow_ros2._bootstrap import ensure_follow_core

ensure_follow_core()

from follow_msgs.msg import FollowCommand, UwbRange  # noqa: E402

from follow_ros2.uwb_protocol import (  # noqa: E402
    Estop,
    FrameReader,
    Gesture,
    GestureId,
    Heartbeat,
    RangeReport,
)

# GESTURE -> (mode, follow_angle_rad override). None means "no override,
# use the node's configured default". COME has no direct FollowMode -- it is
# interpreted as FOLLOW with follow_angle_rad=0 (stand in front of the
# operator's direction of travel, rather than the default trailing angle of
# pi) since "come here" is a request to approach, not to trail. This mapping
# is a judgment call: the wire spec defines the gesture IDs but not their
# follow-behaviour semantics.
_GESTURE_MODE = {
    GestureId.FOLLOW: (FollowCommand.MODE_FOLLOW, None),
    GestureId.STOP: (FollowCommand.MODE_HOLD, None),
    GestureId.ORBIT: (FollowCommand.MODE_ORBIT, None),
    GestureId.LAND: (FollowCommand.MODE_LAND, None),
    GestureId.COME: (FollowCommand.MODE_FOLLOW, 0.0),
}


class UwbNode(Node):
    def __init__(self) -> None:
        super().__init__("uwb_node")

        # VERIFY against the actual band hardware/cabling -- this default is
        # a placeholder, not a confirmed VOXL 2 UART assignment.
        self.declare_parameter("serial_port", "/dev/ttyHS1")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("poll_rate_hz", 100.0)
        self.declare_parameter("read_chunk_bytes", 4096)

        self.declare_parameter("range_topic", "/uwb/range")
        self.declare_parameter("command_topic", "/follow/command")

        self.declare_parameter("range_std_m", 0.10)
        # Scale/units for GESTURE.confidence are not specified by the wire
        # protocol beyond "uint8" -- assumed 0-100 here. VERIFY against the
        # band firmware and adjust gesture_min_confidence accordingly.
        self.declare_parameter("gesture_min_confidence", 60)

        self._range_std_m = float(self.get_parameter("range_std_m").value)
        self._gesture_min_confidence = int(
            self.get_parameter("gesture_min_confidence").value
        )

        self._pub_range = self.create_publisher(
            UwbRange, self.get_parameter("range_topic").value, 10
        )
        # depth=20, not 1: command traffic (esp. ESTOP) must never be dropped
        # for being "just the latest of a burst" -- see module docstring.
        self._pub_command = self.create_publisher(
            FollowCommand, self.get_parameter("command_topic").value, 20
        )

        self._reader = FrameReader()
        self._last_estop_pressed = False
        self._last_heartbeat_log = 0.0

        port = self.get_parameter("serial_port").value
        baud = int(self.get_parameter("baud_rate").value)
        try:
            # timeout=0: non-blocking reads: read() returns immediately with
            # whatever is available (possibly nothing), which is what lets
            # this run off a ROS timer on the executor thread with no risk
            # of stalling the node.
            self._ser = serial.Serial(port, baudrate=baud, timeout=0)
            self.get_logger().info(f"uwb_node: opened {port} @ {baud} baud")
        except serial.SerialException as exc:
            self._ser = None
            self.get_logger().error(
                f"uwb_node: could not open serial port {port!r}: {exc}. "
                "Node will idle -- fix the port/permissions and restart."
            )

        rate_hz = float(self.get_parameter("poll_rate_hz").value)
        self._chunk = int(self.get_parameter("read_chunk_bytes").value)
        self.create_timer(1.0 / rate_hz, self._on_poll)

    # -- serial polling -------------------------------------------------------

    def _on_poll(self) -> None:
        if self._ser is None:
            return
        try:
            data = self._ser.read(self._chunk)
        except serial.SerialException as exc:
            self.get_logger().error(f"uwb_node: serial read failed: {exc}")
            return
        if not data:
            return
        self._reader.feed(data)
        for pkt in self._reader.read_all():
            self._dispatch(pkt)

    def _dispatch(self, pkt) -> None:
        now = self._now()
        if isinstance(pkt, RangeReport):
            self._publish_range(pkt, now)
        elif isinstance(pkt, Gesture):
            self._on_gesture(pkt, now)
        elif isinstance(pkt, Estop):
            self._on_estop(pkt, now)
        elif isinstance(pkt, Heartbeat):
            self._on_heartbeat(pkt, now)
        # Ack: nothing to do -- it is the band acknowledging something we
        # sent it, and this node does not currently send commands to the
        # band (see README for haptic-feedback scope).

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- packet handlers -------------------------------------------------------

    def _publish_range(self, pkt: RangeReport, now: float) -> None:
        msg = UwbRange()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.range_m = float(pkt.range_m)
        msg.tag_id = int(pkt.tag_id)
        msg.std_m = self._range_std_m
        msg.quality = int(pkt.quality)
        self._pub_range.publish(msg)

    def _on_gesture(self, pkt: Gesture, now: float) -> None:
        if pkt.confidence < self._gesture_min_confidence:
            self.get_logger().debug(
                f"uwb_node: gesture {pkt.gesture_id} below confidence "
                f"threshold ({pkt.confidence} < {self._gesture_min_confidence}), ignored"
            )
            return
        mapping = _GESTURE_MODE.get(pkt.gesture_id)
        if mapping is None:
            self.get_logger().warn(f"uwb_node: unknown gesture_id {pkt.gesture_id}")
            return
        mode, follow_angle_override = mapping
        cmd = FollowCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.mode = mode
        cmd.source = FollowCommand.SOURCE_GESTURE
        cmd.standoff_m = float("nan")
        cmd.height_m = float("nan")
        cmd.follow_angle_rad = (
            float("nan") if follow_angle_override is None else float(follow_angle_override)
        )
        self._pub_command.publish(cmd)

    def _on_estop(self, pkt: Estop, now: float) -> None:
        # Published immediately, in this same callback -- see module
        # docstring: an ESTOP must never wait behind a queue or a timer.
        cmd = FollowCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.source = FollowCommand.SOURCE_ESTOP_BUTTON
        cmd.standoff_m = float("nan")
        cmd.height_m = float("nan")
        cmd.follow_angle_rad = float("nan")

        if pkt.pressed:
            cmd.mode = FollowCommand.MODE_ESTOP
            self._pub_command.publish(cmd)
            self._last_estop_pressed = True
            self.get_logger().warn(
                f"uwb_node: ESTOP pressed (press_count={pkt.press_count}) -- publishing ESTOP"
            )
        else:
            if self._last_estop_pressed:
                # Release edge: the "clear the latch" sentinel. See module
                # docstring -- this exact (mode, source) pair is special-cased
                # by follow_node._on_command.
                cmd.mode = FollowCommand.MODE_IDLE
                self._pub_command.publish(cmd)
                self.get_logger().info("uwb_node: ESTOP released -- publishing clear sentinel")
            self._last_estop_pressed = False

    def _on_heartbeat(self, pkt: Heartbeat, now: float) -> None:
        # Defensive redundancy: the band's own firmware thinks ESTOP is
        # latched, but we have not (yet) seen a corresponding ESTOP-pressed
        # packet -- assume the pressed packet was lost and assert ESTOP now
        # rather than silently trusting a possibly-stale local state.
        if pkt.estop_latched and not self._last_estop_pressed:
            cmd = FollowCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.mode = FollowCommand.MODE_ESTOP
            cmd.source = FollowCommand.SOURCE_ESTOP_BUTTON
            cmd.standoff_m = float("nan")
            cmd.height_m = float("nan")
            cmd.follow_angle_rad = float("nan")
            self._pub_command.publish(cmd)
            self._last_estop_pressed = True
            self.get_logger().error(
                "uwb_node: heartbeat reports estop_latched with no prior ESTOP "
                "packet seen -- asserting ESTOP defensively"
            )

        # Manual throttle (avoids relying on any post-Foxy rclpy logging
        # convenience kwargs): log battery/health at most once every 5s.
        now_wall = time.monotonic()
        if now_wall - self._last_heartbeat_log > 5.0:
            self._last_heartbeat_log = now_wall
            self.get_logger().info(
                f"uwb_node: heartbeat battery_mv={pkt.battery_mv} imu_ok={pkt.imu_ok} "
                f"uwb_ok={pkt.uwb_ok} estop_latched={pkt.estop_latched}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UwbNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
