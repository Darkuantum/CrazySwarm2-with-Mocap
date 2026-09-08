"""detector_bridge -- turns a 2D detection box into follow_msgs/PersonBearing.

INPUT SHIM NOTE: voxl-tflite-server publishes person detections, but its
message shape is not something this package depends on directly. The obvious
choice, vision_msgs/Detection2DArray, is deliberately AVOIDED: vision_msgs is
a separate community package not guaranteed to ship in voxl-ros2-foxy's apt
set, and its Detection2D bbox representation has changed across vision_msgs
major versions (v1 vs v2/v3), which is exactly the kind of dependency this
project cannot afford to have silently mismatch on the vehicle.

Instead this node subscribes to follow_msgs/DetectionBox2D -- a small message
defined in this repo's own follow_msgs package, built from only
sensor_msgs/RegionOfInterest (guaranteed present in Foxy) plus a score and a
class id. Whatever process bridges voxl-tflite-server's real output onto
ROS 2 must publish that shape on `detection_topic`; if the actual bridge
already speaks vision_msgs, a small translator node (not included here, since
it depends on which vision_msgs version is actually installed on your image)
should sit between it and this one.

Bearing convention (must match follow_core/types.py and follow_core/ekf.py
exactly): with bbox centroid pixel (u, v) and intrinsics (fx, fy, cx, cy),

    x = (u - cx) / fx
    y = (v - cy) / fy
    z = 1.0
    azimuth_rad   = atan2(x, z)
    elevation_rad = atan2(-y, hypot(x, z))

The default intrinsics below are a PLACEHOLDER for a 640x480 stream and are
almost certainly wrong for your actual camera -- replace them with a real
calibration (e.g. from `voxl-configure-cameras` / a checkerboard calibration)
via the `fx`, `fy`, `cx`, `cy` parameters before trusting any bearing this
node produces.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node

from follow_ros2._bootstrap import ensure_follow_core

ensure_follow_core()

from follow_msgs.msg import DetectionBox2D, PersonBearing  # noqa: E402

from follow_ros2.frames import bearing_from_camera_vector  # noqa: E402

# Placeholder-only defaults for a 640x480 stream. REPLACE with real
# calibration -- these are not measured for any specific VOXL 2 camera.
_DEFAULT_FX = 460.0
_DEFAULT_FY = 460.0
_DEFAULT_CX = 320.0
_DEFAULT_CY = 240.0


class DetectorBridge(Node):
    def __init__(self) -> None:
        super().__init__("detector_bridge")

        self.declare_parameter("detection_topic", "/detector/boxes")
        self.declare_parameter("bearing_topic", "/person/bearing")

        # Camera intrinsics -- SEE MODULE DOCSTRING. These defaults are for a
        # generic 640x480 stream only and must be replaced with a real
        # calibration for your camera.
        self.declare_parameter("fx", _DEFAULT_FX)
        self.declare_parameter("fy", _DEFAULT_FY)
        self.declare_parameter("cx", _DEFAULT_CX)
        self.declare_parameter("cy", _DEFAULT_CY)

        # Only detections of this class id are turned into bearings. Default
        # 0 matches a person-only detector model; if using a multi-class COCO
        # model, set this to COCO's "person" class id (0) or whatever your
        # label map assigns -- CHECK YOUR MODEL'S LABEL MAP, do not assume.
        self.declare_parameter("person_class_id", 0)
        self.declare_parameter("min_score", 0.3)

        self._fx = float(self.get_parameter("fx").value)
        self._fy = float(self.get_parameter("fy").value)
        self._cx = float(self.get_parameter("cx").value)
        self._cy = float(self.get_parameter("cy").value)
        self._person_class_id = int(self.get_parameter("person_class_id").value)
        self._min_score = float(self.get_parameter("min_score").value)

        self._pub = self.create_publisher(
            PersonBearing, self.get_parameter("bearing_topic").value, 10
        )
        self.create_subscription(
            DetectionBox2D,
            self.get_parameter("detection_topic").value,
            self._on_detection,
            10,
        )

        self.get_logger().info(
            f"detector_bridge up: fx={self._fx} fy={self._fy} cx={self._cx} "
            f"cy={self._cy} (VERIFY these against real calibration), "
            f"person_class_id={self._person_class_id}"
        )

    def _on_detection(self, msg: DetectionBox2D) -> None:
        if msg.class_id != self._person_class_id:
            return
        if msg.score < self._min_score:
            return

        roi = msg.roi
        u = float(roi.x_offset) + float(roi.width) * 0.5
        v = float(roi.y_offset) + float(roi.height) * 0.5

        x = (u - self._cx) / self._fx
        y = (v - self._cy) / self._fy
        z = 1.0
        azimuth_rad, elevation_rad = bearing_from_camera_vector(x, y, z)

        out = PersonBearing()
        out.header = msg.header
        out.azimuth_rad = float(azimuth_rad)
        out.elevation_rad = float(elevation_rad)
        # No direct pixel-noise-to-angle-noise model here; a fixed std with
        # confidence-based inflation is handled downstream in follow_core's
        # EKF via the `score` field (see ekf.update_bearing). Widen the fixed
        # std a little for boxes near the image edge, where a centroid is a
        # worse proxy for the person's actual bearing (partial box).
        near_edge = (
            roi.x_offset <= 2
            or roi.y_offset <= 2
            or (roi.x_offset + roi.width) >= (2 * self._cx - 2)
            or (roi.y_offset + roi.height) >= (2 * self._cy - 2)
        )
        out.std_rad = float(0.04 if near_edge else 0.02)
        out.score = float(msg.score)
        # DetectionBox2D carries no per-detection track id (only a class
        # id); left at 0 unless/until the upstream bridge adds one.
        out.detection_id = 0
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectorBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
