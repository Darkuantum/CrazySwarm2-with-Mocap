"""Frame math shared by follow_node and detector_bridge.

No ROS imports here (only numpy) so it stays trivially unit-testable, and
deliberately avoids `tf_transformations` / `transforms3d`: neither ships in
the base `voxl-ros2-foxy` debian set, and pulling in an extra apt/pip
dependency on the vehicle for six lines of quaternion algebra is not worth
the deployment risk. FOXY COMPATIBILITY NOTE: keep it this way -- do not
"modernise" this into a tf_transformations import.

Frame conventions match follow_core/types.py exactly:
  world   right-handed, Z up (ENU-like).
  body    X forward, Y left, Z up (FLU).
  camera  REP-103 optical: X right, Y down, Z forward.
"""
from __future__ import annotations

import numpy as np

# Static world-frame conversion NED -> ENU and body-frame conversion
# FRD -> FLU. Both are the same matrix (a rotation of pi about the axis
# bisecting +x/+y), which is not a coincidence: axis 3 flips sign and axes
# 1/2 swap in both conversions. This is the standard PX4<->ROS static
# transform (see px4_ros_com / mavros frame-conversion notes).
_NED_ENU = np.array(
    [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
)
_FRD_FLU = _NED_ENU  # same matrix, kept as a separate name for readability


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """geometry_msgs/Quaternion (x, y, z, w) -> 3x3 rotation matrix.

    Standard closed-form conversion; normalises defensively so a slightly
    denormalised quaternion from a driver doesn't silently scale the result.
    """
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ]
    )


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Fixed-axis roll/pitch/yaw (radians) -> 3x3 rotation matrix.

    Convention R = Rz(yaw) @ Ry(pitch) @ Rx(roll), the same one
    tf2's static_transform_publisher uses for its rpy arguments -- used here
    for the camera mounting-rotation parameter so it can be copy-pasted from
    a `ros2 run tf2_ros static_transform_publisher` invocation.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def yaw_from_matrix(r_world_body: np.ndarray) -> float:
    """World heading (rad) of the body +X axis, projected to the world XY
    plane. Matches follow_core's VehicleState.yaw definition."""
    return float(np.arctan2(r_world_body[1, 0], r_world_body[0, 0]))


def ned_frd_to_enu_flu(
    position: np.ndarray, velocity: np.ndarray, r_ned_frd: np.ndarray
):
    """Convert a pose+velocity expressed in world-NED/body-FRD (PX4-native,
    e.g. an un-converted qvio feed) into world-ENU/body-FLU (what
    follow_core expects).

    Returns (position_enu, velocity_enu, r_world_body_enu_flu).
    """
    position_enu = _NED_ENU @ np.asarray(position, dtype=float).reshape(3)
    velocity_enu = _NED_ENU @ np.asarray(velocity, dtype=float).reshape(3)
    # R2 (_FRD_FLU) is symmetric, so R2^T == R2.
    r_enu_flu = _NED_ENU @ np.asarray(r_ned_frd, dtype=float).reshape(3, 3) @ _FRD_FLU
    return position_enu, velocity_enu, r_enu_flu


def bearing_from_camera_vector(x: float, y: float, z: float):
    """azimuth/elevation of a direction vector in the camera optical frame.

    Mirrors follow_core.ekf._bearing_from_vector exactly (duplicated only
    because that function is private to follow_core) -- keep the two in
    sync if the convention ever changes:
      azimuth   = atan2(x, z)
      elevation = atan2(-y, hypot(x, z))
    """
    azimuth = float(np.arctan2(x, z))
    elevation = float(np.arctan2(-y, np.hypot(x, z)))
    return azimuth, elevation
