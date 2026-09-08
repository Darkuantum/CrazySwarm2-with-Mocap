# Direction to a detected person, in the camera optical frame (REP-103: X
# right, Y down, Z forward). Mirrors follow_core.types.BearingMeasurement.
#
# azimuth_rad   = atan2(x, z)              -- positive to the right
# elevation_rad = atan2(-y, hypot(x, z))   -- positive upward
#
# Carries no target identity of its own -- identity comes from gating against
# the UWB-anchored estimate inside follow_core.

std_msgs/Header header
float32 azimuth_rad
float32 elevation_rad
float32 std_rad
float32 score
# Detector-assigned track/detection id, for debugging multi-person scenes.
# Not consumed by follow_core (which only sees one bearing at a time).
uint16 detection_id
