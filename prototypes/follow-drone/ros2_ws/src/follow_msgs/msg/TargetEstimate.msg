# Fused operator estimate, in the world frame (right-handed, Z up -- see
# follow_core/types.py). Mirrors follow_core.types.TargetEstimate.

uint8 HEALTH_TRACKING=0
uint8 HEALTH_DEGRADED=1
uint8 HEALTH_LOST=2

std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
# Row-major 6x6 covariance over [x, y, z, vx, vy, vz], matching
# TargetEstimate.covariance. Follows the same 36-element convention as
# nav_msgs/Odometry's pose/twist covariance fields.
float64[36] covariance
uint8 health
