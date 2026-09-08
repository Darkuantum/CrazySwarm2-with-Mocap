# Operator command with provenance. Mirrors follow_core.types.Command /
# FollowMode / CommandSource. Precedence between sources is decided entirely
# inside follow_core.arbiter.CommandArbiter -- this message only carries the
# facts, it does not encode who wins.

uint8 MODE_IDLE=0
uint8 MODE_FOLLOW=1
uint8 MODE_ORBIT=2
uint8 MODE_HOLD=3
uint8 MODE_LAND=4
uint8 MODE_ESTOP=5

uint8 SOURCE_TABLET=0
uint8 SOURCE_GESTURE=1
uint8 SOURCE_ESTOP_BUTTON=2
uint8 SOURCE_INTERNAL=3

std_msgs/Header header
uint8 mode
uint8 source

# Optional per-command overrides of FollowParams defaults. Since ROS 2 has no
# native "optional float", NaN is the sentinel meaning "no override, use the
# node's configured default" -- detected with a plain `x != x` float32
# comparison, and it round-trips cleanly through YAML/CLI tooling. NaN is the
# ONLY sentinel value for all three fields below: a negative sentinel would
# work for standoff_m/height_m (physically non-negative) but not for
# follow_angle_rad, which is a signed angle in (-pi, pi] and can legitimately
# be negative. Publishers that build this message by hand should use
# `float('nan')`; see follow_ros2/follow_node.py for the receiving side.
float32 standoff_m
float32 height_m
float32 follow_angle_rad
