# Human/dashboard-facing summary of what follow_node is doing right now.
#
# `mode` uses the FollowCommand.MODE_* constants and `health` uses the
# TargetEstimate.HEALTH_* constants (deliberately not redeclared here, to
# keep one definition of each enum across the interface package).

std_msgs/Header header
uint8 mode
uint8 health
float32 range_to_target_m
float32 speed_cap_m_s
uint32 range_accepted
uint32 range_rejected
uint32 bearing_accepted
uint32 bearing_rejected
# Short machine-readable-ish explanation, e.g. "target lost" or
# "cmd:tablet" -- mirrors follow_core.follower.FollowerOutput.reason.
string reason
