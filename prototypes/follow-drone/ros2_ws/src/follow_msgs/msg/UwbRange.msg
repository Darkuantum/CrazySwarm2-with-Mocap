# A single two-way-ranging return from the operator's UWB tag.
#
# Mirrors follow_core.types.RangeMeasurement. tag_id is an unambiguous radio
# identity -- it is what anchors the follow lock against a visually similar
# bystander, so it must be forwarded even when only one tag is in use.

std_msgs/Header header
float32 range_m
uint16 tag_id
float32 std_m
# Vendor/driver quality indicator, 0 (worst) - 255 (best). Not used by
# follow_core directly today; carried through for logging/diagnostics.
uint8 quality
