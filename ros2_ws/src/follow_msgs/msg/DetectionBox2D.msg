# Minimal 2D detection box, used as detector_bridge's input shim.
#
# vision_msgs is deliberately NOT used here: it is a separate community
# package not guaranteed to be present in voxl-ros2-foxy's apt set, and its
# Detection2D/Detection2DArray shape has changed across vision_msgs releases
# (v1 vs v2/v3 disagree on bbox representation). This message uses only
# sensor_msgs (guaranteed present in Foxy) plus two primitive fields, which is
# enough for a bbox-centroid-to-bearing conversion and nothing else.
#
# Whatever bridges voxl-tflite-server's actual output onto ROS 2 must publish
# this shape (or detector_bridge must be pointed at an equivalent existing
# topic) -- see follow_ros2/detector_bridge.py's module docstring.

std_msgs/Header header
sensor_msgs/RegionOfInterest roi
float32 score
uint16 class_id
# Human-readable label, e.g. "person". Filtering in detector_bridge is by
# class_id (see its `person_class_id` parameter); label is carried for
# logging/debugging only.
string label
