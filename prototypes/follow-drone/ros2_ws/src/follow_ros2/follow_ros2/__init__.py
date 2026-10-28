"""ROS 2 adapter nodes around the platform-independent `follow_core` package.

Everything under follow_ros2 is a thin translator: ROS messages in, calls into
follow_core.Follower, ROS messages out. No follow logic is reimplemented here
-- see follow_core/ for that, and see this package's README.md for the
Foxy-vs-Humble constraint that shapes every file below.
"""
