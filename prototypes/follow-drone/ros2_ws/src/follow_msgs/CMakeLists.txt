cmake_minimum_required(VERSION 3.5)
project(follow_msgs)

# rosidl_generate_interfaces below is available identically on Foxy and
# Humble; nothing in this file is distro-specific.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UwbRange.msg"
  "msg/PersonBearing.msg"
  "msg/TargetEstimate.msg"
  "msg/FollowCommand.msg"
  "msg/FollowStatus.msg"
  "msg/DetectionBox2D.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
