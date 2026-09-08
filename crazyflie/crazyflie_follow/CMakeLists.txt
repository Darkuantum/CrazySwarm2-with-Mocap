cmake_minimum_required(VERSION 3.8)
project(crazyflie_follow)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Installs the python package and generates console-script shims from the
# entry points declared in setup.cfg (same mechanism as crazyflie_shows /
# crazyflie_examples).
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Everything the node needs at runtime, resolved via
# get_package_share_directory('crazyflie_follow') -- never an absolute path.
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
