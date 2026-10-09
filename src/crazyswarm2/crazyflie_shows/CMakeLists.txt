cmake_minimum_required(VERSION 3.8)
project(crazyflie_shows)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Installs the python package and generates console-script shims from the
# entry points declared in setup.cfg (same mechanism as crazyflie_examples).
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Everything the show needs at runtime, resolved via
# get_package_share_directory('crazyflie_shows') -- never an absolute path.
# No `config` here on purpose: the fleet and mocap config are the `crazyflie`
# package's, read directly (see launch/show_launch.py). A copy in this package
# is a second fleet to keep in sync, and it drifted (2026-10-02: no cf8).
install(DIRECTORY
  launch
  data
  DESTINATION share/${PROJECT_NAME}/
)

# Mission-console views (one YAML per show; docs/MISSIONS.md). Nothing in the
# shows reads them. The console prefers the source copy when the install is a
# --symlink-install, so editing a spec needs no rebuild; this copy is for an
# install that is not.
install(DIRECTORY
  missions
  DESTINATION share/${PROJECT_NAME}/
)

# Lab diagnostics, executable in place: ros2 run crazyflie_shows mocap_diag.sh
install(PROGRAMS
  scripts/mocap_diag.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
