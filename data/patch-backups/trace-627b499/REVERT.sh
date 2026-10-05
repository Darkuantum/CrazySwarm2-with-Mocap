#!/bin/bash
# Undo the packet-trace instrumentation. Both routes work; git is exact because
# every touched file was clean against HEAD (627b499) when the patch was applied.
cd /home/jeremy/CrazySwarm2-with-Mocap
# Route 1 - git (preferred):
git restore src/crazyswarm2/crazyflie/src/crazyflie_server.cpp src/crazyswarm2/crazyflie/deps/crazyflie_tools/crazyflie_cpp/crazyflie-link-cpp/src/CrazyradioThread.cpp src/crazyswarm2/crazyflie/deps/crazyflie_tools/crazyflie_cpp/crazyflie-link-cpp/include/crazyflieLinkCpp/Connection.h src/crazyswarm2/crazyflie/deps/crazyflie_tools/crazyflie_cpp/crazyflie-link-cpp/src/ConnectionImpl.h src/crazyswarm2/crazyflie/deps/crazyflie_tools/crazyflie_cpp/crazyflie-link-cpp/src/Connection.cpp src/crazyswarm2/crazyflie/deps/crazyflie_tools/crazyflie_cpp/include/crazyflie_cpp/Crazyflie.h src/crazyswarm2/crazyflie/deps/crazyflie_tools/crazyflie_cpp/src/Crazyflie.cpp 
# Route 2 - if git state has moved on, copy the saved originals back:
#   cd data/patch-backups/trace-627b499 && find . -type f ! -name REVERT.sh -exec cp --parents {} /home/jeremy/CrazySwarm2-with-Mocap/ \;
./scripts/build.sh crazyflie
