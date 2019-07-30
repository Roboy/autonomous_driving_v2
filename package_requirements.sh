#!/bin/sh
apt-get update  # To get the latest package lists
apt-get install cmake ros-melodic-sbg-driver python-wstool python-rosdep ninja-build ros-melodic-abseil-cpp ros-melodic-geometry2 libarmadillo-dev ros-melodic-rviz python-catkin-tools libapr1-dev libboost-atomic-dev libboost-system-dev -y
#etc.
