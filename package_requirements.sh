#!/bin/sh
apt-get update  # To get the latest package lists
apt-get install ros-kinetic-map-server python-wstool python-rosdep ros-kinetic-abseil-cpp ros-kinetic-geometry2 libarmadillo-dev ros-kinetic-moveit-msgs ros-kinetic-rviz ros-kinetic-cv-bridge ros-kinetic-rgbd-launch libgazebo7-dev ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control ros-kinetic-ros-controllers ros-kinetic-navigation ros-kinetic-sbg-driver python-catkin-tools pyyaml -y
#etc.
