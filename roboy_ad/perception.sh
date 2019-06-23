#!/bin/bash
clear
source ${HOME}/catkin_ws/devel/setup.bash
roslaunch roboy_ad sensors.launch
docker start localization_mw

