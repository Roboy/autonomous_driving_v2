A customized [ROS Navigation Stack](http://wiki.ros.org/navigation) for Rickshaw control.

## Background

**/map** - world frame

**base_link** - frame of the tricycle

## Usage

To start navigation stack with the real hardware run `roslaunch roboy_navigation nav_lidar.launch`

To start a simulation you should run
 1. `roslaunch roboy_models gaz_rickshaw.launch`
 2. `roslaunch roboy_navigation nav_static.launch`
 
 
## Testing Steering

1. `rosrun roboy_navingation test_steering.py`
2. `rqt_plot /test_angle`