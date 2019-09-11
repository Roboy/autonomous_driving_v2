# Autonomous Driving V2

This is [Roboy's](https://roboy.org) Autonomous Driving Team's main repository from summer semester 2019. It is a follow-up on the [Autonomous Driving Team's work from the winter semester 2018/19](https://github.com/Roboy/autonomous_driving).

## Devel

This is the devel branch for planning which will be deployed in the docker container ad-planning. Check out the [docker branch](https://github.com/Roboy/autonomous_driving_v2/tree/docker) for more info.
You can also build this branch locally if you don't want to work in a docker container.

```
git clone -b devel_planning https://github.com/Roboy/autonomous_driving_v2.git 
mv ./autonomous_driving_v2 ./src
cd src
git submodule init
git submodule update
cd ..
catkin build
source devel/setup.bash
```

## Packages of the ad-planning docker:

### Navigation
This package is used to test the navigation stack in simulation with rviz visualization.
If you want to run the simulation, use the following command:
<pre>
roslaunch navigation demo_quadtree.launch rviz:=true map_configfile:="<i>MapConfigfile.yaml</i>" goal_tolerance_local_planner:=<i>xx</i>
</pre>
The following maps are available:
- MW.yaml (Maschinenwesen building of TUM indoor)
- Interims.yaml (area around Interims II lecture hall of TUM)
- MidnightSurprise.yaml

The parameter goal_tolerance_local_planner is optional. It specifies the tolerance in the xy-plane from the final goal position for the local planner to stop the rickshaw. You can also start the launch file without specifying a tolerance (default value will be used in this case).
<pre>
roslaunch navigation demo_quadtree.launch rviz:=true map_configfile:="<i>MapConfigfile.yaml</i>"
</pre>

### Quadtreeplanner
This package contains the custom implementation of a quadtree cell decomposition based planner with path refinement to meet the non-holonomic constraints of the rickshaw. Details about the implementation of the planning algorithm can be found in our wiki at https://github.com/Roboy/autonomous_driving_v2/wiki/Path_Planning

### Roboy_communication
This submodule contains the roboy_communication package which includes all Roboy specific ROS message types and ROS service types.

### Roboy_models
This package is required to run the simulation of the navigation stack. It contains a .urdf model of the rickshaw.

### Roboy_navigation
This package contains the launch-files, config-files and data (static maps of the environment) to run the navigation stack with the real HW (i.e. roboy rickshaw).
If you want to run the SW on the roboy rickshaw, use the following command:
<pre>
roslaunch roboy_navigation nav_static.launch rviz:=false map_configfile:="<i>MapConfigfile.yaml</i>"  goal_tolerance_local_planner:=<i>xx</i>
</pre>
The following maps are available:
- MW.yaml (Maschinenwesen building of TUM indoor)
- Interims.yaml (area around Interims II lecture hall of TUM
- MidnightSurprise.yaml

The parameter goal_tolerance_local_planner is optional. It specifies the tolerance in the xy-plane from the final goal position for the local planner to stop the rickshaw.

If you want to visualize the map in rviz on a remote PC, do the following steps in a terminal on your remote PC:
<pre>
export ROS_MASTER_URI=http://ROS_MASTER_IP:11311/
export ROS_HOSTNAME=REMOTE_IP
export ROS_IP=REMOTE_IP

rosrun rviz rviz -d <i>path_to_the_repo</i>/src/roboy_navigation/nav.rviz
</pre>

### Simulation
This package is required to run the simulation of the navigation stack. It contains scripts that are used in the simulation mode.
