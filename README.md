# Autonomous Driving V2

This is [Roboy's](https://roboy.org) Autonomous Driving Team's main repository from summer semester 2019. It is a follow-up on the [Autonomous Driving Team's work from the winter semester 2018/19](https://github.com/Roboy/autonomous_driving).

## Devel

This is the devel branch for planning which will be deployed in the docker container. Check out the docker [docker branch](https://github.com/Roboy/autonomous_driving_v2/tree/docker) for more info.

## Packages of the ad-planning docker:

### Navigation
This package is used to test the navigation stack in simulation with rviz visualization.

### Quadtreeplanner
This package contains the custom implementation of a quadtree cell decomposition based planner with path refinement to meet the non-holonomic constraints of the rickshaw. Details about the implementation of the planning algorithm can be found in our wiki at ToDo:Link to WIKI page.

### Roboy_communication
This submodule contains the roboy_communication package which includes all Roboy specific ROS message types and ROS service types.

### Roboy_models
This package is required to run the simulation of the navigation stack. It contains a .urdf model of the rickshaw.

### Roboy_navigation
This package contains the launch-files, config-files and data (static maps of the environment) to run the navigation stack with the real HW (i.e. roboy rickshaw).

### Simulation
This package is required to run the simulation of the navigation stack. It contains scripts that are used in the simulation mode.
