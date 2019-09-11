# Demo Day March 9th, 2019

## Preparation

### Leia
Build the pure localization docker as stated [here](https://github.com/Roboy/autonomous_driving/tree/devel/dockers/localization_mw).

Build the autonomous driving workspace as stated [here](https://github.com/Roboy/autonomous_driving_src/blob/master/README.md).

### Hardware

1. Connect to the FPGA that controls myo-muscles for steering

2. Configure ROS to run with roscore on Leia

3. Start roboy plexus

### Map
Have a map of your environment in the `.pbstream` file format.

### Bike
Position bike physically somewhere on the map. Pure Localization can take up to 7 seconds to converge after you start driving so make sure you have enough free space ahead. 

## Demo

1. Launch roscore
```
roscore
```
### Localization 

2. Launch Sensor nodes
```
roslaunch roboy_ad sensors.launch
```
check if they are up and working properly through `rostopic echo /scan`

3. Launch Cartographer in Pure Localization Mode
```
docker start localization_mw
```

### Navigation
4. Start hardware controllers
```
roslaunch roboy_navigation driving.launch
``` 
5. Start ROS navigation stack
```
roslaunch roboy_navigation nav_lidar.launch
```
6. If you're running scripts on leia, start a separate rviz window on your machine 
```
rosrun rviz rviz -d path/to/catkin/workspace/src/roboy_navigation nav.rviz
```
7. Set `2D Nav Goal` in RVIZ

## What it should look like
(The GIFs might take some time to load.)
![Autonomous](https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/autonomous_roboy.gif)
![Demo](https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/demo.gif)

## Troubleshooting
I ran all the scripts, but the bike is not moving. What should I do? 

* Check if the rickshaw is localized properly on the map. If not, try manually moving the rickshaw forward a few meters meters until it does.
* Try setting navigation goal in RVIZ and check wheter a global plan to the goal is created (blue curve).
* Check if navigation software is working by listening to `rostopic echo /cmd_vel`. 
* Check that control commands for the hardware are being issued: `rostopic echo /roboy/control/GPIO` for rickshaw motor and `rostopic echo /roboy/middleware/MotorCommand` for the steering. 
* If control commands are being published, I have bad news for you: the problem is most probably in the hardware. 
* Check yellow LED on the motorboards is running. If not, restart the motorboards

If you've done all the steps and the bike is still not moving, grab someone from the hardware team and ask them to check the hardware. 