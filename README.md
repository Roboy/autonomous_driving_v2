# Autonomous Driving V2

This is [Roboy's](https://roboy.org) Autonomous Driving Team's control repository from summer semester 2019.
It is a follow-up on the [Autonomous Driving Team's work from the winter semester 2018/19](https://github.com/Roboy/autonomous_driving).

## Control

This is the devel branch for control which will be deployed in the docker container ad-control. Check out the [docker branch](https://github.com/Roboy/autonomous_driving_v2/tree/docker) for more info.
You can also build this branch locally if you don't want to work in a docker container.

```
git clone -b devel_control https://github.com/Roboy/autonomous_driving_v2.git 
mv ./autonomous_driving_v2 ./src
cd src
git submodule init
git submodule update
cd ..
catkin build
source devel/setup.bash
```

## Packages of the ad-control docker:

### Communications
This package includes all ROS-Messages and Services used by the control.

### Navigation
This package is used to receive commands from the docker ad-planning and convert them into steering-/driving-commands.
This package also includes python-scripts to calibrate the angle-sensor of the rickshaw and test the steering/driving controls.

#### Calibration
In order to calibrate the angle sensor run:
<pre>
rosrun roboy_navigation angle_sensor_calibration.py
</pre>
Follow the instructions provided.
To check if the calibration was succesful check the config-file created by the calibration script at:
<pre>
cd src/roboy_navigation/config
vim calibration.yaml
</pre>
If this was not successful check if the angle is published correctly.
<pre>
rostopic echo /roboy/middleware/SteeringAngle
</pre>
raw_angles should contain the right data.

#### Testing Control
In order the test the control run:
<pre>
roslaunch roboy_navigation startup_test.launch sim:=False drive:=True
</pre>
More detailed instructions are provided on our confluence page.

#### Simulation Control
If you want to test rewrite the control code and test it, you can run
<pre>
roslaunch roboy_navigation startup_test.launch sim:=True drive:=False
</pre> 
This will simulate the rickshaw as an first-order delay element. With logging you can check if the controller is working.

