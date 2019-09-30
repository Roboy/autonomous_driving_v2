# Autonomous Driving V2

This is [Roboy's](https://roboy.org) Autonomous Driving Team's main repository from summer semester 2019. It is a follow-up on the [Autonomous Driving Team's work from the winter semester 2018/19](https://github.com/Roboy/autonomous_driving).
Note: Due to some issues with Sphinx-documentation, the provided html documentation will slightly differ in appearence from the source code provided here, but the functionalty is not affected

## Control

This is the control branch for planning which will be deployed in the docker container ad-control. Check out the [master branch](https://github.com/Roboy/autonomous_driving_v2/tree/master) for more info.
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

### Roboy_Navigation
This package contains all the code necessary for controlling the rickshaw. It contains the controllers for the myo-muscles as well as the controller for the main driving motor. It also contains the code to read the steering angle of the bike and send the driving commands to the FPGA on the bike.
If everything is set up (electronics and FPGA) and you simply want to start the controll run
```
roslaunch roboy_navigation controller_start.launch
```
inside the docker container on the bike.
For more detailed usage look below and into the wiki file.

### Roboy_communication
This submodule contains the roboy_communication package which includes all Roboy specific ROS message types and ROS service types.

### common_utilities
This submodule is only needed if you want know how and want to use rqt-plugins

## Usage

### FPGA
Connect to the FPGA
Set the ROS settings
<pre>
export ROS_MASTER_URI=http://<i>IP-Computer</i>:11311/
export ROS_HOSTNAME=<i>IP-FPGA</i>
export ROS_IP=<i>IP-FPGA</i>
</pre>
check if Computer is in /etc/hosts
```
vim /etc/hosts
```

In order to activate the motor start roboy_plexus. Only start it AFTER! a roscore is running on Leia. 
```
roboy_plexus
```

### Computer
Connect to computer

Start a roscore on the computer
```
roscore
```

Reconnect to the computer in another terminal and go to the directory with the dockerfiles
To build run
```
docker build -t ad-control -f control.Dockerfile .
```

If you want to completely rebuild the dockerfile
```
docker build -t ad-control -f control.Dockerfile . --no-cache
```

To run, start and execute the docker-container run
```
docker run -it -d --network=host --name ad-control ad-control:latest bash
docker start ad-control
docker exec -it ad-control bash
```

Now you should be inside the docker prompt
#### Optional: Calibration
In order to recalibrate the angle sensor 
```
rosrun roboy_navigation angle_sensor_calibration.py
```

Follow the instructions in the terminal and the configfile will be created automatically.
#### Startup-Sequence
To launch the startup sequence run
```
roslaunch roboy_navigation startup_test.launch sim:=False drive:=True
```

If you want to use other parameters, then the ones in the configuration files run
```
roslaunch roboy_navigation startup_test.launch left_motor_id:="" right_motor_id:="" fpga_id:="" sample_rate:="" kp:="" ki:="" kd:="" max_disp:="" min_disp:=""
```
