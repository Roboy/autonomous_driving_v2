# I. Prerequisites & Hardware

Before trying to work with lidar or IMU, follow the [roboy_ad README](https://github.com/Roboy/autonomous_driving_src/blob/master/roboy_ad/README.md). For details on the hardware used, see the [hardware](https://github.com/Roboy/autonomous_driving/wiki/Hardware) section.

## I.1. Lidar
For [our lidar](https://github.com/Roboy/autonomous_driving/wiki/Hardware%3A-Lidar), the [launch file](https://github.com/Roboy/autonomous_driving_src/blob/master/roboy_ad/launch/sick_lms_155.launch) provided boots up the lidar and starts publishing ROS messages under lidar factory settings. Note that you will need to set the correct IP adress in the script. 

The [Sick Scan ROS package](http://wiki.ros.org/sick_scan) being build for using our code is kind of unstable. Sometimes (actually, almost always), it crashes and requires you to stop the node and retry booting it even though all parameters are correct. 

Test your setup through visualization in RVIZ and see if it matches your environment.

## I.2. IMU
Our [IMU](https://github.com/Roboy/autonomous_driving/wiki/Hardware%3A-IMU) can be set up as stated [here](https://github.com/Roboy/autonomous_driving_src/tree/devel/roboy_ad#imu). It is optional for Cartographer but helps a lot (more on that later). The [SBG_driver](http://wiki.ros.org/sbg_driver) has some special preconfigured settings like a left-handed coordinate system, includes french special characters in the ROS messages and you will need to think twice about the direction of gravity (compare to [ROS specifications for IMU](http://www.ros.org/reps/rep-0145.html#data-sources)). We have a [conversion script](https://github.com/Roboy/autonomous_driving_src/blob/devel/roboy_ad/imu/imu_remapping.py) putting things in the right setting for us. 

Test your setup through `rostopic echo /imu` and rotate the IMU such that every coordinate axis points down once. If the direction of gravity does not intuitively make sense to you right away but think about it another time. 