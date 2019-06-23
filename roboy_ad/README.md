# Roboy AD ROS package
This package can do many things needed for Roboy AD project.

## IMU 
This is to convert the [custom ROS message](http://docs.ros.org/api/sbg_driver/html/msg/SbgImuData.html) of a [SBG Systems Ellipse2-A](https://github.com/Roboy/autonomous_driving/wiki/Hardware%3A-IMU) IMU unit to a standard [ROS Kinetic IMU sensor message](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Imu.html) for the use in [Google Cartographer](https://github.com/Roboy/cartographer_ros/tree/roboy).

### Install
```
sudo apt-get install ros-kinetic-sbg-driver
```
Add your session `username`to the dialout group and **reboot your machine**.
```
sudo adduser username dialout
```
Clone this repository into your catkin/src folder and build using
```
catkin build roboy_ad
source devel/setup.bash
```

### Run
Launch IMU script:
```
roslaunch roboy_ad imu_converter.launch
```
The IMU will publish topic `/imu_data` which is converted to `/imu` by our script.

## Lidar
Before launching, it is required to set the LIDAR IP adress accordingly in the `launch` file (i.e. 192.168.0.42). 
```
roslaunch roboy_ad sick_lms_155.launch
```

## Cartographer Visualization
You will need to [`build cartographer_rviz`](https://github.com/Roboy/cartographer_ros/tree/roboy) on your machine for this to work.

```
roslaunch roboy_ad rviz_cartographer.launch
```
