# Autonomous Driving V2

This is [Roboy's](https://roboy.org) Autonomous Driving Team's main repository from summer semester 2019. It is a follow-up on the [Autonomous Driving Team's work from the winter semester 2018/19](https://github.com/Roboy/autonomous_driving).

## Devel

This is the devel branch for integrating the sensor data into our software stack. This branch will be deployed in the docker container ad-sensors. Check out the [master branch](https://github.com/Roboy/autonomous_driving_v2/tree/master) for more info.
You can also build this branch locally if you don't want to work in a docker container.

```
mkdir /PATH/TO/LOCATION/WHERE/TO/BUILD/CATKIN-WORKSPACE
git clone -b devel_sensors https://github.com/Roboy/autonomous_driving_v2.git 
mv ./autonomous_driving_v2 ./src
cd src
chmod +x package_requirements.sh
bash package_requirements.sh
cd /PATH/TO/LOCATION/WHERE/TO/INSTALL/LIVOX-SDK
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK/build
cmake ..
make
make install
cd /PATH/TO/CATKIN-WORKSPACE/FOLDER
cd src
git submodule init
git submodule update
catkin config --extend /opt/ros/$ROS_DISTRO
cd ..
catkin build
cd /PATH/TO/CATKIN-WORKSPACE/FOLDER/src/roboy_ad
chmod +x src/fixBag3D.py
chmod +x imu/imu_remapping.py
chmod +x src/lidar_remapping.py
adduser root dialout
```

## Packages of the ad-sensors docker:

### Livox ROS Driver
This package is used to get the Livox Mid-100 Lidar PointCloud data. For more information on how to use this package, please refer to the [official documentation](https://github.com/Livox-SDK/livox_ros_driver).

### Roboy AD
This package integrates both Lidar and IMU sensor into very few launch files that need to be executed in order to receive and record the sensors' data. It furthermore involves scripts that remap and repair the ROS messages that contain the sensors' data.
In order to run both sensors (Lidar & IMU) simply execute the following launch file:
```
roslaunch roboy_ad sensors.launch rviz:=false
```
The sensors will start publishing their data to the topics /points2 and /imu
If you additionally want to record this data, wait until the first launch file is finished starting (make sure, no errors occured and only messages like "\[ INFO\] \[xxxxxxxxx.xxxxxxxxx\]: 80000" are prompted). This may take a few seconds. Afterwards, you can launch the second launch file, which will immediately start recording the data:
```
roslaunch roboy_ad data_recording.launch dir:=/DIRECTORY/WHERE/TO/SAVE/FILE/$(date +%Y-%m-%d-%H-%M-%S).bag
```

The recorded data will be saved as a bag file with the date and time of recording as its name.

## Scripts in the roboy ad package:

### imu_remapping.py

This script subscribes to the IMU topic (`/imu_data`) provided by the SBG ROS Driver and converts the proprietary SBG IMU Data format to the standard ROS sensor_msgs/Imu message format. The new messages are published to a new topic called `/imu`.

### lidar_remapping.py

This script subscribes to the Lidar PointCloud2 topic (`/livox/lidar`) provided by the Livox ROS Driver and sets the time stamp of each message to the current ROS time. This is necessary, as in its current set-up no sync signal is provided to the Lidar and thus the Lidar's internal time does not match the ROS time. The new messages are then published to a new topic called `/points2`.

### fixBag3D.py

This script takes bag files as an input and performs some repairing steps to the ROS messages, so that they are compliant with Google Cartographer.
Mainly, it aligns the time stamps of Lidar and IMU messages so that they are in the same time range and correct order. Furthermore, it also remaps the `livox/lidar` topic to `/points2` if not already happened. This scripts works supplementary to the two previously named scripts on offline data. For more information on how the time stamp remapping works see [the corresponding Confluence page](https://devanthro.atlassian.net/wiki/spaces/SS19/pages/491290693/Data+preprocessing) (access might only be possible, if you have access to Roboy's Confluence)
