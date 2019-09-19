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
