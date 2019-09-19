# Autonomous Driving V2

This is [Roboy's](https://roboy.org) Autonomous Driving Team's main repository from summer semester 2019. It is a follow-up on the [Autonomous Driving Team's work from the winter semester 2018/19](https://github.com/Roboy/autonomous_driving).

## Devel

This is the devel branch for SLAM which will be deployed in the docker container ad-slam. Check out the [master branch](https://github.com/Roboy/autonomous_driving_v2/tree/master) for more info.
You can also build this branch locally if you don't want to work in a docker container.

```
sudo apt-get update
sudo apt-get install ninja-build python-catkin-tools python-wstool python-rosdep ros-melodic-catkin ros-melodic-map-server ros-melodic-geometry2 ros-melodic-moveit-msgs

mkdir /PATH/TO/LOCATION/WHERE/TO/BUILD/CATKIN-WORKSPACE
cd /PATH/TO/LOCATION/WHERE/TO/BUILD/CATKIN-WORKSPACE

wstool init src
wstool merge -t src https://raw.githubusercontent.com/Roboy/autonomous_driving_v2/devel_slam/cartographer_ros.rosinstall
wstool update -t src

src/cartographer/scripts/install_proto3.sh
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

catkin config --extend /opt/ros/melodic
catkin build

touch /root/.bashrc && echo 'source /home/ros/devel/setup.bash' >> /root/.bashrc
```

## Packages of the ad-slam docker:
