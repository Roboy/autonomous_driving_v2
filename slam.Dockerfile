FROM ros:melodic-robot

WORKDIR /home/ros

# Install system libraries
RUN apt-get update && \
    apt-get install -y sudo ninja-build python-catkin-tools python-wstool python-rosdep ros-melodic-abseil-cpp ros-melodic-catkin ros-melodic-map-server ros-melodic-geometry2 ros-melodic-moveit-msgs net-tools

# Install vim
RUN apt install vim -y

# Initialize ROS
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'

# Copy autonomous driving code
WORKDIR /home/ros/src
RUN git clone https://github.com/Roboy/cartographer_ros.git
#WORKDIR /home/ros/src

RUN cd cartographer_ros/ && \
    git checkout adv2

# Compile cartographer
WORKDIR /home/ros
RUN wstool init src && \
    wstool merge -t src https://raw.githubusercontent.com/Roboy/cartographer_ros/roboy/cartographer_ros.rosinstall && \
    wstool update -t src

RUN src/cartographer/scripts/install_proto3.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y 

# Build catkin workspace
RUN catkin config --extend /opt/ros/melodic
RUN catkin build
RUN /bin/bash -c 'source devel/setup.bash'

# Update Cartographer_ROS roboy
RUN cd src/cartographer_ros/ && \
    git checkout adv2 && \
    git pull
