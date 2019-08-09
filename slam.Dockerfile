FROM ros:melodic-robot

WORKDIR /home/ros

# Install system libraries
RUN apt-get update && \
    apt-get install -y sudo vim ninja-build python-catkin-tools python-wstool python-rosdep net-tools 
RUN apt-get install -y ros-melodic-abseil-cpp ros-melodic-catkin ros-melodic-map-server ros-melodic-geometry2 ros-melodic-moveit-msgs 

# Initialize ROS
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'

# Copy autonomous driving code
#RUN mkdir src && cd src
#RUN git clone https://github.com/Roboy/autonomous_driving_v2.git cartographer_ros
#WORKDIR /home/ros/src

#RUN cd cartographer_ros/ && \
#    git checkout devel_slam

# Compile cartographer
RUN cd /home/ros
RUN wstool init src && \
    wstool merge -t src https://raw.githubusercontent.com/Roboy/autonomous_driving_v2/devel_slam/cartographer_ros.rosinstall && \
    wstool update -t src

RUN src/cartographer/scripts/install_proto3.sh && \
    rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y 

# Build catkin workspace
RUN catkin config --extend /opt/ros/melodic
RUN catkin build
RUN /bin/bash -c 'source devel/setup.bash'

# Update Cartographer_ROS roboy
#RUN cd src/cartographer_ros/ && \
#    git checkout devel_slam && \
#    git pull
