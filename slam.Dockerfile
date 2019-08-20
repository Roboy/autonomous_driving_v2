FROM ros:melodic-robot

WORKDIR /home/ros

# Install system libraries
RUN apt-get update && \
    apt-get install -y sudo apt-utils ninja-build python-catkin-tools python-wstool python-rosdep ros-melodic-catkin ros-melodic-map-server ros-melodic-geometry2 ros-melodic-moveit-msgs net-tools

# Compile cartographer
WORKDIR /home/ros
RUN wstool init src && \
    wstool merge -t src https://raw.githubusercontent.com/Roboy/autonomous_driving_v2/devel_slam/cartographer_ros.rosinstall && \
    wstool update -t src

RUN src/cartographer/scripts/install_proto3.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# Build catkin workspace
RUN catkin config --extend /opt/ros/melodic && \
    catkin build

RUN touch /root/.bashrc && \
    echo 'source /home/ros/devel/setup.bash' >> /root/.bashrc && \
    echo 'export ROS_MASTER_URI=http://192.168.0.105:11311' >> /root/.bashrc && \
    echo 'export ROS_HOSTNAME=192.168.0.105' >> /root/.bashrc && \
    echo 'export ROS_IP=192.168.0.105' >> /root/.bashrc
