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

# Get and install Abseil
#WORKDIR /home/abseil
#RUN git clone https://github.com/abseil/abseil-cpp
#WORKDIR /home/abseil/abseil-cpp/build
#RUN cmake -DCMAKE_INSTALL_PREFIX=/home/abseil/abseil-cpp .. && \
#    make install -j
#WORKDIR /home/abseil/abseil-cpp
#RUN cp ./build/CMakeFiles/Export/lib/cmake/absl/abslTargets.cmake .

WORKDIR /home/ros/src
RUN git clone https://github.com/abseil/abseil-cpp
WORKDIR /home/ros

# Build catkin workspace
RUN catkin config --extend /opt/ros/melodic && \
  #  export absl_DIR=/home/abseil/abseil-cpp && \
  #  export CXXFLAGS=-isystem\ /home/abseil/abseil-cpp/include && \
  #  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/abseil/abseil-cpp/include && \
  #  catkin build --cmake-args -DCMAKE_CXX_FLAGS="-I /home/abseil/abseil-cpp/include"
  catkin build
