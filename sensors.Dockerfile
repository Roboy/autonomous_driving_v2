FROM ros:melodic-robot

# Set working directory
WORKDIR /home/ros

# Get autonomous driving src folder from GIT
RUN git clone -b devel_sensors https://github.com/Roboy/autonomous_driving_v2.git  ./src

# Install required packages
RUN chmod +x src/package_requirements.sh
RUN bash src/package_requirements.sh

# Get Livox SDK from GIT
RUN cd /
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git &&  \
	cd Livox-SDK/build && \
	cmake .. && \
 	make && \
	make install

# Prepare git submodules
RUN cd src && \
git submodule init && \
git submodule update

# build ros package source
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO && \
    catkin build

RUN chmod +x /home/ros/src/roboy_ad/src/fixBag3D.py
RUN chmod +x /home/ros/src/roboy_ad/imu/imu_remapping.py
RUN chmod +x /home/ros/src/roboy_ad/src/lidar_remapping.py
RUN adduser root dialout
