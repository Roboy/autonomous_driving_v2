FROM ros:kinetic-robot

# Set working directory
WORKDIR /home

# Get autonomous driving src folder from GIT
RUN git clone -b devel https://github.com/Roboy/autonomous_driving_v2.git  ./src

RUN cp ./src/package_requirements.sh ./package_requirements.sh

# Set up Kinetic keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install required packages
RUN chmod +x ./package_requirements.sh && \
 ./package_requirements.sh

RUN apt install python-pip -y && \
  pip install simple-pid --user

# Prepare git submodules
RUN cd src && \
git submodule init && \
git submodule update

# remove cartographer as it is run in the other docker container anyways
RUN rm -r ./src/cartographer_ros

# build ros package source
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO && \
    catkin build
