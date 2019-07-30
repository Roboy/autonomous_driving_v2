FROM ros:kinetic-robot

# Set working directory
WORKDIR /home

# Get autonomous driving src folder from GIT
RUN git clone -b devel https://github.com/Roboy/autonomous_driving_v2.git  ./src

# Set up Kinetic keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install required packages
RUN chmod +x src/package_requirements.sh
RUN bash src/package_requirements.sh

RUN apt install python-pip -y
RUN pip install -r src/pip_requirements.txt --user

# Prepare git submodules
RUN cd src && \
git submodule init && \
git submodule update

# build ros package source
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO && \
    catkin build
