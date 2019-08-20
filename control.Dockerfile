FROM ros:kinetic-robot

# Set working directory
WORKDIR /home

# Get autonomous driving src folder from GIT
RUN git clone -b devel_control https://github.com/Roboy/autonomous_driving_v2.git  ./src

# Set up Kinetic keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install required packages
RUN chmod +x src/package_requirements.sh
RUN bash src/package_requirements.sh

RUN apt update && \
    apt install -y python-pip vim

RUN pip install -r src/pip_requirements.txt --user

# Prepare git submodules
RUN cd src && \
git submodule init && \
git submodule update

# build ros package source
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO && \
    catkin build

RUN touch /root/.bashrc && \
    echo 'source /home/ros/devel/setup.bash' >> /root/.bashrc && \
    echo 'export ROS_MASTER_URI=http://192.168.0.105:11311' >> /root/.bashrc && \
    echo 'export ROS_HOSTNAME=192.168.0.105' >> /root/.bashrc && \
    echo 'export ROS_IP=192.168.0.105' >> /root/.bashrc
