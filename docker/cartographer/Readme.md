# Docker Adaptions for 3D Lidar

Five files have been added to the Cartographer ROS in order to make 3D Lidar SLAM possible inside the docker container:

 - robot description file (*.urdf)
 - cartographer configuration file (*.lua)
 - rviz  configuration file (*.rviz)
 - launch file for cartographer (*.launch)
 - launch file for rviz (*.launch)

In order to get it to work, the *cartographer_devel* docker container hast to be built according to last [semester's documentation](https://github.com/Roboy/autonomous_driving/tree/master/dockers/cartographer_devel). Afterwards the files can be downloaded and copied into the container.
So, download the files from this folder into your Downloads folder and copy them into the right directories.
Three files are copied into the docker container:

    sudo docker cp ~/Downloads/testrig.urdf cartographer_devel:/home/ros/src/cartographer_ros/cartographer_ros/urdf/testrig.urdf
    sudo docker cp ~/Downloads/testrig.lua cartographer_devel:/home/ros/src/cartographer_ros/cartographer_ros/configuration_files/testrig.lua
    sudo docker cp ~/Downloads/testrig3D_indoor_offline.launch cartographer_devel:/home/ros/src/cartographer_ros/cartographer_ros/launch/testrig3D_indoor_offline.launch

And two files are copied into the roboy_ad package on the machine where rviz should run:

    sudo docker cp ~/Downloads/rviz_cartographer3d.launch ~catkin_ws/src/roboy_ad/launch/rviz_cartographer3d.launch
    sudo docker cp ~/Downloads/demo_3d.rviz ~catkin_ws/src/roboy_ad/rviz/demo_3d.rviz

In order to start SLAM, roscore needs to run on host

    roscore
 
rviz needs to run either on the remote PC or also on host and is started with this command

    roslaunch roboy_ad rviz_cartographer3d.launch
    
After starting the container and enter the docker shell

    sudo docker start cartographer_devel
    sudo docker exec -it cartographer_devel bash
    source devel/setup.bash

the SLAM can be run with the following command (replacing `<PATH/TO/BAGFILE>` with the actual path to where the bagfile is stored in the container):

    roslaunch cartographer_ros testrig3D_indoor_offline.launch bag_filenames:=<PATH/TO/BAGFILE>


