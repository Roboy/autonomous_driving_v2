# Migration to 3D Lidar

Four files have been added to the Cartographer ROS in order to make 3D Lidar SLAM possible:

 - robot description file (*.urdf)
 - cartographer configuration file (*.lua)
 - rviz  configuration file (*.rviz)
 - launch file (*.launch)

In order to get it to work, the files need to be copied into the right directories. The description assumes that the Cartographer ROS package has already been built according to [last semester's description](https://github.com/Roboy/cartographer_ros/tree/f49dbbb6f9260edfc9bd976402559b84d626fa31). 

So, download the files from this Github folder into your Downloads folder and copy them into the right directories:

    cp ~/Downloads/testrig.urdf ~/catkin_ws/src/cartographer_ros/cartographer_ros/urdf/testrig.urdf
    cp ~/Downloads/testrig.lua ~/catkin_ws/src/cartographer_ros/cartographer_ros/configuration_files/testrig.lua
    cp ~/Downloads/demo_3d.rviz ~/catkin_ws/src/cartographer_ros/cartographer_ros/configuration_files/demo_3d.rviz
    cp ~/Downloads/testrig3D.launch ~/catkin_ws/src/cartographer_ros/cartographer_ros/launch/testrig3D.launch

Afterwards, SLAM can be run using the following command (replacing `<PATH/TO/BAGFILE>` with the actual path to where the bagfile is stored on your computer):
    
    roslaunch cartographer_ros testrig3D.launch bag_filenames:=<PATH/TO/BAGFILE>
