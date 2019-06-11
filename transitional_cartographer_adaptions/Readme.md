# Migration to 3D Lidar

## Preprocessing the data

As the recorded bag files do not work with Cartographer out of the box, a modification is necessary. For this purpose, the *fixBag3D.py* script has been created. <br>
After downloading it, copying it to the *roboy_ad* package

    cp ~/Downloads/fixBag3D.py ~/catkin_ws/src/roboy_ad/src/fixBag3D.py

and making the file executable

    chmod +x ~/catkin_ws/src/roboy_ad/src/fixBag3D.py
    
it can be run simply using this command

    rosrun roboy_ad fixBag3D.py <PATH/TO/BAGFILE> --ref_topic <TOPIC>

where the reference topic can be any topic in the bag (e.g. */points2*). <br>
The resulting bagfile is saved in the same directory as the original one and named `<oldname>_sort.bag`.

Now, this resulting file should not show any warnings or issues when checked with `cartographer_rosbag_validate`.

## SLAM

The preprocessed data can then be used for SLAM. Four files have been added to the Cartographer ROS in order to make 3D Lidar SLAM possible:

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
