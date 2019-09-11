# Sick Scan LMS 1XX

<img align="right" src="https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/overview/sick_lidar.png" width="250">

The most important sensor for this project is the [SICK Scan LMS](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/lms1xx/lms111-10100/p/p109842) (LMS151-10100) 2-dimensional lidar. 

The sensor gives us 2D laserscan information (LaserScan or PointCloud2 ROS messages) which allows us to detect moving or non-moving objects @ 25 Hz or 50 Hz in an 270° angle with a maximum range of 50 m. It is currently mounted in the front of the bike slightly above ground level. 

We use it mainly for SLAM, i.e. the bike creates a 2D map of the environment where it is driving and it localises itself on that map (more info about that in section "Software"). Additionally, the ROS local planner can take the raw scanning data as an input for obstacle avoidance, i.e. deviating from the global path to navigate around an obstacle or pedestrian that is in the way. 

For more information about the Lidar itself, see  [SICK's webpage](https://www.sick.com/us/en/detection-and-ranging-solutions/2d-lidar-sensors/lms1xx/lms151-10100/p/p141840). Details about SLAM and other software applications can be found in the following section "Software".