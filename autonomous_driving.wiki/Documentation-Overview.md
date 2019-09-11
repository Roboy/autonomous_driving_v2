# Introduction
Teaching Roboy to autonomously drive the Roboy Bike is the next step in mobility for the cutest robot in town. The Autonomous Driving Team has equipped Roboy and his bike with the latest sensor technology ranging from Lidar to 3D cameras in order to SLAM their way around the campus of Technical University of Munich countless times allowing the finest global and local planning algorithms to guide Roboy on his smooth and well-PID-controlled path to his destination. 

If you want to be able to follow Roboy on his bike trip this article should give you an overview of the nuts and bolts. Feel free to dig deeper and gain a better understanding by following the links provided in this article. You should find everything from here. Everything related to the software can be found in the [main Github repository](https://github.com/Roboy/autonomous_driving) and a [separate Github repository](https://github.com/Roboy/autonomous_driving_src) only for the ROS nodes.The README gives you extensive instructions on how to install, run and tweak the software required for the autonomous ride.


# Who is this article for?
It is written for:

* future and present Roboyans
* anyone interested in robotics or autonomous driving
* adventurous ROS programmers
* in general tech savvy people
* you, thinking about building a humanoid soft robot that autonomously drives a rickshaw to [serve ice cream in the future](https://roboy.org/#roadmap)

to get a quick understanding about the project without the need for a deep-dive.

The goal of this article is that the reader understands within 5-10 minutes what this project is about and comprehends what the components of the system are and what they do. It does not require previous knowledge about autonomous driving technology. Since the Roboy project is based on ROS it is helpful to have some experience if you want to deep dive into the features but not required to understand this article. If you want to learn about ROS the official documentation is a great place to start.


**Quick note to all non-Roboyans:**

All of this work was done from October 2018 until March 2019 by students at Technical University of Munich as part of the [Roboy project](https://roboy.org/). Check out the "Team" paragraph at the bottom of the page and feel free to contact us. Please note that this article was also originally written for the internal documentation of Roboy in Confluence. We edited this article to put it here on the Github Wiki. While we try to make everything accessible that is required for the project you might come across a link to a very specific article that is not accessible for people outside of the Roboy Project. If you feel the need that this information is important for your understanding please contact us.

# Our Vision
As part of the Roboy project the team created a vision video at the beginning of the semester to outline the goal of the semester:

[![Vision Video](http://img.youtube.com/vi/N-CuiXY5ZXo/0.jpg)](http://www.youtube.com/watch?v=N-CuiXY5ZXo "Vision Video")


The vision of this project was simple: We want to enable Roboy to autonomously drive a passenger from the U-Bahn station Garching Forschungszentrum to the Roboy Lab. Teaching autonomous driving to Roboy also provides the base for next semester's goal where the Roboy Bike will be modified to an ice cream bike so Roboy can serve cold treats in summer.

Therefore we partnered with the Bike to Rickshaw team that is mainly responsible for building the Roboy Bike, attaching the required sensors and electronics and supported us with all the hardware problems. They also equipped the Roboy Bike with a "ghost mode" allowing the bike to operate without Roboy by steering with the same MyoMuscle units that are also in use by Roboy and accelerating with the motor that is built into the bike.

In parallel, the Driverless Driver team aims to provide Roboy with the means to pedal and steer the bike.

As an extra feature, team Tele Rickshaw wants to enable Roboy and his Roboy Bike to be remote controlled in VR.

# Semester Results
What we achieved (or currently at least planned until the end of the hackathon):
* mounted and integrated lidar
* mounted and integrated 3D camera 
* calibrated camera to lidar for extrinsic paramers
* created a map and localized on the map with SLAM (by using and tuning Google Cartographer) 
* wrote a custom global planner customized for rickshaw dynamics
* used local planner to navigate around obstacles
* tuned a PID controller to steer the bike
* dockerized parts of the packages that are hard to build
* demo in an indoor environent where the Roboy Bike autonomously moved to the navigation goal set in ROS

What could be done for Autonomous Driving 2.0 next semester:
* use 3D camera to detect road information (curbs, lines etc.), obstacles and pedestrians
* upgrade from 2D to 3D lidar
* 3D SLAM
* 3D map
* implement a multi dimensional occupancy grid map to signify preferred paths (prefer sidewalk over street) and adapt planner to this preference 

## Roboy Finals Video
At the end of the semester a final video was shot showing the features of the autonomous Roboy Bike:

[![Final Video](http://img.youtube.com/vi/KdW6pN4xFSY/0.jpg)](http://www.youtube.com/watch?v=KdW6pN4xFSY "Final Video")

## Demo Video
Quick demonstration showing the autonomous movement of the bike upon setting a navigation goal in ROS:

[![Demo Video](http://img.youtube.com/vi/NFuIMAr0DO4/0.jpg)](http://www.youtube.com/watch?v=NFuIMAr0DO4 "Demo Video")




# Hardware and Sensor Overview

In order to understand the core of our work, i.e. the software, one needs to first get an understanding of the hardware, sensors and what data they provide. This section should give all the necessary information for that.

## Lidar: Sick Scan LMS 1XX

<img align="left" src="https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/overview/sick_lidar.png" width="250">

The most important sensor for this project is the [SICK Scan LMS](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/lms1xx/lms111-10100/p/p109842) (LMS151-10100) 2-dimensional lidar. 

The sensor gives us 2D laserscan information (LaserScan or PointCloud2 ROS messages) which allows us to detect moving or non-moving objects @ 25 Hz or 50 Hz in an 270° angle with a maximum range of 50 m. It is currently mounted in the front of the bike slightly above ground level. 

We use it mainly for SLAM, i.e. the bike creates a 2D map of the environment where it is driving and it localises itself on that map (more info about that in section "Software"). Additionally, the ROS local planner can take the raw scanning data as an input for obstacle avoidance, i.e. deviating from the global path to navigate around an obstacle or pedestrian that is in the way. 

For more information about the Lidar itself, see  [SICK's webpage](https://www.sick.com/us/en/detection-and-ranging-solutions/2d-lidar-sensors/lms1xx/lms151-10100/p/p141840). Details about SLAM and other software applications can be found in the following section "Software".



## IMU: SBG Ellipse2-A
<img align="left" src="https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/overview/imu.png" width="250">

To augment our Lidar recordings, we are also using a SBG Ellipse2-A Inertia Measurement Unit. 

Ellipse2-A is a cost-effective high performance Attitude and Heading Reference System (AHRS). Factory calibrated from -40 to 85°C, this robust inertial motion sensor provides Roll, Pitch, Heading, and Heave data. The accuracies are 0.1° Roll and Pitch over 360° and 1° Heading (Internal Magnetometers).

Due to the setting of our environment and robot, we encounter situations where the Lidar is oriented in a way which makes it more or less blind. This leads to the SLAM algorithm wandering off in some random direction. We use the IMU to add additional information about our robot's dynamics and improve our Mapping.

For more information about the IMU itself, see [SBG System's website](https://www.sbg-systems.com/products/ellipse-2-series/). Details about SLAM and other software applications can be found in the following section "Software".



## 3D Camera: Intel Realsense D435

<img align="left" src="https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/overview/realsense.png" width="250">

The 3D/depth camera Intel Realsense D435 gives additional information about everything that is in front of the bike.

It provides regular 2D RGB video information as well as a depth stream (think of a picture but instead of color values behind every pixel you get the distance in cm) enabled by sending out an IR pattern and receiving the reflection up to a range of 10 m. Both of these streams can be combined to form a colored point cloud (XYZRGB).

Based on this information you could detect and localise road information (curbs, lane markings, etc.) and pedestrians or cars. Because of the limited time this semester those projects could not be realised (see section "Future Development and Ideas"). However, the base was set in order to use this information and to overlay objects detected and localized by the 3D camera on top of the map created by lidar SLAM.

Information of the extrinsic lidar-to-camera calibration can be found in the section "Software". Details about the Realsense can be found in a [separate](https://github.com/Roboy/autonomous_driving/wiki/Hardware%3A-Calibration) article in this wiki.



## Steering angle sensor

<img align="left" src="https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/overview/angle_sensor.jpg" width="250">

To measure the current steering angle a Hall-effect sensor is needed.

For this, our cooperating Team Bike To Rickshaw installed a Hall-effect sensor on the bike. By measuring the magnetic field this sensor picks up the change in angle towards the neutral position. The angle is then published to the ROS network.

In order to achieve accurate steering by implementing a steering controller (see Software Overview section) a steering angle sensor is needed to get the current angle. 


## Myo-muscles for steering

<img align="left" src="https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/overview/myo-muscle.png" width="250">


To keep it very simple, the consist out of a winch motor that can pull a string. This mimics the way human muscles contract and by using two muscles working in opposite directions (antagonist muscles, compare biceps and triceps) back-and-forth movement is possible.

By placing one Myo-muscles on each side the Roboy Bike can steer itself without the help of Roboy. Together with the angle sensor and the PID steering controller ghost mode steering (steering without the help of Roboy) can be achieved.





# Software Overview
Now to the core of the project: the Software. This section should give you a good overview of the different software modules. For details, please follow the provided links to other articles.

For instructions on how to install, run and tweak the code, start with the READMEs of our repositories: the [ROS repository](https://github.com/Roboy/autonomous_driving_src) (containing all required ROS nodes as either git submodules or own implementations from the team) and the [main repository](https://github.com/Roboy/autonomous_driving) (everything else required to get the Roboy Bike running).


## Localization and Mapping: Lidar SLAM with Google cartographer

<img align="left" src="https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/overview/cartographer.png" width="250">

The first steps in our software is to create a map and localize the bike on that map by using the [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) tool [Google Cartographer](https://github.com/googlecartographer/cartographer).

To enable autonomous driving Roboy needs to get an understanding of his environment, i.e. the static and dynamic objects around him. This starts with building a map that describes your static environment. Roboy then uses it to localize himself and other moving objects (pedestrians, cars) on the map and plan the path to his desired destination accordingly. We use the laserscan data from the lidar sensor to record data while we are driving around to build a map. Once the whole map of an area is built Roboy can then at a later stage compare the current laserscan to the previously built map to figure out at which position he currently is. The combination of building a map and localizing yourself on the map is called SLAM (Simultaneous Localization And Mapping).

We use the [ROS implementation](https://github.com/googlecartographer/cartographer_ros) of Google Cartographer for that. For general information about the tool check out the [Cartographer ROS Read the Docs site](https://google-cartographer-ros.readthedocs.io/en/latest/).




## Path planning

With the map constructed by SLAM as a basis Roboy can plan its path on it. In general, path planning is divided into global and local planning. In a nut shell, this means that the global plan finds the ideal path on a static map. When Roboy then actually drives on the path it constantly monitors moving objects (pedestrians, cars, etc.) and adapts the global plan if needed. This is called the local plan and is updated every few seconds.

The ROS navigation stack provides an all-in-one solution to solve both of these problems. However, the ROS global planner does not consider some restrictions (e.g. that the bike can not turn in place and therefore we can't plan a u-turn. This is why our code has a custom global planner but still uses the local planner from the ROS navigation stack.



### Global planning

<img align="left" src="https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/overview/global_planner_4_3.gif" width="250">

Global planning finds the ideal path on a static map that was generated by the SLAM in the previous steps, i.e. this planning step does not consider dynamic obstacles such as pedestrians.

However, the global planner of the [ROS navigation stack](http://wiki.ros.org/navigation) was not applicable for our case because it assumes that the bike can turn in place which is not possible. Instead the custom implementation [astar_ackermann_planner](https://github.com/melkonyan/astar_ackermann_planner/tree/master) was made that considers the fact that the bike has a minimum turning radius.



### Local planning

<img align="left" src="https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/overview/local_planner_4_3.gif" width="250">

Local planning allows obstacle avoidance (e.g. driving around pedestrians) by considering dynamic obstacles such that the global path is adapted to the local path.

Here we use the local planner provided by the ROS navigation stack. It subscribes to the raw laserscan data provided by the lidar and plans a path around an obstacle in case it is picked up by the lidar.

For more information read the article about the [ROS navigation stack](http://wiki.ros.org/navigation) in general or the [base_local_planner](http://wiki.ros.org/base_local_planner?distro=melodic). 



## Extrinsic calibration Lidar-to-Camera

<img align="left" src="https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/overview/calibration.png" width="250">

To jointly use information gained from the lidar and camera the extrinsic parameters (relative rotation and translation) between both of them needs to be calculated.

In the future, we want to detect more information in our environment like static road information (detect and localize curbs, lane marking, street signs) or dynamically moving objects (cars, pedestrians). We plan to detect these objects with Intel Realsense camera but in order to use them alongside with the map information (for path planning etc.) we need to bring them to the same coordinate system. The process of extrinsic calibration allows us to find out the relative position and translation by recording several samples of a checkered calibration board. To the current state no object detection features where developed or integrated in this project but the foundation was built for next semester.

Since the calibration procedure is a delicate process where small errors are crucial we create a [step-by-step tutorial](https://github.com/Roboy/autonomous_driving/wiki/Hardware%3A-Calibration) in this wiki that explains the correct usage of the MATLAB tools for calibration and the sample recording. Please find all the details in this tutorial.





## Dockerization

<img align="left" src="https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/overview/docker.png" width="250">

As we experienced a lot of problems building certain ROS nodes on different system we created a Docker container that virtualizes and reliably builds the most critical parts of the code.

In doing this we do not add any new features to the project but we want to make the building process more convenient by removing time consuming dependency fixing. Also we configured the Docker containers in a way that ROS messages can be shared between the host and the Docker containers. We decided against dockerizing all the ROS nodes that directly interface with hardware since this could present problems and these nodes are not the ones that are difficult to build.

If you are not familiar with Docker that [Docker docs](https://docs.docker.com/) are a great way to start. For instructions on how to get the Docker containers in our project running we created a separate README in the 'dockers' [folder](https://github.com/Roboy/autonomous_driving/tree/master/dockers) of our repository.

# The team 
<img src="https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/overview/team2.jpeg" width="700">

[Lennart Haller](https://github.com/lennarthaller) (Tech Lead), [Jonas Kerber](https://github.com/jonas-kerber), [Alex Melkonyan](https://github.com/melkonyan), [Christoph Killing](https://github.com/christqoh)