# 3D Camera: Intel Realsense D435

<img align="right" src="https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/overview/realsense.png" width="250">

The 3D/depth camera Intel Realsense D435 gives additional information about everything that is in front of the bike.

It provides regular 2D RGB video information as well as a depth stream (think of a picture but instead of color values behind every pixel you get the distance in cm) enabled by sending out an IR pattern and receiving the reflection up to a range of 10 m. Both of these streams can be combined to form a colored point cloud (XYZRGB).

Based on this information you could detect and localise road information (curbs, lane markings, etc.) and pedestrians or cars. Because of the limited time this semester those projects could not be realised (see section "Future Development and Ideas"). However, the base was set in order to use this information and to overlay objects detected and localized by the 3D camera on top of the map created by lidar SLAM.

Information of the extrinsic lidar-to-camera calibration can be found in the section "Software". Details about the Realsense can be found in a [separate](https://github.com/Roboy/autonomous_driving/wiki/Hardware%3A-Calibration) article in this wiki.

