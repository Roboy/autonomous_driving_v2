# SBG Ellipse2-A
<img align="right" src="https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/overview/imu.png" width="250">

To augment our Lidar recordings, we are also using a SBG Ellipse2-A Inertia Measurement Unit. 

Ellipse2-A is a cost-effective high performance Attitude and Heading Reference System (AHRS). Factory calibrated from -40 to 85°C, this robust inertial motion sensor provides Roll, Pitch, Heading, and Heave data. The accuracies are 0.1° Roll and Pitch over 360° and 1° Heading (Internal Magnetometers).

Due to the setting of our environment and robot, we encounter situations where the Lidar is oriented in a way which makes it more or less blind. This leads to the SLAM algorithm wandering off in some random direction. We use the IMU to add additional information about our robot's dynamics and improve our Mapping.

For more information about the IMU itself, see [SBG System's website](https://www.sbg-systems.com/products/ellipse-2-series/). Details about SLAM and other software applications can be found in the following section "Software".