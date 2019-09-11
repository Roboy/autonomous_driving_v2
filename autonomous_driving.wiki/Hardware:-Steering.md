# Steering angle sensor

<img align="right" src="https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/overview/angle_sensor.jpg" width="250">

To measure the current steering angle a Hall-effect sensor is needed.

For this, our cooperating Team Bike To Rickshaw installed a Hall-effect sensor on the bike. By measuring the magnetic field this sensor picks up the change in angle towards the neutral position. The angle is then published to the ROS network.

In order to achieve accurate steering by implementing a steering controller (see Software Overview section) a steering angle sensor is needed to get the current angle. 
