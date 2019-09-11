# II. Recording Data

The easy answer: `rosbag record /scan /imu`. 

However, what could possibly go wrong?

## II.1. Understand your hardware
Two examples about hardware limitations:

- The [Lidar we used](https://github.com/Roboy/autonomous_driving/wiki/Hardware%3A-Lidar) had a nominal range of 50m. However, we experienced that it was not able to see a matt black door 5m away from it.
- An IMU measures accelerations. If you give the bike a quick ride over very bumpy ground your recordings will have so much noise that it renders all IMU data unusable.

Keeping those two examples in mind, try to estimate your expected sensor performance in your environment.

## II.2. Understand your setup
In our case, the Lidar had to be mounted in a way such that its 2D plane of sight was approximately 15cm above ground. Like this, the tiniest tilts in the pavement would lead to the laser beam hitting the ground a few meters away. Even more, to decouple the Lidar from overall bike vibrations, we had to spring-mount it. This lead to small tilting around every axis whilst in operation and posed a significant drawback of our setup. 

## II.3. Understand your environment
This mostly applies for outdoor scenarios. Walk along the paths you want to map and drive on later and try to set yourself into the Lidar's perspective. What will it see? Is the pavement tilted? Is the ground higher right next to the path? Will the Lidar after all see e.g. a building several meters away or will the laser scan just detect the grass in front of it? This is a good way to estimate your setups capabilities. Take all of those into consideration and adapt the Lidar's position if necessary. 

Going back to hardware limitations, consider your realistic lidar range. Due to tilting and visibility conditions this might significantly differ from capabilities stated. Consequently, you might need to get rather close to obstacles to actually detect them in a way Cartographer will recognize them again later. Also, what paving is used? What will the IMU record? Having a rough ground might render the IMU useless due to noise.

Furthermore, consider your environment dynamics. Are there plenty of humans walking about? Cars or Trucks driving by? Cartographer can deal with that. However, humans that walk, stop to watch you and then suddenly walk again can be understood as features. So in case you have a feature-poor environment (like outdoors in grassland with just a few trees) make sure this doesn't happen as it will mess up your recording.