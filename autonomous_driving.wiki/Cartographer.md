# Google Cartographer



To enable autonomous driving, Roboy needs to get an understanding of his environment, i.e. the static and dynamic objects around him. To achieve this, we start by creating a map that describes our static environment. Once a map of an area is built, we are then able to compare the current laserscan to the previously built map to figure out at which position we currently are in. After being localized, we utilize the map to plan the path to the desired destination. 


<img align="right" src="https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/overview/cartographer.png" width="250">

We use the laserscan range measurements from the Lidar to construct a map while we are driving around. [Google Cartographer](https://github.com/googlecartographer/cartographer_ros) is implementing this so-called `Simultaneous Localization and Mapping` algorithm, short [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping). It also provides a [ROS implementation](https://github.com/googlecartographer/cartographer_ros). For general information about the tool check out the [Cartographer ROS Read the Docs site](https://google-cartographer-ros.readthedocs.io/en/latest/).

We are using Cartographer ROS release `1.0`. If you use an other release, syntax will differ (compare [the cartographer documentation](https://google-cartographer.readthedocs.io/en/latest/configuration.html)). This is the reason why everything we did happend on the `roboy` branch of [our own fork](https://github.com/Roboy/cartographer_ros/tree/roboy).

To start, build the Cartographer as stated in the [README](https://github.com/Roboy/cartographer_ros/tree/roboy#building-cartographer). The following documentation is purely supplemental to the tuning methodology suggested in [Cartographer ROS Documentation](https://google-cartographer-ros.readthedocs.io/en/latest/). However, if you have never worked with cartorapher before, this will be your most straight-forward introduction.