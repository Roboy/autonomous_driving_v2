# IV. Localization

![pure_localization.gif](https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/cartographer/pure_localization.gif)

## IV.1 Map data
It is required that you have a clear map of your environment in the `.pbstream` file format created as stated in the [mapping section](https://github.com/Roboy/autonomous_driving/wiki/Cartographer%3A-III_Mapping).

Hint: For quick localization when demoing, begin the trajectory you use for mapping in the same place your demo trajectory will begin. This eases convergence of pure localization. 

## IV.2 Tuning
For tuning global localization, you need to consider two scenarios. 

First, where on the map are you? A new trajectory will always begin in the maps origin, so in the point where you started your trajectory when recording the map. To localize yourself globally, a certain subset of maps from the whole map is compared against the submaps you just recorded. If the resemblance is above a certain `global_localization_min_score`, the robot will be localized in some place on the map and jump there. 

Second, once localized on the map you do not want to drive "blindly" but check which of the submaps of the map are a good match to what you currently see. In the gif above, this is depicted through orange and yellow lines. Those constraints can be tuned through setting global constraint `min_score` and search window. 

Parameters for the [demo](https://github.com/Roboy/autonomous_driving/wiki/Examples) are set in the according [localization file](https://github.com/Roboy/cartographer_ros/blob/roboy/cartographer_ros/configuration_files/roboy_mw_localization.lua).

More on pure localization can be found [here](https://github.com/Roboy/cartographer_ros/tree/roboy#pure-localization). A small tuning hint: The angular search window will allow rotations vs the original map, so you can keep that bit small. 