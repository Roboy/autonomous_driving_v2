# III. Creating a map

![SLAM.gif](https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/cartographer/SLAM.gif)

## III.1. Basic Cartographer Configuration
First, you need to define the relations between your robot and your sensors in the according [.urdf file](https://github.com/Roboy/cartographer_ros/blob/roboy/cartographer_ros/urdf/roboy.urdf). Note that the IMU must not have an offset towards the base_link frame, which is the origin of your robot. 

## III.2. Tune Scan Matching
This vastly depends on your robots local dynamics. Is it subject to non-holonomic constraints? Will it go very fast? Based on such considerations, you can set search windows to find scan correlations in as well as scan matching weights. Search windows are rather intuitive, considering the weights however, how easy do you want the scan matching to shift scans around? Remember: if you set a too high weight, your robot will be stuck in one place with the environment "passing by". But setting it too low the scan matcher is able to place scans just somewhere within the search window.

In the end, you will get the relative motion between scans. This information is accumulated in so called `submaps` consisting of a certain number of scan messages. 

When tuning, it is vital that you avoid any broken submaps as cartographer will most likely not recover from it.  Each submap has to have clear indications on free and occupied space with sharp obstacle boundaries. 

In our case, parameters are set in [rickshaw_local.lua](https://github.com/Roboy/cartographer_ros/blob/roboy/cartographer_ros/configuration_files/rickshaw_local.lua). The Rickshaw will not suddenly rotate or show a very dynamic acceleration behaviour. Search windows and weights have been tuned accordingly to that. 

## III.3. Tuning Scan Matching with IMU
Due to the setting of our environment and robot, we can encounter situations where the Lidar is oriented in a way which makes it more or less blind. This leads to the SLAM algorithm wandering off in some random direction. We use the IMU to add additional information about our robot's dynamics and improve our Mapping.

After you tuned the scan matching to get decent submaps out of most situations, you can improve the results by [including data from an IMU](https://github.com/Roboy/cartographer_ros/blob/67c0b9c8d8315b6feeab01aa5731421cb2f5dfdd/cartographer_ros/configuration_files/roboy_mw.lua#L5-L13). 
If using an IMU, set tracking frame to `imu`, otherwise, you can use `base_link`. Also, do not use `use_online_correlative_scan_matching` if you set `use_imu_data`. 

## III.4. Tuning global optimization
This depends on your environment. Do submaps contain distinct landmarks? Are you able to see certain landmarks from several submaps? Does your environment contain unique global landmarks? The goal to achieve is called `loop closure`. This means, cartographer will, through global optimization, recognize a previously-visited location and updating the map accordingly. This can be a problem because model or algorithm errors might assign wrong constraints or simply miss the previously seen location.

Cartographer computes a sort of measurement similarity and matches those locations with a certain ease as set in `loop_closure_[...]_weight` when a match is detected. 

The general tuning procedure is the same as for the local case. Adapt search windows and weights to your and the environments needs. In our case, parameters for the Mechanical Engineering building are set in [roboy_mw.lua](https://github.com/Roboy/cartographer_ros/blob/roboy/cartographer_ros/configuration_files/roboy_mw.lua). 

## III.5. Save a map

When being run with the `offline`-node set, Cartographer will save a `.pbstream`-file next to the loaded `.bag`-file upon finishing the optimization. 
If cartographer is run in online mode, follow the instructions [here](https://github.com/Roboy/cartographer_ros/tree/roboy#saving) to save a `.pbstream`-file.

When running pure localization as described in the next chapter, Cartographer will load a specified `.pbstream`-file as its map localization is being performed on. It will also publish that map on the `/map` topic. 

If you prefere or need to convert a `.pbstream`-file into a `.yaml` and `.pgm` pair, i.e. for manually publishing through `map-server`, see [here](https://github.com/Roboy/cartographer_ros/tree/roboy#publishing). This comes in very handy when you need to [make certain adaptions to the map](https://github.com/Roboy/cartographer_ros/blob/roboy/README.rst#editing) such as additional obstacles which might be required i.e. by the global planner. 