# Overview

The extrinsic calibration is needed to get the relative translation and rotation between camera and lidar. This enables us to overlay information gathered by the camera (curbs etc.) on the map that is provided by lidar slam. The calibration is done with the implemention of this paper. It is a 5-step process.

1. [radlocc_collect](https://github.com/bernardomig/radlocc_calibration) (ROS): This is a rosnode of a package that allows to record sample shots (camera + lidar data) that are used for calibration.
2. [RADOCC](https://github.com/SubMishMar/RADOCCToolbox) (MATLAB, [official documentation](http://www-personal.acfr.usyd.edu.au/akas9185/AutoCalib/AutoCamDoc/index.html)): Gives us the camera intrinsic parameters out of the recorded samples. The camera intrinsics are needed for getting the extrinsics in step 3
3. [RADLOCC](https://github.com/bernardomig/radlocc_calibration) (MATLAB, [official documentation](http://www-personal.acfr.usyd.edu.au/akas9185/AutoCalib/AutoLaserCamDoc/index.html)): Calculates extrinsics. It's a little difficult to use that tool. More in detail below.
4. [radlocc_convert](https://github.com/bernardomig/radlocc_calibration) (ROS): This converts the MATLAB results to a format usable in ROS
5. static_transform_publisher (ROS): Output the results of the calibration as tf messages.


# Tutorial
## Example dataset

The following [zip file](https://github.com/Roboy/autonomous_driving/raw/wiki-material/wiki-material/calibration/roboy_calibration_example_dataset.zip) contains a dataset of pictures that were recorded at Roboy and that were used in the creation of this tutorial.

## Comment for recording datasets for calibration
The recording of the samples should be done in an open space that allows to move around a lot with the board (to the sides and different distances to camera). For all the Roboyans: ground floor at Roboy is a good idea.

To create a variety of board positions do a combination things:

1. Move left and right
2. Move forward and backwards
3. Rotate board left and right
4. Rotate board up and down
5. Always make sure that all squares are visible (also don't occlude squares with your fingers)

As written in the [paper]() the calibration error converges after 20 samples. I would recommend to take around 25-30 sample shots in step 1 because some of the samples might not be detected by RADLOCC.

IMPORTANT: record in the same camera resolution that you will use later


Bad example | Good example
--- | ---
<img src= "https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/calibration/recording_badexample_photo.png" width="600"> | <img src= "https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/calibration/recording_goodexample_photo.png" width="600">
The background has a lot of surfaces similar to a board (e.g. TV) that are also close to the board, so the Auto Select in RADLOCC mistakens them for the checker board. | Here we have a lot of open space and the board is clearly identifiable. Auto Select should be able to identify the board in most of the pictures and if you manually select the failes scans this should easily be possible.
<img src= "https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/calibration/recording_badexample_plot.png" width="400"> | <img src= "https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/calibration/recording_goodexample_plot.png" width="400">
MATLAB plot of board positions across all samples (RADLOCC): As you can see the environment only allowed to have samples in a tight area with samples having not a lot of variation.  | As you can see the samples are spread to a bigger area. The increased variation can then lead to more accurate results.


Another important thing is to only record the calibration board in either landscape or portrait orientation. Decide for one (here: portrait). Make sure to never rotate the board for more than 45 degrees, otherwise the tool produces bad results. 



Okay | Not okay
--- | ---
<img src= "https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/calibration/recording_orientation_okay.png" width="400"> | <img src= "https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/calibration/recording_orientation_notokay.png" width="400">


## Calibration process
### 1. Sample collection (radlocc_collect)
#### What you need
* 1 or 2 people
* a checker-board
* a ruler to measure the size of the squares on board (The one I used has squares 108x108 mm)
* an open space
* SICK lidar
* camera
* switch
* power-supply for lidar (should be around 16V)
* a setup where lidar and camera don't change their relative position and rotation
* ROS

#### Process description
If you have 2 people:
1. Read **"Some notes about environment if sample recording"** first
2. Setup lidar and camera
3. Run camera nodes: _**"roslaunch realsense2_camera rs_camera.launch"**_ (change .launch file if you want to change resolution or FPS, always choose same resolution as you will use later)
4. Run lidar nodes: _**"roslaunch sick_scan sick_lms_1xx.launch -use-binary-protocol"**_
5. Verify that both data streams are correct: _**"rostopic echo /scan"**_ and _**"rostopic echo /camera/color/image_raw"**_ → if your terminal is full of numbers you are correct
6. Create directory for your samples: _**"mkdir sample_dir"**_
7. Start radlocc_collect: _**"rosrun radlocc_calibration radlocc_collect laserscan:=/scan image:=/camera/color/image_raw _output_dir:=sample_dir"**_
8. Follow the information in terminal. Press enter for every sample. If you have enough samples type "q" and enter.
Person A moves to different positions with board in hand 
Person B presses enter for every shot (25-30 shots recommended)
9. Everything you need for RADOCC and RADLOCC is stored in sample_dir

If you are alone:
1. Read **"Some notes about environment if sample recording"** first
2. Setup lidar and camera
3. Run camera nodes: _**"roslaunch realsense2_camera rs_camera.launch"**_ (change .launch file if you want to change resolution or FPS, always choose same resolution as you will use later)
4. Run lidar nodes: _**"roslaunch sick_scan sick_lms_1xx.launch -use-binary-protocol"**_
5. Verify that both data streams are correct: _**"rostopic echo /scan"**_ and _**"rostopic echo /camera/color/image_raw"**_ → if your terminal is full of numbers you are correct
6. Open **rviz**: Panels > Displays, Add > By topic: /camera/color/image_raw → This shows you the camera stream
7. Start recording a rosbag: _**"rosbag record /scan /camera/color/image_raw -O samples.bag" **_(make sure to have enough space on your system (2-3 GB))
8. Move around with the board in hand. Record different positions and rotations while constantly checking in **rviz** if the board is fully in the picture. **Stand still** for a second whenever you would like to take a shot (25-30 shots recommended).
9. Stop recording rosbag: **Ctrl+C** in terminal window
10. Start radlocc_collect: _**"rosrun radlocc_calibration radlocc_collect laserscan:=/scan image:=/camera/color/image_raw _output_dir:=sample_dir"**_
11. Replay rosbag: _**"rosbag play samples.bag"**_
12. In **radlocc_collect**: press enter whenever you are standing still. Type "q" and enter once you're done
13. Everything you need for RADOCC and RADLOCC is stored in sample_dir
14. remove Rosbag: _**"rm samples.bag"**_ (you don't want it to take up so much space on your system)

After you are finished transfer "sample_dir" to whatever system your MATLAB is on.


###  2. Camera intrinsics (RADOCC)
#### What you need
* sample folder created in 1
* MATLAB (system does not need to be Linux and you don't need ROS (I did step 1 on Linux VM and steps 2&3 on MacOS host))

#### Process description
Official documentation here, but since I am a MATLAB noob myself, I decided to make a step-by-step tutorial for dummies to save other people some time:

1. Type _**clear**_ into the MATLAB console, to clear the workspace variables. This could prevent potential errors if you did an unsuccessful calibration attempt before.
2. **Copy** the "sample_dir" into the RADOCC directory  (here called "RADOCCToolbox")
3. On MATLAB terminal, navigate to "sample_dir": _**"cd RADOCCToolbox/sample_dir"**_
4. Type _**"calib"**_ and enter. A GUI opens.
5. On GUI, press **"Read images"** and follow instructions in terminal. If no images were found check if you're in "sample_dir" (Can use "pwd" to check)
6. On GUI, press **"Extract Grid corners"** and follow instructions on console. It might happen that it fails to read some images. If you want to keep calibration error small, make sure that at least 20 images are detected correctly. (Remember that our calibration board has 108x108mm)
7. On Gui, press **"Calibration"**. This saves calibration results in workspace
8. (Optional) Press **"Show Extrinsics"** to check the plausibility of results
9. On GUI, press **save**. 
10. **Move** the just created file **"Calib_Results.mat"** to the RADLOCC directory (watchout: that's the other directory) (here called RADLOCC_dir)

If something goes wrong in MATLAB and you need to redo it, always type **"clear"** first. This clears the MATLAB Workspace variable and can be helpful.

### 3. Camera-Lidar Extrinsics (RADLOCC)
#### What you need
sample folder created in 1
MATLAB (system does not need to be Linux and you don't need ROS (I did steps 1&4 on Linux VM and steps 2&3 on MacOS host))
 "Calib_Results.mat" from 2
patience;)
#### Process description
Official documentation here, but since I am a MATLAB noob myself, I decided to make a step-by-step tutorial for dummies to save other people some time:

1. **Copy** the "sample_dir" into the RADLOCC directory  (here called "RADLOCC")
2. Make sure you copied **"Calib_Results.mat"** into  "RADLOCC" (see 2 RADOCC tutorial)
3. Make sure that there are no other folders with recorded samples (e.g. folder that contain the image PNGs, "laser.txt" and "image_stamps.txt"). Delete them to prevent errors.
4. Type _**clear**_ into the MATLAB console, to clear the workspace variables. This could prevent potential errors if you did an unsuccessful calibration attempt before.
5. On MATLAB folder browser, right-click "RADLOCC" and **"Add to Path: Selected folders and subfolders"**
6. On MATLAB terminal, navigate to "RADLOCC_dir": _**"cd RADLOCC_dir"**_
7. Type **"RADLOCC"** and enter. A GUI opens.
8. On GUI, press **"Read Data"** and follow instructions. If it says "File does not exist", make sure that you spelled it out correctly and the files are in the MATLAB path.
9. On GUI, press **"Auto select"**. (Now it gets a little tricky. I will now describe how I personally reached good results. Can differ according to your recorded samples)
10. Enter **default values** first
11. When it says **"Please validate the selected board lines in the following scans"** change the number of the selection if necessary.
12. Answer **"y"** to **"Would you like to manually select the failed scans?"** and choose select appropriate regions in opened window.
13. Inspect the figure of the board positions. If they don't look plausible something went wrong.
14. Number of planes selected is printed on console. For the calibration error to converge the number should be **20**. If you are far off you should consider recording new samples with radlocc_collect.
15. On GUI, press **"Calibrate"**. 
16. On GUI, press **"Laser onto Image"** and inspect a few image numbers if they look plausible.
17. On GUI, press **save**.
18. **Move** the just created file **"LaserCalibResults.mat"** to your ROS system.

If something goes wrong in MATLAB and you need to redo it, always type "clear" first. This clears the MATLAB Workspace and can be helpful.

Example of Laser onto image (I don't know what the big ellipses should mean, probably a bug):

![Example: Laser onto image](https://github.com/Roboy/autonomous_driving/blob/wiki-material/wiki-material/calibration/laserontoimage_example.png)


### 4. Convert calibration results to ROS format (radlocc_convert)
#### What you need
* ROS system
* "LaserCalibResults.mat" from 3

#### Process description
1. **Move "LaserCalibResults.mat"** from step 3 to your ROS system
2. _**"rosrun radlocc_calibration radlocc_convert --format yaml FOLDER_CONTAINING_LASER_CALIB"**_
3. find translation (xyz) and rotation (rpy) on terminal output (Note that if your try it with the same dataset the results might vary slightly depending on the manual selection of the failed scans)

xyz: [0.60065, 1.69362, 2.65144]

rpy: [ 2.45415, -1.23914, -0.95749]


**Important:** The axes in ROS coordinate system are different for bodies and for optical systems (read [REP 103](http://www.ros.org/reps/rep-0103.html)):
* x: (bodies: forward, optical: right)
* y: (bodies: left, optical: down)
* z: (bodies: up, optical: forward) 

This means that in our calibration the lidar in relation to the camera (optical frame) is 2.65m in front of the camera, 1.69m below the camera and 0.6m to the right of the camera.

Mapping of XYZ and RPY from optical to body frame:
* x<sub>body</sub> = z<sub>opt</sub>
* y<sub>body</sub> = -x<sub>opt</sub>
* z<sub>body</sub> = -y<sub>opt</sub>


### 5 Publish tf messages
Finally, to publish tf messages in ROS you can run the static_transform_publisher from the tf package. It has the following form:


`rosrun tf static_transform_publisher X Y Z YAW PITCH ROLL LIDAR_FRAMENAME CAMERA_FRAMENAME UPDATE_FREQUENCY`


As the transform is static, we choose a low update frequency of 1000 ms:


`rosrun tf static_transform_publisher 0.60065 1.69362 2.65144 -0.95749 -1.23914 2.45415 laser camera_color_optical_frame 1000`


You can take a look in rviz now and visualize the tf frames and check of their relative position and translation looks plausible.



Once you are sure that you did the calibration right you can add the following line to your roslaunch file:

_&lt;node pkg="tf" type="static_transform_publisher" name="calibration_broadcaster" args="X Y Z YAW PITCH ROLL LIDAR_FRAMENAME CAMERA_FRAMENAME UPDATE_FREQUENCY" /&gt;_
