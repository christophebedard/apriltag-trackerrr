# apriltag_trackerrr

Track AprilTags with an RRR robot manipulator.

# Prerequisites

1. [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

2. libv4l
   ````
   sudo apt-get install libv4l-dev
   ````

## Initial setup

1. Create workspace directory and init the catkin workspace
   ````
   mkdir -p ~/trackrrr_ws/src
   cd ~/trackrrr_ws/src
   catkin_init_workspace
   ````

2. Clone this repo in the `~/trackrrr_ws/src` folder
   ````
   cd ~/trackrrr_ws/src
   git clone git@github.com:christophebedard/apriltag_trackerrr.git
   ````  

3. Clone `camera_umd` in the `~/trackrrr_ws/src` folder  
   ````
   cd ~/trackrrr_ws/src
   git clone git@github.com:ktossell/camera_umd.git
   ````  

3. Compile
   ````
   cd ~/trackrrr_ws
   catkin_make
   ````

### Camera calibration

Follow [this tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). Put the parameters in the `cfg/camera.yaml` file.