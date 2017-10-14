# apriltag_trackerrr

Track AprilTags with an RRR robot manipulator.

# Prerequisites

1. [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

2. libv4l and qt-build
   ````
   sudo apt-get install libv4l-dev ros-kinetic-qt-build
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

3. Clone the following packages in the `~/trackrrr_ws/src` folder  
   ````
   cd ~/trackrrr_ws/src
   git clone https://github.com/ros-drivers/camera_umd.git
   git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
   git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
   git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
   git clone https://github.com/RIVeR-Lab/apriltags_ros.git
   ````  
   `camera_umd` (deprecated) is used instead of `libuvc_camera` because of [this](https://github.com/ros-drivers/libuvc_ros/issues/15)

3. Compile
   ````
   cd ~/trackrrr_ws
   catkin build
   ````

### Camera calibration

Follow [this tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). Put the parameters in the `cfg/camera.yaml` file.

### Dynamixel parameters

For the AX-12A, use `1000000` for `baud_rate` and `1.0` for `protocol_version`.