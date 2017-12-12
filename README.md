# trackerrr

Track tags and other features with an RRR robot manipulator.

# Prerequisites

1. [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

2. These
   ````
   sudo apt-get install libv4l-dev \
   ros-kinetic-qt-build \
   ros-kinetic-gazebo-ros-control \
   ros-kinetic-position-controllers \
   ros-kinetic-robotis-math \
   ros-kinetic-moveit
   ````

## Initial setup

1. Create workspace directory and init the catkin workspace
   ````
   mkdir -p ~/tracker_ws/src
   cd ~/tracker_ws/src
   catkin_init_workspace
   ````

2. Clone this repo in the `~/tracker_ws/src` folder
   ````
   cd ~/tracker_ws/src
   git clone https://github.com/christophebedard/trackerrr.git
   ````  

3. Clone the following packages in the `~/tracker_ws/src` folder  
   ````
   cd ~/tracker_ws/src
   git clone https://github.com/ros-drivers/camera_umd.git
   git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
   git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
   git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
   git clone https://github.com/christophebedard/open_manipulator.git
   ````  
   `camera_umd` (deprecated) is used instead of `libuvc_camera` because of [this](https://github.com/ros-drivers/libuvc_ros/issues/15)

4. Additionally, clone these to:  
   1. track AprilTags  
      ````
      cd ~/tracker_ws/src
      git clone https://github.com/RIVeR-Lab/apriltags_ros.git
      ````

5. Compile
   ````
   cd ~/tracker_ws
   catkin build
   ````

6. Source  
   ````
   cd ~/tracker_ws
   source devel/setup.bash
   ````

### Camera calibration

Follow [this tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). Put the parameters in the `cfg/camera.yaml` file.

### Dynamixel parameters

For the AX-12A, use `1000000` for `baud_rate` and `1.0` for `protocol_version`.