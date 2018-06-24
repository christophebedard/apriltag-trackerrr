# trackerrr [![Build Status](https://travis-ci.org/christophebedard/trackerrr.svg?branch=kinetic-devel)](https://travis-ci.org/christophebedard/trackerrr)

Track tags and other features with an RRR robot manipulator.

## Prerequisites

1. [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

## Initial setup

1. Create workspace directory and init the catkin workspace
   ````
   mkdir -p ~/tracker_ws/src
   cd ~/tracker_ws/src
   catkin_init_workspace
   ````

2. Clone this repo in the `src/` directory
   ````
   cd ~/tracker_ws/src
   git clone https://github.com/christophebedard/trackerrr.git
   ````

3. Get source dependencies with
   ````
   cd ~/tracker_ws/src
   wstool init
   wstool merge trackerrr/dependencies.rosinstall
   wstool up
   ````
   Or clone the following repositories in the `src/` directory
   ````
   cd ~/tracker_ws/src
   git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
   git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
   git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
   git clone https://github.com/christophebedard/open_manipulator.git
   git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
   ````

4. Get package dependencies with
   ````
   cd ~/tracker_ws
   rosdep install -y --from-paths src --ignore-src --rosdistro kinetic
   ````
   Or install them with
   ````
   sudo apt-get install libv4l-dev \
   ros-kinetic-qt-build \
   ros-kinetic-gazebo-ros-control \
   ros-kinetic-position-controllers \
   ros-kinetic-robotis-math \
   ros-kinetic-moveit \
   ros-kinetic-camera-umd
   ````  
   `camera_umd` (deprecated) is used instead of `libuvc_camera` because of [this](https://github.com/ros-drivers/libuvc_ros/issues/15)

### Feature selection

5. Additionally, clone at least one of these, in order to:
   1. track AprilTags
      ````
      cd ~/tracker_ws/src
      git clone https://github.com/RIVeR-Lab/apriltags_ros.git
      ````

   2. track objects with YOLO
      ````
      cd ~/tracker_ws/src
      git clone https://github.com/leggedrobotics/darknet_ros.git
      ````  
      Follow instructions [here](https://github.com/leggedrobotics/darknet_ros).

## Build

1. Compile
   ````
   cd ~/tracker_ws
   catkin build
   ````

   To get better performance, compile a release build
   ````
   cd ~/tracker_ws
   catkin build -DCMAKE_BUILD_TYPE=Release
   ````

2. Source
   ````
   cd ~/tracker_ws
   source devel/setup.bash
   ````

## Launch files

Some of the main launch files:

* `tracker_tag.launch`
   * tracks an AprilTag (`apriltags_ros` needed!)
   * *main arguments*
      * `dof` : degrees of freedom of the robot [1]
      * `target_tag_id` : ID of target tag [27]

* `tracker_yolo.launch`
   * tracks an object (`darknet_ros` needed!)
   * *main arguments*
      * `dof` : degrees of freedom of the robot [1]
      * `target_object` : name/class of target object

* `joy_test.launch`
   * drives the robot with a controller (why not)
   * *main argument*
      * `dof` : degrees of freedom of the robot [1]

## Documentation

Generate documentation with  
````
doxygen trackerrr/Doxyfile
````

Or read it online [here](https://christophebedard.github.io/trackerrr/).

## Additional info

### Camera calibration

Follow [this tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). Put the parameters in the `trackerrr/cfg/camera.yaml` file.

### Dynamixel parameters

For model AX-12A, use `1000000` for `baud_rate` and `1.0` for `protocol_version`.
