# apriltag_trackerrr

Track AprilTags with an RRR robot manipulator.

# Prerequisites

1. [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)


## Initial setup

1. Create workspace directory and init the catkin workspace
   ````
   mkdir -p ~/trackrrr_ws/src
   cd ~/trackrrr_ws/src
   catkin_init_workspace
   ````

2. Clone this repo in the `~/trackrrr_ws/src` folder
   ````
   git clone git@github.com:christophebedard/apriltag_trackerrr.git
   cd apriltag_trackerrr/
   ````  

3. Compile
   ````
   cd ~/trackrrr_ws
   catkin_make
   ````