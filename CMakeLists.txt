cmake_minimum_required(VERSION 2.8.3)
project(trackerrr)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  message_generation
  sensor_msgs
  uvc_camera
  jpeg_streamer
  cv_bridge
  image_geometry
  image_transport
  dynamixel_workbench_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
  LIBRARIES trackerrr
  CATKIN_DEPENDS roscpp rospy tf std_msgs std_srvs geometry_msgs sensor_msgs uvc_camera jpeg_streamer cv_bridge image_geometry image_transport dynamixel_workbench_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## Specify libraries to link a library or executable target against
add_executable(dh_to_tf
    src/dh_to_tf.cpp
)
target_link_libraries(dh_to_tf
    ${catkin_LIBRARIES}
)

add_executable(dynamixel_interface_node
    src/dynamixel_interface.cpp
)
target_link_libraries(dynamixel_interface_node
    ${catkin_LIBRARIES}
)

# apriltag tracking
find_package(apriltags_ros)
if(apriltags_ros_FOUND)
    message("apriltags_ros found for tagtracker_node")
    add_executable(tagtracker_node
        src/TagTracker_node.cpp
        src/TagTracker.cpp
        src/Tracker.cpp
    )
    include_directories(
        include
        ${apriltags_ros_INCLUDE_DIRS}
    )
    target_link_libraries(tagtracker_node
        ${apriltags_ros_INCLUDE_DIRS}
        ${apriltags_ros_LIBRARIES}
        ${catkin_LIBRARIES}
    )
else()
    message(WARNING "apriltags_ros NOT found: will not build tagtracker_node")
endif()
