cmake_minimum_required(VERSION 2.8.3)
project(trackerrr)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    tf
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    control_toolbox
    uvc_camera
    jpeg_streamer
    cv_bridge
    image_geometry
    image_transport
    dynamixel_sdk
    dynamixel_workbench_controllers
    dynamixel_workbench_operators
    dynamixel_workbench_single_manager
    dynamixel_workbench_single_manager_gui
    dynamixel_workbench_toolbox
    dynamixel_workbench_msgs
    open_manipulator_description
    open_manipulator_dynamixel_ctrl
    open_manipulator_moveit
    open_manipulator_position_ctrl
    open_manipulator_msgs
    message_generation
    message_runtime
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rospy tf std_msgs std_srvs geometry_msgs sensor_msgs control_toolbox uvc_camera jpeg_streamer cv_bridge image_geometry image_transport dynamixel_sdk dynamixel_workbench_controllers dynamixel_workbench_operators dynamixel_workbench_single_manager dynamixel_workbench_single_manager_gui dynamixel_workbench_toolbox dynamixel_workbench_msgs open_manipulator_description open_manipulator_dynamixel_ctrl open_manipulator_moveit open_manipulator_position_ctrl open_manipulator_msgs message_generation message_runtime
)

###########
## Build ##
###########

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

# dh_to_tf
add_executable(dh_to_tf
    src/dh_to_tf.cpp
)
target_link_libraries(dh_to_tf
    ${catkin_LIBRARIES}
)

# dynamixel_interface_node
add_executable(dynamixel_interface_node
    src/dynamixel_interface.cpp
)
target_link_libraries(dynamixel_interface_node
    ${catkin_LIBRARIES}
)

# dynamixelsim_node
add_executable(dynamixelsim_node
    src/DynamixelSim_node.cpp
    src/DynamixelSim.cpp
    src/MotorSim.cpp
    src/BoundedPID.cpp
)
target_link_libraries(dynamixelsim_node
    ${catkin_LIBRARIES}
)

# motorjoy_node
add_executable(motorjoy_node
    src/MotorJoy.cpp
)
target_link_libraries(motorjoy_node
    ${catkin_LIBRARIES}
)

#######################
## optional features ##
#######################

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

# YOLO tracking
find_package(darknet_ros)
if(darknet_ros_FOUND)
    message("darknet_ros found for yolotracker_node")
    add_executable(yolotracker_node
        src/YoloTracker_node.cpp
        src/YoloTracker.cpp
        src/Tracker.cpp
    )
    include_directories(
        include
        ${darknet_ros_INCLUDE_DIRS}
    )
    target_link_libraries(yolotracker_node
        ${darknet_ros_INCLUDE_DIRS}
        ${darknet_ros_LIBRARIES}
        ${catkin_LIBRARIES}
    )
else()
    message(WARNING "darknet_ros NOT found: will not build yolotracker_node")
endif()
