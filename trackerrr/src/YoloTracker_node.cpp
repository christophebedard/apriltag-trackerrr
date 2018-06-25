/**
 * \file YoloTracker_node.cpp
 * \brief YoloTracker node
 * \author christophebedard
 */

#include "trackerrr/YoloTracker.h"

/**
 * \brief Entry point for yolotracker_node
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "yolotracker");
    ros::NodeHandle n;
    
    YoloTracker yolo_tracker(n);
    
    try
    {
        yolo_tracker.spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[YOLOTRACKER] Runtime error: " << e.what());
        return 1;
    }    
    return 0;
}