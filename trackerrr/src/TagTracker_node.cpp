/**
 * \file TagTracker_node.cpp
 * \brief TagTracker node
 * \author christophebedard
 */

#include "trackerrr/TagTracker.h"

/**
 * \brief Entry point for tagtracker_node
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "tagtracker");
    ros::NodeHandle n;
    
    TagTracker tag_tracker(n);
    
    try
    {
        tag_tracker.spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[TAGTRACKER] Runtime error: " << e.what());
        return 1;
    }    
    return 0;
}