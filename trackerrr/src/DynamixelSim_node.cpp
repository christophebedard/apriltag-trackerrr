/**
 * \file DynamixelSim_node.cpp
 * \brief DynamixelSim node
 * \author christophebedard
 */

#include "trackerrr/DynamixelSim.h"

/**
 * \brief Entry point for dynamixelsim_node
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamixelsim");
    ros::NodeHandle n;
    
    DynamixelSim dynamixelSim(n);
    
    try
    {
        dynamixelSim.spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[DYNAMIXELSIM] Runtime error: " << e.what());
        return 1;
    }    
    return 0;
}