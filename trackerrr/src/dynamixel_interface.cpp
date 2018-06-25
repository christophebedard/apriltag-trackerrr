/**
 * \file dynamixel_interface.cpp
 * \brief NOT USED (basic interface for dynamixel_workbench_single_manager/single_manager.launch, used by dynamixel_single.launch)
 * \author christophebedard
 */

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include "dynamixel_workbench_msgs/AX.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

static const std::string NODE_NAME = "dynamixel_interface";
static const std::string DYNAMIXEL_INFO_TOPIC_NAME = "/dynamixel/AX_12A";
static const std::string POSITION_COMMAND_TOPIC_NAME = "command";
static const std::string DYNAMIXEL_POSITION_COMMAND_SERVICE_NAME = "/dynamixel/command";
static const std::string POSITION_TOPIC_NAME = "q1";
static const double PI = 3.1415;
static const double DYNAMIXEL_POSITION_RES_MIN = 0.0;
static const double DYNAMIXEL_POSITION_RES_MAX = 1023.0;
static const double DYNAMIXEL_POSITION_ANGLE_MIN = 0.0;
static const double DYNAMIXEL_POSITION_ANGLE_MAX = (5.0/3.0)*PI;

ros::Subscriber angle_sub;
ros::Subscriber position_command_sub;
ros::Publisher position_pub;
ros::ServiceClient position_command_srv;

int last_pos = 0;

/**
 * \brief Callback function for Dynamixel AX-12A info message
 *
 * Republishes equivalent in radians.
 *
 * \param msg   constptr to dynamixel AX info message
 */
void infoCallback(const dynamixel_workbench_msgs::AX::ConstPtr& msg) {
    std_msgs::Float64 pos_msg;
    pos_msg.data = (((msg->Present_Position - DYNAMIXEL_POSITION_RES_MIN)/(DYNAMIXEL_POSITION_RES_MAX - DYNAMIXEL_POSITION_RES_MIN))*(DYNAMIXEL_POSITION_ANGLE_MAX - DYNAMIXEL_POSITION_ANGLE_MIN)) + DYNAMIXEL_POSITION_ANGLE_MIN;
    position_pub.publish(pos_msg);
}

/**
 * \brief Callback function for position command
 *
 * Calls Dynamixel position command from position command in radians.
 *
 * \param msg   constptr to angle command message
 */
void positionCommandCallback(const std_msgs::Float64::ConstPtr& msg) {
    int new_pos = (int)((DYNAMIXEL_POSITION_RES_MAX - DYNAMIXEL_POSITION_RES_MIN)*((msg->data - DYNAMIXEL_POSITION_ANGLE_MIN)/(DYNAMIXEL_POSITION_ANGLE_MAX - DYNAMIXEL_POSITION_ANGLE_MIN))) + DYNAMIXEL_POSITION_RES_MIN;

    // if new position command is out of bounds, send last (valid) command
    // TODO: send command equal to closest min/max bound
    if ((new_pos > (int)DYNAMIXEL_POSITION_RES_MAX) || (new_pos < (int)DYNAMIXEL_POSITION_RES_MIN)) {
        new_pos = last_pos;
    } else {
        last_pos = new_pos;
    }

    dynamixel_workbench_msgs::DynamixelCommand srv;
    srv.request.command = "addr";
    srv.request.addr_name = "goal_position";
    srv.request.value = new_pos;
    position_command_srv.call(srv);
}

/**
 * \brief Function for torque enabling
 *
 * Calls service with correct parameter.
 */
void enableTorque() {
    dynamixel_workbench_msgs::DynamixelCommand srv;
    srv.request.command = "addr";
    srv.request.addr_name = "torque_enable";
    srv.request.value = 1;
    position_command_srv.call(srv);
}

/**
 * \brief Entry point for dynamixel_interface_node
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    angle_sub = n.subscribe(DYNAMIXEL_INFO_TOPIC_NAME, 100, &infoCallback);
    position_command_sub = n.subscribe(POSITION_COMMAND_TOPIC_NAME, 100, &positionCommandCallback);
    
    position_pub = n.advertise<std_msgs::Float64>(POSITION_TOPIC_NAME, 10);

    position_command_srv = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>(DYNAMIXEL_POSITION_COMMAND_SERVICE_NAME);
    ros::Duration(0.5).sleep();
    enableTorque();

    ros::spin();
    
    return 0;
}