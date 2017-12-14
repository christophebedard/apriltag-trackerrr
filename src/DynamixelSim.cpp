/**
 * \file DynamixelSim.cpp
 * \brief DynamixelSim class implementation
 * \author christophebedard
 */

#include "trackerrr/DynamixelSim.h"

DynamixelSim::DynamixelSim(ros::NodeHandle& n)
    : n_(n), rate_(LOOP_RATE)
{
    // get params
    ros::NodeHandle n_p("~");
    double p, i, d, velMax;
    n_p.getParam("dof", dof_);
    n_p.getParam("present_jointstate_topic", presentJointStateTopic_);
    n_p.getParam("goal_jointstate_topic", goalJointStateTopic_);
    n_p.getParam("p", p);
    n_p.getParam("i", i);
    n_p.getParam("d", d);
    n_p.getParam("vel_max", velMax);

    // setup subscribers
    goalJointState_sub_ = n_.subscribe(goalJointStateTopic_, 10, &DynamixelSim::goalJointStateCallback, this);

    // setup publishers
    presentJointState_pub_ = n_.advertise<sensor_msgs::JointState>(presentJointStateTopic_, 1);

    // setup motors
    for (int i = 0; i < dof_; i++) {
        motors_.push_back(new MotorSim(DYNAMIXEL_POSITION_MIN, DYNAMIXEL_POSITION_MAX, 0.0, p, i, d, velMax));
        positions_.push_back(0.0);
    }
}

DynamixelSim::~DynamixelSim() {
    // delete motors
    for (auto m : motors_) {
        delete m;
    }
}

/*===========================
 * Callbacks
 *===========================*/

void DynamixelSim::goalJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // extract and send setpoints to motors
    for (int i = 0; i < dof_; i++) {
        motors_[i]->setSetpoint(msg->position[i]);
    }
}


/*===========================
 * Update
 *===========================*/

void DynamixelSim::updateMotors() {
    for (int i = 0; i < dof_; i++) {
        motors_[i]->update(rate_.expectedCycleTime());
    }
}

void DynamixelSim::updatePositions() {
    for (int i = 0; i < dof_; i++) {
        positions_[i] = motors_[i]->getPosition();
    }
}

void DynamixelSim::publishPresentJointState() {
    sensor_msgs::JointState state_msg;
    state_msg.position = positions_;
    presentJointState_pub_.publish(state_msg);
}

void DynamixelSim::update() {
    updateMotors();
    updatePositions();
    publishPresentJointState();
}

void DynamixelSim::spinOnce() {
    update();
    ros::spinOnce();
}

void DynamixelSim::spin() {    
    while (ros::ok()) {
        spinOnce();
        
        if (!rate_.sleep()) {
            ROS_WARN("[DYNAMIXELSIM] Loop running slowly.");
        }
    }
}
