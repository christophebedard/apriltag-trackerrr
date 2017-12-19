/**
 * \file MotorJoy.cpp
 * \brief MotorJoy class implementation
 * \author christophebedard
 */

#include "trackerrr/MotorJoy.h"

MotorJoy::MotorJoy(ros::NodeHandle& n)
    : n_(n), rate_(LOOP_RATE)
{
    // get params
    ros::NodeHandle n_p("~");
    std::string joyVelTopic, goalJointStateTopic, currentJointStateTopic;
    n_p.getParam("dof", dof_);
    n_p.getParam("/teleop/piloting/topic_name", joyVelTopic);
    n_p.getParam("goal_jointstate_topic", goalJointStateTopic);
    n_p.getParam("current_jointstate_topic", currentJointStateTopic);

    // subscribers
    vel_sub_ = n.subscribe(joyVelTopic, 10, &MotorJoy::velCallback, this);
    pos_sub_ = n.subscribe(currentJointStateTopic, 10, &MotorJoy::jointstateCallback, this);

    // publishers
    presentJointState_pub_ = n.advertise<sensor_msgs::JointState>(goalJointStateTopic, 100);

    // init
    for (int i = 0; i < dof_; i++) {
        posCurrent_.push_back(0.0);
        nextGoalState_.push_back(0.0);
        vel_.push_back(0.0);
    }
}

MotorJoy::~MotorJoy() {
}

/*===========================
 * Callbacks
 *===========================*/

void MotorJoy::velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // extract angular velocities
    switch (dof_) {
        case 2:
            vel_[1] = msg->angular.y;
        case 1:
            vel_[0] = msg->angular.z;
            break;
        default:
            vel_[0] = 0.0;
            break;
    }
}

void MotorJoy::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // extract angle states
    posCurrent_ = msg->position;
}

/*===========================
 * Update
 *===========================*/

void MotorJoy::updatePosition() {
    // get next position
    switch (dof_) {
        case 2:
            nextGoalState_[1] = posCurrent_[1] + (vel_[1] * rate_.expectedCycleTime().toSec());
        case 1:
            nextGoalState_[0] = posCurrent_[0] + (vel_[0] * rate_.expectedCycleTime().toSec());
            break;
        default:
            nextGoalState_[0] = 0.0;
            break;
    }

    // TODO: check position validity
    // e.g.
    /*
    if (nextPos < posMin_) {
        // below minimum, set to minimum
        posCurrent_ = posMin_;
    } else if (nextPos > posMax_) {
        // above maximum, set to maximum
        posCurrent_ = posMax_;
    } else {
        posCurrent_ = nextPos;
    }
    */
}

void MotorJoy::publishGoalJointState() {
    sensor_msgs::JointState state_msg;
    state_msg.position = nextGoalState_;
    presentJointState_pub_.publish(state_msg);
}

void MotorJoy::update() {
    updatePosition();
    publishGoalJointState();
}

void MotorJoy::spinOnce() {
    update();
    ros::spinOnce();
}

void MotorJoy::spin() {
    while (ros::ok()) {
        spinOnce();
        
        if (!rate_.sleep()) {
            ROS_WARN("[MOTORJOY] Loop running slowly.");
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motorjoy");
    ros::NodeHandle n;
    
    MotorJoy motorJoy(n);
    
    try
    {
        motorJoy.spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[MOTORJOY] Runtime error: " << e.what());
        return 1;
    }    
    return 0;
}