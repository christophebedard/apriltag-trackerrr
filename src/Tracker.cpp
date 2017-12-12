/**
 * \file Tracker.cpp
 * \brief Tracker class implementation
 * \author christophebedard
 */

#include "trackerrr/Tracker.h"

Tracker::Tracker(ros::NodeHandle& n)
    : n_(n)
{
    // get params
    ros::NodeHandle n_p("~");
    n_p.getParam("dof", dof_);

    // setup publishers
    joint_state_command_pub_ = n.advertise<sensor_msgs::JointState>(JOINT_STATE_COMMAND_TOPIC_NAME, 100);

    // setup DoF-dependent stuff
    for (int i = 0; i < dof_; i++) {
        target_pose_pubs_.push_back(n.advertise<geometry_msgs::PoseStamped>(TARGET_POSE_TOPIC_NAME_PREFIX + std::to_string(i+1), 10));
        frames_.push_back(JOINT_TF_NAME_PREFIX + std::to_string(i));
    }
}

Tracker::~Tracker() {
}

/*===========================
 * Utilities
 *===========================*/

void Tracker::clearCommandAngles() {
    angles_.clear();
}

std::vector<geometry_msgs::PoseStamped> Tracker::createPoseStampedVectorFromAngles(const std::vector<double>& angles) {
    std::vector<geometry_msgs::PoseStamped> vec;
    for (int i = 0; i < dof_; i++) {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(angles[i]);
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = frames_[i];
        vec.push_back(pose_msg);
    }
    return vec;
}

sensor_msgs::JointState Tracker::createJointStateFromAngles(const std::vector<double>& angles) {
    sensor_msgs::JointState state_msg;
    state_msg.position = angles;
    return state_msg;
}

void Tracker::publishTargetPoses(const std::vector<geometry_msgs::PoseStamped>& poses) {
    for (int i = 0; i < dof_; i++) {
        target_pose_pubs_[i].publish(poses[i]);
    }
}

/*===========================
 * Update
 *===========================*/

void Tracker::update() {
    // publish results, if there are any
    if (!angles_.empty()) {
        joint_state_command_pub_.publish(createJointStateFromAngles(angles_));
        publishTargetPoses(createPoseStampedVectorFromAngles(angles_));
    }
}


