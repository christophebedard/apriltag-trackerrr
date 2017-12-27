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
    std::string goalJointStateTopic, resetSrvName;
    n_p.getParam("dof", dof_);
    n_p.getParam("camera", cameraName_);
    n_p.getParam("goal_jointstate_topic", goalJointStateTopic);
    n_p.getParam("reset_srv_name", resetSrvName);

    // setup publishers
    joint_state_command_pub_ = n.advertise<sensor_msgs::JointState>(goalJointStateTopic, 100);

    // setup services
    reset_srv_ = n.advertiseService(resetSrvName, &Tracker::resetCallback, this);

    // setup DoF-dependent stuff
    for (int i = 0; i < dof_; i++) {
        target_pose_pubs_.push_back(n.advertise<geometry_msgs::PoseStamped>(TARGET_POSE_TOPIC_NAME_PREFIX + std::to_string(i+1), 10));
        frames_.push_back(JOINT_TF_NAME_PREFIX + std::to_string(i));
        angles_.push_back(0.0);
    }

    // reset motors
    reset();
}

Tracker::~Tracker() {
}

/*===========================
 * Callbacks
 *===========================*/

bool Tracker::resetCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    reset();
    return true;
}

/*===========================
 * Utilities
 *===========================*/

void Tracker::reset() {
    std::vector<double> resetAngles;
    for (int i = 0; i < dof_; i++) {
        resetAngles.push_back(0.0);
    }
    joint_state_command_pub_.publish(createJointStateFromAngles(resetAngles));
    publishTargetPoses(createPoseStampedVectorFromAngles(resetAngles));
}

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
    if (isTargetDetected()) {
        // lookup transforms
        std::vector<tf::StampedTransform> transforms;
        try {
            for (int i = 0; i < dof_; i++) {
                tf::StampedTransform stf;
                tf_listener_.waitForTransform(frames_[i], getTargetTfName(), ros::Time(0), ros::Duration(0.5));
                tf_listener_.lookupTransform(frames_[i], getTargetTfName(), ros::Time(0), stf);
                // assuming the transform lookup works is (probably) a bad idea
                transforms.push_back(stf);
            }

            // calculate angles
            for (int i = 0; i < dof_; i++) {
                angles_[i] = atan2(transforms[i].getOrigin().y(), transforms[i].getOrigin().x());
            }

            // publish
            joint_state_command_pub_.publish(createJointStateFromAngles(angles_));
            publishTargetPoses(createPoseStampedVectorFromAngles(angles_));
        } catch (tf::TransformException ex) {
            ROS_ERROR("Error looking up tag transform: %s", ex.what());
        }
    }
}


