/**
 * \file dh_to_tf.cpp
 * \brief Helper node that subscribes to a JointState topic and publishes the corresponding transforms
 * \author christophebedard
 */

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

static const std::string NODE_NAME = "dh_to_tf";
static const std::string JOINT_STATE_TOPIC = "/motors/present_states";
static const std::string END_EFFECTOR_TF_NAME = "/camera";
static const std::string FRAME_TF_NAME_PREFIX = "/joint";
static const std::string ANGLE_TOPIC_PREFIX = "q";
static const int COL_OFFSET = 0;
static const int COL_D      = 1;
static const int COL_A      = 2;
static const int COL_ALPHA  = 3;
static const int COL_TOTAL  = 4;

int dof;
std::vector<double> dh_matrix;
std::vector<std::string> child_frames, frames, angle_topics;
std::vector<double> offset, d, a, alpha;
double end_effector_x, end_effector_y, end_effector_z, end_effector_roll, end_effector_pitch, end_effector_yaw;
tf::Transform end_effector_tf;

/**
 * \brief JointState callback
 *
 * Publishes corresponding transforms
 *
 * \param msg   constptr to JointState message
 */
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    static tf::TransformBroadcaster br;
    for (int i = 1; i <= dof; i++) {
        int idx = i - 1;
        double theta = offset[idx] + msg->position[idx];

        tf::Matrix3x3 rot(cos(theta), -sin(theta)*cos(alpha[idx]),  sin(theta)*sin(alpha[idx]),
                          sin(theta),  cos(theta)*cos(alpha[idx]), -cos(theta)*sin(alpha[idx]),
                                   0,             sin(alpha[idx]),             cos(alpha[idx]));
        tf::Vector3 trans(a[idx]*cos(theta),
                          a[idx]*sin(theta),
                                    d[idx]);
        tf::Transform transform(rot, trans);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child_frames[idx], frames[idx]));
    }
    // also publish end effector tf
    br.sendTransform(tf::StampedTransform(end_effector_tf, ros::Time::now(), frames[dof-1], END_EFFECTOR_TF_NAME));
}

/*
void angleCallback(const std_msgs::Float64::ConstPtr& msg) {
    double theta = offset + msg->data;

    tf::Matrix3x3 rot(cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),
                      sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),
                               0,             sin(alpha),             cos(alpha));
    tf::Vector3 trans(a*cos(theta),
                      a*sin(theta),
                                 d);
    tf::Transform transform(rot, trans);

    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child_frame, frame));
}
*/

/**
 * \brief Entry point for dh_to_tf
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    ros::NodeHandle n_p("~");
    n_p.getParam("dof", dof);
    n_p.getParam("/dh_matrix", dh_matrix);
    n_p.getParam("/end_effector/x", end_effector_x);
    n_p.getParam("/end_effector/y", end_effector_y);
    n_p.getParam("/end_effector/z", end_effector_z);
    n_p.getParam("/end_effector/roll", end_effector_roll);
    n_p.getParam("/end_effector/pitch", end_effector_pitch);
    n_p.getParam("/end_effector/yaw", end_effector_yaw);

    for (int i = 1; i <= dof; i++) {
        child_frames.push_back(FRAME_TF_NAME_PREFIX + std::to_string(i - 1));
        frames.push_back(FRAME_TF_NAME_PREFIX + std::to_string(i));
        //angle_topics.push_back(ANGLE_TOPIC_PREFIX + std::to_string(i));

        offset.push_back(dh_matrix[COL_TOTAL * (i - 1) + COL_OFFSET]);
        d.push_back(dh_matrix[COL_TOTAL * (i - 1) + COL_D]);
        a.push_back(dh_matrix[COL_TOTAL * (i - 1) + COL_A]);
        alpha.push_back(dh_matrix[COL_TOTAL * (i - 1) + COL_ALPHA]);
    }

    // create end effector tf
    end_effector_tf.setOrigin(tf::Vector3(end_effector_x, end_effector_y, end_effector_z));
    tf::Quaternion q;
    q.setRPY(end_effector_roll, end_effector_pitch, end_effector_yaw);
    end_effector_tf.setRotation(q);

    //ros::Subscriber angle_sub = n.subscribe(angle_topic, 10, &angleCallback);
    ros::Subscriber joint_state_sub = n.subscribe(JOINT_STATE_TOPIC, 10, &jointStateCallback);
    
    ros::spin();
    
    return 0;
}