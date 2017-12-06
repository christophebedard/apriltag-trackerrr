/**
 * \file TagTracker.cpp
 * \brief TagTracker class implementation
 * \author christophebedard
 */

#include "trackerrr/TagTracker.h"

TagTracker::TagTracker(ros::NodeHandle& n, int dof)
    : n_(n), it_(n), dof_(dof)
{
    // setup subscribers
    image_sub_ = it_.subscribeCamera("/"+CAMERA_NAMESPACE+"/"+IMAGE_RECT_TOPIC_NAME, 100, &TagTracker::imageCallback, this);
    tag_detect_sub_ = n_.subscribe("/"+CAMERA_NAMESPACE+"/"+TAG_DETECTIONS_TOPIC_NAME, 100, &TagTracker::tagPositionCallback, this);

    // setup publishers
    track_image_pub_ = it_.advertise("/"+CAMERA_NAMESPACE+"/"+"tracking_error", 1);
    joint_state_command_pub_ = n.advertise<sensor_msgs::JointState>(JOINT_STATE_COMMAND_TOPIC_NAME, 100);
    
    // setup DoF-dependent stuff
    for (int i = 0; i < dof_; i++) {
        tag_target_pose_pubs_.push_back(n.advertise<geometry_msgs::PoseStamped>(TAG_TARGET_POSE_TOPIC_NAME_PREFIX + std::to_string(i+1), 10));
        frames_.push_back(JOINT_TF_NAME_PREFIX + std::to_string(i));
    }
}

TagTracker::~TagTracker() {
}

/*===========================
 * Utilities
 *===========================*/

std::vector<geometry_msgs::PoseStamped> TagTracker::createPoseStampedVectorFromAngles(const std::vector<double>& angles) {
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

void TagTracker::publishTargetPoses(const std::vector<geometry_msgs::PoseStamped>& poses) {
    for (int i = 0; i < dof_; i++) {
        tag_target_pose_pubs_[i].publish(poses[i]);
    }
}

sensor_msgs::JointState TagTracker::createJointStateFromAngles(const std::vector<double>& angles) {
    sensor_msgs::JointState state_msg;
    state_msg.position = angles;
    return state_msg;
}

bool TagTracker::isTagDetected(int tag_id) {
    for (apriltags_ros::AprilTagDetection tag : detected_tags_) {
        if (tag.id == tag_id) { return true; }
    }
    return false;
}

/*===========================
 * Callbacks
 *===========================*/

void TagTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info) {
    // convert image to opencv image type
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // update camera model
    cam_model_.fromCameraInfo(cam_info);

    int middle_x = cv_ptr->image.cols / 2;
    int middle_y = cv_ptr->image.rows / 2;
    cv::Point2d middle(middle_x, middle_y);
    //cv::circle(cv_ptr->image, cv::Point(middle_x, middle_y), 50, CV_RGB(255,0,0));

    // draw
    for (apriltags_ros::AprilTagDetection tag : detected_tags_) {
        cv::Point3d pt_cv(tag.pose.pose.position.x, tag.pose.pose.position.y, tag.pose.pose.position.z);
        cv::Point2d uv = cam_model_.project3dToPixel(pt_cv);
        cv::arrowedLine(cv_ptr->image, middle, uv, CV_RGB(255,0,0));
    }

    // output modified video stream
    track_image_pub_.publish(cv_ptr->toImageMsg());
}

void TagTracker::tagPositionCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg) {
    // update vector
    detected_tags_ = msg->detections;
}

/*===========================
 * Update
 *===========================*/

void TagTracker::update() {
    if (!detected_tags_.empty()) {
        if (isTagDetected(TARGET_TAG_ID)) {
            // lookup transforms
            std::vector<tf::StampedTransform> transforms;
            try {
                for (int i = 0; i < dof_; i++) {
                    tf::StampedTransform stf;
                    tf_listener_.waitForTransform(frames_[i], TAG_TF_NAME_PREFIX+std::to_string(TARGET_TAG_ID), ros::Time(0), ros::Duration(0.5));
                    tf_listener_.lookupTransform(frames_[i], TAG_TF_NAME_PREFIX+std::to_string(TARGET_TAG_ID), ros::Time(0), stf);
                    // assuming the transform lookup works is (probably) a bad idea
                    transforms.push_back(stf);
                }
            } catch (tf::TransformException ex) {
                ROS_ERROR("Error looking up tag transform: %s", ex.what());
            }

            // calculate angles
            std::vector<double> angles;
            for (int i = 0; i < dof_; i++) {
                angles.push_back(atan2(transforms[i].getOrigin().y(), transforms[i].getOrigin().x()));
            }

            // publish results
            joint_state_command_pub_.publish(createJointStateFromAngles(angles));
            publishTargetPoses(createPoseStampedVectorFromAngles(angles));
        }

    } else {
        // empty; didn't find tag
    }
}

void TagTracker::spinOnce() {
    update();
    ros::spinOnce();
}

void TagTracker::spin() {
    ros::Rate rate(LOOP_RATE);
    
    while (ros::ok()) {
        spinOnce();
        
        if (!rate.sleep()) {
            ROS_WARN("[TAGTRACKER] Loop running slowly.");
        }
    }
}

// ---------------------------

int main(int argc, char** argv) {
    ros::init(argc, argv, "tagtracker");
    ros::NodeHandle n;
    
    ros::NodeHandle n_p("~");
    int dof;
    n_p.getParam("dof", dof);
    
    TagTracker tag_tracker(n, dof);
    
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