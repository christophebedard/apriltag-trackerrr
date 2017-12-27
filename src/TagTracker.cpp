/**
 * \file TagTracker.cpp
 * \brief TagTracker class implementation
 * \author christophebedard
 */

#include "trackerrr/TagTracker.h"

TagTracker::TagTracker(ros::NodeHandle& n)
    : Tracker(n)
{
    // get params
    ros::NodeHandle n_p("~");
    n_p.getParam("target_tag_id", targetTagID_);

    // setup subscribers
    tag_detect_sub_ = n_.subscribe("/"+cameraName_+"/"+TAG_DETECTIONS_TOPIC_NAME, 100, &TagTracker::tagPositionCallback, this);
}

TagTracker::~TagTracker() {
}

/*===========================
 * Utilities
 *===========================*/

bool TagTracker::isTargetDetected() const {
    return isTagDetected(targetTagID_);
}

bool TagTracker::isTagDetected(int tag_id) const {
    for (apriltags_ros::AprilTagDetection tag : detected_tags_) {
        if (tag.id == tag_id) { return true; }
    }
    return false;
}

std::string TagTracker::getTargetTfName() const {
    return TAG_TF_NAME_PREFIX + std::to_string(targetTagID_);
}

/*===========================
 * Callbacks
 *===========================*/

void TagTracker::tagPositionCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg) {
    // update vector
    detected_tags_ = msg->detections;
}

/*===========================
 * Update
 *===========================*/

void TagTracker::update() {
    Tracker::update();
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
