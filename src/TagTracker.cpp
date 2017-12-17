/**
 * \file TagTracker.cpp
 * \brief TagTracker class implementation
 * \author christophebedard
 */

#include "trackerrr/TagTracker.h"

TagTracker::TagTracker(ros::NodeHandle& n)
    : Tracker(n), it_(n)
{
    // get params
    ros::NodeHandle n_p("~");
    n_p.getParam("target_tag_id", targetTagID_);

    // setup subscribers
    image_sub_ = it_.subscribeCamera("/"+CAMERA_NAMESPACE+"/"+IMAGE_RECT_TOPIC_NAME, 100, &TagTracker::imageCallback, this);
    tag_detect_sub_ = n_.subscribe("/"+CAMERA_NAMESPACE+"/"+TAG_DETECTIONS_TOPIC_NAME, 100, &TagTracker::tagPositionCallback, this);

    // setup publishers
    track_image_pub_ = it_.advertise("/"+CAMERA_NAMESPACE+"/"+"tracking_error", 1);
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
    if (isTagDetected(targetTagID_)) {
        try {
            // lookup target tag transform (with ID)
            tf::StampedTransform stf;
            tf_listener_.waitForTransform("/camera", TAG_TF_NAME_PREFIX+std::to_string(targetTagID_), ros::Time(0), ros::Duration(0.5));
            tf_listener_.lookupTransform("/camera", TAG_TF_NAME_PREFIX+std::to_string(targetTagID_), ros::Time(0), stf);
            // re-broadcast (for base class)
            tf_br_.sendTransform(tf::StampedTransform(tf::Transform(stf.getRotation(), stf.getOrigin()), ros::Time::now(), "/camera", TARGET_TF_NAME));
        } catch (tf::TransformException ex) {
            ROS_ERROR("Error looking up tag transform: %s", ex.what());
        }
    }

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
