/**
 * \file YoloTracker.cpp
 * \brief YoloTracker class implementation
 * \author christophebedard
 */

#include "trackerrr/YoloTracker.h"

YoloTracker::YoloTracker(ros::NodeHandle& n)
    : Tracker(n), it_(n)
{
    // get params
    ros::NodeHandle n_p("~");
    n_p.getParam("target_object", targetObject_);
    n_p.getParam("detection_topic", objectDetectionTopic_);
    n_p.getParam("found_objects_topic", objectFoundTopic_);
    n_p.getParam("image_topic", imageTopic_);

    // setup subscribers
    objectDetect_sub_ = n_.subscribe(objectDetectionTopic_, 10, &YoloTracker::objectCallback, this);
    objectFound_sub_ = n_.subscribe(objectFoundTopic_, 10, &YoloTracker::objectFoundCallback, this);
    image_sub_ = it_.subscribeCamera(imageTopic_, 10, &YoloTracker::imageCallback, this);

    // init
    objectsFound_ = 0;
}

YoloTracker::~YoloTracker() {
}

/*===========================
 * Utilities
 *===========================*/

bool YoloTracker::isTargetDetected() const {
    return isObjectDetected(targetObject_);
}

bool YoloTracker::isObjectDetected(std::string object) const {
    if (objectsFound_ > (int8_t)0) {
        for (darknet_ros_msgs::BoundingBox box : detectedObjects_) {
            if (box.Class == object) { return true; }
        }
    }
    return false;
}

cv::Point2d YoloTracker::getObjectPosition2d(std::string object) {
    for (darknet_ros_msgs::BoundingBox box : detectedObjects_) {
        if (box.Class == object) {
            cv::Point2d pos((box.xmax + box.xmin)/2.0,
                            (box.ymax + box.ymin)/2.0);
            return pos;
        }
    }
    return cv::Point2d(0.0, 0.0);
}

/*===========================
 * Callbacks
 *===========================*/

void YoloTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info) {
    // update camera model
    cam_model_.fromCameraInfo(cam_info);
}

void YoloTracker::objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) {
    detectedObjects_ = msg->boundingBoxes;
}

void YoloTracker::objectFoundCallback(const std_msgs::Int8::ConstPtr& msg) {
    objectsFound_ = msg->data;
}

/*===========================
 * Update
 *===========================*/

void YoloTracker::update() {
    if (isObjectDetected(targetObject_)) {
        // compute 3d position of object wrt camera
        cv::Point3d pos3d = cam_model_.projectPixelTo3dRay(getObjectPosition2d(targetObject_));

        // broadcast corresponding tf
        tf::Transform tf;
        tf::Quaternion q;
        tf.setOrigin(tf::Vector3(pos3d.x, pos3d.y, pos3d.z));
        q.setRPY(0.0, 0.0, 0.0);
        tf.setRotation(q);
        tf_br_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "/"+cameraName_, TARGET_TF_NAME));
    }

    Tracker::update();
}

void YoloTracker::spinOnce() {
    update();
    ros::spinOnce();
}

void YoloTracker::spin() {
    ros::Rate rate(LOOP_RATE);
    
    while (ros::ok()) {
        spinOnce();
        
        if (!rate.sleep()) {
            ROS_WARN("[YOLOTRACKER] Loop running slowly.");
        }
    }
}
