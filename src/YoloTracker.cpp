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

    // setup publishers
    trackImage_pub_ = it_.advertise("/camera/tracking_error", 1);

    // init
    objectsFound_ = 0;
}

YoloTracker::~YoloTracker() {
}

/*===========================
 * Utilities
 *===========================*/

bool YoloTracker::isObjectDetected(std::string object) {
    if (objectsFound_ > (int8_t)0) {
        for (darknet_ros_msgs::BoundingBox box : detectedObjects_) {
            if (box.Class == object) { return true; }
        }
    }
    return false;
}

/*===========================
 * Callbacks
 *===========================*/

void YoloTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info) {
    // update camera model
    cam_model_.fromCameraInfo(cam_info);
    
    // debug drawing
    /*
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
    if (isObjectDetected(targetObject_)) {
        int middle_x = cv_ptr->image.cols / 2;
        int middle_y = cv_ptr->image.rows / 2;
        cv::Point2d middle(middle_x, middle_y);
        for (darknet_ros_msgs::BoundingBox box : detectedObjects_) {
            double x = (box.xmax + box.xmin)/2.0;
            double y = (box.ymax + box.ymin)/2.0;
            cv::Point2d uv(x, y);
            cv::arrowedLine(cv_ptr->image, middle, uv, CV_RGB(255,0,0));
        }
    }
    // output modified video stream
    trackImage_pub_.publish(cv_ptr->toImageMsg());
    */
}

void YoloTracker::objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) {
    // update vector
    detectedObjects_ = msg->boundingBoxes;
}

void YoloTracker::objectFoundCallback(const std_msgs::Int8::ConstPtr& msg) {
    objectsFound_ = msg->data;
}

/*===========================
 * Update
 *===========================*/

void YoloTracker::update() {
    // first clear command angles
    Tracker::clearCommandAngles();

    // check detections
    if (isObjectDetected(targetObject_)) {
        // compute 3d position of object wrt camera

        // broadcast corresponding tf

        // lookup transforms for each frame
        /*
        std::vector<tf::StampedTransform> transforms;
        try {
            for (int i = 0; i < dof_; i++) {
                tf::StampedTransform stf;
                tf_listener_.waitForTransform(frames_[i], TAG_TF_NAME_PREFIX+std::to_string(targetTagID_), ros::Time(0), ros::Duration(0.5));
                tf_listener_.lookupTransform(frames_[i], TAG_TF_NAME_PREFIX+std::to_string(targetTagID_), ros::Time(0), stf);
                // assuming the transform lookup works is (probably) a bad idea
                transforms.push_back(stf);
            }
        } catch (tf::TransformException ex) {
            ROS_ERROR("Error looking up tag transform: %s", ex.what());
        }
        */

        // calculate angles
        for (int i = 0; i < dof_; i++) {
            angles_.push_back(atan2(transforms[i].getOrigin().y(), transforms[i].getOrigin().x()));
        }
    } else {
        // empty; didn't find tag
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
