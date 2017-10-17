#include "apriltag_trackerrr/tagtracker.h"

TagTracker::TagTracker(ros::NodeHandle& n)
    : n_(n), it_(n), loop_hz_(LOOP_RATE)
{
    // setup subscribers
    image_sub_ = it_.subscribeCamera("/"+CAMERA_NAMESPACE+"/"+IMAGE_RECT_TOPIC_NAME, 100, &TagTracker::imageCallback, this);
    tag_detect_sub_ = n_.subscribe("/"+CAMERA_NAMESPACE+"/"+TAG_DETECTIONS_TOPIC_NAME, 100, &TagTracker::tagPositionCallback, this);

    // setup publishers
    track_image_pub_ = it_.advertise("/"+CAMERA_NAMESPACE+"/"+"tracking_error", 1);
}

TagTracker::~TagTracker()
{

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
        // not empty; found tag(s)
        ROS_INFO_STREAM("found tag(s) :");
        int size = detected_tags_.size();
        ROS_INFO_STREAM(size);

        for (apriltags_ros::AprilTagDetection tag : detected_tags_) {
            ROS_INFO_STREAM(tag.id);
        }

    } else {
        // empty; didn't find tag
        //ROS_INFO_STREAM("DID NOT find tag");
    }
}

void TagTracker::spinOnce()
{
    update();
    ros::spinOnce();
}

void TagTracker::spin()
{
    ros::Rate rate(loop_hz_);
    
    while (ros::ok())
    {
        spinOnce();
        
        is_running_slowly_ = !rate.sleep();
        if (is_running_slowly_)
        {
            ROS_WARN("[TAGTRACKER] Loop running slowly.");
        }
    }
}

// ---------------------------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tagtracker");
    ros::NodeHandle n;
    
    //ros::NodeHandle n_p("~");
    //int robot_id;
    //n_p.getParam("robot_id", robot_id);
    
    TagTracker tag_tracker(n);
    
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