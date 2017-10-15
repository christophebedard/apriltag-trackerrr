#include "apriltag_trackerrr/tagtracker.h"

TagTracker::TagTracker(ros::NodeHandle& n)
    : n_(n), it_(n)
{
    image_sub_ = it_.subscribe("/camera/image_rect", 1, &TagTracker::imageCallback, this);
    image_pub_ = it_.advertise("/camera/tracking_error", 1);
}

TagTracker::~TagTracker()
{

}

void TagTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
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
    
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) {
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    }

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
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
    
    ros::spin();
    
    return 0;
}