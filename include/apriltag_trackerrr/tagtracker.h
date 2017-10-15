#ifndef TAG_TRACKER_H
#define TAG_TRACKER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class TagTracker
{
    private:
        ros::NodeHandle n_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_;

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);


    protected:


    public:
        TagTracker(ros::NodeHandle& n);
        ~TagTracker();

};

#endif  // APRILTAG_TRACKERRR_TAG_TRACKER_H