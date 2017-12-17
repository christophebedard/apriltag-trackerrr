#ifndef YOLO_TRACKER_H
#define YOLO_TRACKER_H

/**
 * \file YoloTracker.h
 * \brief YoloTracker class declaration
 * \author christophebedard
 */

#include "trackerrr/Tracker.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <std_msgs/Int8.h>

static const double LOOP_RATE = 20.0;

/** \class YoloTracker
 * \brief class which tracks an object with YOLO.
 */
class YoloTracker : public Tracker
{
    public:
        /**
         * \brief YoloTracker constructor.
         *
         * \param n : node handle.
         */
        YoloTracker(ros::NodeHandle& n);
        
        /**
         * \brief YoloTracker destructor.
         */
        ~YoloTracker();

        /**
         * \brief ROS spin. Called only once (by node); contains ROS while loop.
         */
        void spin();


    protected:
        /*===========================
         * Update
         *===========================*/
        /**
         * \brief Update; called every spinOnce().
         */
        void update();

        /**
         * \brief ROS spin once, called on every loop.
         */
        void spinOnce();

        /*===========================
         * Utilities
         *===========================*/
        /**
         * \brief Check if target is currently detected.
         */
        bool isTargetDetected() const;


    private:
        std::string targetObject_; /**< name of target object */
        std::string objectDetectionTopic_; /**< topic name for darknet BoundingBoxes msg */
        std::string objectFoundTopic_; /**< topic name for darknet object found msg */
        std::string imageTopic_;  /**< topic name for camera image */

        std::vector<darknet_ros_msgs::BoundingBox> detectedObjects_; /**< vector containing detection messages */
        int8_t objectsFound_; /**< number of objects found in latest image */

        image_transport::ImageTransport it_; /**< image transport */
        image_geometry::PinholeCameraModel cam_model_; /**< camera model */
        
        /*===========================
         * Subscribers
         *===========================*/
        ros::Subscriber objectDetect_sub_; /**< YOLO detection subscriber */
        ros::Subscriber objectFound_sub_; /**< YOLO object found subscriber */
        image_transport::CameraSubscriber image_sub_; /**< camera image subscriber */
        tf::TransformListener tf_listener_; /**< tf listener */

        /*===========================
         * Publishers
         *===========================*/
        tf::TransformBroadcaster tf_br_; /**< pose tf publisher */
        image_transport::Publisher trackImage_pub_; /**< publisher for image with tracking error info */

        /*===========================
         * Callbacks
         *===========================*/
        /**
         * \brief Callback class method for YOLO object detection.
         *
         * \param msg : constptr to darknet BoundingBoxes message.
         */
        void objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);

        /**
         * \brief Callback class method for YOLO object found message.
         *
         * \param msg : constptr to std_msgs::Int8 message.
         */
        void objectFoundCallback(const std_msgs::Int8::ConstPtr& msg);

        /**
         * \brief Callback class method for camera image.
         *
         * \param msg : constptr to image message.
         * \param cam_info : constptr to camera info message.
         */
        void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);

        /*===========================
         * Utilities
         *===========================*/
        /**
         * \brief Check if object is detected.
         *
         * \param object : name of object to look for.
         *
         * \return detection result.
         */
        bool isObjectDetected(std::string object) const;

        /**
         * \brief Get 2D position of object in image.
         *
         * \param object : name of object to look for.
         *
         * \return position.
         */
        cv::Point2d getObjectPosition2d(std::string object);

};

#endif  // TRACKERRR_YOLO_TRACKER_H