#ifndef TAG_TRACKER_H
#define TAG_TRACKER_H

/**
 * \file TagTracker.h
 * \brief TagTracker class declaration
 * \author christophebedard
 */

#include "trackerrr/Tracker.h"
#include "apriltags_ros/AprilTagDetectionArray.h"

static const double LOOP_RATE = 20.0;
static const std::string CAMERA_NAMESPACE = "camera"; /**< namespace for camera */
static const std::string IMAGE_RECT_TOPIC_NAME = "image_rect"; /**< topic name for image rect */
static const std::string TAG_DETECTIONS_TOPIC_NAME = "tag_detections"; /**< topic name for tag detections message */
static const std::string TAG_TF_NAME_PREFIX = "/tag_"; /**< tf name prefix for tags */

/** \class TagTracker
 * \brief class which tracks an apriltag.
 *
 * (something here).
 */
class TagTracker : public Tracker
{
    public:
        /**
         * \brief TagTracker constructor.
         *
         * \param n : node handle.
         */
        TagTracker(ros::NodeHandle& n);
        
        /**
         * \brief TagTracker destructor.
         */
        ~TagTracker();

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
        int targetTagID_; /**< id of tag to target */

        std::vector<apriltags_ros::AprilTagDetection> detected_tags_; /**< vector containing detection messages */

        image_transport::ImageTransport it_; /**< image transport */
        image_geometry::PinholeCameraModel cam_model_; /**< camera model */
        
        /*===========================
         * Subscribers
         *===========================*/
        image_transport::CameraSubscriber image_sub_; /**< camera image subscriber */
        ros::Subscriber tag_detect_sub_; /**< Apriltag detection subscriber */

        /*===========================
         * Publishers
         *===========================*/
        image_transport::Publisher track_image_pub_; /**< publisher for image with tracking error info */
        tf::TransformBroadcaster tf_br_; /**< tf broadcaster */

        /*===========================
         * Callbacks
         *===========================*/
        /**
         * \brief Callback class method for camera image.
         *
         * \param msg : constptr to image message.
         * \param cam_info : constptr to camera info message.
         */
        void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);

        /**
         * \brief Callback class method for apriltag detection.
         *
         * \param msg : constptr to apriltag detection array message.
         */
        void tagPositionCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg);

        /*===========================
         * Utilities
         *===========================*/
        /**
         * \brief Check if tag ID is detected.
         *
         * \param tag_id : id of tag to look for.
         *
         * \return detection result.
         */
        bool isTagDetected(int tag_id) const;

};

#endif  // TRACKERRR_TAG_TRACKER_H