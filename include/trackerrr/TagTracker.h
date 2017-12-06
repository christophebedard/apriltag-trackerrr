#ifndef TAG_TRACKER_H
#define TAG_TRACKER_H

/**
 * \file TagTracker.h
 * \brief TagTracker class declaration
 * \author christophebedard
 */

#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "apriltags_ros/AprilTagDetectionArray.h"

static const double LOOP_RATE = 10.0;
static const std::string CAMERA_NAMESPACE = "camera";
static const std::string IMAGE_RECT_TOPIC_NAME = "image_rect";
static const std::string TAG_DETECTIONS_TOPIC_NAME = "tag_detections";
static const std::string JOINT_TF_NAME_PREFIX = "/joint";
static const std::string TAG_TARGET_POSE_TOPIC_NAME_PREFIX = "/target";
static const std::string TAG_TF_NAME_PREFIX = "/tag_";
static const std::string JOINT_STATE_COMMAND_TOPIC_NAME = "/motors/goal_states";
static const int TARGET_TAG_ID = 27;

/** \class TagTracker
 * \brief class which tracks an apriltag.
 *
 * (something here).
 */
class TagTracker
{
    public:
        /**
         * \brief TagTracker constructor.
         *
         * \param n : node handle.
         * \param dof : degrees of freedom.
         */
        TagTracker(ros::NodeHandle& n, int dof);
        
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


    private:
        ros::NodeHandle n_; /**< node handle */

        int dof_; /**< degrees of freedom */
        std::vector<std::string> frames_; /**< vector containing DOF frames */

        std::vector<apriltags_ros::AprilTagDetection> detected_tags_; /**< vector containing detection messages */

        image_transport::ImageTransport it_; /**< image transport */
        image_geometry::PinholeCameraModel cam_model_; /**< camera model */
        
        /*===========================
         * Subscribers
         *===========================*/
        image_transport::CameraSubscriber image_sub_; /**< camera image subscriber */
        ros::Subscriber tag_detect_sub_; /**< Apriltag detection subscriber */
        tf::TransformListener tf_listener_; /**< tf listener */

        /*===========================
         * Publishers
         *===========================*/
        image_transport::Publisher track_image_pub_; /**< publisher for image with tracking error info */
        ros::Publisher joint_state_command_pub_; /**< publisher for joint state command */
        std::vector<ros::Publisher> tag_target_pose_pubs_; /**< publishers for tag target pose */

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
         * \brief Create PoseStamped messages from angles and frames.
         *
         * \param angles : vector of angles (corresponding to DOFs).
         *
         * \return resulting vector of poses.
         */
        std::vector<geometry_msgs::PoseStamped> createPoseStampedVectorFromAngles(const std::vector<double>& angles);

        /**
         * \brief Create JointState message from angles vector.
         *
         * \param angles : vector of angles (corresponding to DOFs).
         *
         * \return resulting jointstate message.
         */
        sensor_msgs::JointState createJointStateFromAngles(const std::vector<double>& angles);

        /**
         * \brief Check if tag ID is detected.
         *
         * \param tag_id : id of tag to look for.
         *
         * \return detection result.
         */
        bool isTagDetected(int tag_id);

        /**
         * \brief Publish target poses from vector.
         *
         * \param poses : vector of posestamped messages to publish.
         */
        void publishTargetPoses(const std::vector<geometry_msgs::PoseStamped>& poses);

};

#endif  // TRACKERRR_TAG_TRACKER_H