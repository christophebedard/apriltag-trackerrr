#ifndef TAG_TRACKER_H
#define TAG_TRACKER_H

#include "apriltags_ros/AprilTagDetectionArray.h"
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
#include <vector>

static const double LOOP_RATE = 10.0;
static const std::string CAMERA_NAMESPACE = "camera";
static const std::string IMAGE_RECT_TOPIC_NAME = "image_rect";
static const std::string TAG_DETECTIONS_TOPIC_NAME = "tag_detections";
static const std::string JOINT_TF_NAME_PREFIX = "/joint";
static const std::string TAG_TARGET_POSE_TOPIC_NAME_PREFIX = "/target";
static const std::string TAG_TF_NAME_PREFIX = "/tag_";
static const std::string JOINT_STATE_COMMAND_TOPIC_NAME = "/motors/goal_states";
static const int TARGET_TAG_ID = 27;

class TagTracker
{
    public:
        /*
         * Constructor
         */
        TagTracker(ros::NodeHandle& n, int dof);
        /*
         * Destructor
         */
        ~TagTracker();

        /*
         * ROS spin. Called only once (by node); contains ROS while loop
         */
        void spin();


    protected:
        /*===========================
         * Update
         *===========================*/
        /*
         * Update; called every spinOnce()
         */
        void update();

        /*
         * ROS spin once, called on every loop
         */
        void spinOnce();


    private:
        ros::NodeHandle n_;
        bool is_running_slowly_;
        double loop_hz_;

        int dof_;
        std::vector<std::string> frames_;

        std::vector<apriltags_ros::AprilTagDetection> detected_tags_;

        tf::TransformListener tf_listener_;
        image_transport::ImageTransport it_;
        image_geometry::PinholeCameraModel cam_model_;
        /*===========================
         * Subscribers
         *===========================*/
        /* Camera image subscriber */
        image_transport::CameraSubscriber image_sub_;
        /* Apriltag detection subscriber */
        ros::Subscriber tag_detect_sub_;

        /*===========================
         * Publishers
         *===========================*/
        /* Publisher for image with tracking error info */
        image_transport::Publisher track_image_pub_;
        /* Publisher for joint state command */
        ros::Publisher joint_state_command_pub_;
        /* Publishers for tag target pose */
        std::vector<ros::Publisher> tag_target_pose_pubs_;

        /*===========================
         * Callbacks
         *===========================*/
        /*
         * Callback class method for camera image
         */
        void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);

        /*
         * Callback class method for apriltag detection
         */
        void tagPositionCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg);

        /*===========================
         * Utilities
         *===========================*/
        /*
         * Create PoseStamped messages from angles and frames
         */
        std::vector<geometry_msgs::PoseStamped> createPoseStampedVectorFromAngles(std::vector<double> angles);

        /*
         * Create JointState message from angles vector
         */
        sensor_msgs::JointState createJointStateFromAngles(std::vector<double> angles);

        /*
         * Check if tag ID is detected
         */
        bool isTagDetected(int tag_id);

        /*
         * Publish target poses from vector
         */
        void publishTargetPoses(std::vector<geometry_msgs::PoseStamped> poses);

};

#endif  // APRILTAG_TRACKERRR_TAG_TRACKER_H