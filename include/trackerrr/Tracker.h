#ifndef TRACKER_H
#define TRACKER_H

/**
 * \file Tracker.h
 * \brief Tracker class declaration
 * \author christophebedard
 */

#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string JOINT_STATE_COMMAND_TOPIC_NAME = "/motors/goal_states"; /**< topic for position commands */
static const std::string TARGET_POSE_TOPIC_NAME_PREFIX = "/target_q"; /**< topic prefix for position commands pose */
static const std::string JOINT_TF_NAME_PREFIX = "/joint"; /**< topic name prefix for joint position */
static const std::string TARGET_TF_NAME = "/target"; /**< tf name for target (which derived class will broadcast) */

/** \class Tracker
 * \brief abstract class for tracking.
 *
 * Expects a derived class to publish /target tf, then it updates the angles_ vector with joint position commands.
 */
class Tracker
{
    public:
        /**
         * \brief Tracker constructor.
         *
         * \param n : node handle.
         */
        Tracker(ros::NodeHandle& n);
        
        /**
         * \brief Tracker destructor.
         */
        ~Tracker();

        /**
         * \brief ROS spin. Called only once (by node); contains ROS while loop.
         */
        virtual void spin() =0;

    protected:
        ros::NodeHandle n_; /**< node handle */

        int dof_; /**< degrees of freedom */
        std::vector<std::string> frames_; /**< vector containing DOF frames */
        std::vector<double> angles_; /**< vector containing target angles for next command */

        tf::TransformListener tf_listener_; /**< tf listener */
        tf::TransformBroadcaster tf_br_; /**< tf broadcaster */
        
        /*===========================
         * Update
         *===========================*/
        /**
         * \brief Update; called every spinOnce().
         */
        virtual void update();

        /**
         * \brief ROS spin once, called on every loop.
         */
        virtual void spinOnce() =0;

        /*===========================
         * Utilities
         *===========================*/
        /**
         * \brief Check if target is currently detected.
         */
        virtual bool isTargetDetected() const =0;
    
    private:
        /*===========================
         * Publishers
         *===========================*/
        ros::Publisher joint_state_command_pub_; /**< publisher for joint state command */
        std::vector<ros::Publisher> target_pose_pubs_; /**< publishers for target pose */

        /*===========================
         * Utilities
         *===========================*/
        /**
         * \brief Clears command angles from vector.
         */
        void clearCommandAngles();
        
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
         * \brief Publish target poses from vector.
         *
         * \param poses : vector of posestamped messages to publish.
         */
        void publishTargetPoses(const std::vector<geometry_msgs::PoseStamped>& poses);
};


#endif  // TRACKERRR_TRACKER_H