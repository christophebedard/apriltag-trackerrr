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
        virtual bool isTargetDetected() const;

        /**
         * \brief Get tf name of target to track (implementation).
         */
        virtual std::string getTargetTfName() const;

    private:
        int targetTagID_; /**< id of tag to target */

        std::vector<apriltags_ros::AprilTagDetection> detected_tags_; /**< vector containing detection messages */
        
        /*===========================
         * Subscribers
         *===========================*/
        ros::Subscriber tag_detect_sub_; /**< Apriltag detection subscriber */

        /*===========================
         * Callbacks
         *===========================*/
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