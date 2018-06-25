#ifndef DYNAMIXELSIM_H
#define DYNAMIXELSIM_H

/**
 * \file DynamixelSim.h
 * \brief DynamixelSim class declaration
 * \author christophebedard
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "trackerrr/MotorSim.h"

static const double LOOP_RATE = 30.0;
static const double PI = 3.1415;
static const double DYNAMIXEL_POSITION_ANGLE_RESOLUTION = (5.0/3.0)*PI;
static const double DYNAMIXEL_POSITION_MIN = -DYNAMIXEL_POSITION_ANGLE_RESOLUTION/2.0;
static const double DYNAMIXEL_POSITION_MAX = DYNAMIXEL_POSITION_ANGLE_RESOLUTION/2.0;

/**
 * \class DynamixelSim
 * \brief class which simulates one or many dynamixel motors
 *
 * Emulates dynamixel_controller.
 */
class DynamixelSim
{
    public:
        /**
         * \brief DynamixelSim constructor
         *
         * \param n     node handle
         */
        DynamixelSim(ros::NodeHandle& n);
        
        /**
         * \brief DynamixelSim destructor
         */
        ~DynamixelSim();

        /**
         * \brief ROS spin. Called only once (by node); contains ROS while loop.
         */
        void spin();

    private:
        ros::NodeHandle n_; ///< node handle
        ros::Rate rate_; ///< loop rate

        int dof_; ///< degrees of freedom
        std::vector<MotorSim*> motors_; ///< motors
        std::vector<double> positions_; ///< motors positions

        std::string presentJointStateTopic_; ///< present state topic
        std::string goalJointStateTopic_; ///< goal state topic

        /*===========================
         * Subscribers
         *===========================*/
        ros::Subscriber goalJointState_sub_; ///< goal jointstate subscriber

        /*===========================
         * Publishers
         *===========================*/
        ros::Publisher presentJointState_pub_; ///< present jointstate publisher

        /*===========================
         * Callbacks
         *===========================*/
        /**
         * \brief Goal JointState callback
         *
         * \param msg   constptr to JointState message
         */
        void goalJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

        /*===========================
         * Update
         *===========================*/
        /**
         * \brief Update
         */
        void update();

        /**
         * \brief Update motor velocities
         */
        void updateMotors();

        /**
         * \brief Update motor positions
         */
        void updatePositions();

        /**
         * \brief Publish present jointstate
         */
        void publishPresentJointState();

        /**
         * \brief ROS spin once, called on every loop
         */
        void spinOnce();
};

#endif // DYNAMIXELSIM_H