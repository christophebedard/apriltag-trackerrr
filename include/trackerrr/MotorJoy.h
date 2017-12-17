#ifndef MOTORJOY_H
#define MOTORJOY_H

/**
 * \file MotorJoy.h
 * \brief MotorJoy class declaration
 * \author christophebedard
 */

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"

static const double LOOP_RATE = 30.0;

/** \class MotorJoy
 * \brief class which helps control motors with a controller/joy.
 */
class MotorJoy
{
    public:
        /**
         * \brief MotorJoy constructor.
         *
         * \param posMin : minimum position value (rad).
         */
        MotorJoy(ros::NodeHandle& n);
        
        /**
         * \brief MotorJoy destructor.
         */
        ~MotorJoy();

        /**
         * \brief ROS spin. Called only once (by node); contains ROS while loop.
         */
        void spin();

    private:
        ros::NodeHandle& n_; /**< node handle */
        ros::Rate rate_; /**< rate */

        int dof_; /**< degrees of freedom */

        std::vector<double> posCurrent_; /**< current positions */
        double posInit_; /**< initial position */
        
        std::vector<double> vel_; /**< current angular velocities */

        /*===========================
         * Publishers
         *===========================*/
        ros::Publisher presentJointState_pub_; /**< publisher for joint state command */

        /*===========================
         * Subscribers
         *===========================*/
        ros::Subscriber vel_sub_; /**< controller/joy vel subscriber */

        /*===========================
         * Callbacks
         *===========================*/
        /**
         * \brief Callback class method for Twist message.
         *
         * \param msg : constptr to Twist message.
         */
        void velCallback(const geometry_msgs::Twist::ConstPtr& msg);
        
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

        /**
         * \brief Update position.
         */
        void updatePosition();

        /**
         * \brief Publish goal state.
         */
        void publishGoalJointState();

};

#endif // MOTORJOY_H