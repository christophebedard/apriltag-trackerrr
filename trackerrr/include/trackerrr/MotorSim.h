#ifndef MOTORSIM_H
#define MOTORSIM_H

/**
 * \file MotorSim.h
 * \brief MotorSim class declaration
 * \author christophebedard
 */

#include <ros/ros.h>
#include "trackerrr/BoundedPID.h"

/** \class MotorSim
 * \brief class which simulates a motor which is controlled with a PID.
 */
class MotorSim
{
    public:
        /**
         * \brief MotorSim constructor.
         *
         * \param posMin : minimum position value (rad).
         * \param posMax : maximum position value (rad).
         * \param posInit : initial position (rad).
         * \param p : proportional gain.
         * \param i : integral gain.
         * \param d : derivative gain.
         * \param velMax : maximum angular velocity.
         */
        MotorSim(double posMin, double posMax, double posInit, double p, double i, double d, double velMax);
        
        /**
         * \brief MotorSim destructor.
         */
        ~MotorSim();

        /**
         * \brief Set setpoint value.
         *
         * \param setpoint : new angle position setpoint.
         */
        void setSetpoint(double setpoint);

        /**
         * \brief Get current position.
         *
         * \return current position.
         */
        double getPosition();

        /**
         * \brief Update velocity and position.
         *
         * \param dt : elapsed time since last update.
         */
        void update(ros::Duration dt);

    private:
        double posSetpoint_; /**< current goal position */
        double posCurrent_; /**< current position */
        double posInit_; /**< initial position */
        double posMin_; /**< minimum position */
        double posMax_; /**< maximum position */
        
        double vel_; /**< current angular velocity */
        BoundedPID* pid_vel_; /**< PID */

        /**
         * \brief Update position.
         *
         * \param dt : elapsed time since last update.
         */
        void updatePosition(ros::Duration dt);

        /**
         * \brief Update velocity.
         *
         * \param dt : elapsed time since last update.
         */
        void updateVel(ros::Duration dt);

};

#endif // MOTORSIM_H