/**
 * \file MotorSim.cpp
 * \brief MotorSim class implementation
 * \author christophebedard
 */

#include "trackerrr/MotorSim.h"

MotorSim::MotorSim(double posMin, double posMax, double posInit, double p, double i, double d, double velMax)
    : posMin_(posMin),
      posMax_(posMax),
      posInit_(posInit),
      posCurrent_(posInit),
      posSetpoint_(posInit),
{
    // setup PID
    pid_vel_ = new BoundedPID(-velMax, velMax, p, i, d, 0.0, -0.0, false);
}

MotorSim::~MotorSim() {
    delete pid_vel_;
}


void MotorSim::setSetpoint(double setpoint) {
    posSetpoint_ = setpoint;
}

double MotorSim::getPosition() {
    return posCurrent_;
}

/*===========================
 * Update
 *===========================*/

void MotorSim::updatePosition(ros::Duration dt) {
    posCurrent_ += vel_ * dt.toSec();
}

void MotorSim::updateVel(ros::Duration dt) {
    vel_ = pid_vel_->computeCommand(posSetpoint_ - posCurrent_, dt);
}

void Quad::update(ros::Duration dt) {
    updateVel(dt);
    updatePosition(dt);
}
