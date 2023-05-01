/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cmath>
#include <iostream>

#include "differential.hpp"

#define R (0.12f)    // radius of the robot (in m)
#define r (0.04f)    // radius of the wheel (in m)

Differential::Differential(const float vx0, const float vy0, const float yaw0) noexcept :
  m_AxleAngularVelocityLeftMutex{},
  m_AxleAngularVelocityRightMutex{},
  isAxleAngularVelocityLeftNew{false},
  isAxleAngularVelocityRightNew{false},
  AxleAngularVelocityLeft{0.0f},
  AxleAngularVelocityRight{0.0f},
  vx{vx0},
  vy{vy0},
  yaw_rate{0.0f},
  yaw{yaw0},
  vl{0.0f},
  vr{0.0f}
{
}

void Differential::setAxleAngularVelocityLeft(const opendlv::proxy::AxleAngularVelocityRequest& axle_ang_vel_left) noexcept
{
  std::lock_guard<std::mutex> lock(m_AxleAngularVelocityLeftMutex);

  // TODO: ideally a queue with timestamp must be maintained
  this->AxleAngularVelocityLeft = axle_ang_vel_left.axleAngularVelocity();
  this->isAxleAngularVelocityLeftNew = true;    // new data flag set
}

void Differential::setAxleAngularVelocityRight(const opendlv::proxy::AxleAngularVelocityRequest& axle_ang_vel_right) noexcept
{  
  std::lock_guard<std::mutex> lock(m_AxleAngularVelocityRightMutex);

  // TODO: ideally a queue with timestamp must be maintained
  this->AxleAngularVelocityRight = axle_ang_vel_right.axleAngularVelocity();
  this->isAxleAngularVelocityRightNew = true;   // new data flag set
}

opendlv::sim::KinematicState Differential::step(double dt) noexcept
{
  opendlv::sim::KinematicState k_state;   // default
  
  // if we have new data the update else state = prev_state
  if(this->isAxleAngularVelocityLeftNew == true && this->isAxleAngularVelocityRightNew == true)
  {
    // acquire MUTEX
    // std::lock_guard<std::mutex> lock1(m_AxleAngularVelocityLeftMutex);
    // std::lock_guard<std::mutex> lock2(m_AxleAngularVelocityRightMutex);
    
    // convert axle speed to wheel speed 
    this->vl = this->AxleAngularVelocityLeft * r;
    this->vr = this->AxleAngularVelocityRight * r;

    // calculations for yaw_rate vx vy from lecture notes
    float v = (this->vl + this->vr) / 2.0f;
    this->yaw_rate = (this->vr - this->vl) / (2*R);    // phi_dot
    this->vx = v;
    this->vy = 0.0f;

    // // new data flag clear
    this->isAxleAngularVelocityLeftNew = false;
    this->isAxleAngularVelocityRightNew = false;

    (void)dt;
  }
  
  // compute kinematic state    
  k_state.vx(static_cast<float>(this->vx));
  k_state.vy(static_cast<float>(this->vy));
  k_state.yawRate(static_cast<float>(this->yaw_rate));
  return k_state;
}


