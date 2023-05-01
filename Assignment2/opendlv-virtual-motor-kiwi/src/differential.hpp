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

#ifndef DIFFERENTIAL_HPP
#define DIFFERENTIAL_HPP

#include <mutex>

#include "opendlv-standard-message-set.hpp"

class Differential {
  private:
    std::mutex m_AxleAngularVelocityLeftMutex;    // to avoid shared access
    std::mutex m_AxleAngularVelocityRightMutex;

    volatile bool isAxleAngularVelocityLeftNew;   // to avoid repeated update on no new data
    volatile bool isAxleAngularVelocityRightNew;
    
    // float AxleAngularVelocityLeft;    // actual vales of axle velocities
    // float AxleAngularVelocityRight;
    volatile float vl;	// rad/s
    volatile float vr;	// rad/s
    // state
    float vx;		        // m/s
    float vy;		        // m/s
    float yaw_rate;	    // rad/s
    float yaw;          // rad
    // input
    
    
  public:
    Differential(const float vx0 = 0.0f, const float vy0 = 0.0f, const float yaw0 = 0.0f) noexcept;
    ~Differential() {};
    void setAxleAngularVelocityLeft(const opendlv::proxy::AxleAngularVelocityRequest& axle_ang_vel_left) noexcept;
    void setAxleAngularVelocityRight(const opendlv::proxy::AxleAngularVelocityRequest& axle_ang_vel_right) noexcept;
    opendlv::sim::KinematicState step(const double dt) noexcept;
};

#endif // DIFFERENTIAL_HPP


