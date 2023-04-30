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

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#define INPUT_ID_LEFT_WHEEL  0
#define INPUT_ID_RIGHT_WHEEL 1

#define T1 3000000.0f
#define T2 10000000.0f

int32_t main(int32_t argc, char **argv) 
{ 
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

  // Check that the mandatory command line arguments are present.
  if (0 == commandlineArguments.count("cid")
      || 0 == commandlineArguments.count("freq")) {
    std::cerr << argv[0] << " tests the Kiwi platform by sending actuation "
      << "commands and reacting to sensor input." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --freq=<Integration frequency> "
      << "--cid=<OpenDaVINCI session> [--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --freq=10 --cid=111" << std::endl;
    retCode = 1;
  } else {
    // Extract the command line arguments.
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    uint16_t const CID = std::stoi(commandlineArguments["cid"]);
    float const FREQ = std::stof(commandlineArguments["freq"]);
    
    opendlv::sim::KinematicState kinematicState{};

    auto onKinematicState{[&VERBOSE, &kinematicState](cluon::data::Envelope &&envelope) {
      uint32_t const senderStamp = envelope.senderStamp();
      if(senderStamp == INPUT_ID_LEFT_WHEEL || senderStamp == INPUT_ID_RIGHT_WHEEL)
      {
        auto k_state = cluon::extractMessage<opendlv::sim::KinematicState>(std::move(envelope));
        kinematicState.vx(k_state.vx());
        kinematicState.vy(k_state.vy());
        kinematicState.vz(k_state.vz());
        kinematicState.rollRate(k_state.rollRate());
        kinematicState.pitchRate(k_state.pitchRate());
        kinematicState.yawRate(k_state.yawRate());

        if (VERBOSE) {
          std::cout << "Kinematic state : "
            << " has velocity [vx=" << kinematicState.vx() 
            << ", vy=" << kinematicState.vy() << ", vz=" << kinematicState.vz() 
            << "] with the rotation rate [rollRate=" 
            << kinematicState.rollRate() << ", pitchRate=" 
            << kinematicState.pitchRate() << ", yawRate=" 
            << kinematicState.yawRate() << "]." << std::endl;
        }
      }
    }};
    
    // Createing the OD4 session.
    cluon::OD4Session od4{CID};
    od4.dataTrigger(opendlv::sim::kinematicState::ID(), onKinematicState);

    // Lambda function to run at a specified frequency.
    auto atFrequency{[&VERBOSE, &od4]() -> bool
    {
      cluon::data::TimeStamp sampleTime = cluon::time::now();
      float sampleTime_float = static_cast<float>(sampleTime.microseconds());

      opendlv::proxy::AxleAngularVelocityRequest axle_ang_vel_left, axle_ang_vel_right;
      if(sampleTime_float >= 0.0 && sampleTime_float <= T1)
      {
        axle_ang_vel_left.axleAngularVelocity(0.0f);
        axle_ang_vel_right.axleAngularVelocity(v0*(sampleTime_float/T1));
      } 
      else if(sampleTime_float > T1 && sampleTime_float <= T2)
      {
        axle_ang_vel_left.axleAngularVelocity(v0*((sampleTime_float - T1)/T2));
        axle_ang_vel_right.axleAngularVelocity(v0);
      } 
      else
      {
        axle_ang_vel_left.axleAngularVelocity(0.0f);
        axle_ang_vel_right.axleAngularVelocity(0.0f); 
      }
      od4.send(axle_ang_vel_left,  sampleTime, INPUT_ID_LEFT_WHEEL);
      // sleep(1);
      od4.send(axle_ang_vel_right, sampleTime, INPUT_ID_RIGHT_WHEEL);

      return true;
    }};

    // This will block until Ctrl+C is pressed.
    od4.timeTrigger(FREQ, atFrequency);
  }
  
  return retCode;
}

