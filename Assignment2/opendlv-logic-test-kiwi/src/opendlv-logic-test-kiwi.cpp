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

#define T1 3.0f
#define T2 10.0f

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
    
    auto onKinematicState{[](cluon::data::Envelope &&envelope) {
      uint32_t const senderStamp = envelope.senderStamp();
      if(senderStamp == INPUT_ID_LEFT_WHEEL || senderStamp == INPUT_ID_RIGHT_WHEEL)
      {
        auto kinematicState = cluon::extractMessage<opendlv::sim::KinematicState>(std::move(envelope));

        if (VERBOSE) {
          std::cout << "Kinematic state with id " << FRAME_ID
            << " is at velocity [vx=" << kinematicState.vx() 
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
    auto atFrequency{[&VERBOSE, &behavior, &od4]() -> bool
    {
      cluon::data::TimeStamp sampleTime = cluon::time::now();

      opendlv::proxy::AxleAngularVelocityRequest axle_ang_vel_left, axle_ang_vel_right;
      if(sampleTime >= 0.0 && sampleTime <= T1)
      {
        axle_ang_vel_left = 0.0f;
        axle_ang_vel_right = v0*(sampleTime/T1);
      } 
      else if(sampleTime > T1 && sampleTime <= T2)
      {
        axle_ang_vel_left = v0*((sampleTime - T1)/T2);
        axle_ang_vel_right = v0;
      } 
      else
      {
        axle_ang_vel_left = 0.0f;
        axle_ang_vel_right = 0.0f; 
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

