/*
 * Copyright (C) 2019 Ola Benderius
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

#include <iostream>
#include <ctime> 			// for time_t, gmtime, strftime
#include <chrono>			// for system_clock, to_time_t
#include <sstream>		// for stringstream
#include <iomanip>		// for put_time
#include <fstream> 		// for ofstream
#include <string>

#include "cluon-complete.hpp"
#include "tme290-sim-grass-msg.hpp"

bool doStorePos{true};
bool returnToPos{false};
bool towardsRight{true};
bool charging{false};
bool goingToCharge{false};
bool goToCharger{false};
bool stopRun{false};

uint32_t storedX{0};
uint32_t storedY{0};

const double rainLimit{0.4};
std::string hourlyMeanFileName{"hourlyData"};
std::string batteryFileName{"batteryData"};

void initCSVFile(std::string& fileName) 
{
  auto now = std::chrono::system_clock::now();
  std::time_t currentTime = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::gmtime(&currentTime), "%m-%d_%H-%M");

  fileName = "data/" + fileName + "_" + ss.str() + ".csv";
	return;
}

void appendToCSV(const std::string& line, const std::string& fileName) 
{
  std::ofstream file(fileName, std::ios_base::app);

  if (!file.is_open()) {
    file.open(fileName);
    file.close();

    file.open(fileName, std::ios_base::app);
  }

  if (file) {
    file << line << '\n';
    file.close();
  }
}

double getBatteryLimit(tme290::grass::Sensors& msg) 
{
  uint32_t x{msg.i()}; 
  uint32_t y{msg.j()};
  int32_t distX{0};
  int32_t distY{0};
  double distance{0};

  if (y < 17) {
    distance = sqrt(pow(x, 2) + pow(y, 2));
  } else {
    distX = abs(x - 22);
    distY = abs(y - 17);
    distance = sqrt(pow(distX, 2) + pow(distY, 2));
    distance += sqrt(pow(22, 2) + pow(17, 2));
  }

  return distance*0.005;
}

void cutGrass(tme290::grass::Sensors& msg, tme290::grass::Control& control) 
{
  uint32_t x{msg.i()}; 
  uint32_t y{msg.j()};

  if (x == 39 && y == 39) {
    storedX = 23;
    storedY = 18;
    doStorePos = false;
    goToCharger = true;
    towardsRight = true;
    return;
  } else if (x == 39 && y == 18) {
    storedX = 0;
    storedY = 0;
    doStorePos = false;
    goToCharger = true;
    towardsRight = true;
    return;
  } 

  if (towardsRight) {
    if (x < 39) {
      control.command(4);
    } else {
      towardsRight = false;
      control.command(6);
    }
  } else {
    if (x == 0) {
      towardsRight = true;
      control.command(6);
    } else if (y == 17 && x == 22) {
      control.command(6);
    } else {
      control.command(8);
    }
  }
}

void stayInCell(tme290::grass::Control& control) 
{
  control.command(0);
}

void chargeRobot(tme290::grass::Sensors& msg, tme290::grass::Control& control) 
{
  uint32_t x{msg.i()}; 
  uint32_t y{msg.j()};

  goingToCharge = x != 0 && y != 0;

  if (doStorePos) {
    storedX = x;
    storedY = y;
    doStorePos = false;
  }
  if (x == 0 && y == 0 && !charging) {
      std::string line{std::to_string(msg.time()) + "," +
                       std::to_string(msg.battery())};
      appendToCSV(line, batteryFileName);
  }

  if (x == 0 && y == 0 && msg.battery() < 1.0) {
    charging = true;
    goToCharger = false;
    control.command(0);
  } else if (msg.battery() >= 1.0) {
    charging = false;
    doStorePos = true;
    returnToPos = true;
  } else if (y < 17 ) {
    if (y == 0) {
      control.command(8);
    } else if (x == 0) {
      control.command(2);
    } else {
      control.command(1);
    }
  } else {
    if (x > 22) {
      control.command(1);
    } else if (x == 22) {
      control.command(2);
    } else  {
      if (y == 18) {
        control.command(4);
      } else {
        control.command(3);
      }
    }
  }
}

void returnToPosition(tme290::grass::Sensors& msg, tme290::grass::Control& control) 
{
  uint32_t x{msg.i()};
  uint32_t y{msg.j()};

  if (x == storedX && y == storedY) {
    returnToPos = false;
    return;
  }
  
  if (storedY < 17) {
    if (x == storedX) {
      control.command(6);
    } else if (y == storedY) {
      control.command(4);
    } else {
      control.command(5);
    }
  } else {
    if (x < 22 && y < 16) {
        control.command(5);
      } else if (x < 22 && y == 16) {
        control.command(4);
      }
    else if (storedX >= 22) {
      if (x == storedX) {
        control.command(6);
      } else if (y == storedY) {
        control.command(4);
      } else {
        control.command(5);
      }
    } else {
      if (y < 18) {
        control.command(6);
      } else if (y == storedY) {
        control.command(8);
      } else if (x == storedX) {
        control.command(6);
      } else {
        control.command(7);
      }
    }
  }
}

int32_t main(int32_t argc, char **argv) 
{
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid")) {
    std::cerr << argv[0] 
      << " is a lawn mower control algorithm." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDLV session>" 
      << "[--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --verbose" << std::endl;
    retCode = 1;
  } else {
    bool const verbose{commandlineArguments.count("verbose") != 0};
    uint16_t const cid = std::stoi(commandlineArguments["cid"]);
    
    cluon::OD4Session od4{cid};

		int32_t timeStep{0};		// TODO: some variable initializations
		double batteryLimit{0};

		// TODO: some CSV operations here...
		initCSVFile(hourlyMeanFileName);
		initCSVFile(batteryFileName);

    auto onSensors{[&od4, &someVariable](cluon::data::Envelope &&envelope)
      {
        auto msg = cluon::extractMessage<tme290::grass::Sensors>(
            std::move(envelope));
        timeStep++;		// TODO: replace some variable

        tme290::grass::Control control;

				batteryLimit = getBatteryLimit(msg);

        if (msg.battery() < batteryLimit || (msg.rain() >= rainLimit && !returnToPos && msg.i() + msg.j() != 0)) {
          goToCharger = true;
        } 
        
        if ((goToCharger && !charging) || goingToCharge || charging) {
          chargeRobot(msg, control);
        } else if (returnToPos){
          returnToPosition(msg, control);
        } else {
          if (timeStep % 2 == 0) {
            stayInCell(control);
          } else {
            cutGrass(msg, control);
          }
        }

        std::cout << "Rain reading " << msg.rain() << ", direction (" <<
         msg.rainCloudDirX() << ", " << msg.rainCloudDirY() << ")" << std::endl; 

        od4.send(control);
				// TODO: some stop message
				if (msg.time() == 288000) {
          stopRun = true;
        }
      }};

    auto onStatus{[&verbose](cluon::data::Envelope &&envelope)
      {
        auto msg = cluon::extractMessage<tme290::grass::Status>(
            std::move(envelope));
        if (verbose) {
          std::cout  << "Status at time " << msg.time() << ": " 
            << msg.grassMean() << "/" << msg.grassMax() << std::endl;
        }

				// TODO: some CSV thing
				if (msg.time() % 60 == 0) {
          std::string line{std::to_string(msg.time() / 60) + "," +
                           std::to_string(msg.grassMean())};
          appendToCSV(line, hourlyMeanFileName);
        }

      }};

    od4.dataTrigger(tme290::grass::Sensors::ID(), onSensors);
    od4.dataTrigger(tme290::grass::Status::ID(), onStatus);

    if (verbose) {
      std::cout << "All systems ready, let's cut some grass!" << std::endl;
    }

    tme290::grass::Control control;
    control.command(0);
    od4.send(control);

		// TODO: some minor stuff in while
    while (od4.isRunning() && !stopRun) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    retCode = 0;
  }
  return retCode;
}
