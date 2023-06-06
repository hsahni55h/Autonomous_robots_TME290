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

#define WALL_LENGTH   (22)
#define WALL_WIDTH    (17)


bool record_position{true};
bool home_position{false};
bool move_right{true};
bool charge_level{false};
bool charge_battery{false};
bool return_charging_dock{false};
bool stop_execution{false};

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

  if (y < WALL_WIDTH) {
    distance = sqrt(pow(x, 2) + pow(y, 2));
  } else {
    distX = abs(x - WALL_LENGTH);
    distY = abs(y - WALL_WIDTH);
    distance = sqrt(pow(distX, 2) + pow(distY, 2));
    distance += sqrt(pow(WALL_LENGTH, 2) + pow(WALL_WIDTH, 2));
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
    record_position = false;
    return_charging_dock = true;
    move_right = true;
    return;
  } else if (x == 39 && y == 18) {
    storedX = 0;
    storedY = 0;
    record_position = false;
    return_charging_dock = true;
    move_right = true;
    return;
  } 

  if (move_right) {
    if (x < 39) {
      control.command(4);
    } else {
      move_right = false;
      control.command(6);
    }
  } else {
    if (x == 0) {
      move_right = true;
      control.command(6);
    } else if (y == WALL_WIDTH && x == WALL_LENGTH) {
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

  charge_battery = x != 0 && y != 0;

  if (record_position) {
    storedX = x;
    storedY = y;
    record_position = false;
  }
  if (x == 0 && y == 0 && !charge_level) {
      std::string line{std::to_string(msg.time()) + "," +
                       std::to_string(msg.battery())};
      appendToCSV(line, batteryFileName);
  }

  if (x == 0 && y == 0 && msg.battery() < 1.0) {
    charge_level = true;
    return_charging_dock = false;
    control.command(0);
  } else if (msg.battery() >= 1.0) {
    charge_level = false;
    record_position = true;
    home_position = true;
  } else if (y < WALL_WIDTH ) {
    if (y == 0) {
      control.command(8);
    } else if (x == 0) {
      control.command(2);
    } else {
      control.command(1);
    }
  } else {
    if (x > WALL_LENGTH) {
      control.command(1);
    } else if (x == WALL_LENGTH) {
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
    home_position = false;
    return;
  }
  
  if (storedY < WALL_WIDTH) {
    if (x == storedX) {
      control.command(6);
    } else if (y == storedY) {
      control.command(4);
    } else {
      control.command(5);
    }
  } else {
    if (x < WALL_LENGTH && y < 16) {
        control.command(5);
      } else if (x < WALL_LENGTH && y == 16) {
        control.command(4);
      }
    else if (storedX >= WALL_LENGTH) {
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

    auto onSensors{[&od4, &timeStep, &batteryLimit](cluon::data::Envelope &&envelope)
      {
        auto msg = cluon::extractMessage<tme290::grass::Sensors>(
            std::move(envelope));
        timeStep++;		// TODO: replace some variable

        tme290::grass::Control control;

				batteryLimit = getBatteryLimit(msg);

        if (msg.battery() < batteryLimit || (msg.rain() >= rainLimit && !home_position && msg.i() + msg.j() != 0)) {
          return_charging_dock = true;
        } 
        
        if ((return_charging_dock && !charge_level) || charge_battery || charge_level) {
          chargeRobot(msg, control);
        } else if (home_position){
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
          stop_execution = true;
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
    while (od4.isRunning() && !stop_execution) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    retCode = 0;
  }
  return retCode;
}
