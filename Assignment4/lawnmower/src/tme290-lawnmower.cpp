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

using namespace std;
using namespace tme290::grass;

#include "cluon-complete.hpp"
#include "tme290-sim-grass-msg.hpp"

#define WALL_LENGTH   (22)
#define WALL_WIDTH    (17)

#define within_boundary   control.command(0)

bool record_position{true};
bool home_position{false};

bool move_right{true};
bool return_charging_dock{false};

bool charge_level{false};
bool charge_battery{false};

bool stop_execution{false};

uint32_t position_x{0};
uint32_t position_y{0};

const double rain_threshold{0.4};
string filename_hourly_average{"hourly_average_data"};
string filename_battery{"battery_data"};

void generate_csv_file(string& fileName);
void add_data_csv(const string& line, const string& fileName);

double battery_level(Sensors& msg);
void charging_battery(Sensors& msg, Control& control);

void cutting_grass(Sensors& msg, Control& control);
void home_position(Sensors& msg, Control& control);


int32_t main(int32_t argc, char **argv) 
{
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if(0 == commandlineArguments.count("cid")) {
    cerr << argv[0] 
      << " is a lawn mower control algorithm." << endl;
    cerr << "Usage:   " << argv[0] << " --cid=<OpenDLV session>" 
      << "[--verbose]" << endl;
    cerr << "Example: " << argv[0] << " --cid=111 --verbose" << endl;
    retCode = 1;
  } else {
    bool const verbose{commandlineArguments.count("verbose") != 0};
    uint16_t const cid = stoi(commandlineArguments["cid"]);
    
    cluon::OD4Session od4{cid};

		double batteryLimit{0};
		int32_t timeStep{0};

		generate_csv_file(filename_hourly_average);
		generate_csv_file(filename_battery);

    auto onSensors{[&od4, &timeStep, &batteryLimit](cluon::data::Envelope &&envelope)
      {
        auto msg = cluon::extractMessage<Sensors>(
            move(envelope));
        timeStep++;

        Control control;

				batteryLimit = battery_level(msg);

        if(msg.battery() < batteryLimit || (msg.rain() >= rain_threshold && !home_position && msg.i() + msg.j() != 0)) {
          return_charging_dock = true;
        } 
        
        if((return_charging_dock && !charge_level) || charge_battery || charge_level) {
          charging_battery(msg, control);
        } else if(home_position){
          home_position(msg, control);
        } else {
          if(timeStep % 2 == 0) {
            within_boundary(control);
          } else {
            cutting_grass(msg, control);
          }
        }

        cout << "Rain reading " << msg.rain() << ", direction (" <<
         msg.rainCloudDirX() << ", " << msg.rainCloudDirY() << ")" << endl; 

        od4.send(control);
				if(msg.time() == 288000) {
          stop_execution = true;
        }
      }};

    auto onStatus{[&verbose](cluon::data::Envelope &&envelope)
      {
        auto msg = cluon::extractMessage<Status>(
            move(envelope));
        if(verbose) {
          cout  << "Status at time " << msg.time() << ": " 
            << msg.grassMean() << "/" << msg.grassMax() << endl;
        }

				if(msg.time() % 60 == 0) {
          string line{to_string(msg.time() / 60) + "," +
                           to_string(msg.grassMean())};
          add_data_csv(line, filename_hourly_average);
        }

      }};

    od4.dataTrigger(Sensors::ID(), onSensors);
    od4.dataTrigger(Status::ID(), onStatus);

    if(verbose) {
      cout << "All systems ready, let's cut some grass!" << endl;
    }

    Control control;
    control.command(0);
    od4.send(control);

		while (od4.isRunning() && !stop_execution) {
      this_thread::sleep_for(chrono::milliseconds(1000));
    }

    retCode = 0;
  }
  return retCode;
}


void generate_csv_file(string& fileName) 
{
  auto tick = chrono::system_clock::now();
  time_t time = chrono::system_clock::to_time_t(tick);

  stringstream ss;
  ss << put_time(gmtime(&time), "%m-%d_%H-%M");

  fileName = "data/" + fileName + "_" + ss.str() + ".csv";
	return;
}

void add_data_csv(const string& line, const string& fileName) 
{
  ofstream file(fileName, ios_base::app);

  if(!file.is_open()) {
    file.open(fileName);
    file.close();

    file.open(fileName, ios_base::app);
  }

  if(file) {
    file << line << '\n';
    file.close();
  }
}


double battery_level(Sensors& msg) 
{
  uint32_t x{msg.i()}; 
  uint32_t y{msg.j()};
  int32_t distX{0};
  int32_t distY{0};
  double distance{0};

  if(y < WALL_WIDTH) {
    distance = sqrt(pow(x, 2) + pow(y, 2));
  } else {
    distX = abs(x - WALL_LENGTH);
    distY = abs(y - WALL_WIDTH);
    distance = sqrt(pow(distX, 2) + pow(distY, 2));
    distance += sqrt(pow(WALL_LENGTH, 2) + pow(WALL_WIDTH, 2));
  }

  return distance*0.005;
}

void charging_battery(Sensors& msg, Control& control)
{
  uint32_t x{msg.i()}; 
  uint32_t y{msg.j()};

  charge_battery = x != 0 && y != 0;

  if(record_position) {
    position_x = x;
    position_y = y;
    record_position = false;
  }
  if(x == 0 && y == 0 && !charge_level) {
      string line{to_string(msg.time()) + "," +
                  to_string(msg.battery())};
      add_data_csv(line, filename_battery);
  }

  if(x == 0 && y == 0 && msg.battery() < 1.0) {
    charge_level = true;
    return_charging_dock = false;
    control.command(0);
  } else if(msg.battery() >= 1.0) {
    charge_level = false;
    record_position = true;
    home_position = true;
  } else if(y < WALL_WIDTH ) {
    if(y == 0) {
      control.command(8);
    } else if(x == 0) {
      control.command(2);
    } else {
      control.command(1);
    }
  } else {
    if(x > WALL_LENGTH) {
      control.command(1);
    } else if(x == WALL_LENGTH) {
      control.command(2);
    } else  {
      if(y == 18) {
        control.command(4);
      } else {
        control.command(3);
      }
    }
  }
}


void cutting_grass(Sensors& msg, Control& control) 
{
  uint32_t x{msg.i()}; 
  uint32_t y{msg.j()};

  if(x == 39 && y == 39) {
    position_x = 23;
    position_y = 18;
    record_position = false;
    return_charging_dock = true;
    move_right = true;
    return;
  } else if(x == 39 && y == 18) {
    position_x = 0;
    position_y = 0;
    record_position = false;
    return_charging_dock = true;
    move_right = true;
    return;
  } 

  if(move_right) {
    if(x < 39) {
      control.command(4);
    } else {
      move_right = false;
      control.command(6);
    }
  } else {
    if(x == 0) {
      move_right = true;
      control.command(6);
    } else if(y == WALL_WIDTH && x == WALL_LENGTH) {
      control.command(6);
    } else {
      control.command(8);
    }
  }
}

void home_position(Sensors& msg, Control& control) 
{
  uint32_t x{msg.i()};
  uint32_t y{msg.j()};

  if(x == position_x && y == position_y) {
    home_position = false;
    return;
  }
  
  if(position_y < WALL_WIDTH) {
    if(x == position_x) {
      control.command(6);
    } else if(y == position_y) {
      control.command(4);
    } else {
      control.command(5);
    }
  } else {
    if(x < WALL_LENGTH && y < 16) {
        control.command(5);
      } else if(x < WALL_LENGTH && y == 16) {
        control.command(4);
      }
    else if(position_x >= WALL_LENGTH) {
      if(x == position_x) {
        control.command(6);
      } else if(y == position_y) {
        control.command(4);
      } else {
        control.command(5);
      }
    } else {
      if(y < 18) {
        control.command(6);
      } else if(y == position_y) {
        control.command(8);
      } else if(x == position_x) {
        control.command(6);
      } else {
        control.command(7);
      }
    }
  }
}

