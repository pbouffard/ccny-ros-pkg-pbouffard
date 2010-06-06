/*
 *  AscTec Autopilot Interface
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  William Morris <morris@ee.ccny.cuny.edu>
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>

#include <ros/ros.h>

#include "asctec_autopilot/crc16.h"
#include "asctec_autopilot/serial_interface.h"
#include "asctec_autopilot/asctec_autopilot.h"

namespace asctec_autopilot
{
  AutoPilot::AutoPilot ()
  {
//    nh_.param<std::string>("port",    port,    "/dev/ttyUSB0");
//    SerialInterface serialInterface_ = SerialInterface("/dev/ttyUSB0",57600);
    SerialInterface serialInterface_ = SerialInterface();
    pollingEnabled_ = false;
    timer_ = nh_.createTimer(ros::Duration(1.0/std::max(freq, 1.0)), &AutoPilot::spin, this);
  }
  AutoPilot::~AutoPilot ()
  {
  }

  // enablePolling(REQ_LL_STATUS,1);
  void
  AutoPilot::enablePolling(uint16_t request, uint16_t interval)
  {
    switch(request) {
      case REQ_LL_STATUS:
        this->interval_LL_STATUS_ = interval;
      case REQ_IMU_RAWDATA:
        this->interval_IMU_RAWDATA_ = interval;
      case REQ_IMU_CALCDATA:
        this->interval_IMU_CALCDATA_ = interval;
      case REQ_RC_DATA:
        this->interval_RC_DATA_ = interval;
    }
  }

  void
  AutoPilot::buildRequest()
  {
    if (this->interval_LL_STATUS_ != 0 && ((this->requestCount_ - this->offset_LL_STATUS_) % this->interval_LL_STATUS_) == 0) {
      requestPackets_ |= this->REQ_LL_STATUS;
    }
    if (this->interval_IMU_RAWDATA_ != 0 && ((this->requestCount_ - this->offset_IMU_RAWDATA_) % this->interval_IMU_RAWDATA_) == 0) {
      requestPackets_ |= this->REQ_IMU_RAWDATA;
    }
    if (this->interval_IMU_CALCDATA_ != 0 && ((this->requestCount_ - this->offset_IMU_CALCDATA_) % this->interval_IMU_CALCDATA_) == 0) {
      requestPackets_ |= this->REQ_IMU_CALCDATA;
    }
    if (this->interval_RC_DATA_ != 0 && ((this->requestCount_ - this->offset_RC_DATA_) % this->interval_RC_DATA_) == 0) {
      requestPackets_ |= this->REQ_RC_DATA;
    }
  }
void AutoPilot::spin(const ros::TimerEvent& e)
{
//  serialInterface_.getPackets(&this);
}

}


int
main (int argc, char **argv)
{
  ros::init (argc, argv, "publisher");
  ros::NodeHandle n;
//  ros::Publisher calcdataPublisher;
//  calcdataPublisher = n.advertise <asctec_autopilot::IMUCalcdata >("PelicanIMUCalcdata", 100);
  ros::Rate loop_rate (10);

  asctec_autopilot::AutoPilot::AutoPilot autopilot;
  autopilot.enablePolling(autopilot.REQ_LL_STATUS,4);
  autopilot.enablePolling(autopilot.REQ_IMU_RAWDATA,1);
  autopilot.enablePolling(autopilot.REQ_RC_DATA,4);

  while (ros::ok ())
  {
    // sleep
    ros::spinOnce ();
    loop_rate.sleep ();
  }
  return 0;
}

