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
#include "asctec_autopilot/autopilot.h"
#include "asctec_autopilot/telemetry.h"
#include "asctec_autopilot/serialinterface.h"
#include "asctec_autopilot/IMUCalcData.h"
#include "asctec_autopilot/LLStatus.h"

namespace asctec
{
  AutoPilot::AutoPilot ()
  {
//      std::string port;
//      int speed;
//      nh_.param<std::string>("serial_port",    port,    "/dev/ttyUSB0");
//      nh_.param<int>("serial_speed",    speed,    57600);
//      SerialInterface serialInterface = SerialInterface(port,speed);
      timer_ = nh_.createTimer (ros::Duration (1.0 / std::max (freq, 1.0)), &AutoPilot::spin, this);
  }
  AutoPilot::~AutoPilot ()
  {
    ROS_DEBUG ("Destroying AutoPilot Interface");
  }

  // enablePolling(REQ_LL_STATUS,1);



  void AutoPilot::spin (const ros::TimerEvent & e)
  {
    ROS_DEBUG("spin()");
    tele_->buildRequest ();
    tele_->requestCount_++;
    serialInterface_->getPackets(tele_);
    tele_->publishPackets();
  }

}


int main (int argc, char **argv)
{
  ros::init (argc, argv, "autopilot");
  ros::NodeHandle n;
//  ros::Publisher calcdataPublisher;
//  calcdataPublisher = n.advertise <asctec_autopilot::IMUCalcdata >("PelicanIMUCalcdata", 100);
  ros::Rate loop_rate (10);

  asctec::AutoPilot::AutoPilot autopilot;
  asctec::SerialInterface::SerialInterface serial;
  asctec::Telemetry::Telemetry tele;
  autopilot.tele_ = &tele;
  autopilot.tele_->enablePolling(asctec::RequestTypes::IMU_CALCDATA,1);
  autopilot.tele_->enablePolling(asctec::RequestTypes::IMU_RAWDATA,2,0);
  autopilot.tele_->enablePolling(asctec::RequestTypes::LL_STATUS,2,1);
  autopilot.tele_->enablePolling(asctec::RequestTypes::RC_DATA,5);
  autopilot.serialInterface_ = &serial;
  while (ros::ok ())
  {
    // sleep
    ros::spinOnce ();
    loop_rate.sleep ();
  }
  return 0;
}
