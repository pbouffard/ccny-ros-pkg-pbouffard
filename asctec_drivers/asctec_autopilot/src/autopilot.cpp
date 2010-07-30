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

#include "asctec_autopilot/autopilot.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "autopilot");
  asctec::AutoPilot::AutoPilot autopilot;
  ros::spin();
  return 0;
}

namespace asctec
{
  AutoPilot::AutoPilot ()
  {
    ROS_INFO ("Creating AutoPilot Interface");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // **** get parameters
    
    if (!nh_private.getParam ("freq", freq_))
      freq_ = 50.0;

    if (!nh_private.getParam ("port", port_))
      port_ = "/dev/ttyUSB0";

    if (!nh_private.getParam ("speed", speed_))
      speed_ = 57600;

    if(freq_ <= 0.0) ROS_FATAL("Invalid frequency param");

    ros::Duration d(1.0/freq_);

    // **** set up intefaces

    serialInterface_ = new asctec::SerialInterface::SerialInterface(port_, speed_);
    telemetry_ = new asctec::Telemetry::Telemetry();

    // **** enable polling

    telemetry_->enablePolling(asctec::RequestTypes::LL_STATUS, 10, 2) ;
    //telemetry_->enablePolling(asctec::RequestTypes::IMU_RAWDATA, 10, 2);
    //telemetry_->enablePolling(asctec::RequestTypes::IMU_CALCDATA, 10, 4);
    //telemetry_->enablePolling(asctec::RequestTypes::RC_DATA, 10, 6);
    //telemetry_->enablePolling(asctec::RequestTypes::CONTROLLER_OUTPUT, 10, 8);
    //telemetry_->enablePolling(asctec::RequestTypes::GPS_DATA, 10);
    telemetry_->enableControl( 10 , 4);
    telemetry_->CTRL_INPUT_.pitch = 0;
    telemetry_->CTRL_INPUT_.roll = 0;
    telemetry_->CTRL_INPUT_.yaw = 0;
    telemetry_->CTRL_INPUT_.thrust = 0;
    telemetry_->CTRL_INPUT_.ctrl = 0x0000;
    telemetry_->CTRL_INPUT_.chksum = telemetry_->CTRL_INPUT_.pitch + telemetry_->CTRL_INPUT_.roll + telemetry_->CTRL_INPUT_.yaw + telemetry_->CTRL_INPUT_.thrust + telemetry_->CTRL_INPUT_.ctrl + 0xAAAA;
    ROS_INFO("CTRL_INPUT_.chksum: %i", (short) telemetry_->CTRL_INPUT_.chksum);

    timer_ = nh_private.createTimer (d, &AutoPilot::spin, this);
  }

  AutoPilot::~AutoPilot ()
  {
    ROS_INFO ("Destroying AutoPilot Interface");
  }

  void AutoPilot::spin (const ros::TimerEvent& e)
  {
    ROS_INFO("spin()");
    
    telemetry_->buildRequest ();
    telemetry_->requestCount_++;
    serialInterface_->getPackets(telemetry_);
    telemetry_->publishPackets();
    serialInterface_->sendCommand(telemetry_);
    telemetry_->controlCount_++;
  }
}

