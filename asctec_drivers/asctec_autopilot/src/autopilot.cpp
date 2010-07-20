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

    telemetry_->enablePolling(asctec::RequestTypes::IMU_CALCDATA, 1);
    //telemetry_->enablePolling(asctec::RequestTypes::IMU_RAWDATA,  2, 0);
    //telemetry_->enablePolling(asctec::RequestTypes::LL_STATUS,    2, 1);
    //telemetry_->enablePolling(asctec::RequestTypes::RC_DATA,      5);

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
  }
}

