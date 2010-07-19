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
  asctec::SerialInterface::SerialInterface serial;
  asctec::Telemetry::Telemetry tele;
  autopilot.tele_ = &tele;

  // enable polling

  autopilot.tele_->enablePolling(asctec::RequestTypes::IMU_CALCDATA, 1);
  //autopilot.tele_->enablePolling(asctec::RequestTypes::IMU_RAWDATA,  2, 0);
  //autopilot.tele_->enablePolling(asctec::RequestTypes::LL_STATUS,    2, 1);
  //autopilot.tele_->enablePolling(asctec::RequestTypes::RC_DATA,      5);

  autopilot.serialInterface_ = &serial;

  ros::spin();
/*
  while (ros::ok ())
  {
    // sleep
    ros::spinOnce ();
    loop_rate.sleep ();
  }*/
  return 0;
}

namespace asctec
{
  AutoPilot::AutoPilot ()
  {
    ROS_INFO ("Creating AutoPilot Interface");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // **** parameters
    
    if (!nh_private.getParam ("freq", freq_))
      freq_ = 50.0;

    ROS_INFO("Frequency is %f Hz", freq_);

    timer_ = nh_private.createTimer (ros::Duration(1.0 / freq_), &AutoPilot::spin, this);

  }

  AutoPilot::~AutoPilot ()
  {
    ROS_INFO ("Destroying AutoPilot Interface");
  }

  void AutoPilot::spin (const ros::TimerEvent& e)
  {
    ROS_INFO("spin()");
    tele_->buildRequest ();
    tele_->requestCount_++;
    serialInterface_->getPackets(tele_);
    tele_->publishPackets();
  }
}

