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
  ros::spin ();
  return 0;
}

namespace asctec
{
  AutoPilot::AutoPilot ()
  {
    ROS_INFO ("Creating AutoPilot Interface");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private ("~");

    // **** get parameters

    if (!nh_private.getParam ("freq", freq_))
      freq_ = 10.0;

    if (!nh_private.getParam ("port", port_))
      port_ = "/dev/ttyUSB0";

    if (!nh_private.getParam ("speed", speed_))
      speed_ = 57600;

    if (!nh_private.getParam ("enable_LL_STATUS", enable_LL_STATUS_))
      enable_LL_STATUS_ = false;
    if (!nh_private.getParam ("enable_IMU_RAWDATA", enable_IMU_RAWDATA_))
      enable_IMU_RAWDATA_ = false;
    if (!nh_private.getParam ("enable_IMU_CALCDATA", enable_IMU_CALCDATA_))
      enable_IMU_CALCDATA_ = false;
    if (!nh_private.getParam ("enable_RC_DATA", enable_RC_DATA_))
      enable_RC_DATA_ = false;
    if (!nh_private.getParam ("enable_CONTROLLER_OUTPUT", enable_CONTROLLER_OUTPUT_))
      enable_CONTROLLER_OUTPUT_ = false;
    if (!nh_private.getParam ("enable_GPS_DATA", enable_GPS_DATA_))
      enable_GPS_DATA_ = false;
    if (!nh_private.getParam ("enable_GPS_DATA_ADVANCED", enable_GPS_DATA_ADVANCED_))
      enable_GPS_DATA_ADVANCED_ = false;

    if (!nh_private.getParam ("interval_LL_STATUS", interval_LL_STATUS_))
      interval_LL_STATUS_ = 1;
    if (!nh_private.getParam ("interval_IMU_RAWDATA", interval_IMU_RAWDATA_))
      interval_IMU_RAWDATA_ = 1;
    if (!nh_private.getParam ("interval_IMU_CALCDATA", interval_IMU_CALCDATA_))
      interval_IMU_CALCDATA_ = 1;
    if (!nh_private.getParam ("interval_RC_DATA", interval_RC_DATA_))
      interval_RC_DATA_ = 1;
    if (!nh_private.getParam ("interval_CONTROLLER_OUTPUT", interval_CONTROLLER_OUTPUT_))
      interval_CONTROLLER_OUTPUT_ = 1;
    if (!nh_private.getParam ("interval_GPS_DATA", interval_GPS_DATA_))
      interval_GPS_DATA_ = 1;
    if (!nh_private.getParam ("interval_GPS_DATA_ADVANCED", interval_GPS_DATA_ADVANCED_))
      interval_GPS_DATA_ADVANCED_ = 1;

    if (!nh_private.getParam ("offset_LL_STATUS", offset_LL_STATUS_))
      offset_LL_STATUS_ = 0;
    if (!nh_private.getParam ("offset_IMU_RAWDATA", offset_IMU_RAWDATA_))
      offset_IMU_RAWDATA_ = 0;
    if (!nh_private.getParam ("offset_IMU_CALCDATA", offset_IMU_CALCDATA_))
      offset_IMU_CALCDATA_ = 0;
    if (!nh_private.getParam ("offset_RC_DATA", offset_RC_DATA_))
      offset_RC_DATA_ = 0;
    if (!nh_private.getParam ("offset_CONTROLLER_OUTPUT", offset_CONTROLLER_OUTPUT_))
      offset_CONTROLLER_OUTPUT_ = 0;
    if (!nh_private.getParam ("offset_GPS_DATA", offset_GPS_DATA_))
      offset_GPS_DATA_ = 0;
    if (!nh_private.getParam ("offset_GPS_DATA_ADVANCED", offset_GPS_DATA_ADVANCED_))
      offset_GPS_DATA_ADVANCED_ = 0;

    if (freq_ <= 0.0)
      ROS_FATAL ("Invalid frequency param");

    ros::Duration d (1.0 / freq_);

    // **** set up intefaces

    serialInterface_ = new asctec::SerialInterface::SerialInterface (port_, speed_);
    telemetry_ = new asctec::Telemetry::Telemetry ();

    // **** enable polling
    if(enable_LL_STATUS_)
      telemetry_->enablePolling (asctec::RequestTypes::LL_STATUS, interval_LL_STATUS_, offset_LL_STATUS_);
    if(enable_RC_DATA_)
      telemetry_->enablePolling (asctec::RequestTypes::RC_DATA, interval_RC_DATA_, offset_RC_DATA_);
    if(enable_CONTROLLER_OUTPUT_)
      telemetry_->enablePolling (asctec::RequestTypes::CONTROLLER_OUTPUT, interval_CONTROLLER_OUTPUT_, offset_CONTROLLER_OUTPUT_);
    if(enable_IMU_RAWDATA_)
      telemetry_->enablePolling(asctec::RequestTypes::IMU_RAWDATA, interval_IMU_RAWDATA_, offset_IMU_RAWDATA_);
    if(enable_IMU_CALCDATA_)
      telemetry_->enablePolling (asctec::RequestTypes::IMU_CALCDATA, interval_IMU_CALCDATA_, offset_IMU_CALCDATA_);
    if(enable_GPS_DATA_)
      telemetry_->enablePolling (asctec::RequestTypes::GPS_DATA, interval_GPS_DATA_, offset_GPS_DATA_);
    if(enable_GPS_DATA_ADVANCED_)
      telemetry_->enablePolling (asctec::RequestTypes::GPS_DATA_ADVANCED, interval_GPS_DATA_ADVANCED_,  offset_GPS_DATA_ADVANCED_);

    telemetry_->enableCommanding (10, 2);
    telemetry_->CTRL_INPUT_.pitch = 0;
    telemetry_->CTRL_INPUT_.roll = 0;
    telemetry_->CTRL_INPUT_.yaw = 0;
    telemetry_->CTRL_INPUT_.thrust = 0;
    telemetry_->CTRL_INPUT_.ctrl = 0x0000;
    telemetry_->CTRL_INPUT_.chksum =
      telemetry_->CTRL_INPUT_.pitch + telemetry_->CTRL_INPUT_.roll + telemetry_->CTRL_INPUT_.yaw +
      telemetry_->CTRL_INPUT_.thrust + telemetry_->CTRL_INPUT_.ctrl + 0xAAAA;
    ROS_INFO ("CTRL_INPUT_.chksum: %i", (short) telemetry_->CTRL_INPUT_.chksum);

    timer_ = nh_private.createTimer (d, &AutoPilot::spin, this);
  }

  AutoPilot::~AutoPilot ()
  {
    ROS_INFO ("Destroying AutoPilot Interface");
  }

  void AutoPilot::spin (const ros::TimerEvent & e)
  {
    ROS_INFO ("spin()");
    telemetry_->buildRequest ();
    telemetry_->requestCount_++;
    serialInterface_->getPackets (telemetry_);
    telemetry_->publishPackets ();
    serialInterface_->sendCommand (telemetry_);
    telemetry_->commandCount_++;
  }
}
