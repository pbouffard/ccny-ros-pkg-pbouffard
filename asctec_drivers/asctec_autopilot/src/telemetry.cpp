/*
 *  AscTec Autopilot Telemetry
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
#include "asctec_autopilot/telemetry.h"

namespace asctec
{
  Telemetry::Telemetry ()
  {
    requestCount_ = 0;          
    pollingEnabled_ = false;
    requestPackets_ = 0;
    memset(requestInterval_, 0, REQUEST_TYPES*sizeof(uint8_t));
    memset(requestOffset_, 0, REQUEST_TYPES*sizeof(uint8_t));
    REQUEST_BITMASK[RequestTypes::LL_STATUS] = 0x0001;
    REQUEST_BITMASK[RequestTypes::IMU_RAWDATA] = 0x0002;
    REQUEST_BITMASK[RequestTypes::IMU_CALCDATA] = 0x0004;
    REQUEST_BITMASK[RequestTypes::RC_DATA] = 0x0008;
    REQUEST_BITMASK[RequestTypes::CONTROLLER_OUTPUT] = 0x0010;
    REQUEST_BITMASK[RequestTypes::GPS_DATA] = 0x0080;
    REQUEST_BITMASK[RequestTypes::WAYPOINT] = 0x0100;
    REQUEST_BITMASK[RequestTypes::GPS_DATA_ADVANCED] = 0x0200;
    REQUEST_BITMASK[RequestTypes::CAM_DATA] = 0x0800;
  }
  Telemetry::~Telemetry ()
  {
  }

  void Telemetry::buildRequest ()
  {
    ROS_DEBUG ("Telemetry::buildRequest()");
    // Clear previous packet request
    requestPackets_ ^= requestPackets_;
    for (int i = 0; i < REQUEST_TYPES; i++)
    {
      if (requestInterval_[i] != 0 && ((requestCount_ - requestOffset_[i]) % requestInterval_[i] == 0))
        requestPackets_ |= REQUEST_BITMASK[i];
    }
  }
  void Telemetry::publishPackets()
  {
    for (int i = 0; i < REQUEST_TYPES; i++)
    {
      if (requestInterval_[i] != 0 && ((requestCount_ - requestOffset_[i]) % requestInterval_[i] == 0))
      {
        switch (i)
        {
          case RequestTypes::LL_STATUS:
            copyLL_STATUS();
            requestPublisher_[i].publish(LLStatus_);
            break;
          case RequestTypes::IMU_CALCDATA:
            copyIMU_CALCDATA();
            requestPublisher_[i].publish(IMUCalcData_);
            break;
          case RequestTypes::GPS_DATA:
            copyGPS_DATA();
            requestPublisher_[i].publish(GPSData_);
            break;
          case RequestTypes::RC_DATA:
            copyRC_DATA();
            requestPublisher_[i].publish(RCData_);
            break;
          case RequestTypes::CONTROLLER_OUTPUT:
            copyCONTROLLER_OUTPUT();
            requestPublisher_[i].publish(CTRLOut_);
            break;
          default:
            ROS_DEBUG("Unable to publish unknown type");
        }
      }
    }
  }

  void Telemetry::enablePolling (RequestType msg, uint8_t interval, uint8_t offset)
  {
    ros::NodeHandle nh_private("~");
    switch (msg)
    {
      case RequestTypes::LL_STATUS:
        requestPublisher_[msg] = nh_private.advertise<asctec_msgs::LLStatus>(requestToString(msg).c_str(), 10);
        break;
//      case RequestTypes::IMU_RAWDATA: {
      case RequestTypes::IMU_CALCDATA:
        requestPublisher_[msg] = nh_private.advertise<asctec_msgs::IMUCalcData>(requestToString(msg).c_str(), 10);
        break;
      case RequestTypes::RC_DATA:
        requestPublisher_[msg] = nh_private.advertise<asctec_msgs::RCData>(requestToString(msg).c_str(), 10);
        break;
      case RequestTypes::GPS_DATA:
        requestPublisher_[msg] = nh_private.advertise<asctec_msgs::GPSData>(requestToString(msg).c_str(), 10);
        break;
      case RequestTypes::CONTROLLER_OUTPUT:
        requestPublisher_[msg] = nh_private.advertise<asctec_msgs::CTRLOut>(requestToString(msg).c_str(), 10);
        break;
    }

    ROS_INFO("Publishing %s data on topic: %s", requestToString(msg).c_str(),requestToString(msg).c_str ());
    ROS_DEBUG ("Telemetry::enablePolling()");
    requestInterval_[msg] = interval;
    requestOffset_[msg] = offset;
    pollingEnabled_ = true;
  }

std::string Telemetry::requestToString(RequestTypes::RequestType t)
{
   switch (t)
   {
      case RequestTypes::LL_STATUS:    { return "LL_STATUS";    }
      case RequestTypes::IMU_RAWDATA:    { return "IMU_RAWDATA";    }
      case RequestTypes::IMU_CALCDATA:    { return "IMU_CALCDATA";    }
      case RequestTypes::RC_DATA:    { return "RC_DATA";    }
      case RequestTypes::GPS_DATA:    { return "GPS_DATA";    }
      case RequestTypes::CONTROLLER_OUTPUT:    { return "CONTROLLER_OUTPUT";    }
   }
   return "Unknown";
}


  void Telemetry::dumpLL_STATUS() {
    ROS_INFO("LL_STATUS");
    ROS_INFO("--------------------------------");
    ROS_INFO("battery_voltage_1:%d",LL_STATUS_.battery_voltage_1);
    ROS_INFO("battery_voltage_2:%d",LL_STATUS_.battery_voltage_2);
    ROS_INFO("status:%d",LL_STATUS_.status);
    ROS_INFO("cpu_load:%d",LL_STATUS_.cpu_load);
    ROS_INFO("compass_enabled:%d",LL_STATUS_.compass_enabled);
    ROS_INFO("chksum_error:%d",LL_STATUS_.chksum_error);
    ROS_INFO("flying:%d",LL_STATUS_.flying);
    ROS_INFO("motors_on:%d",LL_STATUS_.motors_on);
    ROS_INFO("flightMode:%d",LL_STATUS_.flightMode);
    ROS_INFO("up_time:%d",LL_STATUS_.up_time);
  }
  void Telemetry::copyLL_STATUS() {
    LLStatus_.battery_voltage_1 =  LL_STATUS_.battery_voltage_1;
    LLStatus_.battery_voltage_2 = LL_STATUS_.battery_voltage_2;
    LLStatus_.status = LL_STATUS_.status;
    LLStatus_.cpu_load = LL_STATUS_.cpu_load;
    LLStatus_.compass_enabled = LL_STATUS_.compass_enabled;
    LLStatus_.chksum_error = LL_STATUS_.chksum_error;
    LLStatus_.flying = LL_STATUS_.flying;
    LLStatus_.motors_on = LL_STATUS_.motors_on;
    LLStatus_.flightMode = LL_STATUS_.flightMode;
    LLStatus_.up_time = LL_STATUS_.up_time;
  }
  void Telemetry::dumpIMU_RAWDATA() {
    ROS_INFO("IMU_RAWDATA");
    ROS_INFO("--------------------------------");
    ROS_INFO("pressure:%d",IMU_RAWDATA_.pressure);
    ROS_INFO("gyro_x:%d",IMU_RAWDATA_.gyro_x);
    ROS_INFO("gyro_y:%d",IMU_RAWDATA_.gyro_y);
    ROS_INFO("gyro_z:%d",IMU_RAWDATA_.gyro_z);
    ROS_INFO("mag_x:%d",IMU_RAWDATA_.mag_x);
    ROS_INFO("mag_y:%d",IMU_RAWDATA_.mag_y);
    ROS_INFO("mag_z:%d",IMU_RAWDATA_.mag_z);
    ROS_INFO("acc_x:%d",IMU_RAWDATA_.acc_x);
    ROS_INFO("acc_y:%d",IMU_RAWDATA_.acc_y);
    ROS_INFO("acc_z:%d",IMU_RAWDATA_.acc_z);
    ROS_INFO("temp_gyro:%d",IMU_RAWDATA_.temp_gyro);
    ROS_INFO("temp_ADC:%d",IMU_RAWDATA_.temp_ADC);
  }
  void Telemetry::dumpIMU_CALCDATA() {
    ROS_INFO("IMU_CALCDATA");
    ROS_INFO("--------------------------------");
    ROS_INFO("angle_nick:%d",IMU_CALCDATA_.angle_nick);
    ROS_INFO("angle_roll:%d",IMU_CALCDATA_.angle_roll);
    ROS_INFO("angle_yaw:%d",IMU_CALCDATA_.angle_yaw);
    ROS_INFO("angvel_nick:%d",IMU_CALCDATA_.angvel_nick);
    ROS_INFO("angvel_roll:%d",IMU_CALCDATA_.angvel_roll);
    ROS_INFO("angvel_yaw:%d",IMU_CALCDATA_.angvel_yaw);
    ROS_INFO("acc_x_calib:%d",IMU_CALCDATA_.acc_x_calib);
    ROS_INFO("acc_y_calib:%d",IMU_CALCDATA_.acc_y_calib);
    ROS_INFO("acc_z_calib:%d",IMU_CALCDATA_.acc_z_calib);
    ROS_INFO("acc_x:%d",IMU_CALCDATA_.acc_x);
    ROS_INFO("acc_y:%d",IMU_CALCDATA_.acc_y);
    ROS_INFO("acc_z:%d",IMU_CALCDATA_.acc_z);
    ROS_INFO("acc_angle_nick:%d",IMU_CALCDATA_.acc_angle_nick);
    ROS_INFO("acc_angle_roll:%d",IMU_CALCDATA_.acc_angle_roll);
    ROS_INFO("acc_absolute_value:%d",IMU_CALCDATA_.acc_absolute_value);
    ROS_INFO("Hx:%d",IMU_CALCDATA_.Hx);
    ROS_INFO("Hy:%d",IMU_CALCDATA_.Hy);
    ROS_INFO("Hz:%d",IMU_CALCDATA_.Hz);
    ROS_INFO("mag_heading:%d",IMU_CALCDATA_.mag_heading);
    ROS_INFO("speed_x:%d",IMU_CALCDATA_.speed_x);
    ROS_INFO("speed_y:%d",IMU_CALCDATA_.speed_y);
    ROS_INFO("speed_z:%d",IMU_CALCDATA_.speed_z);
    ROS_INFO("height:%d",IMU_CALCDATA_.height);
    ROS_INFO("dheight:%d",IMU_CALCDATA_.dheight);
    ROS_INFO("dheight_reference:%d",IMU_CALCDATA_.dheight_reference);
    ROS_INFO("height_reference:%d",IMU_CALCDATA_.height_reference);
  }
  void Telemetry::copyIMU_CALCDATA() {
    IMUCalcData_.angle_nick = IMU_CALCDATA_.angle_nick;
    IMUCalcData_.angle_roll = IMU_CALCDATA_.angle_roll;
    IMUCalcData_.angle_yaw = IMU_CALCDATA_.angle_yaw;
    IMUCalcData_.angvel_nick = IMU_CALCDATA_.angvel_nick;
    IMUCalcData_.angvel_roll = IMU_CALCDATA_.angvel_roll;
    IMUCalcData_.angvel_yaw = IMU_CALCDATA_.angvel_yaw;
    IMUCalcData_.acc_x_calib = IMU_CALCDATA_.acc_x_calib;
    IMUCalcData_.acc_y_calib = IMU_CALCDATA_.acc_y_calib;
    IMUCalcData_.acc_z_calib = IMU_CALCDATA_.acc_z_calib;
    IMUCalcData_.acc_x = IMU_CALCDATA_.acc_x;
    IMUCalcData_.acc_y = IMU_CALCDATA_.acc_y;
    IMUCalcData_.acc_z = IMU_CALCDATA_.acc_z;
    IMUCalcData_.acc_angle_nick = IMU_CALCDATA_.acc_angle_nick;
    IMUCalcData_.acc_angle_roll = IMU_CALCDATA_.acc_angle_roll;
    IMUCalcData_.acc_absolute_value = IMU_CALCDATA_.acc_absolute_value;
    IMUCalcData_.Hx = IMU_CALCDATA_.Hx;
    IMUCalcData_.Hy = IMU_CALCDATA_.Hy;
    IMUCalcData_.Hz = IMU_CALCDATA_.Hz;
    IMUCalcData_.mag_heading = IMU_CALCDATA_.mag_heading;
    IMUCalcData_.speed_x = IMU_CALCDATA_.speed_x;
    IMUCalcData_.speed_y = IMU_CALCDATA_.speed_y;
    IMUCalcData_.speed_z = IMU_CALCDATA_.speed_z;
    IMUCalcData_.height = IMU_CALCDATA_.height;
    IMUCalcData_.dheight = IMU_CALCDATA_.dheight;
    IMUCalcData_.dheight_reference = IMU_CALCDATA_.dheight_reference;
    IMUCalcData_.height_reference = IMU_CALCDATA_.height_reference;
  }
  void Telemetry::dumpRC_DATA() {
    ROS_INFO("RC_DATA");
    ROS_INFO("--------------------------------");
    ROS_INFO("channels_in: %d %d %d %d %d %d %d %d",RC_DATA_.channels_in[0],RC_DATA_.channels_in[1],
      RC_DATA_.channels_in[2],RC_DATA_.channels_in[3],RC_DATA_.channels_in[4],RC_DATA_.channels_in[5],
      RC_DATA_.channels_in[6],RC_DATA_.channels_in[7]);
    ROS_INFO("channels_out: %d %d %d %d %d %d %d %d",RC_DATA_.channels_out[0],RC_DATA_.channels_out[1],
      RC_DATA_.channels_out[2],RC_DATA_.channels_out[3],RC_DATA_.channels_out[4],RC_DATA_.channels_out[5],
      RC_DATA_.channels_out[6],RC_DATA_.channels_out[7]);
    ROS_INFO("lock:%d",RC_DATA_.lock);
  }

  void Telemetry::copyRC_DATA() {
    for(int i=0;i<8;i++){
      RCData_.channels_in[i] = RC_DATA_.channels_in[i];
      RCData_.channels_out[i] = RC_DATA_.channels_out[i];
    }
    RCData_.lock = RC_DATA_.lock;
  }
  void Telemetry::dumpGPS_DATA() {
    ROS_INFO("GPS_DATA");
    ROS_INFO("--------------------------------");
    ROS_INFO("latitude:%d",GPS_DATA_.latitude);
    ROS_INFO("longitude:%d",GPS_DATA_.longitude);
    ROS_INFO("height:%d",GPS_DATA_.height);
    ROS_INFO("speed_x:%d",GPS_DATA_.speed_x);
    ROS_INFO("speed_y:%d",GPS_DATA_.speed_y);
    ROS_INFO("heading:%d",GPS_DATA_.heading);
    ROS_INFO("horizontal_accuracy:%d",GPS_DATA_.horizontal_accuracy);
    ROS_INFO("vertical_accuracy:%d",GPS_DATA_.vertical_accuracy);
    ROS_INFO("speed_accuracy:%d",GPS_DATA_.speed_accuracy);
    ROS_INFO("numSV:%d",GPS_DATA_.numSV);
    ROS_INFO("status:%d",GPS_DATA_.status);
  }
  void Telemetry::copyGPS_DATA() {
    GPSData_.latitude = GPS_DATA_.latitude;
    GPSData_.longitude = GPS_DATA_.longitude;
    GPSData_.height = GPS_DATA_.height;
    GPSData_.speed_x = GPS_DATA_.speed_x;
    GPSData_.speed_y = GPS_DATA_.speed_y;
    GPSData_.heading = GPS_DATA_.heading;
    GPSData_.horizontal_accuracy = GPS_DATA_.horizontal_accuracy;
    GPSData_.vertical_accuracy = GPS_DATA_.vertical_accuracy;
    GPSData_.speed_accuracy = GPS_DATA_.speed_accuracy;
    GPSData_.numSV = GPS_DATA_.numSV;
    GPSData_.status = GPS_DATA_.status;
  }
  void Telemetry::copyCONTROLLER_OUTPUT() {
    CTRLOut_.pitch = CONTROLLER_OUTPUT_.nick;
    CTRLOut_.yaw = CONTROLLER_OUTPUT_.yaw;
    CTRLOut_.roll = CONTROLLER_OUTPUT_.roll;
    CTRLOut_.thrust = CONTROLLER_OUTPUT_.thrust;
  }
}
