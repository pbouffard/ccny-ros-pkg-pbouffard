/*
 *  AscTec Autopilot IMU Calibration
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
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <ros/message_operations.h>
#include <ros/message.h>

#include "asctec_autopilot/publisher.h"
#include "asctec_autopilot/asctec_autopilot.h"
#include "asctec_autopilot/IMUCalcdata.h"

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "publisher");
  ros::NodeHandle n;
  ros::Publisher calcdataPublisher;
  calcdataPublisher = n.advertise <asctec_autopilot::IMUCalcdata >("PelicanIMUCalcdata", 100);
  ros::Rate loop_rate (10);

  asctec_autopilot::IMUCalcdata rosIMUCalcdata;

  // set up serial reader

  asctec_autopilot::SerialInterface serialInterface = asctec_autopilot::SerialInterface("/dev/ttyUSB0",57600);
  IMU_CALCDATA serialIMUCalcdata;

  while (ros::ok ())
  {
    if (serialInterface.getIMUCalcdata (serialIMUCalcdata))
    {
      rosIMUCalcdata.angle_roll = serialIMUCalcdata.angle_roll;
      rosIMUCalcdata.angle_pitch = serialIMUCalcdata.angle_nick;
      rosIMUCalcdata.angle_yaw = serialIMUCalcdata.angle_yaw;

      rosIMUCalcdata.acc_x_calib = serialIMUCalcdata.acc_x_calib;
      rosIMUCalcdata.acc_y_calib = serialIMUCalcdata.acc_y_calib;
      rosIMUCalcdata.acc_z_calib = serialIMUCalcdata.acc_z_calib;

      rosIMUCalcdata.acc_x = serialIMUCalcdata.acc_x;
      rosIMUCalcdata.acc_y = serialIMUCalcdata.acc_y;
      rosIMUCalcdata.acc_z = serialIMUCalcdata.acc_z;

      rosIMUCalcdata.height = serialIMUCalcdata.height;

      rosIMUCalcdata.height_reference = serialIMUCalcdata.height_reference;

      // pubish message
      calcdataPublisher.publish (rosIMUCalcdata);

      // print that we published
      ROS_INFO ("scan published");
    }
    else
    {
      ROS_INFO ("failed to read serial data");
    }

    // sleep
    ros::spinOnce ();
    loop_rate.sleep ();
  }
  return 0;
}

