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

#ifndef ASCTEC_AUTOPILOT_AUTOPILOT_H
#define ASCTEC_AUTOPILOT_AUTOPILOT_H

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
#include "asctec_autopilot/serialinterface.h"

namespace asctec
{
  class AutoPilot
  {
    private:

      ros::Timer timer_;
      double freq_;

    public:

      AutoPilot ();
     ~AutoPilot ();

      SerialInterface *serialInterface_;
      Telemetry *tele_;

      void enablePolling (uint16_t request, uint16_t interval);
      void spin (const ros::TimerEvent & e);

      // CONVERSION FACTORS
      static const double PEL_TO_ROS_ANGLE = (1.0 / 1000.0) * 3.14159265 / 180.0; // converts to rad
      static const double PEL_TO_ROS_ANGVEL = (1.0 / 64.8) * 3.14159265 / 180.0;  // convetts to rad/s
      static const double PEL_TO_ROS_ACC = (1.0 / 10000.0) * 9.81;        // converts to m/s^s
      static const double PEL_TO_ROS_HEIGHT = (1.0 / 1000.0);     // converts to m

  };                            // end class AutoPilot
}                               //end namespace asctec_autopilot
#endif
