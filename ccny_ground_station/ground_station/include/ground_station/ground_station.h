/*
 *  Ground Station for CityFlyer CCNY project
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Gautier Dumonteil <gautier.dumonteil@gmail.com>
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

/**
 * @file ground_station.h 
 * @brief Program that link ROS with Gtk
 * @author Gautier Dumonteil <gautier.dumonteil@gmail.com>
 * @version 0.1
 * @date 06/06/2010
 *
 * Ground Station for CityFlyer CCNY project
 * Copyright (C) 2010, CCNY Robotics Lab
 * http://robotics.ccny.cuny.edu
 *  
 */

#ifndef CCNY_GROUND_STATION_GROUND_STATION_H
#define CCNY_GROUND_STATION_GROUND_STATION_H

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <glib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <asctec_msgs/GPSData.h>
#include <asctec_msgs/IMUCalcData.h>
#include <asctec_msgs/LLStatus.h>
#include <asctec_msgs/Height.h>
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>
#include <sensor_msgs/Imu.h>

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include <ground_station/gui/ground_station_appdata.h>
#include <ground_station/gui/gtkaltimeter.h>
#include <ground_station/gui/gtkvariometer.h>
#include <ground_station/gui/gtkcompass.h>
#include <ground_station/gui/gtkgauge.h>
#include <ground_station/gui/gtkbargauge.h>
#include <ground_station/gui/gtkturncoordinator.h>
#include <ground_station/gui/gtkartificialhorizon.h>
#include <ground_station/gui/gpsd_viewer_osd.h>

const std::string imuTopic = "/asctec_proc/imu";
const std::string heightTopic = "/asctec_proc/pressure_height";
const std::string imuCalcDataTopic = "/autopilot/IMU_CALCDATA";
const std::string gpsDataTopic = "/autopilot/GPS_DATA";
const std::string llStatusTopic = "/autopilot/LL_STATUS";

#define RAD2DEG(RAD) ((RAD)*((180.)/(M_PI)))

/**
 * @struct arg
 * @brief Allow to pass the arguments of the program to a pthread.<br>
 * Allow to use ROS params even ROS is in a child thread of Gtk.
 */
struct arg
{
  int argc;
  char **argv;
};

ros::Subscriber imuSub;
ros::Subscriber heightSub;
ros::Subscriber imuCalcDataSub;
ros::Subscriber gpsDataSub;
ros::Subscriber llStatusSub;
ros::Subscriber gpsFixSub;

sensor_msgs::Imu imuData_;
asctec_msgs::Height heightData_;
asctec_msgs::IMUCalcData imuCalcData_;
asctec_msgs::GPSData gpsData_;
asctec_msgs::LLStatus llStatus_;

void *startROS (void *);
void imuCalcDataCallback (const asctec_msgs::IMUCalcDataConstPtr &);
void gpsDataCallback (const asctec_msgs::GPSDataConstPtr &);
void llStatusCallback (const asctec_msgs::LLStatusConstPtr &);
gboolean widgets_update (gpointer dat);

#endif
