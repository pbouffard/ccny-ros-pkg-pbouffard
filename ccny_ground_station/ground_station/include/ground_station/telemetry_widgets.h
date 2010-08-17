/*
 *  Gauges Ground Station for CityFlyer CCNY project
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
 * @file telemetry_widgets.h 
 * @brief Program that link ROS with Gtk
 * @author Gautier Dumonteil <gautier.dumonteil@gmail.com>
 * @version 0.1
 * @date 06/06/2010
 *
 * Gauges Ground Station for CityFlyer CCNY project
 * Copyright (C) 2010, CCNY Robotics Lab
 * http://robotics.ccny.cuny.edu
 * 
 * This program provides an Gtk window with several widgets,<br>
 * each widget is link to a ROS topic. 
 * Gtk and ROS possess there main loop, so in order to run both<br>
 * of them, we run first Gtk in the main program and then create<br>
 * a child thread for ROS. 
 * Gtk is not "thread safe", if we want to access to an Gtk resource<br>
 * we have to put Gtk in a "thread aware" mode. This is allowed in <br>
 * in part by the following functions:
 * - gdk_threads_enter ();
 * - gdk_threads_leave ();
 *  
 */

#ifndef CCNY_GROUND_STATION_GAUGES_GROUND_STATION_H
#define CCNY_GROUND_STATION_GAUGES_GROUND_STATION_H

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <glib.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include <ground_station/gui/telemetry_widgets_appdata.h>
#include <ground_station/gui/gtkaltimeter.h>
#include <ground_station/gui/gtkvariometer.h>
#include <ground_station/gui/gtkcompass.h>
#include <ground_station/gui/gtkgauge.h>
#include <ground_station/gui/gtkbargauge.h>
#include <ground_station/gui/gtkturncoordinator.h>
#include <ground_station/gui/gtkartificialhorizon.h>

const std::string imuTopic = "/imu";

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

sensor_msgs::Imu imuData_;

void *startROS (void *);
void updateAltitudeCallback (const geometry_msgs::PoseConstPtr &);
gboolean window_update (gpointer dat);

#endif
