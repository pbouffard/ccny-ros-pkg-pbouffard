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
 * @file gauges_ground_station_appdata.h
 * @brief Class object pass between ROS thread and Gtk thread.
 */

#ifndef GAUGES_GROUND_STATION_APP_DATA_H_
#define GAUGES_GROUND_STATION_APP_DATA_H_

#include <gtk/gtk.h>
/**
 * @class AppData
 * @brief class containing all the parameter needed in order to link ROS
 * with Gtk.
 */
class AppData
{
public:

  // **** main window
  GtkWidget * window;
  bool ros_param_read;
  bool widget_created;
  bool grayscale_color;
  bool radial_color;

  // **** altimeter
  GtkWidget *alt;
  int altimeter_step_value;
  bool altimeter_unit_is_feet;
  
    // **** variometer
  GtkWidget * vario;
  int variometer_step_value;
  bool variometer_unit_is_feet;

  // **** compass
  GtkWidget *comp;

  // **** gauge1 gauge
  GtkWidget *gauge1;
  char gauge1_name_f[FILENAME_MAX];
  int gauge1_start_value;
  int gauge1_end_value;
  int gauge1_initial_step;
  double gauge1_sub_step;
  int gauge1_drawing_step;
  char gauge1_color_strip_order[FILENAME_MAX];
  int gauge1_green_strip_start;
  int gauge1_yellow_strip_start;
  int gauge1_red_strip_start;

  // **** gauge2 gauge
  GtkWidget *gauge2;
  char gauge2_name_f[FILENAME_MAX];
  int gauge2_start_value;
  int gauge2_end_value;
  int gauge2_initial_step;
  double gauge2_sub_step;
  int gauge2_drawing_step;
  
  // **** bar gauge widget
  GtkWidget * bg;
  char widget_name[FILENAME_MAX];
  char name_bar_gauge1_f[FILENAME_MAX];
  char name_bar_gauge2_f[FILENAME_MAX];
  char name_bar_gauge3_f[FILENAME_MAX];
  int bar_number;
  int start_value_bar_1;
  int start_value_bar_2;
  int start_value_bar_3;
  int end_value_bar_1;
  int end_value_bar_2;
  int end_value_bar_3;
  int green_strip_start_1;
  int yellow_strip_start_1;
  
  // **** artificial horizon 
  GtkWidget * arh;
  
  // **** turn coordinator
  GtkWidget * tc;  

};

#endif
