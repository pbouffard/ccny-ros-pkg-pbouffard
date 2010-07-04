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

#ifndef APP_DATA_H_
#define APP_DATA_H_

#include <gtk/gtk.h>

class AppData
{
public:

  // **** main window
  GtkWidget * window;
  gboolean ros_param_read;
  gboolean widget_created;

  // **** altimeter
  GtkWidget *alt;
  gint altimeter_step_value;
  gboolean altimeter_unit_is_feet;
  gboolean altimeter_inv_color;

  // **** compass
  GtkWidget *comp;
  gboolean compass_inv_color;

  // **** battery gauge
  GtkWidget *battery;
  gboolean battery_gauge_inv_color;

  // **** velocity gauge
  GtkWidget *vel;
  gboolean velocity_gauge_inv_color;
  
  // **** bar gauge widget
  GtkWidget * bg;
  gboolean bg_inv_color;
  
  // **** artificial horizon widget widget
  GtkWidget * arh;
  gboolean arh_inv_color;

};

#endif
