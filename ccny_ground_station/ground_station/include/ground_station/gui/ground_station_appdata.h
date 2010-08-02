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
 * @file gauges_ground_station_appdata.h
 * @brief Class object pass between ROS thread and Gtk thread.
 */

#ifndef _GROUND_STATION_APP_DATA_H_
#define _GROUND_STATION_APP_DATA_H_

#include <gtk/gtk.h>
#include <osm-gps-map.h>
#include <ground_station/gui/gpsd_viewer_osd.h>

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

  // -------------------------------------------------------------------
  // -------------------------------------------------------------------
  // **** Tab 1: Telemetry
  GtkWidget * widget_table;

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
  GtkWidget *comp2;  
  
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
  
  // -------------------------------------------------------------------
  // -------------------------------------------------------------------
  // **** Tab 2: GpsdViewer
  OsmGpsMap * map;
  OsmGpsMapSource_t map_provider;
  OsmGpsMapTrack * current_track;
  GpsdViewerOsd * osd;
  GtkWidget *map_box;
  GtkWidget *map_container;
  
  bool draw_path;
  int map_zoom_max;
  int map_current_zoom;
  const char *repo_uri;
  const char *friendly_name;
  char *cachedir;

};

#endif
