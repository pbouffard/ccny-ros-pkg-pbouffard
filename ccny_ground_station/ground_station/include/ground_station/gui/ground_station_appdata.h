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
  GtkWidget *notebook;
  bool ros_param_read;
  bool widget_created;
  bool grayscale_color;
  bool radial_color;
  bool fullscreen;
  int current_page;
  int telemetry_refresh_rate;
  char icon_directory[FILENAME_MAX];

  // **** Icon 64*64
  GdkPixbuf *leftarrow_icon_64;
  GdkPixbuf *rightarrow_icon_64;
  GdkPixbuf *record_icon_64;
  GdkPixbuf *pause_icon_64;
  GdkPixbuf *stop_icon_64;
  GdkPixbuf *refresh_icon_64;
  GdkPixbuf *status_ok_icon_64;
  GdkPixbuf *status_fail_icon_64;
  GdkPixbuf *record_g_icon_64;

  // **** Icon resized (Status bar)
  GtkWidget *record_icon;
  GtkWidget *record_g_icon;
  GtkWidget *status_ok_icon_motor;
  GtkWidget *status_fail_icon_motor;
  GtkWidget *status_ok_icon_gps;
  GtkWidget *status_fail_icon_gps;
  GtkWidget *status_ok_icon_flying;
  GtkWidget *status_fail_icon_flying;

  // -------------------------------------------------------------------
  // -------------------------------------------------------------------
  // **** Status bar
  GtkWidget *box_MotorStatus;
  GtkWidget *box_Flying;
  GtkWidget *box_Gps;
  GtkWidget *flightMode_label;
  GtkWidget *upTime_label;
  GtkWidget *cpuLoad_label;
  GtkWidget *box_RecordStatus;

  // -------------------------------------------------------------------
  // -------------------------------------------------------------------
  // **** Tab 1: Telemetry
  GtkWidget *widget_table;

  // **** altimeter
  GtkWidget *alt;
  int altimeter_step_value;
  bool altimeter_unit_is_feet;

  // **** variometer
  GtkWidget *vario;
  int variometer_step_value;
  bool variometer_unit_is_feet;

  // **** compass
  GtkWidget *comp;
  GtkWidget *comp2;

  // **** gauge widget
  GtkWidget *gauge1;
  char gauge1_name_f[FILENAME_MAX];
  char gauge1_color_strip_order[FILENAME_MAX];
  int gauge1_start_value;
  int gauge1_end_value;
  int gauge1_initial_step;
  double gauge1_sub_step;
  int gauge1_drawing_step;
  int gauge1_green_strip_start;
  int gauge1_yellow_strip_start;
  int gauge1_red_strip_start;

  // **** artificial horizon 
  GtkWidget *arh;

  // **** Option Popup
  bool telemetry_opt_popup_state;
  GtkWidget *telemetry_option_popup;
  GtkWidget *btn_open_telemetry_option_popup;
  GtkWidget *btn_close_telemetry_option_popup;

  // -------------------------------------------------------------------
  // -------------------------------------------------------------------
  // **** Tab 2: GpsdViewer
  OsmGpsMap *map;
  OsmGpsMapSource_t map_provider;
  OsmGpsMapTrack *uav_track;
  GpsdViewerOsd *osd;
  GtkWidget *map_box;
  GtkWidget *map_container;

  bool draw_path;
  bool lock_view;
  int map_zoom_max;
  int map_current_zoom;
  const char *repo_uri;
  const char *friendly_name;
  char *cachedir;

  // **** Option Popup
  bool gps_opt_popup_state;
  GtkWidget *gpsd_option_popup;
  GtkWidget *btn_open_gpsd_option_popup;
  GtkWidget *btn_close_gpsd_option_popup;

  // -------------------------------------------------------------------
  // -------------------------------------------------------------------
  // **** Tab 3: ROSBag Record
  pid_t rosbag_pid;
  GtkListStore *topicsList;
  GtkWidget *cmd_line_entry;
  GtkWidget *prefix_entry;
  GtkWidget *info_textview;
  bool recording;

  char *rosbag_record_cmd;
  char *file_prefix;
  char cmd_line[FILENAME_MAX];
  char list_topic[FILENAME_MAX];
  char rosbag_rec_path[FILENAME_MAX];

  GtkWidget *update_btn;
  GtkWidget *record_stop_btn;
};

#endif
