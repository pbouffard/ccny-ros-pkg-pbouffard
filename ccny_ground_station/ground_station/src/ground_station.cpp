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
 * @file ground_station.cpp 
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

#include <ground_station/ground_station.h>

AppData *data;

void imuCallback (const sensor_msgs::ImuConstPtr & imu)
{
  // **** get GTK thread lock
  gdk_threads_enter ();
  imuData_ = (*imu);

  double yaw, pitch, roll;
  btQuaternion orientation; 
  orientation.setX(imuData_.orientation.x);
  orientation.setY(imuData_.orientation.y);
  orientation.setZ(imuData_.orientation.z);
  orientation.setW(imuData_.orientation.w);
  btMatrix3x3(orientation).getRPY(roll, pitch, yaw);

  ROS_DEBUG ("Yaw %f, Pitch %f, Roll %f, RollBis %f", RAD2DEG(yaw), RAD2DEG(pitch), RAD2DEG(roll),(double)(((int)(RAD2DEG(roll)*1000)+360000)%360000)/1000);

  //~ if (IS_GTK_ALTIMETER (data->alt))
    //~ gtk_altimeter_set_alti (GTK_ALTIMETER (data->alt), (double) (imuCalcData_.height / 1000.));
//~ 
  //~ if (IS_GTK_VARIOMETER (data->vario))
   //~ gtk_variometer_set_value (GTK_VARIOMETER (data->vario), (double) (imuCalcData_.dheight / 1000.)*3600);
//~ 
  //~ if (IS_GTK_COMPASS (data->comp))
    //~ gtk_compass_set_angle (GTK_COMPASS (data->comp), (double) (imuCalcData_.mag_heading / 1000.));

  if (IS_GTK_COMPASS (data->comp2))
    gtk_compass_set_angle (GTK_COMPASS (data->comp2), (double)(((int)(RAD2DEG(yaw)*1000)+360000)%360000)/1000);

  if (IS_GTK_ARTIFICIAL_HORIZON (data->arh))
    gtk_artificial_horizon_set_value (GTK_ARTIFICIAL_HORIZON (data->arh),
                                      (double)(((int)(RAD2DEG(roll)*1000)+360000)%360000)/1000,
                                      (double) -RAD2DEG(pitch));

  // **** release GTK thread lock 
  gdk_threads_leave ();
}


void heightCallback (const asctec_msgs::HeightConstPtr & height)
{
  // **** get GTK thread lock
  gdk_threads_enter ();
  heightData_ = (*height);

  ROS_DEBUG ("Climb %fm/s %fm/min\n", heightData_.climb,heightData_.climb*3600);

  if (IS_GTK_ALTIMETER (data->alt))
    gtk_altimeter_set_alti (GTK_ALTIMETER (data->alt), (double) heightData_.height);

  //~ if (IS_GTK_VARIOMETER (data->vario))
   //~ gtk_variometer_set_value (GTK_VARIOMETER (data->vario), (double) (heightData_.climb)*3600);

  // **** release GTK thread lock 
  gdk_threads_leave ();
}

void imuCalcDataCallback (const asctec_msgs::IMUCalcDataConstPtr & dat)
{
  // **** get GTK thread lock
  gdk_threads_enter ();

  imuCalcData_ = (*dat);
  
  ROS_DEBUG("imuCalcData yaw %f, pitch %f, roll %f\n",(double)imuCalcData_.angle_yaw /1000.,(double)imuCalcData_.angle_nick/1000.,(double)imuCalcData_.angle_roll/1000.);

  if (IS_GTK_VARIOMETER (data->vario))
    gtk_variometer_set_value (GTK_VARIOMETER (data->vario), (double) (imuCalcData_.dheight /1000.)*3600);

  if (IS_GTK_COMPASS (data->comp))
    gtk_compass_set_angle (GTK_COMPASS (data->comp), (double) (imuCalcData_.mag_heading / 1000.));

  // **** release GTK thread lock 
  gdk_threads_leave ();
}

void gpsFixCallback (const gps_common::GPSFix::ConstPtr & msg)
{
  // **** get GTK thread lock
  gdk_threads_enter ();

  gint pixel_x, pixel_y;

  OsmGpsMapPoint *point = osm_gps_map_point_new_degrees (msg->latitude, msg->longitude);
  osm_gps_map_convert_geographic_to_screen (data->map, point, &pixel_x, &pixel_y);

  if (OSM_IS_GPS_MAP (data->map))
  {

    // **** Center map on gps data received
    if (data->lock_view)
    {
      update_uav_pose_osd (data->osd, TRUE, pixel_x, pixel_y);
      osm_gps_map_set_center (data->map, msg->latitude, msg->longitude);
    }
    else
    {
      update_uav_pose_osd (data->osd, FALSE, pixel_x, pixel_y);
      osm_gps_map_gps_clear (data->map);
    }

    // **** Add point to the track
    osm_gps_map_track_add_point (data->uav_track, point);
  }

  // **** release GTK thread lock 
  gdk_threads_leave ();
}

void llStatusCallback (const asctec_msgs::LLStatusConstPtr & dat)
{
  // **** get GTK thread lock
  gdk_threads_enter ();

  llStatus_ = (*dat);
  char buf[FILENAME_MAX];

  // **** update altimeter
  if (IS_GTK_GAUGE (data->gauge1))
    gtk_gauge_set_value (GTK_GAUGE (data->gauge1), (double) (llStatus_.battery_voltage_1 / 1000.));

  /*if(llStatus_.flying==0)
     {
     gtk_container_remove(GTK_CONTAINER(data->box_Flying),GTK_WIDGET (data->status_ok_icon_flying));
     gtk_box_pack_end (GTK_BOX (data->box_Flying),data->status_fail_icon_flying, TRUE, TRUE, 0);
     }
     else 
     {
     gtk_container_remove(GTK_CONTAINER(data->box_Flying),GTK_WIDGET (data->status_fail_icon_flying));
     gtk_box_pack_end (GTK_BOX (data->box_Flying),data->status_ok_icon_flying, TRUE, TRUE, 0);
     }
     if(llStatus_.motors_on==0)
     {
     gtk_container_remove(GTK_CONTAINER(data->box_MotorStatus),GTK_WIDGET (data->status_ok_icon_motor));
     gtk_box_pack_end (GTK_BOX (data->box_MotorStatus),data->status_fail_icon_motor, TRUE, TRUE, 0);
     }
     else 
     {
     gtk_container_remove(GTK_CONTAINER(data->box_MotorStatus),GTK_WIDGET (data->status_fail_icon_motor));
     gtk_box_pack_end (GTK_BOX (data->box_MotorStatus),data->status_ok_icon_motor, TRUE, TRUE, 0);
     } */

  sprintf (buf, "%d", llStatus_.flightMode);
  gtk_label_set_text (GTK_LABEL (data->flightMode_label), buf);

  sprintf (buf, "%d", llStatus_.up_time);
  gtk_label_set_text (GTK_LABEL (data->upTime_label), buf);

  sprintf (buf, "%d", llStatus_.cpu_load);
  gtk_label_set_text (GTK_LABEL (data->cpuLoad_label), buf);

  // **** release GTK thread lock 
  gdk_threads_leave ();
}


/**
 * @fn void *startROS (void *user)
 * @brief ROS thread.
 * 
 * The main program wait until "ros_param_read" in order to allow the <br>
 * ROS params to be also the Gtk Window and Widgets params.
 * Then the ROS thread wait to the widgets creation before subcribing<br>
 * to any topics, avoid to call public widget function for a widget not<br>
 * yet created.
 */
void *startROS (void *user)
{
  if (user != NULL)
  {
    struct arg *p_arg = (arg *) user;

    ros::init (p_arg->argc, p_arg->argv, "ground_station");
    ros::NodeHandle n;

    std::string local_path;
    std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
    ros::NodeHandle n_param ("~");
    XmlRpc::XmlRpcValue xml_marker_center;

    ROS_INFO ("Starting CityFlyer Ground Station");

    // -----------------------------------------------------------------   
    // **** General window parameters
    if (!n_param.getParam ("window_grayscale_color", data->grayscale_color))
      data->grayscale_color = false;
    ROS_DEBUG ("\tWindow grayscale color: %d", data->grayscale_color);

    if (!n_param.getParam ("window_radial_color", data->radial_color))
      data->radial_color = true;
    ROS_DEBUG ("\tWindow radial color: %d", data->radial_color);
    
    if (!n_param.getParam ("telemetry_refresh_rate", data->telemetry_refresh_rate))
      data->telemetry_refresh_rate = 200;
    ROS_DEBUG ("\tTelemetry refresh_rate: %d", data->telemetry_refresh_rate);

    // -----------------------------------------------------------------      
    // **** Altimeter parameters
    if (!n_param.getParam ("altimeter_unit_is_feet", data->altimeter_unit_is_feet))
      data->altimeter_unit_is_feet = true;
    ROS_DEBUG ("\tAltimeter unit is FEET: %d", data->altimeter_unit_is_feet);

    if (!n_param.getParam ("altimeter_step_value", data->altimeter_step_value))
      data->altimeter_step_value = 100;
    ROS_DEBUG ("\tAltimeter step value: %d", data->altimeter_step_value);

    // -----------------------------------------------------------------      
    // **** Variometer parameters
         /**
	  * @todo check the variometer value computation
	  */
    data->variometer_unit_is_feet = data->altimeter_unit_is_feet;
    ROS_DEBUG ("\tVariometer unit is FEET: %d", data->variometer_unit_is_feet);

    if (!n_param.getParam ("variometer_step_value", data->variometer_step_value))
      data->variometer_step_value = 100;
    ROS_DEBUG ("\tVariometer step value: %d", data->variometer_step_value);

    // -----------------------------------------------------------------      
    // **** Gauge1 parameters
    std::string gauge1_name, gauge1_unit, gauge1_color_strip, gauge1_sub_step;

    n_param.param ("gauge1_name", gauge1_name, std::string ("Gauge 1"));
    n_param.param ("gauge1_unit", gauge1_unit, std::string ("(unit)"));
    sprintf (data->gauge1_name_f, "<big>%s</big>\n<span foreground=\"orange\"><i>(%s)</i></span>", gauge1_name.c_str (),
             gauge1_unit.c_str ());
    ROS_DEBUG ("\tGauge 1 name : %s", data->gauge1_name_f);

    if (!n_param.getParam ("gauge1_start_value", data->gauge1_start_value))
      data->gauge1_start_value = 0;
    ROS_DEBUG ("\tGauge 1 start value: %d", data->gauge1_start_value);

    if (!n_param.getParam ("gauge1_end_value", data->gauge1_end_value))
      data->gauge1_end_value = 100;
    ROS_DEBUG ("\tGauge 1 end value: %d", data->gauge1_end_value);

    if (!n_param.getParam ("gauge1_initial_step", data->gauge1_initial_step))
      data->gauge1_initial_step = 10;
    ROS_DEBUG ("\tGauge 1 initial step value: %d", data->gauge1_initial_step);

    /*
     * Can't figure out why I can't use a double in a ROS launch file
     * To avoid wasting time, I use a string.
     * (string)3_45 = (double)3.45
     */
    if (!n_param.getParam ("gauge1_sub_step", gauge1_sub_step))
    {
      data->gauge1_sub_step = 2;
    }
    else
    {
      size_t found = gauge1_sub_step.find_first_of ("_");
      gauge1_sub_step[found] = '.';
      ROS_DEBUG ("\tGauge 1 sub step value: %s", gauge1_sub_step.c_str ());
      try
      {
        data->gauge1_sub_step = boost::lexical_cast < double >(gauge1_sub_step);
      }
      catch (const std::exception &)
      {
        data->gauge1_sub_step = 0;      // **** Will cause a Gtk warning on the gauge
      }
    }

    if (!n_param.getParam ("gauge1_drawing_step", data->gauge1_drawing_step))
      data->gauge1_drawing_step = 10;
    ROS_DEBUG ("\tGauge 1 drawing step value: %d", data->gauge1_drawing_step);

    // **** OPTIONAL
    n_param.param ("gauge1_color_strip_order", gauge1_color_strip, std::string ("YOR"));
    sprintf (data->gauge1_color_strip_order, "%s", gauge1_color_strip.c_str ());
    n_param.getParam ("gauge1_green_strip_start", data->gauge1_green_strip_start);
    n_param.getParam ("gauge1_yellow_strip_start", data->gauge1_yellow_strip_start);
    n_param.getParam ("gauge1_red_strip_start", data->gauge1_red_strip_start);

    // -----------------------------------------------------------------      
    // **** allow widget creation
    data->ros_param_read = true;

    // **** wait to widget creation
    while (!data->widget_created)
    {
      ROS_DEBUG ("Waiting widgets creation");
    }

    // -----------------------------------------------------------------      
    // **** topics subscribing
    ROS_INFO ("Subscribing to topics");
    imuSub = n.subscribe (imuTopic, 1, imuCallback);
    heightSub = n.subscribe (heightTopic, 1, heightCallback);
    imuCalcDataSub = n.subscribe (imuCalcDataTopic, 1, imuCalcDataCallback);
    //~ gpsDataSub = n.subscribe (gpsDataTopic, 1, gpsDataCallback);
    llStatusSub = n.subscribe (llStatusTopic, 1, llStatusCallback);
    gpsFixSub = n.subscribe ("/fix", 1, &gpsFixCallback);

    ROS_INFO ("Spinning");
    ros::spin ();
  }

  // **** stop the gtk main loop
  if (GTK_IS_WIDGET (data->window))
  {
    gtk_main_quit ();
  }
  pthread_exit (NULL);
}

/**
 * @fn gboolean widgets_update(gpointer dat)
 * @brief Gtk function which allow to refresh the widget_table.<br>
 * This allow the child widgets to be redraw also.<br>
 */
gboolean widgets_update (gpointer dat)
{
  gtk_widget_draw (GTK_WIDGET (data->widget_table), NULL);
  return true;
}

void load_icon ()
{
  GtkWidget *img_record, *img_record_g, *img_stop, *img_pause, *img_refresh;
  GtkWidget *img_leftarrow, *img_rightarrow, *img_status_ok, *img_status_fail;

  char icon_file[FILENAME_MAX];
  sprintf (icon_file, "%s/%s", data->icon_directory, "record-64.png");
  img_record = gtk_image_new_from_file (icon_file);
  data->record_icon_64 = gtk_image_get_pixbuf (GTK_IMAGE (img_record));

  sprintf (icon_file, "%s/%s", data->icon_directory, "record_g-64.png");
  img_record_g = gtk_image_new_from_file (icon_file);
  data->record_g_icon_64 = gtk_image_get_pixbuf (GTK_IMAGE (img_record_g));

  sprintf (icon_file, "%s/%s", data->icon_directory, "pause-64.png");
  img_pause = gtk_image_new_from_file (icon_file);
  data->pause_icon_64 = gtk_image_get_pixbuf (GTK_IMAGE (img_pause));

  sprintf (icon_file, "%s/%s", data->icon_directory, "stop-64.png");
  img_stop = gtk_image_new_from_file (icon_file);
  data->stop_icon_64 = gtk_image_get_pixbuf (GTK_IMAGE (img_stop));

  sprintf (icon_file, "%s/%s", data->icon_directory, "refresh-64.png");
  img_refresh = gtk_image_new_from_file (icon_file);
  data->refresh_icon_64 = gtk_image_get_pixbuf (GTK_IMAGE (img_refresh));

  sprintf (icon_file, "%s/%s", data->icon_directory, "rightarrow-64.png");
  img_rightarrow = gtk_image_new_from_file (icon_file);
  data->rightarrow_icon_64 = gtk_image_get_pixbuf (GTK_IMAGE (img_rightarrow));

  sprintf (icon_file, "%s/%s", data->icon_directory, "leftarrow-64.png");
  img_leftarrow = gtk_image_new_from_file (icon_file);
  data->leftarrow_icon_64 = gtk_image_get_pixbuf (GTK_IMAGE (img_leftarrow));

  sprintf (icon_file, "%s/%s", data->icon_directory, "status-ok-64.png");
  img_status_ok = gtk_image_new_from_file (icon_file);
  data->status_ok_icon_64 = gtk_image_get_pixbuf (GTK_IMAGE (img_status_ok));

  sprintf (icon_file, "%s/%s", data->icon_directory, "status-fail-64.png");
  img_status_fail = gtk_image_new_from_file (icon_file);
  data->status_fail_icon_64 = gtk_image_get_pixbuf (GTK_IMAGE (img_status_fail));

  data->status_ok_icon_motor =
    GTK_WIDGET (gtk_image_new_from_pixbuf
                (gdk_pixbuf_scale_simple (data->status_ok_icon_64, 22, 22, GDK_INTERP_HYPER)));
  data->status_fail_icon_motor =
    GTK_WIDGET (gtk_image_new_from_pixbuf
                (gdk_pixbuf_scale_simple (data->status_fail_icon_64, 22, 22, GDK_INTERP_HYPER)));
  data->status_ok_icon_gps =
    GTK_WIDGET (gtk_image_new_from_pixbuf
                (gdk_pixbuf_scale_simple (data->status_ok_icon_64, 22, 22, GDK_INTERP_HYPER)));
  data->status_fail_icon_gps =
    GTK_WIDGET (gtk_image_new_from_pixbuf
                (gdk_pixbuf_scale_simple (data->status_fail_icon_64, 22, 22, GDK_INTERP_HYPER)));
  data->status_ok_icon_flying =
    GTK_WIDGET (gtk_image_new_from_pixbuf
                (gdk_pixbuf_scale_simple (data->status_ok_icon_64, 22, 22, GDK_INTERP_HYPER)));
  data->status_fail_icon_flying =
    GTK_WIDGET (gtk_image_new_from_pixbuf
                (gdk_pixbuf_scale_simple (data->status_fail_icon_64, 22, 22, GDK_INTERP_HYPER)));
  data->record_icon =
    GTK_WIDGET (gtk_image_new_from_pixbuf (gdk_pixbuf_scale_simple (data->record_icon_64, 30, 30, GDK_INTERP_HYPER)));
  data->record_g_icon =
    GTK_WIDGET (gtk_image_new_from_pixbuf (gdk_pixbuf_scale_simple (data->record_g_icon_64, 30, 30, GDK_INTERP_HYPER)));
}

/**
 * @fn int main (int argc, char **argv)
 * @brief Main program & Gtk thread.
 * 
 * Create window and all widgets, then set there parameters to be the <br>
 * ROS params.
 */
int main (int argc, char **argv)
{
  GtkBuilder *builder;
  GdkColor black = { 0, 0, 0, 0 };
  GError *error = NULL;
  char glade_gui_file[FILENAME_MAX];
  int start_zoom = 15;
  char *mapcachedir;
  OsmGpsMapPoint ccny_coord = { 40.818551, -73.948674 };

  struct arg param;
  param.argc = argc;
  param.argv = argv;

  pthread_t rosThread;

  // **** init threads 
  g_thread_init (NULL);
  gdk_threads_init ();
  gdk_threads_enter ();

  // **** init gtk 
  gtk_init (&argc, &argv);

  // **** allocate data structure
  data = g_slice_new (AppData);

  // **** set the glade gui file & set icon directory
  std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
  sprintf (glade_gui_file, "%s/gui/%s", package_path.c_str (), "gui.glade");
  sprintf (data->icon_directory, "%s/gui/icon", package_path.c_str ());
  
  std::string rosbag_path = ros::package::getPath("rosbag");
  sprintf (data->rosbag_rec_path, "%s/bin/record", rosbag_path.c_str ());
  
  data->current_page = 0;
  data->telemetry_opt_popup_state = false;
  data->gps_opt_popup_state = false;
  data->fullscreen = false;
  load_icon ();

  // **** Create new GtkBuilder object
  builder = gtk_builder_new ();
  // **** Load UI from file
  if (!gtk_builder_add_from_file (builder, glade_gui_file, &error))
  {
    g_warning ("%s", error->message);
    g_free (error);
    exit (-1);
  }

  // **** Get main window pointer from UI
  data->window = GTK_WIDGET (gtk_builder_get_object (builder, "window1"));
  gtk_window_set_title (GTK_WINDOW (data->window), "CityFlyer Ground Station");
  gtk_window_set_position (GTK_WINDOW (data->window), GTK_WIN_POS_CENTER);
  gtk_window_set_default_size (GTK_WINDOW (data->window), 1024, 576);

  // **** create ROS thread
  pthread_create (&rosThread, NULL, startROS, &param);

  // **** wait ros finish read params
  while (!data->ros_param_read)
  {
    ROS_DEBUG ("Waiting ROS params");
  }

  // **** Get GtkNotebook objsect
  data->notebook = GTK_WIDGET (gtk_builder_get_object (builder, "notebook1"));

  // #####################################################################
  // #####################################################################
  // **** Tab 1: Telemetry

  // **** create altimeter widgets
  data->alt = gtk_altimeter_new ();
  g_object_set (GTK_ALTIMETER (data->alt),
                "grayscale-color", data->grayscale_color,
                "unit-is-feet", data->altimeter_unit_is_feet,
                "unit-step-value", data->altimeter_step_value, "radial-color", data->radial_color, NULL);

  // **** create compass widgets
  data->comp = gtk_compass_new ();
  g_object_set (GTK_COMPASS (data->comp),
                "grayscale-color", data->grayscale_color, "radial-color", data->radial_color, NULL);

  data->comp2 = gtk_compass_new ();
  g_object_set (GTK_COMPASS (data->comp2),
                "grayscale-color", data->grayscale_color, "radial-color", data->radial_color, NULL);

  data->gauge1 = gtk_gauge_new ();
  g_object_set (GTK_GAUGE (data->gauge1), "name", data->gauge1_name_f, NULL);
  g_object_set (GTK_GAUGE (data->gauge1),
                "grayscale-color", data->grayscale_color,
                "radial-color", data->radial_color,
                "start-value", data->gauge1_start_value,
                "end-value", data->gauge1_end_value,
                "initial-step", data->gauge1_initial_step,
                "sub-step", (gdouble) data->gauge1_sub_step,
                "drawing-step", data->gauge1_drawing_step,
                "color-strip-order", data->gauge1_color_strip_order,
                "green-strip-start", data->gauge1_green_strip_start,
                "yellow-strip-start", data->gauge1_yellow_strip_start,
                "red-strip-start", data->gauge1_red_strip_start, NULL);

  // **** create artificial horizon widgets
  data->arh = gtk_artificial_horizon_new ();
  g_object_set (GTK_ARTIFICIAL_HORIZON (data->arh),
                "grayscale-color", data->grayscale_color, "radial-color", data->radial_color, NULL);

  // **** create variometer widgets
  data->vario = gtk_variometer_new ();
  g_object_set (GTK_VARIOMETER (data->vario),
                "grayscale-color", data->grayscale_color,
                "unit-is-feet", data->variometer_unit_is_feet,
                "unit-step-value", data->variometer_step_value, "radial-color", data->radial_color, NULL);

  data->widget_table = GTK_WIDGET (gtk_builder_get_object (builder, "table_Widgets"));
  gtk_table_attach_defaults (GTK_TABLE (data->widget_table), data->alt, 0, 1, 0, 1);
  gtk_table_attach_defaults (GTK_TABLE (data->widget_table), data->arh, 1, 2, 0, 1);
  gtk_table_attach_defaults (GTK_TABLE (data->widget_table), data->comp, 2, 3, 0, 1);
  gtk_table_attach_defaults (GTK_TABLE (data->widget_table), data->vario, 0, 1, 1, 2);
  gtk_table_attach_defaults (GTK_TABLE (data->widget_table), data->comp2, 1, 2, 1, 2);
  gtk_table_attach_defaults (GTK_TABLE (data->widget_table), data->gauge1, 2, 3, 1, 2);

  gtk_widget_modify_bg (data->alt, GTK_STATE_NORMAL, &black);
  gtk_widget_modify_bg (data->comp, GTK_STATE_NORMAL, &black);
  gtk_widget_modify_bg (data->comp2, GTK_STATE_NORMAL, &black);
  gtk_widget_modify_bg (data->arh, GTK_STATE_NORMAL, &black);
  gtk_widget_modify_bg (data->gauge1, GTK_STATE_NORMAL, &black);
  gtk_widget_modify_bg (data->vario, GTK_STATE_NORMAL, &black);

  data->telemetry_option_popup = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_TelemetryOption"));
  data->btn_open_telemetry_option_popup =
    GTK_WIDGET (gtk_builder_get_object (builder, "button_OpenTelemetryOptionPopup"));
  data->btn_close_telemetry_option_popup =
    GTK_WIDGET (gtk_builder_get_object (builder, "button_CloseTelemetryOptionPopup"));
  gtk_button_set_image (GTK_BUTTON (data->btn_open_telemetry_option_popup),
                        gtk_image_new_from_pixbuf (gdk_pixbuf_scale_simple
                                                   (data->leftarrow_icon_64, 24, 50, GDK_INTERP_HYPER)));
  gtk_button_set_image (GTK_BUTTON (data->btn_close_telemetry_option_popup),
                        gtk_image_new_from_pixbuf (gdk_pixbuf_scale_simple
                                                   (data->rightarrow_icon_64, 24, 50, GDK_INTERP_HYPER)));

  // #####################################################################
  // #####################################################################
  // **** Tab 2: Gps

  // Some GpsdViewer initialisation
  data->draw_path = false;
  data->map_provider = OSM_GPS_MAP_SOURCE_OPENSTREETMAP;
  data->map_zoom_max = 18;
  data->map_current_zoom = start_zoom;
  data->repo_uri = osm_gps_map_source_get_repo_uri (data->map_provider);
  data->friendly_name = osm_gps_map_source_get_friendly_name (data->map_provider);
  data->uav_track = osm_gps_map_track_new ();
  mapcachedir = osm_gps_map_get_default_cache_directory ();
  data->cachedir = g_build_filename (mapcachedir, data->friendly_name, NULL);
  g_free (mapcachedir);

  // Create the OsmGpsMap object
  data->map = (OsmGpsMap *) g_object_new (OSM_TYPE_GPS_MAP,
                                          "map-source", data->map_provider,
                                          "tile-cache", data->cachedir, "proxy-uri", g_getenv ("http_proxy"), NULL);

  //Set the starting coordinates and zoom level for the map
  osm_gps_map_set_zoom (data->map, start_zoom);
  osm_gps_map_set_center (data->map, ccny_coord.rlat, ccny_coord.rlon);

  data->osd = gpsd_viewer_osd_new ();
  g_object_set (GPSD_VIEWER_OSD (data->osd),
                "show-scale", true,
                "show-coordinates", true,
                "show-dpad", true,
                "show-zoom", true, "show-gps-in-dpad", true, "show-gps-in-zoom", false, "dpad-radius", 30, NULL);
  osm_gps_map_layer_add (OSM_GPS_MAP (data->map), OSM_GPS_MAP_LAYER (data->osd));

  data->map_box = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_map_box"));
  data->map_container = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_map_container"));
  gtk_box_pack_start (GTK_BOX (data->map_box), GTK_WIDGET (data->map), TRUE, TRUE, 0);

  data->gpsd_option_popup = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_GpsdOptionPopup"));
  data->btn_open_gpsd_option_popup = GTK_WIDGET (gtk_builder_get_object (builder, "button_OpenGpsdOptionPopup"));
  data->btn_close_gpsd_option_popup = GTK_WIDGET (gtk_builder_get_object (builder, "button_CloseGpsdOptionPopup"));
  gtk_button_set_image (GTK_BUTTON (data->btn_open_gpsd_option_popup),
                        gtk_image_new_from_pixbuf (gdk_pixbuf_scale_simple
                                                   (data->leftarrow_icon_64, 24, 50, GDK_INTERP_HYPER)));
  gtk_button_set_image (GTK_BUTTON (data->btn_close_gpsd_option_popup),
                        gtk_image_new_from_pixbuf (gdk_pixbuf_scale_simple
                                                   (data->rightarrow_icon_64, 24, 50, GDK_INTERP_HYPER)));

  // #####################################################################
  // #####################################################################
  // **** Tab 3: Rec

  data->recording = 0;
  data->rosbag_record_cmd = "rosbag record";
  data->topicsList = GTK_LIST_STORE (gtk_builder_get_object (builder, "liststore_TopicList"));
  data->cmd_line_entry = GTK_WIDGET (gtk_builder_get_object (builder, "entry_CommandLine"));
  data->prefix_entry = GTK_WIDGET (gtk_builder_get_object (builder, "entry_Prefix"));
  data->info_textview = GTK_WIDGET (gtk_builder_get_object (builder, "textview_BagInfo"));
  data->update_btn = GTK_WIDGET (gtk_builder_get_object (builder, "button_UpdateTopicList"));
  data->box_MotorStatus = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_MotorStatus"));
  data->box_Flying = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_Flying"));
  data->box_Gps = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_Gps"));
  data->flightMode_label = GTK_WIDGET (gtk_builder_get_object (builder, "label_FlightModeValue"));
  data->upTime_label = GTK_WIDGET (gtk_builder_get_object (builder, "label_UpTimeValue"));
  data->cpuLoad_label = GTK_WIDGET (gtk_builder_get_object (builder, "label_CpuLoadValue"));
  data->box_RecordStatus = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_RecordStatus"));
  data->record_stop_btn = GTK_WIDGET (gtk_builder_get_object (builder, "button_RecordStop"));

  gtk_box_pack_end (GTK_BOX (data->box_MotorStatus), data->status_ok_icon_motor, TRUE, TRUE, 0);
  gtk_box_pack_end (GTK_BOX (data->box_MotorStatus), data->status_fail_icon_motor, TRUE, TRUE, 0);
  gtk_box_pack_end (GTK_BOX (data->box_Flying), data->status_ok_icon_flying, TRUE, TRUE, 0);
  gtk_box_pack_end (GTK_BOX (data->box_Flying), data->status_fail_icon_flying, TRUE, TRUE, 0);
  gtk_box_pack_end (GTK_BOX (data->box_Gps), data->status_ok_icon_gps, TRUE, TRUE, 0);
  gtk_box_pack_end (GTK_BOX (data->box_Gps), data->status_fail_icon_gps, TRUE, TRUE, 0);
  gtk_box_pack_end (GTK_BOX (data->box_RecordStatus), data->record_icon, TRUE, TRUE, 0);
  gtk_box_pack_end (GTK_BOX (data->box_RecordStatus), data->record_g_icon, TRUE, TRUE, 0);

  gtk_button_set_image (GTK_BUTTON (data->update_btn),
                        gtk_image_new_from_pixbuf (gdk_pixbuf_scale_simple
                                                   (data->refresh_icon_64, 24, 24, GDK_INTERP_HYPER)));
  gtk_button_set_image (GTK_BUTTON (data->record_stop_btn),
                        gtk_image_new_from_pixbuf (gdk_pixbuf_scale_simple
                                                   (data->record_icon_64, 40, 40, GDK_INTERP_HYPER)));

  // Connect signals
  gtk_builder_connect_signals (builder, data);

  // Destroy builder, since we don't need it anymore
  g_object_unref (G_OBJECT (builder));

  // Show window. All other widgets are automatically shown by GtkBuilder
  gtk_widget_show_all (data->window);
  gtk_widget_hide(data->record_icon);
  gtk_widget_hide(data->status_ok_icon_motor);
  gtk_widget_hide(data->status_ok_icon_flying);
  gtk_widget_hide(data->status_ok_icon_gps);
  gtk_widget_hide_all(data->telemetry_option_popup);
  gtk_widget_hide_all(data->gpsd_option_popup);

  // **** allow ROS spinning
  data->widget_created = true;

  // **** udpate all widgets    
  g_timeout_add (data->telemetry_refresh_rate, widgets_update, NULL);

  gtk_main ();
  gdk_threads_leave ();
  return 0;
}
