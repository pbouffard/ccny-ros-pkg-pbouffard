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
 * Gauges Ground Station for CityFlyer CCNY project
 * Copyright (C) 2010, CCNY Robotics Lab
 * http://robotics.ccny.cuny.edu
 *  
 */

#include <ground_station/ground_station.h>

AppData *data;

void imuCalcDataCallback (const asctec_msgs::IMUCalcDataConstPtr & dat)
{
  // **** get GTK thread lock
  gdk_threads_enter ();

  imuCalcData_ = (*dat);

  if (IS_GTK_ALTIMETER (data->alt))
      gtk_altimeter_set_alti (GTK_ALTIMETER (data->alt), (double)(imuCalcData_.height/10.));
  
  if (IS_GTK_VARIOMETER (data->vario))
    gtk_variometer_set_alti (GTK_VARIOMETER (data->vario), (double)(imuCalcData_.height/1000.));

  if (IS_GTK_COMPASS (data->comp))
      gtk_compass_set_angle (GTK_COMPASS (data->comp), (double)(imuCalcData_.mag_heading/1000.));
      
  if (IS_GTK_COMPASS (data->comp2))
      gtk_compass_set_angle (GTK_COMPASS (data->comp2), (double)(imuCalcData_.angle_yaw/1000.));      
  
  if (IS_GTK_ARTIFICIAL_HORIZON (data->arh))
      gtk_artificial_horizon_set_value (GTK_ARTIFICIAL_HORIZON (data->arh), 
													 (double)((imuCalcData_.angle_roll+360000)%360000)/1000.0,
													 (double)(imuCalcData_.angle_nick/1000.));      

  // **** release GTK thread lock 
  gdk_threads_leave ();
}

void gpsFixCallback (const gps_common::GPSFix::ConstPtr & msg)
{
  // **** get GTK thread lock
  gdk_threads_enter ();
  
  gint pixel_x,pixel_y;
  
  OsmGpsMapPoint * point = osm_gps_map_point_new_degrees(msg->latitude,msg->longitude);
  osm_gps_map_convert_geographic_to_screen(data->map, point, &pixel_x, &pixel_y);
     
  if (OSM_IS_GPS_MAP (data->map)){

		// **** Center map on gps data received
		if(data->lock_view)
		{
			update_uav_pose_osd(data->osd,TRUE, pixel_x, pixel_y);
			osm_gps_map_set_center (data->map, msg->latitude, msg->longitude);
		}
		else
		{
			update_uav_pose_osd(data->osd,FALSE, pixel_x, pixel_y);
			osm_gps_map_gps_clear(data->map);
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

  // **** update altimeter
  if (IS_GTK_BAR_GAUGE (data->bg))
      gtk_bar_gauge_set_value(GTK_BAR_GAUGE (data->bg), 1,(double)(llStatus_.battery_voltage_1/1000.));
  
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
    // **** Bar Gauge parameters
    std::string widget_name;
    std::string name_bar_gauge1, name_bar_gauge2, name_bar_gauge3;
    std::string unit_bar_gauge1, unit_bar_gauge2, unit_bar_gauge3;

    n_param.param ("widget_name", widget_name, std::string ("Randow Bar Gauges"));
    sprintf (data->widget_name, "<big>%s</big>", widget_name.c_str ());
    ROS_DEBUG ("\tWidget name : %s", data->name_bar_gauge1_f);

    n_param.param ("name_bar_gauge1", name_bar_gauge1, std::string ("BG1"));
    n_param.param ("unit_bar_gauge1", unit_bar_gauge1, std::string ("(X)"));
    sprintf (data->name_bar_gauge1_f, "<big>%s</big>\n<span foreground=\"orange\"><i>(%s)</i></span>",
             name_bar_gauge1.c_str (), unit_bar_gauge1.c_str ());
    ROS_DEBUG ("\tBar Gauge 1 name : %s", data->name_bar_gauge1_f);

    n_param.param ("name_bar_gauge2", name_bar_gauge2, std::string ("BG2"));
    n_param.param ("unit_bar_gauge2", unit_bar_gauge2, std::string ("(X)"));
    sprintf (data->name_bar_gauge2_f, "<big>%s</big>\n<span foreground=\"orange\"><i>(%s)</i></span>",
             name_bar_gauge2.c_str (), unit_bar_gauge2.c_str ());
    ROS_DEBUG ("\tBar Gauge 2 name : %s", data->name_bar_gauge2_f);

    n_param.param ("name_bar_gauge3", name_bar_gauge3, std::string ("BG3"));
    n_param.param ("unit_bar_gauge3", unit_bar_gauge3, std::string ("(X)"));
    sprintf (data->name_bar_gauge3_f, "<big>%s</big>\n<span foreground=\"orange\"><i>(%s)</i></span>",
             name_bar_gauge3.c_str (), unit_bar_gauge3.c_str ());
    ROS_DEBUG ("\tBar Gauge 3 name : %s", data->name_bar_gauge3_f);

    if (!n_param.getParam ("bar_number", data->bar_number))
      data->bar_number = 1;
    ROS_DEBUG ("\tNum containser of bar gauge: %d", data->bar_number);

    if (!n_param.getParam ("start_value_bar_1", data->start_value_bar_1))
      data->start_value_bar_1 = 100;
    ROS_DEBUG ("\tStart value bar 1: %d", data->start_value_bar_1);

    if (!n_param.getParam ("end_value_bar_1", data->end_value_bar_1))
      data->end_value_bar_1 = 0;
    ROS_DEBUG ("\tEnd value bar 1: %d", data->end_value_bar_1);

    n_param.getParam ("green_strip_start_1", data->green_strip_start_1);
    n_param.getParam ("yellow_strip_start_1", data->yellow_strip_start_1);

    if (!n_param.getParam ("start_value_bar_2", data->start_value_bar_2))
      data->start_value_bar_2 = 100;
    ROS_DEBUG ("\tStart value bar 2: %d", data->start_value_bar_2);

    if (!n_param.getParam ("end_value_bar_2", data->end_value_bar_2))
      data->end_value_bar_2 = 0;
    ROS_DEBUG ("\tEnd value bar 2: %d", data->end_value_bar_2);

    if (!n_param.getParam ("start_value_bar_3", data->start_value_bar_3))
      data->start_value_bar_3 = 100;
    ROS_DEBUG ("\tStart value bar 3: %d", data->start_value_bar_3);

    if (!n_param.getParam ("end_value_bar_3", data->end_value_bar_3))
      data->end_value_bar_3 = 0;
    ROS_DEBUG ("\tEnd value bar 3: %d", data->end_value_bar_3);

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
    imuCalcDataSub = n.subscribe (imuCalcDataTopic, 1, imuCalcDataCallback);
    //~ gpsDataSub = n.subscribe (gpsDataTopic, 1, gpsDataCallback);
    llStatusSub = n.subscribe (llStatusTopic, 1, llStatusCallback);
    gpsFixSub = n.subscribe ("/fix", 1, &gpsFixCallback);

    ROS_INFO ("Spinning");
    ros::spin ();
  }

  // **** stop the gtk main loop
  if (GTK_IS_WIDGET(data->window))
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
  gtk_widget_draw(GTK_WIDGET(data->widget_table), NULL);
  return true;
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
  GError *error = NULL;
  char gui_filename[FILENAME_MAX];
  GdkColor bg_color;
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
  
  // **** get the glade gui file
  std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
  sprintf (gui_filename, "%s/%s", package_path.c_str (), "gui.glade");

  // Create new GtkBuilder object
  builder = gtk_builder_new ();
  // Load UI from file
  if (!gtk_builder_add_from_file (builder, gui_filename, &error))
  {
    g_warning ("%s", error->message);
    g_free (error);
    exit(-1);
  }
  
  // Get main window pointer from UI
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

  // **** create gauge widgets
  data->bg = gtk_bar_gauge_new ();
  g_object_set (GTK_BAR_GAUGE (data->bg), "widget-name", data->widget_name, NULL);
  g_object_set (GTK_BAR_GAUGE (data->bg), "name-bar-1", data->name_bar_gauge1_f, NULL);
  g_object_set (GTK_BAR_GAUGE (data->bg),
                "bar-number", data->bar_number,
                "grayscale-color", data->grayscale_color,
                "radial-color", data->radial_color,
                "start-value-bar-1", data->start_value_bar_1,
                "end-value-bar-1", data->end_value_bar_1,
                "green-strip-start-1", data->green_strip_start_1,
                "yellow-strip-start-1", data->yellow_strip_start_1,
                NULL);

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
  gtk_table_attach_defaults(GTK_TABLE(data->widget_table),data->alt,0,1,0,1);
  gtk_table_attach_defaults(GTK_TABLE(data->widget_table),data->arh,1,2,0,1);
  gtk_table_attach_defaults(GTK_TABLE(data->widget_table),data->bg,2,3,0,1);
  gtk_table_attach_defaults(GTK_TABLE(data->widget_table),data->vario,0,1,1,2);
  gtk_table_attach_defaults(GTK_TABLE(data->widget_table),data->comp,1,2,1,2);
  gtk_table_attach_defaults(GTK_TABLE(data->widget_table),data->comp2,2,3,1,2);
  
  // **** set widget bg color
  bg_color.red = 0.3 *65535;
  bg_color.green = 0.3 *65535;
  bg_color.blue = 0.3 *65535;
  gtk_widget_modify_bg (data->widget_table, GTK_STATE_NORMAL, &bg_color);
  //~ gtk_widget_modify_bg (data->comp, GTK_STATE_NORMAL, &bg_color);
  //~ gtk_widget_modify_bg (data->comp2, GTK_STATE_NORMAL, &bg_color);
  //~ gtk_widget_modify_bg (data->arh, GTK_STATE_NORMAL, &bg_color);
  //~ gtk_widget_modify_bg (data->bg, GTK_STATE_NORMAL, &bg_color);
  //~ gtk_widget_modify_bg (data->vario, GTK_STATE_NORMAL, &bg_color);
  
  // Some GpsdViewer initialisation
  data->draw_path = false;
  data->map_provider = OSM_GPS_MAP_SOURCE_OPENSTREETMAP;
  data->map_zoom_max = 18;
  data->map_current_zoom = start_zoom;
  data->repo_uri = osm_gps_map_source_get_repo_uri (data->map_provider);
  data->friendly_name = osm_gps_map_source_get_friendly_name (data->map_provider);
  data->uav_track = osm_gps_map_track_new();
  mapcachedir = osm_gps_map_get_default_cache_directory ();
  data->cachedir = g_build_filename (mapcachedir, data->friendly_name, NULL);
  g_free (mapcachedir);  

  // Create the OsmGpsMap object
  data->map = (OsmGpsMap *) g_object_new (OSM_TYPE_GPS_MAP, 
					"map-source", data->map_provider,
               "tile-cache", data->cachedir, 
               "proxy-uri", g_getenv ("http_proxy"),NULL);

  //Set the starting coordinates and zoom level for the map
  osm_gps_map_set_zoom (data->map, start_zoom);
  osm_gps_map_set_center (data->map, ccny_coord.rlat, ccny_coord.rlon);

  data->osd = gpsd_viewer_osd_new();
  g_object_set(GPSD_VIEWER_OSD(data->osd),
					"show-scale",true,
               "show-coordinates",true,
               "show-dpad",true,
               "show-zoom",true,
               "show-gps-in-dpad",true,
               "show-gps-in-zoom",false,
               "dpad-radius", 30,
               NULL);                                                                     
  osm_gps_map_layer_add(OSM_GPS_MAP(data->map), OSM_GPS_MAP_LAYER(data->osd));
  
  data->map_box = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_map_box"));
  data->map_container = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_map_container"));
  gtk_box_pack_start (GTK_BOX (data->map_box), GTK_WIDGET (data->map), TRUE, TRUE, 0);
 
  data->box_gpsd_viewer = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_GpsdViewer"));
  data->gpsd_option_popup = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_GpsdOptionPopup"));
  data->btn_gpsd_option_popup = GTK_WIDGET (gtk_builder_get_object (builder, "button_OpenGpsdOptionPopup"));
  g_object_ref(GTK_WIDGET (data->gpsd_option_popup));  		// **** avoid widget to be destroyed when removed from window
  g_object_ref(GTK_WIDGET (data->btn_gpsd_option_popup));	// **** avoid widget to be destroyed when removed from window
  gtk_container_remove(GTK_CONTAINER(data->box_gpsd_viewer),GTK_WIDGET (data->gpsd_option_popup));
  
  data->rosbag_record_cmd="rosbag record";
  data->topicsList = GTK_LIST_STORE(gtk_builder_get_object (builder, "liststore_TopicList"));
  data->cmd_line_entry = GTK_WIDGET(gtk_builder_get_object (builder, "entry_CommandLine"));
  
  // Connect signals
  gtk_builder_connect_signals (builder, data);

  // Destroy builder, since we don't need it anymore
  g_object_unref (G_OBJECT (builder));

  // Show window. All other widgets are automatically shown by GtkBuilder
  gtk_widget_show_all (data->window);

  // **** allow ROS spinning
  data->widget_created = true;

  // **** udpate all widgets    
  g_timeout_add (400, widgets_update, NULL);
  
  //gtk_window_fullscreen(GTK_WINDOW(data->window));
  //gtk_window_unfullscreen(GTK_WINDOW(data->window));

  gtk_main ();
  gdk_threads_leave ();
  return 0;
}
