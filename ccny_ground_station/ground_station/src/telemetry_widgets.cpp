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
 * @file telemetry_widgets.cpp 
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

#include <ground_station/telemetry_widgets.h>

AppData *data;

void updateAltitudeCallback (const geometry_msgs::PoseConstPtr & msg)
{
  // **** get GTK thread lock
  gdk_threads_enter ();

  // **** update altimeter
  if (IS_GTK_ALTIMETER (data->alt))
  {
    gtk_altimeter_set_alti (GTK_ALTIMETER (data->alt), msg->position.z);
  }

  // **** update variometer
  if (IS_GTK_VARIOMETER (data->vario))
    gtk_variometer_set_value (GTK_VARIOMETER (data->vario), ((int)msg->position.z)%100);

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
     * Can't figure out why I can use a double in a ROS launch file
     * To avoid wasting time, I use string.
     * (string)3_45 = (double)3.45
     */
    if (!n_param.getParam ("gauge1_sub_step", gauge1_sub_step))
      data->gauge1_sub_step = 2;
    size_t found = gauge1_sub_step.find_first_of ("_");
    gauge1_sub_step[found] = '.';
    ROS_DEBUG ("\tGauge 1 sub step value: %s", gauge1_sub_step.c_str ());
    try
    {
      data->gauge1_sub_step = boost::lexical_cast < double >(gauge1_sub_step);
    }
    catch (const std::exception &)
    {
      data->gauge1_sub_step = 0;        // **** Will cause a Gtk warning on the gauge
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
    // **** Gauge2 parameters
    std::string gauge2_name, gauge2_unit, gauge2_color_strip, gauge2_sub_step;

    n_param.param ("gauge2_name", gauge2_name, std::string ("Gauge 1"));
    n_param.param ("gauge2_unit", gauge2_unit, std::string ("(unit)"));
    sprintf (data->gauge2_name_f, "<big>%s</big>\n<span foreground=\"orange\"><i>(%s)</i></span>", gauge2_name.c_str (),
             gauge2_unit.c_str ());
    ROS_DEBUG ("\tGauge 1 name : %s", data->gauge2_name_f);

    if (!n_param.getParam ("gauge2_start_value", data->gauge2_start_value))
      data->gauge2_start_value = 0;
    ROS_DEBUG ("\tGauge 1 start value: %d", data->gauge2_start_value);

    if (!n_param.getParam ("gauge2_end_value", data->gauge2_end_value))
      data->gauge2_end_value = 100;
    ROS_DEBUG ("\tGauge 1 end value: %d", data->gauge2_end_value);

    if (!n_param.getParam ("gauge2_initial_step", data->gauge2_initial_step))
      data->gauge2_initial_step = 10;
    ROS_DEBUG ("\tGauge 1 initial step value: %d", data->gauge2_initial_step);

    /*
     * Can't figure out why I can use a double in a ROS launch file
     * To avoid wasting time, I use string.
     * (string)3_45 = (double)3.45
     */
    if (!n_param.getParam ("gauge2_sub_step", gauge2_sub_step))
      data->gauge2_sub_step = 2;
    found = gauge2_sub_step.find_first_of ("_");
    gauge2_sub_step[found] = '.';
    ROS_DEBUG ("\tGauge 1 sub step value: %s", gauge2_sub_step.c_str ());
    try
    {
      data->gauge2_sub_step = boost::lexical_cast < double >(gauge2_sub_step);
    }
    catch (const std::exception &)
    {
      data->gauge2_sub_step = 0;        // **** Will cause a Gtk warning on the gauge
    }

    if (!n_param.getParam ("gauge2_drawing_step", data->gauge2_drawing_step))
      data->gauge2_drawing_step = 10;
    ROS_DEBUG ("\tGauge 1 drawing step value: %d", data->gauge2_drawing_step);

    // -----------------------------------------------------------------      
    // **** Bar Gauge parameters
    std::string widget_name;
    std::string name_bar_gauge1, name_bar_gauge2, name_bar_gauge3;
    std::string unit_bar_gauge1, unit_bar_gauge2, unit_bar_gauge3;

    n_param.param ("bg_widget_name", widget_name, std::string ("Randow Bar Gauges"));
    sprintf (data->widget_name, "<big>%s</big>", widget_name.c_str ());
    ROS_DEBUG ("\tWidget name : %s", data->name_bar_gauge1_f);

    n_param.param ("bg_name_bar_gauge1", name_bar_gauge1, std::string ("BG1"));
    n_param.param ("bg_unit_bar_gauge1", unit_bar_gauge1, std::string ("(X)"));
    sprintf (data->name_bar_gauge1_f, "<big>%s</big>\n<span foreground=\"orange\"><i>(%s)</i></span>",
             name_bar_gauge1.c_str (), unit_bar_gauge1.c_str ());
    ROS_DEBUG ("\tBar Gauge 1 name : %s", data->name_bar_gauge1_f);

    n_param.param ("bg_name_bar_gauge2", name_bar_gauge2, std::string ("BG2"));
    n_param.param ("bg_unit_bar_gauge2", unit_bar_gauge2, std::string ("(X)"));
    sprintf (data->name_bar_gauge2_f, "<big>%s</big>\n<span foreground=\"orange\"><i>(%s)</i></span>",
             name_bar_gauge2.c_str (), unit_bar_gauge2.c_str ());
    ROS_DEBUG ("\tBar Gauge 2 name : %s", data->name_bar_gauge2_f);

    n_param.param ("bg_name_bar_gauge3", name_bar_gauge3, std::string ("BG3"));
    n_param.param ("bg_unit_bar_gauge3", unit_bar_gauge3, std::string ("(X)"));
    sprintf (data->name_bar_gauge3_f, "<big>%s</big>\n<span foreground=\"orange\"><i>(%s)</i></span>",
             name_bar_gauge3.c_str (), unit_bar_gauge3.c_str ());
    ROS_DEBUG ("\tBar Gauge 3 name : %s", data->name_bar_gauge3_f);

    if (!n_param.getParam ("bg_bar_number", data->bar_number))
      data->bar_number = 1;
    ROS_DEBUG ("\tNum containser of bar gauge: %d", data->bar_number);

    if (!n_param.getParam ("bg_start_value_bar_1", data->start_value_bar_1))
      data->start_value_bar_1 = 100;
    ROS_DEBUG ("\tStart value bar 1: %d", data->start_value_bar_1);

    if (!n_param.getParam ("bg_end_value_bar_1", data->end_value_bar_1))
      data->end_value_bar_1 = 0;
    ROS_DEBUG ("\tEnd value bar 1: %d", data->end_value_bar_1);

    n_param.getParam ("bg_green_strip_start_1", data->green_strip_start_1);
    n_param.getParam ("bg_yellow_strip_start_1", data->yellow_strip_start_1);

    if (!n_param.getParam ("bg_start_value_bar_2", data->start_value_bar_2))
      data->start_value_bar_2 = 100;
    ROS_DEBUG ("\tStart value bar 2: %d", data->start_value_bar_2);

    if (!n_param.getParam ("bg_end_value_bar_2", data->end_value_bar_2))
      data->end_value_bar_2 = 0;
    ROS_DEBUG ("\tEnd value bar 2: %d", data->end_value_bar_2);

    if (!n_param.getParam ("bg_start_value_bar_3", data->start_value_bar_3))
      data->start_value_bar_3 = 100;
    ROS_DEBUG ("\tStart value bar 3: %d", data->start_value_bar_3);

    if (!n_param.getParam ("bg_end_value_bar_3", data->end_value_bar_3))
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
    ROS_INFO ("Subscribing to info topic");
    ros::Subscriber sub;
    sub = n.subscribe ("fake_alti", 1, updateAltitudeCallback);

    ROS_INFO ("Spinning");
    ros::spin ();
  }

  // **** stop the gtk main loop
  if (GTK_IS_WINDOW (data->window))
  {
    gtk_main_quit ();
  }
  pthread_exit (NULL);
}

/**
 * @fn gboolean window_update(gpointer dat)
 * @brief Gtk function which allow to refresh the window, futhermore the widgets<br>
 * @todo find an other way to refresh the widgets (window updates should be update childs)
 * widgets.
 */
gboolean window_update (gpointer dat)
{
  // **** update altimeter
  if (IS_GTK_ALTIMETER (data->alt))
    gtk_altimeter_redraw (GTK_ALTIMETER (data->alt));

  // **** update variometer
  if (IS_GTK_VARIOMETER (data->vario))
    gtk_variometer_redraw (GTK_VARIOMETER (data->vario));

  // **** update compass
  if (IS_GTK_COMPASS (data->comp))
    gtk_compass_redraw (GTK_COMPASS (data->comp));

  // **** update gauge
  if (IS_GTK_GAUGE (data->gauge2))
    gtk_gauge_redraw (GTK_GAUGE (data->gauge2));

  // **** update gauge
  if (IS_GTK_GAUGE (data->gauge1))
    gtk_gauge_redraw (GTK_GAUGE (data->gauge1));

  // **** update bar gauge
  if (IS_GTK_BAR_GAUGE (data->bg))
    gtk_bar_gauge_redraw (GTK_BAR_GAUGE (data->bg));

  // **** update turn coordinator
  if (IS_GTK_TURN_COORDINATOR (data->tc))
    gtk_turn_coordinator_redraw (GTK_TURN_COORDINATOR (data->tc));

  // **** artificial horizon 
  if (IS_GTK_ARTIFICIAL_HORIZON (data->arh))
    gtk_artificial_horizon_redraw (GTK_ARTIFICIAL_HORIZON (data->arh));
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
  GtkWidget *vbox;
  GtkWidget *hbox_up;
  GtkWidget *hbox_down;
  GdkColor bg_color;

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

  // **** create window
  data->window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (data->window), "CityFlyer Ground Station");
  gtk_window_set_position (GTK_WINDOW (data->window), GTK_WIN_POS_CENTER);
  gtk_window_set_default_size (GTK_WINDOW (data->window), 1000, 500);

  g_signal_connect (G_OBJECT (data->window), "destroy", G_CALLBACK (gtk_main_quit), NULL);

  vbox = gtk_vbox_new (TRUE, 1);
  gtk_container_add (GTK_CONTAINER (data->window), vbox);

  hbox_up = gtk_hbox_new (TRUE, 1);
  gtk_container_add (GTK_CONTAINER (vbox), hbox_up);

  hbox_down = gtk_hbox_new (TRUE, 1);
  gtk_container_add (GTK_CONTAINER (vbox), hbox_down);

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

  // **** create gauge widgets
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

  // **** create gauge widgets
  data->gauge2 = gtk_gauge_new ();
  g_object_set (GTK_GAUGE (data->gauge2), "name", data->gauge2_name_f, NULL);
  g_object_set (GTK_GAUGE (data->gauge2),
                "grayscale-color", data->grayscale_color,
                "radial-color", data->radial_color,
                "start-value", data->gauge2_start_value,
                "end-value", data->gauge2_end_value,
                "initial-step", data->gauge2_initial_step,
                "sub-step", (gdouble) data->gauge2_sub_step, "drawing-step", data->gauge2_drawing_step, NULL);

  // **** create gauge widgets
  data->bg = gtk_bar_gauge_new ();
  g_object_set (GTK_BAR_GAUGE (data->bg), "widget-name", data->widget_name, NULL);
  g_object_set (GTK_BAR_GAUGE (data->bg), "name-bar-1", data->name_bar_gauge1_f, NULL);
  g_object_set (GTK_BAR_GAUGE (data->bg), "name-bar-2", data->name_bar_gauge2_f, NULL);
  g_object_set (GTK_BAR_GAUGE (data->bg), "name-bar-3", data->name_bar_gauge3_f, NULL);
  g_object_set (GTK_BAR_GAUGE (data->bg),
                "bar-number", data->bar_number,
                "grayscale-color", data->grayscale_color,
                "radial-color", data->radial_color,
                "start-value-bar-1", data->start_value_bar_1,
                "end-value-bar-1", data->end_value_bar_1,
                "green-strip-start-1", data->green_strip_start_1,
                "yellow-strip-start-1", data->yellow_strip_start_1,
                "start-value-bar-2", data->start_value_bar_2,
                "end-value-bar-2", data->end_value_bar_2,
                "start-value-bar-3", data->start_value_bar_3, "end-value-bar-3", data->end_value_bar_3, NULL);

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

  // **** create variometer widgets
  data->tc = gtk_turn_coordinator_new ();
  g_object_set (GTK_TURN_COORDINATOR (data->tc),
                "grayscale-color", data->grayscale_color, "radial-color", data->radial_color, NULL);

  gtk_box_pack_start (GTK_BOX (hbox_up), GTK_WIDGET (data->gauge2), TRUE, TRUE, 0);
  gtk_box_pack_start (GTK_BOX (hbox_up), GTK_WIDGET (data->alt), TRUE, TRUE, 0);
  gtk_box_pack_start (GTK_BOX (hbox_up), GTK_WIDGET (data->arh), TRUE, TRUE, 0);
  gtk_box_pack_start (GTK_BOX (hbox_up), GTK_WIDGET (data->bg), TRUE, TRUE, 0);

  gtk_box_pack_start (GTK_BOX (hbox_down), GTK_WIDGET (data->tc), TRUE, TRUE, 0);
  gtk_box_pack_start (GTK_BOX (hbox_down), GTK_WIDGET (data->gauge1), TRUE, TRUE, 0);
  gtk_box_pack_start (GTK_BOX (hbox_down), GTK_WIDGET (data->comp), TRUE, TRUE, 0);
  gtk_box_pack_start (GTK_BOX (hbox_down), GTK_WIDGET (data->vario), TRUE, TRUE, 0);

  bg_color.red = (guint16) 0.4 *65535;
  bg_color.green = (guint16) 0.4 *65535;
  bg_color.blue = (guint16) 0.4 *65535;
  gtk_widget_modify_bg (data->window, GTK_STATE_NORMAL, &bg_color);
  gtk_widget_modify_bg (data->alt, GTK_STATE_NORMAL, &bg_color);
  gtk_widget_modify_bg (data->comp, GTK_STATE_NORMAL, &bg_color);
  gtk_widget_modify_bg (data->gauge1, GTK_STATE_NORMAL, &bg_color);
  gtk_widget_modify_bg (data->gauge2, GTK_STATE_NORMAL, &bg_color);
  gtk_widget_modify_bg (data->arh, GTK_STATE_NORMAL, &bg_color);
  gtk_widget_modify_bg (data->bg, GTK_STATE_NORMAL, &bg_color);
  gtk_widget_modify_bg (data->vario, GTK_STATE_NORMAL, &bg_color);
  gtk_widget_modify_bg (data->tc, GTK_STATE_NORMAL, &bg_color);

  gtk_widget_show_all (data->window);

  // **** allow ROS spinning
  data->widget_created = true;

  // **** udpate all widgets
  g_timeout_add (100, window_update, NULL);

  gtk_main ();
  gdk_threads_leave ();
  return 0;
}
