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

#include <ground_station/ground_station.h>

AppData *data;

void updateAltimeterCallback(const geometry_msgs::PoseConstPtr& msg)
{
	int angle;
	
	// **** get GTK thread lock
	gdk_threads_enter ();
	
	// **** update altimeter
	if(IS_GTK_ALTIMETER(data->alt))		
	{
		gtk_altimeter_set_alti (GTK_ALTIMETER(data->alt), msg->position.z);
		gtk_altimeter_redraw (GTK_ALTIMETER(data->alt));
	}
	
	// **** update compass
	if(IS_GTK_COMPASS(data->comp))		
	{
		angle=(int)msg->position.z /10 % 360;
		gtk_compass_set_angle(GTK_COMPASS(data->comp), angle);
		gtk_compass_redraw(GTK_COMPASS(data->comp));
	}

	// **** release GTK thread lock 
	gdk_threads_leave ();
}

void *startROS (void * user)
{
   if (user != NULL) 
   { 
      struct arg *p_arg = (arg*)user; 

		ros::init (p_arg->argc, p_arg->argv, "ground_station");
		ros::NodeHandle n;
		
		std::string local_path;
		std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
		ros::NodeHandle n_param ("~");
		XmlRpc::XmlRpcValue xml_marker_center;

		ROS_INFO("Starting CityFlyer Ground Station");

		// **** get parameters

		if (!n_param.getParam("altimeter_unit_is_feet", data->altimeter_unit_is_feet))
			data->altimeter_unit_is_feet = true;
		ROS_INFO ("\tAltimeter unit is FEET: %d", data->altimeter_unit_is_feet);
		
		if (!n_param.getParam("altimeter_step_value", data->altimeter_step_value))
			data->altimeter_step_value = 100;
		ROS_INFO ("\tAltimeter step value: %d", data->altimeter_step_value);
		
		if (!n_param.getParam("altimeter_inv_color", data->altimeter_inv_color))
			data->altimeter_inv_color = false;
		ROS_INFO ("\tAltimeter color inversed: %d", data->altimeter_inv_color);
		
		// **** allow widget creation
		data->ros_param_read = true;
		
		// **** wait to widget creation
		while(!data->widget_created)
		{
			ROS_DEBUG("Waiting widgets creation");
	   }
			
		// **** topics subscribing
		ros::Subscriber sub = n.subscribe("fake_alti", 1, updateAltimeterCallback);
		
		ROS_INFO ("Spinning");
		ros::spin ();
	}
	
	// **** stop the gtk main loop
	if(GTK_IS_WINDOW(data->window))
	{
		gtk_main_quit();
	}
	pthread_exit (NULL);
}

int main (int argc, char **argv)
{
  GtkWidget *hbox;

  struct arg param; 
  param.argc = argc; 
  param.argv = argv; 
	
  pthread_t rosThread;
  
  // **** init threads 
  g_thread_init (NULL);
  gdk_threads_init ();
  gdk_threads_enter ();
 
  // **** init gtk 
  gtk_init(&argc, &argv);
  
  // **** allocate data structure
  data = g_slice_new (AppData);
  
  // **** create window
  data->window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(data->window), "CityFlyer Ground Station");
  gtk_window_set_position(GTK_WINDOW(data->window), GTK_WIN_POS_CENTER);
  gtk_window_set_default_size(GTK_WINDOW(data->window), 700, 500);

  g_signal_connect(G_OBJECT(data->window), "destroy", 
       G_CALLBACK(gtk_main_quit), NULL);

  hbox = gtk_hbox_new(TRUE, 1);
  gtk_container_add(GTK_CONTAINER(data->window), hbox);
  
  // **** create ROS thread
  pthread_create (&rosThread, NULL, startROS, &param);
  
  // **** wait ros finish read params
  while(!data->ros_param_read)
  {
	  ROS_DEBUG("Waiting ROS params");
  }
    
  // **** create widgets
  data->alt = gtk_altimeter_new();
  g_object_set (GTK_ALTIMETER(data->alt), "inverse-color", data->altimeter_inv_color,
														"unit-is-feet", data->altimeter_unit_is_feet,
														"unit-step-value", data->altimeter_step_value,
														"radial_color", 1,
														NULL);

  data->comp = gtk_compass_new();
  g_object_set (GTK_COMPASS(data->comp), "inverse-color", 0,
													  "radial_color", 1,
														NULL);
  
  gtk_box_pack_start(GTK_BOX(hbox), GTK_WIDGET(data->alt), TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox), GTK_WIDGET(data->comp), TRUE, TRUE, 0);

  gtk_widget_show_all(data->window);
  
  // **** allow ROS spinning
  data->widget_created = true;
    
  gtk_main(); 
  gdk_threads_leave ();
  return 0;
}

