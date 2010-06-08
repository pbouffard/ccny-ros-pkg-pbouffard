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

// a supp
GtkWidget *alt1;
int altimeter_value;
bool altimeter_unit_is_feet,altimeter_inv_color;

int main (int argc, char **argv)
{
  struct arg data; 
  data.argc = argc; 
  data.argv = argv; 
	
  pthread_t rosThread;
  pthread_t guiThread;

  pthread_create (&guiThread, NULL, startGUI, NULL);
  sleep (3);
  pthread_create (&rosThread, NULL, startROS, &data);


  // wait guiThread to continu
  pthread_join (guiThread, NULL);
  ros::shutdown ();

  return (0);
}
 
void chatterCallback(const geometry_msgs::PoseConstPtr& msg)
{
	gtk_altimeter_set_alti(alt1, msg->position.z);
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

		if (!n_param.getParam("altimeter_unit_is_feet", altimeter_unit_is_feet))
			altimeter_unit_is_feet = true;
		ROS_INFO ("\tAltimeter unit is FEET: %d", altimeter_unit_is_feet);
		
		if (!n_param.getParam("altimeter_value", altimeter_value))
			altimeter_value = 100;
		ROS_INFO ("\tAltimeter value: %d", altimeter_value);
		
		if (!n_param.getParam("altimeter_inv_color", altimeter_inv_color))
			altimeter_inv_color = false;
		ROS_INFO ("\tAltimeter color inversed: %d", altimeter_inv_color);
		
		// Altimeter widget init
		gtk_altimeter_set_unit(alt1, altimeter_unit_is_feet);
		gtk_altimeter_set_unit_value(alt1, altimeter_value);
		gtk_altimeter_set_color_mode(alt1, altimeter_inv_color);
		sleep(3);
		
		ros::Subscriber chatter_sub = n.subscribe("fake_alti", 1, chatterCallback);
  	
		ROS_INFO ("Spinning");
		ros::spin ();
	}
	pthread_exit (NULL);
}

void *startGUI (void *)
{
  GtkWidget *window;
  GtkWidget *vbox;
  int argc = 0;
  char **argv;

  gtk_init(&argc, &argv);

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(window), "CityFlyer Ground Station");
  gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
  gtk_window_set_default_size(GTK_WINDOW(window), 500, 500);

  g_signal_connect(G_OBJECT(window), "destroy", 
       G_CALLBACK(gtk_main_quit), NULL);

  vbox = gtk_hbox_new(TRUE, 1);
  gtk_container_add(GTK_CONTAINER(window), vbox);
    
  alt1 = gtk_altimeter_new();
  gtk_container_add(GTK_CONTAINER(vbox), alt1);

  g_signal_connect (window, "destroy",
			G_CALLBACK (gtk_main_quit), NULL);
			
  gtk_widget_show_all(window);
  gtk_main();

  pthread_exit (NULL);
}
