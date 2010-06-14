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
GtkWidget *alt;
GtkWidget *comp;
int altimeter_value;
bool altimeter_unit_is_feet,altimeter_inv_color;
static gint gb_stop = FALSE;
gdouble alti = 0;

int main (int argc, char **argv)
{
  struct arg param; 
  param.argc = argc; 
  param.argv = argv; 
	
  pthread_t rosThread;
  pthread_t guiThread;

  pthread_create (&rosThread, NULL, startROS, &param);
  sleep (3);
  pthread_create (&guiThread, NULL, startGUI, NULL);

  // wait guiThread to continu
  pthread_join (guiThread, NULL);
  ros::shutdown ();

  return (0);
}


static void updateAltitudeCallback (GtkAltimeter * alt)
{
  alti++;
  gtk_altimeter_set_alti (alt, alti);
  gtk_altimeter_redraw (alt);
}
 
void chatterCallback(const geometry_msgs::PoseConstPtr& msg)
{
	//if (!gb_stop){
		//gtk_altimeter_set_alti (GTK_ALTIMETER(alt), msg->position.z);
		//gtk_altimeter_redraw (GTK_ALTIMETER(alt));
		//updateAltitudeCallback(GTK_ALTIMETER(alt));
	//}
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
		
		sleep(10);		
		ros::Subscriber chatter_sub = n.subscribe("fake_alti", 1, chatterCallback);
  	
		ROS_INFO ("Spinning");
		ros::spin ();
	}
	pthread_exit (NULL);
}


void *startGUI (void *)
{
  GtkWidget *window;
  GtkWidget *hbox;
  int argc = 0;
  char **argv;

  gtk_init(&argc, &argv);

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(window), "CityFlyer Ground Station");
  gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
  gtk_window_set_default_size(GTK_WINDOW(window), 700, 500);

  g_signal_connect(G_OBJECT(window), "destroy", 
       G_CALLBACK(gtk_main_quit), NULL);

  hbox = gtk_hbox_new(TRUE, 1);
  gtk_container_add(GTK_CONTAINER(window), hbox);
    
  alt = gtk_altimeter_new();
  g_object_set (GTK_ALTIMETER(alt), "inverse-color", 0, "unit-is-feet", 1, "unit-step-value", 100, NULL);

  comp = gtk_compass_new();
  g_object_set (GTK_COMPASS(comp), "inverse-color", 0, NULL);
  
  gtk_box_pack_start(GTK_BOX(hbox), GTK_WIDGET(alt), TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox), GTK_WIDGET(comp), TRUE, TRUE, 0);

  gtk_widget_show_all(window);
    g_timeout_add (100, (GSourceFunc) updateAltitudeCallback, alt);
  
  gtk_main();

  pthread_exit (NULL);
}
