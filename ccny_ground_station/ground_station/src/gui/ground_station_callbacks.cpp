/*
 *  ROS GPSD Map Viewer using OpenStreetMap & OSMGpsMap
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

#include "ground_station/gui/ground_station_callbacks.h"

// #####################################################################
// #####################################################################
// **** Main Window Callbacks

extern "C" G_MODULE_EXPORT void on_mainwindow_destroy (GtkObject * object, AppData * data)
{
	gtk_main_quit ();
}

extern "C" G_MODULE_EXPORT gboolean on_mainwindow_delete_event (GtkWidget *window, GdkEvent *event, AppData * data)
{
	if(data->recording)
	{
		GtkWidget * dialog = gtk_message_dialog_new (GTK_WINDOW(data->window),
                                 GTK_DIALOG_DESTROY_WITH_PARENT,
                                 GTK_MESSAGE_QUESTION,
                                 GTK_BUTTONS_OK_CANCEL,
                                 "Would you like to stop recording?");
		gint result = gtk_dialog_run (GTK_DIALOG (dialog));
		switch (result)
		{
			case GTK_RESPONSE_OK:
				stopRec(data);
				return FALSE;
				break;
			case GTK_RESPONSE_CANCEL:
				gtk_widget_destroy (dialog); 
				return TRUE;                    
				break;
		}
	}
	else 
	{
		return FALSE;
	}	
	
}

extern "C" G_MODULE_EXPORT gboolean on_mainwindow_key_press_event (GtkWidget * widget, GdkEventKey * event,
                                                                   AppData * data)
{
  if (event->type == GDK_KEY_PRESS)
  {
    switch (event->keyval)
    {
      case GDK_F11:
        // **** fullscreen/unfullscreen
        if (data->fullscreen)
        {
          data->fullscreen = false;
          gtk_window_unfullscreen (GTK_WINDOW (data->window));
        }
        else
        {
          data->fullscreen = true;
          gtk_window_fullscreen (GTK_WINDOW (data->window));
        }
        break;
      case GDK_R:
      case GDK_r:
        // **** Start recording
        startRec(data, true);
        break;
      case GDK_S:
      case GDK_s:
        // **** Stop recording
        stopRec(data);
        break;
      case GDK_Left:
        // **** Switch page
        if (data->current_page > 0)
          gtk_notebook_set_current_page (GTK_NOTEBOOK (data->notebook), data->current_page - 1);
        break;
      case GDK_Right:
        // **** Switch page
        if (data->current_page < 2)
          gtk_notebook_set_current_page (GTK_NOTEBOOK (data->notebook), data->current_page + 1);
        break;
      case GDK_space:
        // **** Open option popup menu
        switch (data->current_page)
        {
          case 0:
            if (!data->telemetry_opt_popup_state)
            {
              data->telemetry_opt_popup_state = true;
              gtk_widget_hide_all(data->btn_open_telemetry_option_popup);
				  gtk_widget_show_all(data->telemetry_option_popup);
            }
            else
            {
              data->telemetry_opt_popup_state = false;
              gtk_widget_hide_all(data->telemetry_option_popup);
				  gtk_widget_show_all(data->btn_open_telemetry_option_popup);
            }
            break;
          case 1:
            if (!data->gps_opt_popup_state)
            {
              data->gps_opt_popup_state = true;
              gtk_widget_hide_all(data->btn_open_gpsd_option_popup);
				  gtk_widget_show_all(data->gpsd_option_popup);
				}
            else
            {
              data->gps_opt_popup_state = false;
              gtk_widget_hide_all(data->gpsd_option_popup);
				  gtk_widget_show_all(data->btn_open_gpsd_option_popup);
            }
            break;
        }
        break;
    }
  }
  return false;
}

extern "C" G_MODULE_EXPORT void on_notebook1_switch_page (GtkNotebook * notebook, GtkNotebookPage * page,
                                                          guint page_num, AppData * data)
{
  GtkTreeIter iter;
  GValue topic_name = { 0 }, topic_selected =
  {
  0};
  GValue topic_name_present = { 0 };
  gboolean valid, add_to_list = TRUE;
  char topics_list[255];
  char *pch;

  data->current_page = page_num;
  switch (page_num)
  {
    case 2:
      // **** Update the topic list
      // **** TODO: remove topic when there're not anymore in the "rostopic list" cmd result
      FILE * topics = popen ("rostopic list", "r");

      while (fgets (topics_list, sizeof (topics_list), topics))
      {
        pch = strchr (topics_list, '\n');
        topics_list[pch - topics_list] = ' ';

        g_value_init (&topic_name, G_TYPE_STRING);
        g_value_init (&topic_selected, G_TYPE_BOOLEAN);
        g_value_set_string (&topic_name, topics_list);
        g_value_set_boolean (&topic_selected, FALSE);

        valid = gtk_tree_model_get_iter_first (GTK_TREE_MODEL (data->topicsList), &iter);
        while (valid)
        {
          gtk_tree_model_get_value (GTK_TREE_MODEL (data->topicsList), &iter, 0, &topic_name_present);
          add_to_list = TRUE;
          if (strcmp (g_value_get_string (&topic_name), g_value_get_string (&topic_name_present)) == 0)
          {
            add_to_list = FALSE;
            g_value_unset (&topic_name_present);
            break;
          }
          valid = gtk_tree_model_iter_next (GTK_TREE_MODEL (data->topicsList), &iter);
          g_value_unset (&topic_name_present);
        }

        if (add_to_list)
        {
          gtk_list_store_append (GTK_LIST_STORE (data->topicsList), &iter);
          gtk_list_store_set_value (GTK_LIST_STORE (data->topicsList), &iter, 0, &topic_name);
          gtk_list_store_set_value (GTK_LIST_STORE (data->topicsList), &iter, 1, &topic_selected);
        }
        g_value_unset (&topic_name);
        g_value_unset (&topic_selected);
      }
      pclose (topics);
      break;
  }
}

// #####################################################################
// #####################################################################
// **** Tab 1: Telemetry Callbacks

extern "C" G_MODULE_EXPORT void on_colorbutton_BgColor_color_set (GtkColorButton * button, AppData * data)
{
  GdkColor bg_color;

  gtk_color_button_get_color (button, &bg_color);
  gtk_widget_modify_bg (data->alt, GTK_STATE_NORMAL, &bg_color);
  gtk_widget_modify_bg (data->comp, GTK_STATE_NORMAL, &bg_color);
  gtk_widget_modify_bg (data->comp2, GTK_STATE_NORMAL, &bg_color);
  gtk_widget_modify_bg (data->arh, GTK_STATE_NORMAL, &bg_color);
  gtk_widget_modify_bg (data->gauge1, GTK_STATE_NORMAL, &bg_color);
  gtk_widget_modify_bg (data->vario, GTK_STATE_NORMAL, &bg_color);
}

extern "C" G_MODULE_EXPORT void on_button_OpenTelemetryOptionPopup_clicked (GtkButton * button, AppData * data)
{
  data->telemetry_opt_popup_state = true;
  gtk_widget_hide_all(data->btn_open_telemetry_option_popup);
  gtk_widget_show_all(data->telemetry_option_popup);
}

extern "C" G_MODULE_EXPORT void on_button_CloseTelemetryOptionPopup_clicked (GtkButton * button, AppData * data)
{
  data->telemetry_opt_popup_state = false;
  gtk_widget_hide_all(data->telemetry_option_popup);
  gtk_widget_show_all(data->btn_open_telemetry_option_popup);
}

// #####################################################################
// #####################################################################
// **** Tab 2: GpsdViewer Callbacks

extern "C" G_MODULE_EXPORT void on_checkbuttonDrawCurrentUAVTrack_toggled (GtkToggleButton * togglebutton,
                                                                           AppData * data)
{
  if (data->draw_path)
  {
    data->draw_path = false;
    osm_gps_map_track_remove (data->map, data->uav_track);
  }
  else
  {
    data->draw_path = true;
    osm_gps_map_track_add (data->map, data->uav_track);
  }
}

extern "C" G_MODULE_EXPORT void on_checkbuttonLockViewUAV_toggled (GtkToggleButton * togglebutton, AppData * data)
{
  if (data->lock_view)
    data->lock_view = false;
  else
    data->lock_view = true;
}

extern "C" G_MODULE_EXPORT void on_button_ClearUAVTrack_clicked (GtkButton * button, AppData * data)
{
  osm_gps_map_track_remove (data->map, data->uav_track);
  data->uav_track = osm_gps_map_track_new ();
  if (data->draw_path)
    osm_gps_map_track_add (data->map, data->uav_track);
}

extern "C" G_MODULE_EXPORT void on_button_OpenGpsdOptionPopup_clicked (GtkButton * button, AppData * data)
{
  data->gps_opt_popup_state = true;
  gtk_widget_hide_all(data->btn_open_gpsd_option_popup);
  gtk_widget_show_all(data->gpsd_option_popup);
}

extern "C" G_MODULE_EXPORT void on_button_CloseGpsdOptionPopup_clicked (GtkButton * button, AppData * data)
{
  data->gps_opt_popup_state = false;              
  gtk_widget_hide_all(data->gpsd_option_popup);
  gtk_widget_show_all(data->btn_open_gpsd_option_popup);
}

extern "C" G_MODULE_EXPORT void on_combobox_MapProvider_changed (GtkComboBox * box, AppData * data)
{
  int i;
  char *mapcachedir;
  OsmGpsMapPoint current_coord;

  osm_gps_map_convert_screen_to_geographic (data->map, (gint) (data->map_container)->allocation.width / 2,
                                            (gint) (data->map_container)->allocation.height / 2, &current_coord);

  i = gtk_combo_box_get_active (box);

  switch (i)
  {
    case 0:
      data->map_provider = OSM_GPS_MAP_SOURCE_OPENSTREETMAP;
      data->map_zoom_max = 18;
      break;
    case 1:
      data->map_provider = OSM_GPS_MAP_SOURCE_OPENSTREETMAP_RENDERER;
      data->map_zoom_max = 18;
      break;
    case 2:
      data->map_provider = OSM_GPS_MAP_SOURCE_OSM_PUBLIC_TRANSPORT;
      data->map_zoom_max = 18;
      break;
    case 3:
      data->map_provider = OSM_GPS_MAP_SOURCE_OPENCYCLEMAP;
      data->map_zoom_max = 18;
      break;
    case 4:
      data->map_provider = OSM_GPS_MAP_SOURCE_MAPS_FOR_FREE;
      data->map_zoom_max = 11;
      break;
    case 5:
      data->map_provider = OSM_GPS_MAP_SOURCE_GOOGLE_STREET;
      data->map_zoom_max = 17;
      break;
    case 6:
      data->map_provider = OSM_GPS_MAP_SOURCE_GOOGLE_SATELLITE;
      data->map_zoom_max = 18;
      break;
    case 7:
      data->map_provider = OSM_GPS_MAP_SOURCE_GOOGLE_HYBRID;
      data->map_zoom_max = 18;
      break;
    case 8:
      data->map_provider = OSM_GPS_MAP_SOURCE_VIRTUAL_EARTH_STREET;
      data->map_zoom_max = 17;
      break;
    case 9:
      data->map_provider = OSM_GPS_MAP_SOURCE_VIRTUAL_EARTH_SATELLITE;
      data->map_zoom_max = 17;
      break;
    case 10:
      data->map_provider = OSM_GPS_MAP_SOURCE_VIRTUAL_EARTH_HYBRID;
      data->map_zoom_max = 17;
      break;
    default:
      data->map_provider = OSM_GPS_MAP_SOURCE_OPENSTREETMAP;
      data->map_zoom_max = 17;
  }

  osm_gps_map_track_remove_all (data->map);
  osm_gps_map_layer_remove_all (data->map);

  // Change name of cache directory 
  data->friendly_name = osm_gps_map_source_get_friendly_name (data->map_provider);
  mapcachedir = osm_gps_map_get_default_cache_directory ();
  data->cachedir = g_build_filename (mapcachedir, data->friendly_name, NULL);

  // Change map source and update box & window 
  data->map = (OsmGpsMap *) g_object_new (OSM_TYPE_GPS_MAP,
                                          "map-source", data->map_provider,
                                          "tile-cache", data->cachedir, "proxy-uri", g_getenv ("http_proxy"), NULL);
  gtk_container_remove (GTK_CONTAINER (data->map_container), data->map_box);
  data->map_box = gtk_hbox_new (TRUE, 0);
  gtk_box_pack_start (GTK_BOX (data->map_container), data->map_box, TRUE, TRUE, 0);
  gtk_box_pack_start (GTK_BOX (data->map_box), GTK_WIDGET (data->map), TRUE, TRUE, 0);

  // Set map coordinates & zoom to the current ones
  osm_gps_map_set_center_and_zoom (data->map, (current_coord.rlat / M_PI * 180.0), (current_coord.rlon / M_PI * 180.0),
                                   data->map_current_zoom);

  osm_gps_map_layer_add (OSM_GPS_MAP (data->map), OSM_GPS_MAP_LAYER (data->osd));

  if (data->draw_path)
    osm_gps_map_track_add (data->map, data->uav_track);

  gtk_widget_show_all (data->window);
  gtk_widget_hide(data->record_icon);
  gtk_widget_hide(data->status_ok_icon_motor);
  gtk_widget_hide(data->status_ok_icon_flying);
  gtk_widget_hide(data->status_ok_icon_gps);
  gtk_widget_hide_all(data->telemetry_option_popup);
  gtk_widget_hide_all(data->btn_open_gpsd_option_popup);
}

// #####################################################################
// #####################################################################
// **** Tab 3: ROSBag Record Callbacks

extern "C" G_MODULE_EXPORT void on_treeview2_topics_row_activated (GtkTreeView * test, GtkTreePath * path,
                                                                   GtkTreeViewColumn * column, AppData * data)
{
  GValue topic_name = { 0, }, topic_selected =  { 0, };
  GtkTreeIter iter;
  gboolean valid;

  // **** change the state of topic corresponding to the row
  gtk_tree_model_get_iter (GTK_TREE_MODEL (data->topicsList), &iter, path);
  gtk_tree_model_get_value (GTK_TREE_MODEL (data->topicsList), &iter, 1, &topic_selected);
  if (g_value_get_boolean (&topic_selected))
    gtk_list_store_set (data->topicsList, &iter, 1, FALSE, -1);
  else
    gtk_list_store_set (data->topicsList, &iter, 1, TRUE, -1);
  g_value_unset (&topic_selected);

  // **** generates list of topic to be recorded
  data->list_topic[0] = '\0';
  valid = gtk_tree_model_get_iter_first (GTK_TREE_MODEL (data->topicsList), &iter);
  while (valid)
  {
    gtk_tree_model_get_value (GTK_TREE_MODEL (data->topicsList), &iter, 0, &topic_name);
    gtk_tree_model_get_value (GTK_TREE_MODEL (data->topicsList), &iter, 1, &topic_selected);

    if (g_value_get_boolean (&topic_selected))
      if (strcmp (g_value_get_string (&topic_name), "") == 0)
        sprintf (data->list_topic, "%s", g_value_get_string (&topic_name));
      else
        sprintf (data->list_topic, "%s%s", data->list_topic, g_value_get_string (&topic_name));

    valid = gtk_tree_model_iter_next (GTK_TREE_MODEL (data->topicsList), &iter);
    g_value_unset (&topic_name);
    g_value_unset (&topic_selected);
  }

  data->file_prefix = (char *) gtk_entry_get_text (GTK_ENTRY (data->prefix_entry));
  if (strcmp (data->file_prefix, "") != 0)
  {
    sprintf (data->cmd_line, "%s -o %s %s", data->rosbag_record_cmd, data->file_prefix, data->list_topic);
    gtk_entry_set_text (GTK_ENTRY (data->cmd_line_entry), data->cmd_line);
  }
  else
  {
    sprintf (data->cmd_line, "%s %s", data->rosbag_record_cmd, data->list_topic);
    gtk_entry_set_text (GTK_ENTRY (data->cmd_line_entry), data->cmd_line);
  }
}

extern "C" G_MODULE_EXPORT void on_button_UpdateTopicList_clicked (GtkButton * button, AppData * data)
{
  GtkTreeIter iter;
  GValue topic_name = { 0, }, topic_selected = { 0, };
  GValue topic_name_present = { 0 };
  gboolean valid, add_to_list = TRUE;
  char topics_list[255];
  char *pch;

  FILE *topics = popen ("rostopic list", "r");

  while (fgets (topics_list, sizeof (topics_list), topics))
  {
    pch = strchr (topics_list, '\n');
    topics_list[pch - topics_list] = ' ';

    g_value_init (&topic_name, G_TYPE_STRING);
    g_value_init (&topic_selected, G_TYPE_BOOLEAN);
    g_value_set_string (&topic_name, topics_list);
    g_value_set_boolean (&topic_selected, FALSE);

    valid = gtk_tree_model_get_iter_first (GTK_TREE_MODEL (data->topicsList), &iter);
    while (valid)
    {
      gtk_tree_model_get_value (GTK_TREE_MODEL (data->topicsList), &iter, 0, &topic_name_present);
      add_to_list = TRUE;
      if (strcmp (g_value_get_string (&topic_name), g_value_get_string (&topic_name_present)) == 0)
      {
        add_to_list = FALSE;
        g_value_unset (&topic_name_present);
        break;
      }
      valid = gtk_tree_model_iter_next (GTK_TREE_MODEL (data->topicsList), &iter);
      g_value_unset (&topic_name_present);
    }

    if (add_to_list)
    {
      gtk_list_store_append (GTK_LIST_STORE (data->topicsList), &iter);
      gtk_list_store_set_value (GTK_LIST_STORE (data->topicsList), &iter, 0, &topic_name);
      gtk_list_store_set_value (GTK_LIST_STORE (data->topicsList), &iter, 1, &topic_selected);
    }
    g_value_unset (&topic_name);
    g_value_unset (&topic_selected);
  }
  pclose (topics);
}

extern "C" G_MODULE_EXPORT void on_entry_Prefix_activate (GtkEntry * entry, AppData * data)
{
  data->file_prefix = (char *) gtk_entry_get_text (entry);
  if (strcmp (data->file_prefix, "") != 0)
  {
    sprintf (data->cmd_line, "%s -o %s %s", data->rosbag_record_cmd, data->file_prefix, data->list_topic);
    gtk_entry_set_text (GTK_ENTRY (data->cmd_line_entry), data->cmd_line);
  }
  else
  {
    sprintf (data->cmd_line, "%s %s", data->rosbag_record_cmd, data->list_topic);
    gtk_entry_set_text (GTK_ENTRY (data->cmd_line_entry), data->cmd_line);
  }
}

void stopRec(AppData * data)
{
 if(data->recording)
  {
		int err = kill (data->rosbag_pid, SIGINT);
		ROS_DEBUG("Kill exit code: %d\n", err);
		if(!err)
		{
				data->recording=false;
				
				// **** update GUI
				gtk_button_set_image (GTK_BUTTON (data->record_stop_btn),
									gtk_image_new_from_pixbuf (gdk_pixbuf_scale_simple
																		(data->record_icon_64, 40, 40, GDK_INTERP_HYPER)));
				gtk_widget_show(data->record_g_icon);
				gtk_widget_hide(data->record_icon);
				// **** Add metadata to the bag
				
				// **** mv the bag to directory if available
		}
  }
  else
  {
	  ROS_WARN("Not recording");
  }
}

void startRec(AppData * data, bool start_w_key)
{
	if (strcmp (data->list_topic, "") == 0)
	{
		GtkWidget * dialog = gtk_message_dialog_new (GTK_WINDOW(data->window),
                                 GTK_DIALOG_DESTROY_WITH_PARENT,
                                 GTK_MESSAGE_INFO,
                                 GTK_BUTTONS_OK,
                                 "No topic selected for recording!");
      g_signal_connect_swapped (dialog, "response",
                          G_CALLBACK (gtk_widget_destroy),
                          dialog);
		gtk_dialog_run (GTK_DIALOG (dialog));
	}
	else
	{
		if(!data->recording) 
	   {
			int nb_arg=0,i=1;
			char * pch, copy_list_topic[FILENAME_MAX];
			char *argv[50];
			
			data->recording=true;
		
			// **** update GUI
			gtk_button_set_image (GTK_BUTTON (data->record_stop_btn),
                      gtk_image_new_from_pixbuf (gdk_pixbuf_scale_simple
                                                (data->stop_icon_64, 40, 40, GDK_INTERP_HYPER)));

			gtk_widget_hide(data->record_g_icon);
			gtk_widget_show(data->record_icon);
                                                     
			// **** topics
			strcpy(copy_list_topic,data->list_topic);
			pch = strtok (copy_list_topic," ");
			while (pch != NULL)
			{
				nb_arg++;
				pch = strtok (NULL, " ");
			}
			
			// **** fill args of rosbag cmd
			argv[0]=data->rosbag_rec_path;
			
			// **** check & add prefix if available
			if (strcmp (data->file_prefix, "") == 0)
			{
				nb_arg++;
				argv[nb_arg]=NULL;
			}
			else
			{
				nb_arg+=3;i+=2;
				argv[1]="-o";
				argv[2]=data->file_prefix;
				argv[nb_arg]=NULL;
			}
			
			strcpy(copy_list_topic,data->list_topic);
			pch = strtok (copy_list_topic," ");
			while (pch != NULL)
			{
				argv[i]=pch;
				i++;
				pch = strtok (NULL, " ");
			}
				
			// ****************************
			// **** creation child process
			data->rosbag_pid = fork ();
			if (data->rosbag_pid == -1)
			{
				// **** ouch, fork() failed
				perror ("fork");
				exit (-1);
			}
			else if (data->rosbag_pid == 0)
			{
				// **** child
				ROS_DEBUG("Child: pid = %d\n", (int) getpid ());
				execv(data->rosbag_rec_path, argv); 
				
				// **** exit child. note the use of _exit() instead of exit()
				_exit (-1);
			}
			else
			{
				// **** parent
				ROS_DEBUG("Parent: forked a child with pid = %d\n", (int) data->rosbag_pid);
			}
		}
		else if(!start_w_key)
		{
			stopRec(data);
		}
		else
		{
			ROS_WARN("Already recording!");
		}
	}
}

extern "C" G_MODULE_EXPORT void on_button_RecordStop_clicked (GtkButton * button, AppData * data)
{
	startRec(data,false);
}

extern "C" G_MODULE_EXPORT void on_button_Stop_clicked (GtkButton * button, AppData * data)
{
 
  
  //~ GtkTextBuffer * buffer;
  //~ GtkTextIter start, end;
  //~ 
  //~ buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(data->info_textview));
  //~ gtk_text_buffer_get_bounds (buffer, &start, &end);
  //~ gchar * bag_info = gtk_text_buffer_get_text(buffer,&start,&end,FALSE);
//~ 
  //~ rosbag::Bag bag;
  //~ bag.open("/home/gaitt/Work_CCNY/pkgs/ccny-ros-pkg/ccny_ground_station/ground_station/test.bag", rosbag::bagmode::Append);
//~ 
  //~ std_msgs::String str;
  //~ str.data = bag_info;
//~ 
  //~ bag.write("metadata", ros::Time::now(), str);
  //~ bag.close();

  //~ rosbag::Bag bag;
  //~ bag.open("/home/gaitt/Work_CCNY/pkgs/ccny-ros-pkg/ccny_vision/ar_pose/demo/test.bag", rosbag::bagmode::Append);
//~ 
}
