/*
 *  CityFlyer ground_station
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

#ifndef _GROUND_STATION_CALLBACKS_H
#define _GROUND_STATION_CALLBACKS_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
#include <glade/glade.h>
#include <math.h>

#include "rosbag/recorder.h"
#include "rosbag/exceptions.h"
#include "std_msgs/String.h"

#include "ground_station/gui/ground_station_appdata.h"

using namespace std;

// #####################################################################
// #####################################################################
// **** Main Window Callbacks
extern "C" G_MODULE_EXPORT void on_mainwindow_destroy (GtkObject * object, AppData * data);
extern "C" G_MODULE_EXPORT gboolean on_mainwindow_key_press_event (GtkWidget * widget, GdkEventKey * event,
                                                                   AppData * data);
extern "C" G_MODULE_EXPORT void on_notebook1_switch_page (GtkNotebook * notebook, GtkNotebookPage * page,
                                                          guint page_num, AppData * data);

// #####################################################################
// #####################################################################
// **** Tab 1: Telemetry Callbacks
extern "C" G_MODULE_EXPORT void on_colorbutton_BgColor_color_set (GtkColorButton * button, AppData * data);
extern "C" G_MODULE_EXPORT void on_button_OpenTelemetryOptionPopup_clicked (GtkButton * button, AppData * data);
extern "C" G_MODULE_EXPORT void on_button_CloseTelemetryOptionPopup_clicked (GtkButton * button, AppData * data);

// #####################################################################
// #####################################################################
// **** Tab 2: GpsdViewer Callbacks
extern "C" G_MODULE_EXPORT void on_checkbuttonDrawCurrentUAVTrack_toggled (GtkToggleButton * togglebutton,
                                                                           AppData * data);
extern "C" G_MODULE_EXPORT void on_checkbuttonLockViewUAV_toggled (GtkToggleButton * togglebutton, AppData * data);
extern "C" G_MODULE_EXPORT void on_button_ClearUAVTrack_clicked (GtkButton * button, AppData * data);
extern "C" G_MODULE_EXPORT void on_button_OpenGpsdOptionPopup_clicked (GtkButton * button, AppData * data);
extern "C" G_MODULE_EXPORT void on_button_CloseGpsdOptionPopup_clicked (GtkButton * button, AppData * data);
extern "C" G_MODULE_EXPORT void on_combobox_MapProvider_changed (GtkComboBox * box, AppData * data);

// #####################################################################
// #####################################################################
// **** Tab 3: ROSBag Record Callbacks
extern "C" G_MODULE_EXPORT void on_treeview2_topics_row_activated (GtkTreeView * test, GtkTreePath * path,
                                                                   GtkTreeViewColumn * column, AppData * data);
extern "C" G_MODULE_EXPORT void on_button_UpdateTopicList_clicked (GtkButton * button, AppData * data);
extern "C" G_MODULE_EXPORT void on_entry_Prefix_activate (GtkEntry * entry, AppData * data);
extern "C" G_MODULE_EXPORT void on_button_RecordPause_clicked (GtkButton * button, AppData * data);
extern "C" G_MODULE_EXPORT void on_button_Stop_clicked (GtkButton * button, AppData * data);
void startRec(AppData * data, bool start_w_key);
void stopRec(AppData * data);

#endif
