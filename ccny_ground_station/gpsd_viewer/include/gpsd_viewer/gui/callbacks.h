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

#ifndef CALLBACKS_H
#define CALLBACKS_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <gtk/gtk.h>
#include <glade/glade.h>
#include <math.h>

#include "gpsd_viewer/gui/AppData.h"

using namespace std;

#include "gpsd_viewer/gui/callbacks.h"

extern "C" G_MODULE_EXPORT void on_mainwindow_destroy (GtkObject * object, gpointer user_data);
extern "C" G_MODULE_EXPORT void on_checkbuttonDrawCurrentUAVTrack_toggled (GtkToggleButton * togglebutton, AppData * data);
extern "C" G_MODULE_EXPORT void on_menuitemMapProvider_group_changed (GtkRadioMenuItem * radiobutton, AppData * data);
extern "C" G_MODULE_EXPORT void on_menuitem_LockView_toggled (GtkToggleButton * togglebutton, AppData * data);

#endif
