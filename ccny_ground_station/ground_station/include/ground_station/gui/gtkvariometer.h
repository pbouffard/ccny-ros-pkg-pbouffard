/*
*  Gtk Variometer Widget
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
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file gtkvariometer.h
 * @brief Gtk+ based Variometer Widget
 * @author Gautier Dumonteil <gautier.dumonteil@gmail.com>
 * @version 0.1
 * @date 06/06/2010
 *
 * Gtk Variometer Widget <br>
 * Copyright (C) 2010, CCNY Robotics Lab <br>
 * http://robotics.ccny.cuny.edu <br>
 * 
 * \b Example: Add Variometer widget to an gtkvbox and set some params <br>
 * @code
 * window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
 * vbox = gtk_vbox_new(TRUE, 1);
 * gtk_container_add(GTK_CONTAINER (window), vbox);
 * 
 * vario = gtk_variometer_new();
 * g_object_set(GTK_VARIOMETER (vario),
 *					"inverse-color", false,
 *					"unit-is-feet", true,
 *					"unit-step-value", 1000,
 *					"radial-color", true, NULL);
 * 
 * gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(vario), TRUE, TRUE, 0);
 * gtk_widget_show_all(window);
 * @endcode
 */

#ifndef __GTK_VARIOMETER_H__
#define __GTK_VARIOMETER_H__

#include <gtk/gtk.h>
#include <glib-object.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#define GTK_VARIOMETER_MAX_STRING  256   /* Size of a text string */
#define GTK_VARIOMETER_MODEL_X 300
#define GTK_VARIOMETER_MODEL_Y 300

G_BEGIN_DECLS 

/**
 * @typedef struct GtkVariometerClass 
 * @brief Special Gtk API strucure.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkVariometerClass
{
  GtkDrawingAreaClass parent_class;

} GtkVariometerClass;

/**
 * @typedef struct GtkVariometer 
 * @brief Special Gtk API strucure.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkVariometer
{
  GtkDrawingArea parent;
  
} GtkVariometer;

#define GTK_VARIOMETER_TYPE			(gtk_variometer_get_type ())
#define GTK_VARIOMETER(obj)			(G_TYPE_CHECK_INSTANCE_CAST ((obj), GTK_VARIOMETER_TYPE, GtkVariometer))
#define GTK_VARIOMETER_CLASS(obj)	(G_TYPE_CHECK_CLASS_CAST ((obj), GTK_VARIOMETER, GtkVariometerClass))
#define IS_GTK_VARIOMETER(obj)		(G_TYPE_CHECK_INSTANCE_TYPE ((obj), GTK_VARIOMETER_TYPE))
#define IS_GTK_VARIOMETER_CLASS(obj)	(G_TYPE_CHECK_CLASS_TYPE ((obj), GTK_VARIOMETER_TYPE))
#define GTK_VARIOMETER_GET_CLASS	(G_TYPE_INSTANCE_GET_CLASS ((obj), GTK_VARIOMETER_TYPE, GtkVariometerClass))

extern GType gtk_variometer_get_type (void) G_GNUC_CONST;
extern GtkWidget *gtk_variometer_new (void);
extern void gtk_variometer_set_value (GtkVariometer * vario, gdouble val);
extern void gtk_variometer_redraw (GtkVariometer * vario);

G_END_DECLS
#endif
