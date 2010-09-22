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
 * @version 0.2
 * @date 02/09/2010
 *
 * Gtk Variometer Widget <br>
 * Copyright (C) 2010, CCNY Robotics Lab <br>
 * http://robotics.ccny.cuny.edu <br>
 * 
 * This widget provide an easy to read variometer instrument. <br>
 * The design is volontary based on a real variometer flight instrument <br>
 * in order to be familiar to aircraft and helicopter pilots.<br>
 * 
 * @b Pictures:<br>
 * <table><tr>
 * <th><IMG SRC="http://www.ros.org/wiki/ground_station?action=AttachFile&do=get&target=gtkvariometer.png"></th>
 * <th><IMG SRC="http://www.ros.org/wiki/ground_station?action=AttachFile&do=get&target=gtkvariometer_g.png"></th>
 * </tr></table>
 * 
 * @b Example: <br>
 * Add variometer widget to an gtkvbox and set some params <br>
 * @code
 * window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
 * vbox = gtk_vbox_new(TRUE, 1);
 * gtk_container_add(GTK_CONTAINER (window), vbox);
 * 
 * vario = gtk_variometer_new();
 * g_object_set(GTK_VARIOMETER (vario),
 *		"grayscale-color", false,
 *		"unit-is-feet", true,
 *		"unit-step-value", 1000,
 *		"radial-color", true, NULL);
 * 
 * gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(vario), TRUE, TRUE, 0);
 * gtk_widget_show_all(window);
 * @endcode
 * 
 * The following code show how to change widget's values and redraw it:<br>
 * Note that here tc's type is "GtkWidget *".<br>
 * @code
 * if (IS_GTK_VARIOMETER (vario))
 * {
 *	gtk_variometer_set_alti (GTK_VARIOMETER (vario),altitude);
 *	gtk_variometer_redraw(GTK_VARIOMETER(vario));
 * }		
 * @endcode
 * 
  @b Widget @b Parameters:<br>
 * - "grayscale-colors": boolean, if TRUE, draw the widget with grayscale colors (outdoor use)<br>
 * - "radial-color": boolean, if TRUE, draw a fake light reflexion<br>
 * - "unit-is-feet": boolean, if TRUE, the widget display FEET values else display METER<br>
 * - "unit-step-value", int, define the step value of the altimeter can be 100,1000<br>
 * 
 * @b Widget @b values:<br>
 * - "altitude": double, define the altitude you want to display by the widget - the value is<br>
 * from 0 to 999999.
 * Note that this value need to be the same as the altimeter widget.
 */

#ifndef __GTK_VARIOMETER_H__
#define __GTK_VARIOMETER_H__

#include <gtk/gtk.h>
#include <glib-object.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#define GTK_VARIOMETER_MAX_STRING  256   /* Size of a text string */

G_BEGIN_DECLS 

/**
 * @typedef struct GtkVariometerClass 
 * @brief Special Gtk API strucure. Define an instance the widget class
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
 * @brief Special Gtk API strucure. Define widget's class
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
extern void gtk_variometer_set_value (GtkVariometer * vario, gdouble dheight);
extern void gtk_variometer_redraw (GtkVariometer * vario);

G_END_DECLS
#endif
