/*
*  Gtk Compass Widget
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
 * @file gtkcompass.h
 * @brief Gtk+ based Compass Widget
 * @author Gautier Dumonteil <gautier.dumonteil@gmail.com>
 * @version 0.2
 * @date 02/09/2010
 *
 * Gtk Compass Widget <br>
 * Copyright (C) 2010, CCNY Robotics Lab <br>
 * http://robotics.ccny.cuny.edu <br>
 * 
 * This widget provide an easy to read compass instrument. <br>
 * The design is based on a real compass flight instrument <br>
 * in order to be familiar to aircraft and helicopter pilots.<br>
 * 
 * @b Pictures:<br>
 * <table><tr>
 * <th><IMG SRC="http://www.ros.org/wiki/ground_station?action=AttachFile&do=get&target=gtkcompass.png"></th>
 * <th><IMG SRC="http://www.ros.org/wiki/ground_station?action=AttachFile&do=get&target=gtkcompass_g.png"></th>
 * </tr></table>
 * 
 * @b Example: <br>
 * Add Compass widget to an gtkvbox and set some params <br>
 * @code
 * window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
 * vbox = gtk_vbox_new(TRUE, 1);
 * gtk_container_add(GTK_CONTAINER (window), vbox);
 * 
 * comp = gtk_compass_new();
 * g_object_set(GTK_COMPASS (comp),
 *		"grayscale-color", false,
 *		"radial-color", true, NULL);
 * 
 * gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(comp), TRUE, TRUE, 0);
 * gtk_widget_show_all(window);
 * @endcode
 * 
 * The following code show how to change widget's values and redraw it:<br>
 * Note that here tc's type is "GtkWidget *".<br>
 * @code
 * if (IS_GTK_COMPASS (comp))
 * {
 *	gtk_compass_set_angle (GTK_COMPASS (comp), angle);
 *	gtk_compass_redraw(GTK_COMPASS(comp));
 * }		
 * @endcode
 * 
  @b Widget @b Parameters:<br>
 * - "grayscale-color": boolean, if TRUE, draw the widget with grayscale colors (outdoor use)<br>
 * - "radial-color": boolean, if TRUE, draw a fake light reflexion<br>
 * 
 * @b Widget @b values:<br>
 * - "angle": double, provide the compass's rotation - the value is from 0 to 360.<br>
 * 
 */

#ifndef __GTK_COMPASS_H
#define __GTK_COMPASS_H

#include <gtk/gtk.h>
#include <glib-object.h>
#include <cairo-svg.h>
#include <math.h>
#include <time.h>

#define GTK_COMPASS_MAX_STRING  256     /* Size of a text string */
#define DEG2RAD(DEG) ((DEG)*((M_PI)/(180.0)))

G_BEGIN_DECLS

/**
 * @typedef struct GtkCompassClass 
 * @brief Special Gtk API strucure. Define an instance the widget class
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkCompassClass
{
  GtkDrawingAreaClass parent_class;

} GtkCompassClass;

/**
 * @typedef struct GtkCompass 
 * @brief Special Gtk API strucure. Define widget's class
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkCompass
{
  GtkDrawingArea parent;

} GtkCompass;

#define GTK_COMPASS_TYPE			(gtk_compass_get_type ())
#define GTK_COMPASS(obj)			(G_TYPE_CHECK_INSTANCE_CAST ((obj), GTK_COMPASS_TYPE, GtkCompass))
#define GTK_COMPASS_CLASS(obj)	(G_TYPE_CHECK_CLASS_CAST ((obj), GTK_COMPASS, GtkCompassClass))
#define IS_GTK_COMPASS(obj)		(G_TYPE_CHECK_INSTANCE_TYPE ((obj), GTK_COMPASS_TYPE))
#define IS_GTK_COMPASS_CLASS(obj)	(G_TYPE_CHECK_CLASS_TYPE ((obj), GTK_COMPASS_TYPE))
#define GTK_COMPASS_GET_CLASS	(G_TYPE_INSTANCE_GET_CLASS ((obj), GTK_COMPASS_TYPE, GtkCompassClass))

extern GType gtk_compass_get_type (void) G_GNUC_CONST;
extern GtkWidget *gtk_compass_new (void);
extern void gtk_compass_redraw (GtkCompass * comp);
extern void gtk_compass_set_angle (GtkCompass * comp, gdouble ang);

G_END_DECLS
#endif
