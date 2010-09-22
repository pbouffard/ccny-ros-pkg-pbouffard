/*
*  Gtk Altimeter Widget
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
 * @file gtkaltimeter.h 
 * @brief Gtk+ based Altimeter Widget
 * @author Gautier Dumonteil <gautier.dumonteil@gmail.com>
 * @version 0.2
 * @date 02/09/2010
 *
 * Gtk Altimeter Widget <br>
 * Copyright (C) 2010, CCNY Robotics Lab <br>
 * http://robotics.ccny.cuny.edu <br>
 * 
 * This widget provide an easy to read altimeter instrument. <br>
 * The design is based on a real altimeter flight instrument <br>
 * in order to be familiar to aircraft and helicopter pilots.<br>
 * 
 * @b Pictures:<br>
 * <table><tr>
 * <th><IMG SRC="http://www.ros.org/wiki/ground_station?action=AttachFile&do=get&target=gtkaltimeter.png"></th>
 * <th><IMG SRC="http://www.ros.org/wiki/ground_station?action=AttachFile&do=get&target=gtkaltimeter_g.png"></th>
 * </tr></table>
 * 
 * @b Example: <br>
 * Add Altimeter widget to an gtkvbox and set some params <br>
 * @code
 * window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
 * vbox = gtk_vbox_new(TRUE, 1);
 * gtk_container_add(GTK_CONTAINER (window), vbox);
 * 
 * alt = gtk_altimeter_new();
 * g_object_set(GTK_ALTIMETER (alt),
 *		"grayscale-colors", false,
 *		"unit-is-feet", true,
 *		"unit-step-value", 100,
 *		"radial-color", true, NULL);
 * 
 * gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(alt), TRUE, TRUE, 0);
 * gtk_widget_show_all(window);
 * @endcode
 * 
 * The following code show how to change widget's values and redraw it:<br>
 * Note that here tc's type is "GtkWidget *".<br>
 * @code
 * if (IS_GTK_ALTIMETER (alt))
 * {
 *	gtk_altimeter_set_alti (GTK_ALTIMETER (alt), altitude);
 *	gtk_altimeter_redraw(GTK_ALTIMETER(alt));
 * }		
 * @endcode
 * 
  @b Widget @b Parameters:<br>
 * - "grayscale-colors": boolean, if TRUE, draw the widget with grayscale colors (outdoor use)<br>
 * - "unit-is-feet": boolean, if TRUE, the widget display FEET values else display METER<br>
 * - "unit-step-value", int, define the step value of the altimeter can be 1,10,100<br>
 * - "radial-color": boolean, if TRUE, draw a fake light reflexion<br>
 * 
 * @b Widget @b values:<br>
 * - "altitude": double, define the altitude you want to display by the widget - the value is<br>
 * from 0 to 999999.
 */

#ifndef __GTK_ALTIMETER_H__
#define __GTK_ALTIMETER_H__

#include <gtk/gtk.h>
#include <glib-object.h>
#include <math.h>

#define GTK_ALTIMETER_MAX_STRING  256   /* Size of a text string */

G_BEGIN_DECLS
/**
 * @typedef struct GtkAltimeterClass 
 * @brief Special Gtk API strucure. Define an instance the widget class
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
  typedef struct _GtkAltimeterClass
{
  GtkDrawingAreaClass parent_class;

} GtkAltimeterClass;

/**
 * @typedef struct GtkAltimeter 
 * @brief Special Gtk API strucure. Define widget's class
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkAltimeter
{
  GtkDrawingArea parent;

} GtkAltimeter;

#define GTK_ALTIMETER_TYPE			(gtk_altimeter_get_type ())
#define GTK_ALTIMETER(obj)			(G_TYPE_CHECK_INSTANCE_CAST ((obj), GTK_ALTIMETER_TYPE, GtkAltimeter))
#define GTK_ALTIMETER_CLASS(obj)	(G_TYPE_CHECK_CLASS_CAST ((obj), GTK_ALTIMETER, GtkAltimeterClass))
#define IS_GTK_ALTIMETER(obj)		(G_TYPE_CHECK_INSTANCE_TYPE ((obj), GTK_ALTIMETER_TYPE))
#define IS_GTK_ALTIMETER_CLASS(obj)	(G_TYPE_CHECK_CLASS_TYPE ((obj), GTK_ALTIMETER_TYPE))
#define GTK_ALTIMETER_GET_CLASS	(G_TYPE_INSTANCE_GET_CLASS ((obj), GTK_ALTIMETER_TYPE, GtkAltimeterClass))

extern GType gtk_altimeter_get_type (void) G_GNUC_CONST;
extern GtkWidget *gtk_altimeter_new (void);
extern void gtk_altimeter_set_alti (GtkAltimeter * alt, gdouble alti);
extern void gtk_altimeter_redraw (GtkAltimeter * alt);

G_END_DECLS
#endif
