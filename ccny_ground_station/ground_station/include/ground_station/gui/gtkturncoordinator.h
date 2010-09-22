/*
*  Gtk Turn Coordinator Widget
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
 * @file gtkturncoordinator.h 
 * @brief Gtk+ based Turn Coordinator Widget
 * @author Gautier Dumonteil <gautier.dumonteil@gmail.com>
 * @version 0.2
 * @date 02/09/2010
 *
 * Gtk Turn Coordinator Widget <br>
 * Copyright (C) 2010, CCNY Robotics Lab <br>
 * http://robotics.ccny.cuny.edu <br>
 * 
 * This widget provide an easy to read turn coordinator instrument. <br>
 * The design is based on a real turn coordinator flight instrument <br>
 * in order to be familiar to aircraft and helicopter pilots.<br>
 * 
 * @b Pictures:<br>
 * <table><tr>
 * <th><IMG SRC="http://www.ros.org/wiki/ground_station?action=AttachFile&do=get&target=gtkturncoordinator.png"></th>
 * <th><IMG SRC="http://www.ros.org/wiki/ground_station?action=AttachFile&do=get&target=gtkturncoordinator_g.png"></th>
 * </tr></table>
 * 
 * @b Example: <br>
 * Add Turn Coordinator widget to an gtkvbox and set some params <br>
 * @code
 * window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
 * vbox = gtk_vbox_new(TRUE, 1);
 * gtk_container_add(GTK_CONTAINER (window), vbox);
 * 
 * tc = gtk_turn_coordinator_new();
 * g_object_set(GTK_TURN_COORDINATOR (tc),
 *		"grayscale-colors", false,
 *		"radial-color", true, NULL);
 * 
 * gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(tc), TRUE, TRUE, 0);
 * gtk_widget_show_all(window);
 * @endcode
 * 
 * The following code show how to change widget's values and redraw it:<br>
 * Note that here tc's type is "GtkWidget *".<br>
 * @code
 * if (IS_GTK_TURN_COORDINATOR (tc))
 * {
 *	gtk_turn_coordinator_set_value (GTK_TURN_COORDINATOR (tc), plane_angle,ball_tr);
 *	gtk_turn_coordinator_redraw(GTK_TURN_COORDINATOR(tc));
 * }		
 * @endcode
 * 
  @b Widget @b Parameters:<br>
 * - "grayscale-colors": boolean, if TRUE, draw the widget with grayscale colors (outdoor use)<br>
 * - "radial-color": boolean, if TRUE, draw a fake light reflexion<br>
 * 
 * @b Widget @b values:<br>
 * - "plane_angle": double, provide the plane rotation - the value is from 0 to 360.<br>
 * - "ball_tr": double, provide the ball translation - the value is from -100 to 100
 * 
 */

#ifndef __GTK_TURN_COORDINATOR_H__
#define __GTK_TURN_COORDINATOR_H__

#include <gtk/gtk.h>
#include <glib-object.h>
#include <math.h>

#define GTK_TURN_COORDINATOR_MAX_STRING  256    /* Size of a text string */
#define DEG2RAD(DEG) ((DEG)*((M_PI)/(180.0)))

G_BEGIN_DECLS

/**
 * @typedef struct GtkTurnCoordinatorClass 
 * @brief Special Gtk API strucure. Define an instance the widget class
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
  typedef struct _GtkTurnCoordinatorClass
{
  GtkDrawingAreaClass parent_class;

} GtkTurnCoordinatorClass;

/**
 * @typedef struct GtkTurnCoordinator 
 * @brief Special Gtk API strucure. Define widget's class
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkTurnCoordinator
{
  GtkDrawingArea parent;

} GtkTurnCoordinator;

#define GTK_TURN_COORDINATOR_TYPE			(gtk_turn_coordinator_get_type ())
#define GTK_TURN_COORDINATOR(obj)			(G_TYPE_CHECK_INSTANCE_CAST ((obj), GTK_TURN_COORDINATOR_TYPE, GtkTurnCoordinator))
#define GTK_TURN_COORDINATOR_CLASS(obj)	(G_TYPE_CHECK_CLASS_CAST ((obj), GTK_TURN_COORDINATOR, GtkTurnCoordinatorClass))
#define IS_GTK_TURN_COORDINATOR(obj)		(G_TYPE_CHECK_INSTANCE_TYPE ((obj), GTK_TURN_COORDINATOR_TYPE))
#define IS_GTK_TURN_COORDINATOR_CLASS(obj)	(G_TYPE_CHECK_CLASS_TYPE ((obj), GTK_TURN_COORDINATOR_TYPE))
#define GTK_TURN_COORDINATOR_GET_CLASS	(G_TYPE_INSTANCE_GET_CLASS ((obj), GTK_TURN_COORDINATOR_TYPE, GtkTurnCoordinatorClass))

extern GType gtk_turn_coordinator_get_type (void) G_GNUC_CONST;
extern GtkWidget *gtk_turn_coordinator_new (void);
extern void gtk_turn_coordinator_set_value (GtkTurnCoordinator * vario, gdouble, gdouble);
extern void gtk_turn_coordinator_redraw (GtkTurnCoordinator * vario);

G_END_DECLS
#endif
