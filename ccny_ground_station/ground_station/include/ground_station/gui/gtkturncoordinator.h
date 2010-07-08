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
 * @version 0.1
 * @date 06/06/2010
 *
 * Gtk Turn Coordinator Widget <br>
 * Copyright (C) 2010, CCNY Robotics Lab <br>
 * http://robotics.ccny.cuny.edu <br>
 * 
 * \b Example: Add Turn Coordinator widget to an gtkvbox and set some params <br>
 * @code
 * window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
 * vbox = gtk_vbox_new(TRUE, 1);
 * gtk_container_add(GTK_CONTAINER (window), vbox);
 * 
 * tc = gtk_turn_coordinator_new();
 * g_object_set(GTK_TURN_COORDINATOR (tc),
 *					"inverse-color", false,
 *					"unit-is-feet", true,
 *					"unit-step-value", 1000,
 *					"radial-color", true, NULL);
 * 
 * gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(tc), TRUE, TRUE, 0);
 * gtk_widget_show_all(window);
 * @endcode
 */

#ifndef __GTK_TURN_COORDINATOR_H__
#define __GTK_TURN_COORDINATOR_H__

#include <gtk/gtk.h>
#include <glib-object.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#define GTK_TURN_COORDINATOR_MAX_STRING  256   /* Size of a text string */
#define GTK_TURN_COORDINATOR_MODEL_X 300
#define GTK_TURN_COORDINATOR_MODEL_Y 300

G_BEGIN_DECLS 

/**
 * @typedef struct GtkTurnCoordinatorClass 
 * @brief Special Gtk API strucure.
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
 * @brief Special Gtk API strucure.
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
extern void gtk_turn_coordinator_set_value (GtkTurnCoordinator * vario, gdouble val);
extern void gtk_turn_coordinator_redraw (GtkTurnCoordinator * vario);

G_END_DECLS
#endif
