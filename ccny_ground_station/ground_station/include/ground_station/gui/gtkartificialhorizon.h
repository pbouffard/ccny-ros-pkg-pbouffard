/*
 * Gtk Artificial Horizon Widget
 * Copyright (C) 2010, CCNY Robotics Lab
 * Gautier Dumonteil <gautier.dumonteil@gmail.com>
 * http://robotics.ccny.cuny.edu
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file gtkartificialhorizon.h
 * @brief Gtk+ based Artificial Horizon Widget
 * @author Gautier Dumonteil <gautier.dumonteil@gmail.com>
 * @version 0.1
 * @date 06/06/2010
 *
 * Gtk Artificial Horizon Widget <br>
 * Copyright (C) 2010, CCNY Robotics Lab <br>
 * http://robotics.ccny.cuny.edu <br>
 * 
 * \b Example: Add Artificial Horizon widget to an gtkvbox and set some params <br>
 * @code
 * window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
 * vbox = gtk_vbox_new(TRUE, 1);
 * gtk_container_add(GTK_CONTAINER (window), vbox);
 * 
 * art_hor = gtk_artificial_horizon_new(); 
 * g_object_set(GTK_ARTIFICIAL_HORIZON (art_hor),
 * 				"inverse-color", false,
 * 				"radial-color", true, NULL);
 * 
 * gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(art_hor), TRUE, TRUE, 0);
 * gtk_widget_show_all(window);
 * @endcode
 */

#ifndef __GTK_ARTIFICIAL_HORIZON_H__
#define __GTK_ARTIFICIAL_HORIZON_H__

#include <gtk/gtk.h>
#include <glib-object.h>
#include <math.h>
#include <time.h>

#define GTK_ARTIFICIAL_HORIZON_MAX_STRING  256   // **** Size of a text string 
#define GTK_ARTIFICIAL_HORIZON_MODEL_X 300
#define GTK_ARTIFICIAL_HORIZON_MODEL_Y 300

G_BEGIN_DECLS

/**
 * @typedef struct GtkArtificialHorizonClass 
 * @brief Special Gtk API strucure.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkArtificialHorizonClass
{
  GtkDrawingAreaClass parent_class;

} GtkArtificialHorizonClass;

/**
 * @typedef struct GtkArtificialHorizon 
 * @brief Special Gtk API strucure.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkArtificialHorizon
{
  GtkDrawingArea parent;
  
} GtkArtificialHorizon;

#define GTK_ARTIFICIAL_HORIZON_TYPE			(gtk_artificial_horizon_get_type ())
#define GTK_ARTIFICIAL_HORIZON(obj)			(G_TYPE_CHECK_INSTANCE_CAST ((obj), GTK_ARTIFICIAL_HORIZON_TYPE, GtkArtificialHorizon))
#define GTK_ARTIFICIAL_HORIZON_CLASS(obj)	(G_TYPE_CHECK_CLASS_CAST ((obj), GTK_ARTIFICIAL_HORIZON, GtkArtificialHorizonClass))
#define IS_GTK_ARTIFICIAL_HORIZON(obj)		(G_TYPE_CHECK_INSTANCE_TYPE ((obj), GTK_ARTIFICIAL_HORIZON_TYPE))
#define IS_GTK_ARTIFICIAL_HORIZON_CLASS(obj)	(G_TYPE_CHECK_CLASS_TYPE ((obj), GTK_ARTIFICIAL_HORIZON_TYPE))
#define GTK_ARTIFICIAL_HORIZON_GET_CLASS	(G_TYPE_INSTANCE_GET_CLASS ((obj), GTK_ARTIFICIAL_HORIZON_TYPE, GtkArtificialHorizonClass))

extern GType gtk_artificial_horizon_get_type (void) G_GNUC_CONST;
extern GtkWidget *gtk_artificial_horizon_new (void);
// extern void gtk_artificial_horizon_set_alti (GtkArtificialHorizon * arh, gdouble alti);
extern void gtk_artificial_horizon_redraw (GtkArtificialHorizon * arh);

G_END_DECLS
#endif
