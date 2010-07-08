/*
*  Gtk Bar Gauge Widget
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
 * @file gtkbargauge.h
 * @brief Gtk+ based Bar Gauge Widget
 * @author Gautier Dumonteil <gautier.dumonteil@gmail.com>
 * @version 0.1
 * @date 06/06/2010
 *
 * Gtk Bar Gauge Widget <br>
 * Copyright (C) 2010, CCNY Robotics Lab <br>
 * http://robotics.ccny.cuny.edu <br>
 * 
 * \b Example: Add Bar Gauge widget to an gtkvbox and set some params <br>
 * @code
 * window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
 * vbox = gtk_vbox_new(TRUE, 1);
 * gtk_container_add(GTK_CONTAINER (window), vbox);
 * 
 * bg = gtk_bar_gauge_new ();
 * g_object_set (GTK_BAR_GAUGE (bg), "name-bar-1",
 *					"<big>Bat</big>\n" "<span foreground=\"orange\"><i>(V)</i></span>", NULL);
 * g_object_set (GTK_BAR_GAUGE (bg), "name-bar-2",
 * 				"<big>Aux</big>\n" "<span foreground=\"orange\"><i>(?)</i></span>", NULL);     
 * g_object_set (GTK_BAR_GAUGE (bg), "name-bar-3",
 *             "<big>Aux2</big>\n" "<span foreground=\"orange\"><i>(?)</i></span>", NULL);  
 * g_object_set (GTK_BAR_GAUGE (bg),
 * 				"bar-number", 3,
 * 				"inverse-color", false,
 *					"radial-color", true,
 * 				"start-value-bar-1", 0, 
 * 				"end-value-bar-1", 12, 
 *					"green-strip-start-1", 10,
 * 				"yellow-strip-start-1", 8,
 * 				"start-value-bar-2", 0, 
 *					"end-value-bar-2", 100,
 * 				"start-value-bar-3", 0, 
 * 				"end-value-bar-3", 100, NULL);
 * 
 * gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(bg), TRUE, TRUE, 0);
 * gtk_widget_show_all(window);
 * @endcode
 */

#ifndef __GTK_BAR_GAUGE_H__
#define __GTK_BAR_GAUGE_H__

#include <gtk/gtk.h>
#include <glib-object.h>
#include <math.h>
#include <time.h>

#define GTK_BAR_GAUGE_MAX_STRING  256       /* Size of a text string */
#define GTK_BAR_GAUGE_MODEL_X 300
#define GTK_BAR_GAUGE_MODEL_Y 300

G_BEGIN_DECLS 


/**
 * @typedef struct GtkBarGaugeClass 
 * @brief Special Gtk API strucure.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkBarGaugeClass
{
  GtkDrawingAreaClass parent_class;

} GtkBarGaugeClass;

/**
 * @typedef struct GtkBarGauge 
 * @brief Special Gtk API strucure.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkBarGauge
{
  GtkDrawingArea parent;

} GtkBarGauge;

#define GTK_BAR_GAUGE_TYPE			(gtk_bar_gauge_get_type ())
#define GTK_BAR_GAUGE(obj)			(G_TYPE_CHECK_INSTANCE_CAST ((obj), GTK_BAR_GAUGE_TYPE, GtkBarGauge))
#define GTK_BAR_GAUGE_CLASS(obj)	(G_TYPE_CHECK_CLASS_CAST ((obj), GTK_BAR_GAUGE, GtkBarGaugeClass))
#define IS_GTK_BAR_GAUGE(obj)		(G_TYPE_CHECK_INSTANCE_TYPE ((obj), GTK_BAR_GAUGE_TYPE))
#define IS_GTK_BAR_GAUGE_CLASS(obj)	(G_TYPE_CHECK_CLASS_TYPE ((obj), GTK_BAR_GAUGE_TYPE))
#define GTK_BAR_GAUGE_GET_CLASS	(G_TYPE_INSTANCE_GET_CLASS ((obj), GTK_BAR_GAUGE_TYPE, GtkBarGaugeClass))

extern GType gtk_bar_gauge_get_type (void) G_GNUC_CONST;
extern GtkWidget *gtk_bar_gauge_new (void);
extern void gtk_bar_gauge_set_value (GtkBarGauge * bg, gint bar_index, gdouble val);
extern void gtk_bar_gauge_redraw (GtkBarGauge * bg);

G_END_DECLS
#endif
