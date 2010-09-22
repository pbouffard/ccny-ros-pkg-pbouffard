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
 * @version 0.2
 * @date 02/09/2010
 *
 * Gtk Bar Gauge Widget
 * Copyright (C) 2010, CCNY Robotics Lab <br>
 * http://robotics.ccny.cuny.edu <br>
 * 
 * This widget provide an easy to read bar gauge instrument. <br>
 * This widget is fully comfigurable and useable for several<br>
 * gauge type (Battery Voltage, Current, Velocity, ...).
 * 
 * @b Pictures:<br>
 * <table><tr>
 * <th><IMG SRC="http://www.ros.org/wiki/ground_station?action=AttachFile&do=get&target=gtkbargauge.png"></th>
 * <th><IMG SRC="http://www.ros.org/wiki/ground_station?action=AttachFile&do=get&target=gtkbargauge_g.png"></th>
 * </tr></table>
 * 
 * \b Example:<br>
 * Add Bar Gauge widget to an gtkvbox and set some params <br>
 * @code
 * window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
 * vbox = gtk_vbox_new(TRUE, 1);
 * gtk_container_add(GTK_CONTAINER (window), vbox);
 * 
 * bg = gtk_bar_gauge_new ();
 * g_object_set (GTK_BAR_GAUGE (bg), "name-bar-1",
 *		"<big>Bat</big>\n" "<span foreground=\"orange\"><i>(V)</i></span>", NULL);
 * g_object_set (GTK_BAR_GAUGE (bg), "name-bar-2",
 *		"<big>Aux</big>\n" "<span foreground=\"orange\"><i>(?)</i></span>", NULL);     
 * g_object_set (GTK_BAR_GAUGE (bg), "name-bar-3",
 *		"<big>Aux2</big>\n" "<span foreground=\"orange\"><i>(?)</i></span>", NULL);  
 * g_object_set (GTK_BAR_GAUGE (bg),
 *		"bar-number", 3,
 *		"grayscale-color", false,
 *		"radial-color", true,
 *		"start-value-bar-1", 0, 
 *		"end-value-bar-1", 12, 
 *		"green-strip-start-1", 10,
 *		"yellow-strip-start-1", 8,
 *		"start-value-bar-2", 0, 
 *		"end-value-bar-2", 100,
 *		"start-value-bar-3", 0, 
 *		"end-value-bar-3", 100, NULL);
 * 
 * gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(bg), TRUE, TRUE, 0);
 * gtk_widget_show_all(window);
 * @endcode
 * 
 * The following code show how to change widget's values and redraw it:<br>
 * Note that here tc's type is "GtkWidget *".<br>
 * @code
 * if (IS_GTK_BAR_GAUGE (bg))
 * {
 *	gtk_bar_gauge_set_value (GTK_BAR_GAUGE(bg), index, val);
 *	gtk_turn_coordinator_redraw(GTK_BAR_GAUGE(bg));
 * }		
 * @endcode
 *
 * @b Widget @b Parameters:<br>
 * - "grayscale-color": boolean, if TRUE, draw the widget with grayscale colors (outdoor use)<br>
 * - "radial-color": boolean, if TRUE, draw a fake light reflexion<br>
 * - "widget-name": char, indicates the widget name to be displayed<br>
 * - "bar-number", int, indicates the number of bar gauge to display (1 to 4)<br>
 * - "start-value-bar-X": int, indicates the start value of bar gauge X<br>
 * - "end-value-bar-X": int, indicates the end value of the bar gauge X<br>
 * - "name-bar-X": char, provide the name of the bar gauge X (support<br>
 * pango text attribute)<br>
 * - OPTIONAL - "green-strip-start-X": int, indicates the start of the green <br>
 * for the bar gauge X<br>
 * - OPTIONAL - "yellow-strip-start-X": int, indicates the start of the yellow<br>
 * for the bar gauge X<br>
 * 
 * @b Widget @b values:<br>
 * - "index": the value of the bar you want to update<br>
 * - "val": double, provide the bar drawing according to the start-value<br>
 * and the end-value.
 * 
 */

#ifndef __GTK_BAR_GAUGE_H__
#define __GTK_BAR_GAUGE_H__

#include <gtk/gtk.h>
#include <glib-object.h>
#include <math.h>

#define GTK_BAR_GAUGE_MAX_STRING  256       /* Size of a text string */

G_BEGIN_DECLS 

/**
 * @typedef struct GtkBarGaugeClass 
 * @brief Special Gtk API strucure. Define an instance the widget class
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
 * @brief Special Gtk API strucure. Define widget's class
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
