/*
*  Gtk Gauge Widget
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
 * @file gtkgauge.h
 * @brief Gtk+ based Gauge Widget
 * @author Gautier Dumonteil <gautier.dumonteil@gmail.com>
 * @version 0.2
 * @date 02/09/2010
 *
 * Gtk Gauge Widget <br>
 * Copyright (C) 2010, CCNY Robotics Lab <br>
 * http://robotics.ccny.cuny.edu <br>
 * 
 * This widget provide an easy to read gauge instrument. <br>
 * The design is made to look like to a real gauge<br>
 * flight instrument in order to be familiar to aircraft and<br>
 * helicopter pilots. This widget is fully comfigurable and<br>
 * useable for several gauge type (Battery Voltage, Current,<br> 
 * Velocity, ...).
 * 
 * @b Pictures:<br>
 * <table><tr>
 * <th><IMG SRC="http://www.ros.org/wiki/ground_station?action=AttachFile&do=get&target=gtkgauge.png"></th>
 * <th><IMG SRC="http://www.ros.org/wiki/ground_station?action=AttachFile&do=get&target=gtkgauge_g.png"></th>
 * </tr></table>
 * 
 * \b Example:<br>
 * Add Gauge widget to an gtkvbox and set some params <br>
 * @code
 * window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
 * vbox = gtk_vbox_new(TRUE, 1);
 * gtk_container_add(GTK_CONTAINER (window), vbox);
 * 
 * batt = gtk_gauge_new();
 * g_object_set (GTK_GAUGE (batt), "name",
 *		"<big>Battery voltage</big>\n" "<span foreground=\"orange\"><i>(V)</i></span>", NULL);
 * g_object_set (GTK_GAUGE (batt),
 *		"grayscale-color", false,
 *		"radial-color", true,
 *		"start-value", 0, 
 *		"end-value", 12, 
 *		"initial-step", 2, 
 *		"sub-step", (gdouble) 0.2, 
 *		"drawing-step", 2,
 *		"color-strip-order", "RYG",
 *		"green-strip-start", 10,
 *		"yellow-strip-start", 8,
 *		"red-strip-start", 0, NULL);
 * 
 * gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(batt), TRUE, TRUE, 0);
 * gtk_widget_show_all(window);
 * @endcode
 * 
 * The following code show how to change widget's values and redraw it:<br>
 * Note that here tc's type is "GtkWidget *".<br>
 * @code
 * if (IS_GTK_GAUGE (batt))
 * {
 *	gtk_turn_coordinator_set_value (GTK_GAUGE (batt), value);
 *	gtk_turn_coordinator_redraw(GTK_GAUGE(batt));
 * }		
 * @endcode
 *
 * @b Widget @b Parameters:<br>
 * - "grayscale-color": boolean, if TRUE, draw the widget with grayscale colors (outdoor use)<br>
 * - "radial-color": boolean, if TRUE, draw a fake light reflexion<br>
 * - "start-value": int, indicates the start value of the gauge<br>
 * - "end-value": int, indicates the end value of the gauge<br>
 * - "initial-step": int, fisrt step, divide the gauge range into step<br>
 * - "sub-step": double, sub step, divide the gauge range into sub step<br>
 * WARNING: you need to cast in 'gdouble' when set.<br>
 * - "drawing-step": int, indicates the step where numbers will be drawn<br>
 * - "name": char, provide the gauge's name (support pango text attribute)<br>
 * - OPTIONAL - "color-strip-order": char, allow to reverse the color strip.<br>
 * By default it's YOR (Yellow Orange Red) - possible: GYR,RYG,ROY<br>
 * - OPTIONAL - "green-strip-start": int, indicates the start of the green strip<br>
 * - OPTIONAL - "yellow-strip-start": int, indicates the start of the yellow strip<br>
 * - OPTIONAL - "orange-strip-start": int, indicates the start of the orange strip<br>
 * - OPTIONAL - "red-strip-start": int, indicates the start of the red strip<br>
 * - OPTIONAL - "grayscale-color-strip-order": char, allow to reverse <br>
 * grayscale strip. By default it's WB (White to Black) - possible: BW<br>
 * 
 * @b Widget @b values:<br>
 * - "value": double, provide the hand rotation according to the start-value<br>
 * and the end-value.
 * 
 */

#ifndef __GTK_GAUGE_H__
#define __GTK_GAUGE_H__

#include <gtk/gtk.h>
#include <glib-object.h>
#include <math.h>

#define GTK_GAUGE_MAX_STRING  256       /* Size of a text string */

G_BEGIN_DECLS 

/**
 * @typedef struct GtkGaugeClass 
 * @brief Special Gtk API strucure. Define an instance the widget class
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkGaugeClass
{
  GtkDrawingAreaClass parent_class;

} GtkGaugeClass;

/**
 * @typedef struct GtkGauge 
 * @brief Special Gtk API strucure. Define widget's class
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkGauge
{
  GtkDrawingArea parent;

} GtkGauge;

#define GTK_GAUGE_TYPE			(gtk_gauge_get_type ())
#define GTK_GAUGE(obj)			(G_TYPE_CHECK_INSTANCE_CAST ((obj), GTK_GAUGE_TYPE, GtkGauge))
#define GTK_GAUGE_CLASS(obj)	(G_TYPE_CHECK_CLASS_CAST ((obj), GTK_GAUGE, GtkGaugeClass))
#define IS_GTK_GAUGE(obj)		(G_TYPE_CHECK_INSTANCE_TYPE ((obj), GTK_GAUGE_TYPE))
#define IS_GTK_GAUGE_CLASS(obj)	(G_TYPE_CHECK_CLASS_TYPE ((obj), GTK_GAUGE_TYPE))
#define GTK_GAUGE_GET_CLASS	(G_TYPE_INSTANCE_GET_CLASS ((obj), GTK_GAUGE_TYPE, GtkGaugeClass))

extern GType gtk_gauge_get_type (void) G_GNUC_CONST;
extern GtkWidget *gtk_gauge_new (void);
extern void gtk_gauge_set_value (GtkGauge * gauge, gdouble val);
extern void gtk_gauge_redraw (GtkGauge * gauge);

G_END_DECLS
#endif
