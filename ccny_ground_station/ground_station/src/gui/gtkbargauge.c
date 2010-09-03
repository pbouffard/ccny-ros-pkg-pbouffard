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
 * @file gtkbargauge.c
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

#include <ground_station/gui/gtkbargauge.h>

/**
 * @typedef struct GtkBarGaugePrivate 
 * @brief Special Gtk API strucure. Allow to add a private data<br>
 * for the widget. Defined in the C file in order to be private.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkBarGaugePrivate
{
  /* cairo data */
  gboolean draw_once;
  cairo_surface_t * static_surface;
  cairo_surface_t * dynamic_surface;

  /* widget data */
  gboolean grayscale_color;
  gboolean radial_color;
  gboolean alert1,alert2,alert3,alert4;
  gchar * widget_name;
    
  gint start_value1;
  gint end_value1;  
  gdouble value_bar1;
  gint green_strip_start1;
  gint yellow_strip_start1;
  gchar * name_bar1;
  
  gint start_value2;
  gint end_value2;  
  gdouble value_bar2;
  gint green_strip_start2;
  gint yellow_strip_start2;
  gchar * name_bar2;  
  
  gint start_value3;
  gint end_value3;  
  gdouble value_bar3;
  gint green_strip_start3;
  gint yellow_strip_start3;  
  gchar * name_bar3;  
  
  gint start_value4;
  gint end_value4;  
  gdouble value_bar4;
  gint green_strip_start4;
  gint yellow_strip_start4;  
  gchar * name_bar4;
  
  gint bar_number;

  /* drawing data */
  gdouble x;
  gdouble y;
  gdouble radius;
  GdkColor bg_color_inv;
  GdkColor bg_color_gauge;
  GdkColor bg_color_bounderie;
  GdkColor bg_radial_color_begin_gauge;
  GdkColor bg_radial_color_begin_bounderie;

  gboolean gauge_name_font_size_ok;
  gdouble gauge_name_font_size;

  /* mouse information */
  gboolean b_mouse_onoff;
  GdkPoint mouse_pos;
  GdkModifierType mouse_state;

} GtkBarGaugePrivate;

/**
 * @enum _GTK_BAR_GAUGE_PROPERTY_ID
 * @brief Special Gtk API enum. Allow to identify widget's properties.
 *
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
enum _GTK_BAR_GAUGE_PROPERTY_ID
{
  PROP_0,
  PROP_NAME,
  PROP_BAR_NUMBER,
  PROP_GRAYSCALE_COLOR,
  PROP_RADIAL_COLOR,
  PROP_RADIAL_BAR_PATTERN,
  PROP_NAME1,
  PROP_START_VALUE1,
  PROP_END_VALUE1,
  PROP_GREEN_STRIP_START1,
  PROP_YELLOW_STRIP_START1,  
  PROP_NAME2,
  PROP_START_VALUE2,
  PROP_END_VALUE2,
  PROP_GREEN_STRIP_START2,
  PROP_YELLOW_STRIP_START2,
  PROP_NAME3,  
  PROP_START_VALUE3,
  PROP_END_VALUE3,
  PROP_GREEN_STRIP_START3,
  PROP_YELLOW_STRIP_START3,  
  PROP_NAME4,  
  PROP_START_VALUE4,
  PROP_END_VALUE4,
  PROP_GREEN_STRIP_START4,
  PROP_YELLOW_STRIP_START4,
} GTK_BAR_GAUGE_PROPERTY_ID;

/**
 * @fn G_DEFINE_TYPE (GtkBarGauge, gtk_bar_gauge, GTK_TYPE_DRAWING_AREA);
 * @brief Special Gtk API function. Define a new object type named GtkBarGauge <br>
 * and all preface of the widget's functions calls with gtk_bar_gauge.<br>
 * We are inheriting the type of GtkDrawingArea.<br>
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
G_DEFINE_TYPE (GtkBarGauge, gtk_bar_gauge, GTK_TYPE_DRAWING_AREA);

/**
 * @def GTK_BAR_GAUGE_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GTK_BAR_GAUGE_TYPE, GtkBarGaugePrivate))
 * @brief Special Gtk API define. Add a macro for easy access to the private<br>
 * data struct.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
#define GTK_BAR_GAUGE_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GTK_BAR_GAUGE_TYPE, GtkBarGaugePrivate))

static void gtk_bar_gauge_class_init (GtkBarGaugeClass * klass);
static void gtk_bar_gauge_init (GtkBarGauge * bg);
static void gtk_bar_gauge_destroy (GtkObject * object);
static void gtk_bar_gauge_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec);

static gboolean gtk_bar_gauge_configure_event (GtkWidget * widget, GdkEventConfigure * event);
static gboolean gtk_bar_gauge_expose (GtkWidget * graph, GdkEventExpose * event);
static gboolean gtk_bar_gauge_button_press_event (GtkWidget * widget, GdkEventButton * ev);


static void gtk_bar_gauge_draw_static (GtkWidget * bg, cairo_t * cr);
static void gtk_bar_gauge_draw_base (GtkWidget * bg, cairo_t * cr);
static void gtk_bar_gauge_draw_screws (GtkWidget * bg, cairo_t * cr);

static void gtk_bar_gauge_draw_dynamic (GtkWidget * bg, cairo_t * cr);
static void gtk_bar_gauge_draw_bar (GtkBarGauge * bg, gint index, gdouble x_bar, gdouble y_bar, gdouble width,cairo_t * cr);
static void gtk_bar_gauge_draw_name (GtkBarGauge * bg, gchar * pch_text, GdkRectangle * rect, cairo_t * cr);

static gboolean gtk_bar_gauge_debug = FALSE;
static gboolean gtk_bar_gauge_lock_update = FALSE;

/**
 * @fn static void gtk_bar_gauge_class_init (GtkBarGaugeClass * klass)
 * @brief Special Gtk API function. Function called when the class is<br>
 * initialised. Allow to set certain class wide functions and<br>
 * properties<br>.
 * Allow to override some parent’s expose handler like :<br>
 * - set_property handler<br>
 * - destroy handler<br>
 * - configure_event handler<br>
 * - motion_notify_event handler (not use in this widget)
 * - button_press_event handler (not use in this widget)
 * 
 * Also register the private struct GtkBarGaugePrivate with<br>
 * the class and install widget properties.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_bar_gauge_class_init (GtkBarGaugeClass * klass)
{
  GObjectClass *obj_class = G_OBJECT_CLASS (klass);
  GtkWidgetClass *widget_class = GTK_WIDGET_CLASS (klass);
  GtkObjectClass *gtkobject_class = GTK_OBJECT_CLASS (klass);

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_class_init()");
  }

  /* GObject signal overrides */
  obj_class->set_property = gtk_bar_gauge_set_property;

  /* GtkObject signal overrides */
  gtkobject_class->destroy = gtk_bar_gauge_destroy;

  /* GtkWidget signals overrides */
  widget_class->configure_event = gtk_bar_gauge_configure_event;
  widget_class->expose_event = gtk_bar_gauge_expose;
  widget_class->button_press_event = gtk_bar_gauge_button_press_event;

  g_type_class_add_private (obj_class, sizeof (GtkBarGaugePrivate));
  
  g_object_class_install_property (obj_class,
                                   PROP_NAME,
                                   g_param_spec_string ("widget-name",
                                                        "Gauge name",
                                                        "Gauge name", "...Name not set...", G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_GRAYSCALE_COLOR,
                                   g_param_spec_boolean ("grayscale-color",
                                                         "use grayscale for the widget color",
                                                         "use grayscale for the widget color", FALSE,
                                                         G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_RADIAL_COLOR,
                                   g_param_spec_boolean ("radial-color",
                                                         "the widget use radial color",
                                                         "the widget use radial color", TRUE, G_PARAM_WRITABLE));                                                                                                                                                                                                                           
  g_object_class_install_property (obj_class,
                                   PROP_NAME1,
                                   g_param_spec_string ("name-bar-1",
                                                        "bar 1 name",
                                                        "bar 1 name", "...Name not set...", G_PARAM_WRITABLE));  
  g_object_class_install_property (obj_class,
                                   PROP_BAR_NUMBER,
                                   g_param_spec_int ("bar-number",
                                                     "set the number of bar",
                                                     "set the number of bar",
                                                     1, 4, 1, G_PARAM_WRITABLE)); 
  g_object_class_install_property (obj_class,
                                   PROP_START_VALUE1,
                                   g_param_spec_int ("start-value-bar-1",
                                                     "set the start value for bar 1",
                                                     "set the start value for bar 1",
                                                     0, 10000, 0, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_END_VALUE1,
                                   g_param_spec_int ("end-value-bar-1",
                                                     "set the end value for bar 1",
                                                     "set the end value for bar 1",
                                                     0, 10000, 100, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_NAME2,
                                   g_param_spec_string ("name-bar-2",
                                                        "bar 2 name",
                                                        "bar 2 name", "...Name not set...", G_PARAM_WRITABLE));  
  g_object_class_install_property (obj_class,
                                   PROP_START_VALUE2,
                                   g_param_spec_int ("start-value-bar-2",
                                                     "set the start value for bar 2",
                                                     "set the start value for bar 2",
                                                     0, 10000, 0, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_END_VALUE2,
                                   g_param_spec_int ("end-value-bar-2",
                                                     "set the end value for bar 2",
                                                     "set the end value for bar 2",
                                                     0, 10000, 100, G_PARAM_WRITABLE));                                                     
  g_object_class_install_property (obj_class,
                                   PROP_NAME3,
                                   g_param_spec_string ("name-bar-3",
                                                        "bar 3 name",
                                                        "bar 3 name", "...Name not set...", G_PARAM_WRITABLE));                                                     
  g_object_class_install_property (obj_class,
                                   PROP_START_VALUE3,
                                   g_param_spec_int ("start-value-bar-3",
                                                     "set the start value for bar 3",
                                                     "set the start value for bar 3",
                                                     0, 10000, 0, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_END_VALUE3,
                                   g_param_spec_int ("end-value-bar-3",
                                                     "set the end value for bar 3",
                                                     "set the end value for bar 3",
                                                     0, 10000, 100, G_PARAM_WRITABLE));                                                                                                          
  g_object_class_install_property (obj_class,
                                   PROP_NAME4,
                                   g_param_spec_string ("name-bar-4",
                                                        "bar 4 name",
                                                        "bar 4 name", "...Name not set...", G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_START_VALUE4,
                                   g_param_spec_int ("start-value-bar-4",
                                                     "set the start value for bar 4",
                                                     "set the start value for bar 4",
                                                     0, 10000, 0, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_END_VALUE4,
                                   g_param_spec_int ("end-value-bar-4",
                                                     "set the end value for bar 4",
                                                     "set the end value for bar 4",
                                                     0, 10000, 100, G_PARAM_WRITABLE));                                                     
  
  g_object_class_install_property (obj_class, 
											  PROP_GREEN_STRIP_START1,
                                   g_param_spec_int ("green-strip-start-1",
																	  "set the green strip start of the gauge",
                                                     "set the green strip start of the gauge",
                                                     0, 10000, 0, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class, 
											  PROP_YELLOW_STRIP_START1,
                                   g_param_spec_int ("yellow-strip-start-1",
																	  "set the yellow strip start of the gauge",
                                                     "set the yellow strip start of the gauge",
                                                     0, 10000, 0, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class, 
											  PROP_GREEN_STRIP_START2,
                                   g_param_spec_int ("green-strip-start-2",
																	  "set the green strip start of the gauge",
                                                     "set the green strip start of the gauge",
                                                     0, 10000, 0, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class, 
											  PROP_YELLOW_STRIP_START2,
                                   g_param_spec_int ("yellow-strip-start-2",
																	  "set the yellow strip start of the gauge",
                                                     "set the yellow strip start of the gauge",
                                                     0, 10000, 0, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class, 
											  PROP_GREEN_STRIP_START3,
                                   g_param_spec_int ("green-strip-start-3",
																	  "set the green strip start of the gauge",
                                                     "set the green strip start of the gauge",
                                                     0, 10000, 0, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class, 
											  PROP_YELLOW_STRIP_START3,
                                   g_param_spec_int ("yellow-strip-start-3",
																	  "set the yellow strip start of the gauge",
                                                     "set the yellow strip start of the gauge",
                                                     0, 10000, 0, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class, 
											  PROP_GREEN_STRIP_START4,
                                   g_param_spec_int ("green-strip-start-4",
																	  "set the green strip start of the gauge",
                                                     "set the green strip start of the gauge",
                                                     0, 10000, 0, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class, 
											  PROP_YELLOW_STRIP_START4,
                                   g_param_spec_int ("yellow-strip-start-4",
																	  "set the yellow strip start of the gauge",
                                                     "set the yellow strip start of the gauge",
                                                     0, 10000, 0, G_PARAM_WRITABLE));                                                     
                                                                                                          
  return;
}

/**
 * @fn static void gtk_bar_gauge_init (GtkBarGauge * bg)
 * @brief Special Gtk API function. Function called when the creating a<br>
 * new GtkBarGauge. Allow to initialize some private variables of<br>
 * widget.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_bar_gauge_init (GtkBarGauge * bg)
{
  GtkBarGaugePrivate *priv = NULL;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_init()");
  }
  g_return_if_fail (IS_GTK_BAR_GAUGE (bg));

  priv = GTK_BAR_GAUGE_GET_PRIVATE (bg);

  gtk_widget_add_events (GTK_WIDGET (bg), GDK_BUTTON_PRESS_MASK |
                         GDK_BUTTON_RELEASE_MASK | GDK_POINTER_MOTION_MASK | GDK_POINTER_MOTION_HINT_MASK);
  priv->b_mouse_onoff = FALSE;
    priv->draw_once = FALSE;
  priv->gauge_name_font_size_ok = FALSE;
  priv->widget_name= g_strdup ("-- NO NAME SET --");

  priv->name_bar1 = g_strdup ("NO NAME");
  priv->start_value1 = -1;
  priv->end_value1 = -1;
  priv->green_strip_start1 = -1;
  priv->yellow_strip_start1 = -1;  
  priv->value_bar1=12;
    
  priv->name_bar2 = g_strdup ("NO NAME");
  priv->start_value2 = -1;
  priv->end_value2 = -1;
  priv->green_strip_start2 = -1;
  priv->yellow_strip_start2 = -1;  
  priv->value_bar2=100;
    
  priv->name_bar3 = g_strdup ("NO NAME");
  priv->start_value3 = -1;
  priv->end_value3 = -1;
  priv->green_strip_start3 = -1;
  priv->yellow_strip_start3 = -1;  
  priv->value_bar3=100;
  
  priv->name_bar4 = g_strdup ("NO NAME");
  priv->start_value4 = -1;
  priv->end_value4 = -1;
  priv->green_strip_start4 = -1;
  priv->yellow_strip_start4 = -1;  
  priv->value_bar4=0;  
  
  priv->grayscale_color = TRUE;
  priv->radial_color = FALSE;
  priv->alert1=TRUE;
  priv->alert2=TRUE;
  priv->alert3=TRUE;
  priv->alert4=TRUE;
  
  priv->bar_number=-1;

  priv->bg_color_bounderie.red = 6553.5;        // 0.1 cairo
  priv->bg_color_bounderie.green = 6553.5;
  priv->bg_color_bounderie.blue = 6553.5;
  priv->bg_color_gauge.red = 3276.75;   // 0.05 cairo
  priv->bg_color_gauge.green = 3276.75;
  priv->bg_color_gauge.blue = 3276.75;
  priv->bg_color_inv.red = 45874.5;     // 0.7 cairo
  priv->bg_color_inv.green = 45874.5;
  priv->bg_color_inv.blue = 45874.5;
  priv->bg_radial_color_begin_bounderie.red = 13107;    // 0.2 cairo
  priv->bg_radial_color_begin_bounderie.green = 13107;
  priv->bg_radial_color_begin_bounderie.blue = 13107;
  priv->bg_radial_color_begin_gauge.red = 45874.5;      // 0.7 cairo
  priv->bg_radial_color_begin_gauge.green = 45874.5;
  priv->bg_radial_color_begin_gauge.blue = 45874.5;
  return;
}

/**
 * @fn static gboolean gtk_bar_gauge_configure_event (GtkWidget * widget, GdkEventConfigure * event)
 * @brief Special Gtk API function. Override the _configure_event handler<br>
 * in order to resize the widget when the main window is resized.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static gboolean gtk_bar_gauge_configure_event (GtkWidget * widget, GdkEventConfigure * event)
{
  GtkBarGaugePrivate *priv;
  GtkBarGauge *bg = GTK_BAR_GAUGE (widget);

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_configure_event()");
  }
  g_return_val_if_fail (IS_GTK_BAR_GAUGE (bg), FALSE);
  g_return_val_if_fail (event->type == GDK_CONFIGURE, FALSE);

  priv = GTK_BAR_GAUGE_GET_PRIVATE (bg);
  g_return_val_if_fail (priv != NULL, FALSE);
  
  priv->draw_once = FALSE;
  priv->gauge_name_font_size_ok = FALSE;

  return FALSE;
}

/**
 * @fn static gboolean gtk_bar_gauge_expose (GtkWidget * bg, GdkEventExpose * event)
 * @brief Special Gtk API function. Override of the expose handler.<br>
 * An “expose-event” signal is emitted when the widget need to be drawn.<br>
 * A Cairo context is created for the parent's GdkWindow.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static gboolean gtk_bar_gauge_expose (GtkWidget * bg, GdkEventExpose * event)
{
  GtkBarGaugePrivate *priv;
  GtkWidget *widget = bg;
  cairo_t * cr_final;
  cairo_t * cr_static;
  cairo_t * cr_dynamic;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_expose()");
  }
  g_return_val_if_fail (IS_GTK_BAR_GAUGE (bg), FALSE);

  priv = GTK_BAR_GAUGE_GET_PRIVATE (bg);
  g_return_val_if_fail (priv != NULL, FALSE);

  cr_final = gdk_cairo_create (widget->window);
  
  if(!priv->draw_once)
  {
		priv->static_surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, event->area.width, event->area.height);
		cr_static = cairo_create(priv->static_surface);
  
		cairo_rectangle (cr_static, event->area.x, event->area.y, event->area.width, event->area.height);
		cairo_clip (cr_static);
		gtk_bar_gauge_draw_static(bg,cr_static);
		cairo_destroy (cr_static);
		priv->draw_once=TRUE;
  }

  priv->dynamic_surface = cairo_surface_create_similar (priv->static_surface,CAIRO_CONTENT_COLOR_ALPHA,event->area.width, event->area.height);
  cr_dynamic = cairo_create(priv->dynamic_surface);
  cairo_rectangle (cr_dynamic, event->area.x, event->area.y, event->area.width, event->area.height);
  cairo_clip (cr_dynamic);
  gtk_bar_gauge_draw_dynamic (bg, cr_dynamic);
  cairo_destroy (cr_dynamic);
  
  cairo_set_source_surface(cr_final, priv->static_surface, 0, 0);
  cairo_paint(cr_final);
  cairo_set_source_surface(cr_final, priv->dynamic_surface, 0, 0);
  cairo_paint(cr_final);
  
  cairo_surface_destroy(priv->dynamic_surface);
  cairo_destroy (cr_final);
  return FALSE;
}

/**
 * @fn extern void gtk_bar_gauge_redraw (GtkBarGauge * bg)
 * @brief Special Gtk API function. Redraw the widget when called.
 * 
 * This function will redraw the widget canvas. In order to reexpose the canvas<br>
 * (and cause it to redraw) of our parent class(GtkDrawingArea), it is needed to<br>
 * use gdk_window_invalidate_rect(). The function gdk_window_invalidate_region()<br>
 * need to be called as well. And finaly, in order to make all events happen, it<br>
 * is needed to call gdk_window_process_all_updates().
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
extern void gtk_bar_gauge_redraw (GtkBarGauge * bg)
{
  GtkWidget *widget;
  GdkRegion *region;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_redraw()");
  }
  g_return_if_fail (IS_GTK_BAR_GAUGE (bg));

  widget = GTK_WIDGET (bg);

  if (!widget->window)
    return;

  region = gdk_drawable_get_clip_region (widget->window);
  // **** redraw the window completely by exposing it
  gdk_window_invalidate_region (widget->window, region, TRUE);
  gdk_window_process_updates (widget->window, TRUE);

  gdk_region_destroy (region);
}

/**
 * @fn extern void gtk_bar_gauge_set_value (GtkBarGauge * bg, gint bar_index, gdouble val)
 * @brief Public widget's function that allow the main program/user to<br>
 * set the internal value variable of the widget. 
 * 
 * "index": the value of the bar you want to update<br>
 * "val": double, provide the bar drawing according to the start-value<br>
 * and the end-value.
 */
extern void gtk_bar_gauge_set_value (GtkBarGauge * bg, gint bar_index, gdouble val)
{
  GtkBarGaugePrivate *priv;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_set_value()");
  }
  g_return_if_fail (IS_GTK_BAR_GAUGE (bg));

  priv = GTK_BAR_GAUGE_GET_PRIVATE (bg);
  
  if(!gtk_bar_gauge_lock_update)
  {
		if(bar_index <= priv->bar_number)
		{
			switch(bar_index)
			{
				case 1:
					if((val<priv->start_value1)||(val>priv->end_value1))
					{
						g_warning("GTK BAR GAUGE: gtk_bar_gauge_set_value => value out of range");
					}
					else priv->value_bar1=val;
					break;
				case 2:
					if((val<priv->start_value2)||(val>priv->end_value2))
					{
						g_warning("GTK BAR GAUGE: gtk_bar_gauge_set_value => value out of range");
					}
					else priv->value_bar2=val;
					break;
				case 3:
					if((val<priv->start_value3)||(val>priv->end_value3))
					{
						g_warning("GTK BAR GAUGE: gtk_bar_gauge_set_value => value out of range");
					}
					else priv->value_bar3=val;
					break;
				case 4:
					if((val<priv->start_value4)||(val>priv->end_value4))
					{
						g_warning("GTK BAR GAUGE: gtk_bar_gauge_set_value => value out of range");
					}
					else priv->value_bar4=val;
					break;
			}
		}
		else g_warning("GTK BAR GAUGE: gtk_bar_gauge_set_value => index out of range");
  }
}

/**
 * @fn extern GtkWidget *gtk_bar_gauge_new (void)
 * @brief Special Gtk API function. This function is simply a wrapper<br>
 * for convienience.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
extern GtkWidget *gtk_bar_gauge_new (void)
{
  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_new()");
  }
  return GTK_WIDGET (gtk_type_new (gtk_bar_gauge_get_type ()));
}

/**
 * @fn static void gtk_bar_gauge_draw_static (GtkWidget * bg, cairo_t * cr)
 * @brief Special Gtk API function. This function use the cairo context<br>
 * created before in order to draw scalable graphics. 
 * 
 * See GObject,Cairo and GTK+ references for more informations: 
 * http://library.gnome.org/devel/references.html.en
 */
static void gtk_bar_gauge_draw_static (GtkWidget * bg, cairo_t * cr)
{
  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_draw_static()");
  }
  g_return_if_fail (IS_GTK_BAR_GAUGE (bg));

  gtk_bar_gauge_draw_base (bg,cr);
  gtk_bar_gauge_draw_screws (bg,cr);
}

/**
 * @fn static void gtk_bar_gauge_draw_dynamic (GtkWidget * bg, cairo_t * cr)
 * @brief Special Gtk API function. This function use the cairo context<br>
 * created before in order to draw scalable graphics. 
 * 
 * See GObject,Cairo and GTK+ references for more informations: 
 * http://library.gnome.org/devel/references.html.en
 */
static void gtk_bar_gauge_draw_dynamic (GtkWidget * bg, cairo_t * cr)
{
  GtkBarGaugePrivate *priv;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_draw_dynamic()");
  }
  g_return_if_fail (IS_GTK_BAR_GAUGE (bg));
  priv = GTK_BAR_GAUGE_GET_PRIVATE (bg);
  
  double x, y, radius;
  radius=priv->radius;
  x=priv->x;
  y=priv->y;
  
  switch(priv->bar_number)
  {
	  case 1:
	  	gtk_bar_gauge_draw_bar(GTK_BAR_GAUGE(bg),1,x - 0.3 * radius ,y-0.5*radius,0.6 * radius, cr);
	  break;
	  case 2:
		gtk_bar_gauge_draw_bar(GTK_BAR_GAUGE(bg),1,x - 0.6 * radius ,y-0.5*radius,0.5 * radius, cr);
		gtk_bar_gauge_draw_bar(GTK_BAR_GAUGE(bg),2,x + 0.1 * radius ,y-0.5*radius,0.5 * radius, cr);
		break;
	  case 3:
		gtk_bar_gauge_draw_bar(GTK_BAR_GAUGE(bg),1,x - 0.645 * radius ,y-0.5*radius,0.3 * radius, cr);
		gtk_bar_gauge_draw_bar(GTK_BAR_GAUGE(bg),2,x - 0.145 * radius ,y-0.5*radius,0.3 * radius, cr);
		gtk_bar_gauge_draw_bar(GTK_BAR_GAUGE(bg),3,x + 0.355 * radius ,y-0.5*radius,0.3 * radius, cr);
		break;
	  case 4:
		gtk_bar_gauge_draw_bar(GTK_BAR_GAUGE(bg),1, x - 0.65 * radius ,y-0.5*radius,0.25 * radius, cr);
		gtk_bar_gauge_draw_bar(GTK_BAR_GAUGE(bg),2, x - 0.30 * radius ,y-0.5*radius,0.25 * radius, cr);
		gtk_bar_gauge_draw_bar(GTK_BAR_GAUGE(bg),3, x + 0.05 * radius ,y-0.5*radius,0.25 * radius, cr);
		gtk_bar_gauge_draw_bar(GTK_BAR_GAUGE(bg),4, x + 0.4 * radius ,y-0.5*radius,0.25 * radius, cr);
	   break;
  }
}

/**
 * @fn static void gtk_bar_gauge_draw (GtkWidget * bg)
 * @brief Special Gtk API function. Override the _draw handler of the<br>
 * parent class GtkDrawingArea. This function use the cairo context<br>
 * created before in order to draw scalable graphics. 
 * 
 * See GObject,Cairo and GTK+ references for more informations: 
 * http://library.gnome.org/devel/references.html.en
 */
static void gtk_bar_gauge_draw_base (GtkWidget * bg, cairo_t * cr)
{
  GtkBarGaugePrivate *priv;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_draw_base()");
  }
  g_return_if_fail (IS_GTK_BAR_GAUGE (bg));

  priv = GTK_BAR_GAUGE_GET_PRIVATE (bg);

  // **** Parameters test  
  if(priv->bar_number<0)
  {
	 g_warning ("GTK_BAR_GAUGE : number of bar gauge not set (1-4), enable to paint.");
    return;
  }
  
  if((priv->bar_number==0)||(priv->bar_number>4))
  {
	 g_warning ("GTK_BAR_GAUGE : number of bar gauge out of range (1-4), enable to paint.");
    return;
  }
  
  switch(priv->bar_number)
  {
	  case 4 :
			if ((priv->start_value4 == -1) || (priv->end_value4 == -1))
			{
				g_warning ("GTK_BAR_GAUGE : minimal parameters not set, unable to paint.");
				return;
			}
			if(priv->end_value4 == priv->start_value4)
			{
				g_warning ("GTK_BAR_GAUGE : wrong parameters, unable to paint.");
				return;
			}
	  case 3 :
	  		if ((priv->start_value3 == -1) || (priv->end_value3 == -1))
			{
				g_warning ("GTK_BAR_GAUGE : minimal parameters not set, unable to paint.");
				return;
			}
			if(priv->end_value3 == priv->start_value3)
			{
				g_warning ("GTK_BAR_GAUGE : wrong parameters, unable to paint.");
				return;
			}
	  case 2 :
	  		if ((priv->start_value2 == -1) || (priv->end_value2 == -1))
			{
				g_warning ("GTK_BAR_GAUGE : minimal parameters not set, unable to paint.");
				return;
			}
			if(priv->end_value2 == priv->start_value2)
			{
				g_warning ("GTK_BAR_GAUGE : wrong parameters, unable to paint.");
				return;
			}
	  case 1 :
	  		if ((priv->start_value1 == -1) || (priv->end_value1 == -1))
			{
				g_warning ("GTK_BAR_GAUGE : minimal parameters not set, unable to paint.");
				return;
			}
			if(priv->end_value1 == priv->start_value1)
			{
				g_warning ("GTK_BAR_GAUGE : wrong parameters, unable to paint.");
				return;
			}
		break;
  }
  
  double x, y, rec_x0, rec_y0, rec_width, rec_height, rec_degrees;
  double rec_aspect, rec_corner_radius, rec_radius, radius;
  cairo_pattern_t *pat=NULL;
  GdkRectangle name_box;
  char str[GTK_BAR_GAUGE_MAX_STRING];  

  x = bg->allocation.width / 2;
  y = bg->allocation.height / 2;
  radius = MIN (bg->allocation.width / 2, bg->allocation.height / 2) - 5;

  rec_x0 = x - radius;
  rec_y0 = y - radius;
  rec_width = radius * 2;
  rec_height = radius * 2;
  rec_aspect = 1.0;
  rec_corner_radius = rec_height / 8.0;
  rec_radius = rec_corner_radius / rec_aspect;
  rec_degrees = M_PI / 180.0;

  // **** Bar Gauge base
  cairo_new_sub_path (cr);
  cairo_arc (cr, rec_x0 + rec_width - rec_radius, rec_y0 + rec_radius,
             rec_radius, -90 * rec_degrees, 0 * rec_degrees);
  cairo_arc (cr, rec_x0 + rec_width - rec_radius, rec_y0 + rec_height - rec_radius,
             rec_radius, 0 * rec_degrees, 90 * rec_degrees);
  cairo_arc (cr, rec_x0 + rec_radius, rec_y0 + rec_height - rec_radius,
             rec_radius, 90 * rec_degrees, 180 * rec_degrees);
  cairo_arc (cr, rec_x0 + rec_radius, rec_y0 + rec_radius, rec_radius, 180 * rec_degrees, 270 * rec_degrees);
  cairo_close_path (cr);

  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_bounderie.red / 65535,
                            (gdouble) priv->bg_color_bounderie.green / 65535,
                            (gdouble) priv->bg_color_bounderie.blue / 65535);
    else
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_bounderie.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, (gdouble) priv->bg_color_bounderie.red / 65535,
                                       (gdouble) priv->bg_color_bounderie.green / 65535,
                                       (gdouble) priv->bg_color_bounderie.blue / 65535, 1);
    cairo_set_source (cr, pat);
  }
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  
  // **** widget name plate
  cairo_set_line_width (cr, 0.015 * radius);
  cairo_rectangle(cr, x - 0.65 * radius, y-0.93*radius, 1.3*radius,0.17*radius);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb(cr, 0.6,0.6,0.6);
    else
      cairo_set_source_rgb(cr, 0.4,0.4,0.4);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, 0.9,0.9,0.9, 0.8);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.6,0.6,0.6, 0.8);
    cairo_set_source (cr, pat);
  }
  cairo_fill (cr);
  cairo_stroke (cr);
  
  cairo_arc (cr, x-0.57*radius, y-0.845*radius, 0.05 * radius, 0, 2*M_PI);
  pat = cairo_pattern_create_radial (x-0.57*radius, y-0.845*radius, 0.07*radius,
                                     x-0.57*radius, y-0.845*radius, 0.05*radius);
  cairo_pattern_add_color_stop_rgba (pat,0, 0.6, 0.6, 0.6,0.4);
  cairo_pattern_add_color_stop_rgba (pat,1, 0.2, 0.2, 0.2,0.4);
  cairo_set_source (cr, pat);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
    
  cairo_arc (cr, x-0.57*radius, y-0.845*radius, 0.04 * radius, 0, 2*M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb(cr, 0.5,0.5,0.5);
    else
      cairo_pattern_add_color_stop_rgba (pat, 0, 1-0.7,1-0.7,1-0.7, 1);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, 0.7,0.7,0.7, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.5,0.5,0.5, 1);
    cairo_set_source (cr, pat);
  }
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  
  cairo_arc (cr, x+0.57*radius, y-0.845*radius, 0.05 * radius, 0, 2*M_PI);
  pat = cairo_pattern_create_radial (x+0.57*radius, y-0.845*radius, 0.07*radius,
                                     x+0.57*radius, y-0.845*radius, 0.05*radius);
  cairo_pattern_add_color_stop_rgba (pat,0, 0.6, 0.6, 0.6,0.4);
  cairo_pattern_add_color_stop_rgba (pat,1, 0.2, 0.2, 0.2,0.4);
  cairo_set_source (cr, pat);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  
  cairo_arc (cr, x+0.57*radius, y-0.845*radius, 0.04 * radius, 0, 2*M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb(cr, 0.5,0.5,0.5);
    else
      cairo_pattern_add_color_stop_rgba (pat, 0, 1-0.7,1-0.7,1-0.7, 1);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, 0.7,0.7,0.7, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.5,0.5,0.5, 1);
    cairo_set_source (cr, pat);
  }
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  

  name_box.x = x-0.48*radius;
  name_box.y = y-0.91*radius;
  name_box.width = 0.97*radius;
  name_box.height = 0.14*radius;
  if (!priv->grayscale_color)
		sprintf (str, "<span foreground=\"black\">%s</span>", priv->widget_name);
  else
		sprintf (str, "<span foreground=\"white\">%s</span>", priv->widget_name);
  gtk_bar_gauge_draw_name (GTK_BAR_GAUGE (bg), (gchar *)&str, &name_box, cr);
  
  priv->x = x;
  priv->y = y;
  priv->radius = radius;  
  cairo_pattern_destroy (pat);  
  return;
}

/**
 * @fn static void gtk_bar_gauge_draw_bar (GtkBarGauge * bg, gint index, gdouble x_bar, gdouble y_bar, gdouble width, cairo_t * cr)
 * @brief Private widget's function that draw one bar.
 */
static void gtk_bar_gauge_draw_bar (GtkBarGauge * bg, gint index, gdouble x_bar, gdouble y_bar, gdouble width, cairo_t * cr)
{
  GtkBarGaugePrivate *priv;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_draw_bar()");
  }
  g_return_if_fail (IS_GTK_BAR_GAUGE (bg));

  priv = GTK_BAR_GAUGE_GET_PRIVATE (bg);
  
  char * name =NULL;
  char str[GTK_BAR_GAUGE_MAX_STRING];
  double x,y,radius,value_bar=0, bar_y=0, bar_height=0;
  int start_value_bar=0,end_value_bar=0,green_strip_start=0,yellow_strip_start=0;
  gboolean alert=TRUE;
  cairo_pattern_t *pat=NULL;
  GdkRectangle bar_box,name_box;
 
  x=priv->x;
  y=priv->y;
  radius=priv->radius;
    
  bar_box.x = x_bar;
  bar_box.y = y_bar;
  bar_box.width = width;
  bar_box.height = 1.1 * radius;
  
  switch(index)
  {
	  case 1:
		value_bar=priv->value_bar1;
		start_value_bar=priv->start_value1;
		end_value_bar=priv->end_value1;
		name=priv->name_bar1;
		alert=priv->alert1;
	
		if ((priv->green_strip_start1 == -1) || (priv->yellow_strip_start1 == -1))
		{
			priv->green_strip_start1 = priv->end_value1 - priv->end_value1 / 3;
			priv->yellow_strip_start1 = priv->end_value1 - 2*priv->end_value1 / 3;
		}
		green_strip_start=priv->green_strip_start1;
		yellow_strip_start=priv->yellow_strip_start1;
		break;
	  case 2:
		value_bar=priv->value_bar2;
		start_value_bar=priv->start_value2;
		end_value_bar=priv->end_value2;		
		name=priv->name_bar2;
		alert=priv->alert2;		
		
		if ((priv->green_strip_start2 == -1) || (priv->yellow_strip_start2 == -1))
		{
			priv->green_strip_start2 = priv->end_value2 - priv->end_value2 / 3;
			priv->yellow_strip_start2 = priv->end_value2 - 2*priv->end_value2 / 3;
		}		
		green_strip_start=priv->green_strip_start2;
		yellow_strip_start=priv->yellow_strip_start2;
		break;
	  case 3:
		value_bar=priv->value_bar3;
		start_value_bar=priv->start_value3;
		end_value_bar=priv->end_value3;		
		name=priv->name_bar3;		
		alert=priv->alert3;		
		
		if ((priv->green_strip_start3 == -1) || (priv->yellow_strip_start3 == -1))
		{
			priv->green_strip_start3 = priv->end_value3 - priv->end_value3 / 3;
			priv->yellow_strip_start3 = priv->end_value3 - 2*priv->end_value3 / 3;
		}		
		green_strip_start=priv->green_strip_start3;
		yellow_strip_start=priv->yellow_strip_start3;		
		break;
	  case 4:
		value_bar=priv->value_bar4;
		start_value_bar=priv->start_value4;
		end_value_bar=priv->end_value4;		
		name=priv->name_bar4;		
		alert=priv->alert4;

		if ((priv->green_strip_start4 == -1) || (priv->yellow_strip_start4 == -1))
		{
			priv->green_strip_start4 = priv->end_value4- priv->end_value4 / 3;
			priv->yellow_strip_start4 = priv->end_value4 - 2*priv->end_value4 / 3;
		}		
		green_strip_start=priv->green_strip_start4;
		yellow_strip_start=priv->yellow_strip_start4;	
		break;
  }
     
  bar_y = bar_box.y + ((((gdouble)end_value_bar-(gdouble)value_bar) / (gdouble)end_value_bar)*bar_box.height);
  bar_height = bar_box.height - ((((gdouble)end_value_bar-(gdouble)value_bar) / (gdouble)end_value_bar)*bar_box.height);
   
  if(!priv->grayscale_color)
  {
	  if(value_bar>=green_strip_start)
	  {
			cairo_rectangle(cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (cr,0,1,0, 1);
			cairo_fill (cr);
			cairo_stroke(cr);
	  }
	  else  if((value_bar<green_strip_start)&&(value_bar>=yellow_strip_start))
	  {
			cairo_rectangle(cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (cr,1,1,0, 1);
			cairo_fill (cr);
			cairo_stroke(cr);
	  }
	  else  if(value_bar<=yellow_strip_start)
	  {
			cairo_rectangle(cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (cr,1,0,0, 1);
			cairo_fill (cr);
			cairo_stroke(cr);
	  }
  }
  else
  {
	  if(value_bar>=green_strip_start)
	  {
			cairo_rectangle(cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (cr,1,1,1, 1);			
			cairo_fill (cr);
			cairo_stroke(cr);
	  }
	  else  if((value_bar<green_strip_start)&&(value_bar>=yellow_strip_start))
	  {
			cairo_rectangle(cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (cr,0.5,0.5,0.5, 1);
			
			cairo_fill (cr);
			cairo_stroke(cr);
	  }
	  else  if(value_bar<=yellow_strip_start)
	  {
			cairo_rectangle(cr,bar_box.x,bar_y,bar_box.width,bar_height);
			if(alert) 
			{
				cairo_set_source_rgba (cr,0.,0.,0., 1);
				alert=FALSE;
			}
			else 
			{	
				cairo_set_source_rgba (cr,1.,1.,1., 1);
				alert=TRUE;
			}
			cairo_fill (cr);
			cairo_stroke(cr);
	  }
  }
  
  // **** alpha rect
  if((priv->radial_color) && (!priv->grayscale_color)) 
  {
		pat = cairo_pattern_create_linear (bar_box.x,bar_box.y,bar_box.x+bar_box.width,bar_box.y);
		cairo_pattern_add_color_stop_rgba (pat, 0, 0, 0, 0, 0.4);
		cairo_pattern_add_color_stop_rgba (pat, 0.3, 1, 1, 1, 0.4);
		cairo_pattern_add_color_stop_rgba (pat, 0.5, 0.2, 0.2, 0.2, 0.4);
		cairo_pattern_add_color_stop_rgba (pat, 1, 0, 0, 0, 0.4);
		cairo_rectangle(cr,  bar_box.x, bar_box.y, bar_box.width, bar_box.height);
		cairo_set_source (cr, pat);
		cairo_fill (cr);
		cairo_stroke(cr);
  }
  
  // **** alpha perimeter up
  pat = cairo_pattern_create_linear (bar_box.x,bar_box.y-0.13*radius,bar_box.x,bar_box.y-0.06*radius-0.13*radius);
  cairo_pattern_add_color_stop_rgba (pat, 0, 0.6, 0.6, 0.6, 0.4);
  cairo_pattern_add_color_stop_rgba (pat, 1, 0.1, 0.1, 0.1, 0.4);  
  cairo_rectangle(cr, bar_box.x, bar_box.y-0.06*radius-0.13*radius, bar_box.width, 0.06*radius);
  cairo_set_source (cr, pat);
  cairo_fill (cr);
  cairo_stroke(cr);
  
  cairo_new_sub_path (cr);
  cairo_move_to (cr,bar_box.x,bar_box.y-0.06*radius-0.13*radius);
  cairo_line_to (cr,bar_box.x-0.06*radius,bar_box.y-0.06*radius-0.13*radius);
  cairo_line_to (cr,bar_box.x,bar_box.y-0.13*radius);  
  cairo_line_to (cr,bar_box.x,bar_box.y-0.06*radius-0.13*radius);  
  cairo_close_path (cr);
  cairo_set_source (cr, pat);
  cairo_fill (cr);
  cairo_stroke(cr);
  
  cairo_new_sub_path (cr);
  cairo_move_to (cr,bar_box.x+bar_box.width,bar_box.y-0.06*radius-0.13*radius);
  cairo_line_to (cr,bar_box.x+bar_box.width+0.06*radius,bar_box.y-0.06*radius-0.13*radius);
  cairo_line_to (cr,bar_box.x+bar_box.width,bar_box.y-0.13*radius);  
  cairo_line_to (cr,bar_box.x+bar_box.width,bar_box.y-0.06*radius-0.13*radius);  
  cairo_close_path (cr);
  cairo_set_source (cr, pat);
  cairo_fill (cr);
  cairo_stroke(cr);  
  
  // **** alpha perimeter down
  pat = cairo_pattern_create_linear (bar_box.x,bar_box.y+bar_box.height, bar_box.x,bar_box.y+bar_box.height+0.06*radius);
  cairo_pattern_add_color_stop_rgba (pat, 0, 0.6, 0.6, 0.6, 0.4);
  cairo_pattern_add_color_stop_rgba (pat, 1, 0.1, 0.1, 0.1, 0.4);
  cairo_rectangle(cr, bar_box.x, bar_box.y+bar_box.height, bar_box.width, 0.06*radius);
  cairo_set_source (cr, pat);
  cairo_fill (cr);
  cairo_stroke(cr);
  
  cairo_new_sub_path (cr);
  cairo_move_to (cr,bar_box.x,bar_box.y+bar_box.height+0.06*radius);
  cairo_line_to (cr,bar_box.x-0.06*radius,bar_box.y+bar_box.height+0.06*radius);
  cairo_line_to (cr,bar_box.x,bar_box.y+bar_box.height);  
  cairo_line_to (cr,bar_box.x,bar_box.y+bar_box.height+0.06*radius);  
  cairo_close_path (cr);
  cairo_set_source (cr, pat);
  cairo_fill (cr);
  cairo_stroke(cr);
  
  cairo_new_sub_path (cr);
  cairo_move_to (cr,bar_box.x+bar_box.width,bar_box.y+bar_box.height+0.06*radius);
  cairo_line_to (cr,bar_box.x+bar_box.width+0.06*radius,bar_box.y+bar_box.height+0.06*radius);
  cairo_line_to (cr,bar_box.x+bar_box.width,bar_box.y+bar_box.height);  
  cairo_line_to (cr,bar_box.x+bar_box.width,bar_box.y+bar_box.height+0.06*radius);  
  cairo_close_path (cr);
  cairo_set_source (cr, pat);
  cairo_fill (cr);
  cairo_stroke(cr);  
  
  // **** alpha perimeter left
  pat = cairo_pattern_create_linear (bar_box.x-0.06*radius,bar_box.y,bar_box.x,bar_box.y);
  cairo_pattern_add_color_stop_rgba (pat, 0, 0.1, 0.1, 0.1, 0.4);
  cairo_pattern_add_color_stop_rgba (pat, 1, 0.6, 0.6, 0.6, 0.4);
  cairo_rectangle(cr,  bar_box.x-0.06*radius, bar_box.y-0.13*radius, 0.06*radius, bar_box.height+0.13*radius);
  cairo_set_source (cr, pat);
  cairo_fill (cr);
  cairo_stroke(cr);
  
  cairo_new_sub_path (cr);
  cairo_move_to (cr,bar_box.x-0.06*radius,bar_box.y-0.13*radius);
  cairo_line_to (cr,bar_box.x-0.06*radius,bar_box.y-0.06*radius-0.13*radius);
  cairo_line_to (cr,bar_box.x,bar_box.y-0.13*radius);  
  cairo_line_to (cr,bar_box.x-0.06*radius,bar_box.y-0.13*radius);  
  cairo_close_path (cr);
  cairo_set_source (cr, pat);
  cairo_fill (cr);
  cairo_stroke(cr);
  
  cairo_new_sub_path (cr);
  cairo_move_to (cr,bar_box.x-0.06*radius,bar_box.y+bar_box.height);
  cairo_line_to (cr,bar_box.x-0.06*radius,bar_box.y+bar_box.height+0.06*radius);
  cairo_line_to (cr,bar_box.x,bar_box.y+bar_box.height);  
  cairo_line_to (cr,bar_box.x-0.06*radius,bar_box.y+bar_box.height);  
  cairo_close_path (cr);
  cairo_set_source (cr, pat);
  cairo_fill (cr);
  cairo_stroke(cr);
  
  // **** alpha perimeter right
  pat = cairo_pattern_create_linear ( bar_box.x+bar_box.width,bar_box.y, bar_box.x+bar_box.width+0.06*radius,bar_box.y);
  cairo_pattern_add_color_stop_rgba (pat, 0, 0.6, 0.6, 0.6, 0.4);
  cairo_pattern_add_color_stop_rgba (pat, 1, 0.1, 0.1, 0.1, 0.4);
  cairo_rectangle(cr,  bar_box.x+bar_box.width, bar_box.y-0.13*radius, 0.06*radius, bar_box.height+0.13*radius);
  cairo_set_source (cr, pat);
  cairo_fill (cr);
  cairo_stroke(cr);
  
  cairo_new_sub_path (cr);
  cairo_move_to (cr,bar_box.x+bar_box.width+0.06*radius,bar_box.y-0.13*radius);
  cairo_line_to (cr,bar_box.x+bar_box.width+0.06*radius,bar_box.y-0.06*radius-0.13*radius);
  cairo_line_to (cr,bar_box.x+bar_box.width,bar_box.y-0.13*radius);  
  cairo_line_to (cr,bar_box.x+bar_box.width+0.06*radius,bar_box.y-0.13*radius);  
  cairo_close_path (cr);
  cairo_set_source (cr, pat);
  cairo_fill (cr);
  cairo_stroke(cr);
  
  cairo_new_sub_path (cr);
  cairo_move_to (cr,bar_box.x+bar_box.width+0.06*radius,bar_box.y+bar_box.height);
  cairo_line_to (cr,bar_box.x+bar_box.width+0.06*radius,bar_box.y+bar_box.height+0.06*radius);
  cairo_line_to (cr,bar_box.x+bar_box.width,bar_box.y+bar_box.height);  
  cairo_line_to (cr,bar_box.x+bar_box.width+0.06*radius,bar_box.y+bar_box.height);  
  cairo_close_path (cr);
  cairo_set_source (cr, pat);
  cairo_fill (cr);
  cairo_stroke(cr);
       
	// **** draw gauge value
  name_box.x = bar_box.x+0.025*radius;
  name_box.y = bar_box.y-0.11*radius;
  name_box.width = bar_box.width-0.04*radius;
  name_box.height = 0.11*radius;
  cairo_rectangle(cr, bar_box.x, bar_box.y-0.13*radius, bar_box.width, 0.13*radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb(cr,0.8,0.8,0.8);
  else
    cairo_set_source_rgb(cr,1-0.8,1-0.8,1-0.8);
  cairo_fill (cr);
  cairo_stroke(cr);
   
  cairo_set_line_width (cr, 0.015 * radius);
  cairo_rectangle(cr, bar_box.x, bar_box.y-0.13*radius, bar_box.width, 0.13*radius);
  cairo_set_source_rgb(cr,0.2,0.2,0.2);
  cairo_stroke(cr); 
  
  int val = value_bar *100;
  if (!priv->grayscale_color)
		sprintf (str, "<span foreground=\"black\">%d,%d</span>", (int)(value_bar),(int)(val/10%10)*10+(val%10));
  else
		sprintf (str, "<span foreground=\"white\">%d,%d</span>", (int)(value_bar),(int)(val/10%10)*10+(val%10));
  gtk_bar_gauge_draw_name (GTK_BAR_GAUGE (bg), (gchar *)&str, &name_box,cr);
  
  // **** bar perimeter     
  cairo_set_line_width (cr, 0.015 * radius);
  cairo_rectangle(cr,  bar_box.x, bar_box.y, bar_box.width, bar_box.height);
  cairo_set_source_rgb(cr,0.2,0.2,0.2);
  cairo_stroke(cr);

  // **** draw gauge name
  name_box.x = bar_box.x-0.03*radius;
  name_box.y = bar_box.y+bar_box.height+0.1*radius;
  name_box.width = bar_box.width+0.07*radius;
  name_box.height = 0.26 * radius;

  cairo_set_line_width (cr, 0.015 * radius);
  cairo_rectangle(cr, name_box.x, name_box.y, name_box.width,name_box.height);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb(cr, 0.6,0.6,0.6);
    else
      cairo_set_source_rgb(cr, 0.4,0.4,0.4);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, 0.9,0.9,0.9, 0.8);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.7,0.7,0.7, 0.8);
    cairo_set_source (cr, pat);
  }
  cairo_fill (cr);
  cairo_stroke (cr);
  
  cairo_rectangle(cr,name_box.x,name_box.y,name_box.width,name_box.height);
  cairo_stroke(cr);
  
  cairo_arc (cr, name_box.x+0.035*radius, name_box.y+name_box.height-0.035*radius, 0.025 * radius, 0, 2*M_PI);
  pat = cairo_pattern_create_radial (name_box.x+0.035*radius, name_box.y+name_box.height-0.035*radius, 0.07*radius,
                                     name_box.x+0.035*radius, name_box.y+name_box.height-0.035*radius, 0.025*radius);
  cairo_pattern_add_color_stop_rgba (pat,0, 0.6, 0.6, 0.6,0.4);
  cairo_pattern_add_color_stop_rgba (pat,1, 0.2, 0.2, 0.2,0.4);
  cairo_set_source (cr, pat);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
    
  cairo_arc (cr, name_box.x+0.035*radius, name_box.y+name_box.height-0.035*radius, 0.019 * radius, 0, 2*M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb(cr, 0.5,0.5,0.5);
    else
      cairo_pattern_add_color_stop_rgba (pat, 0, 1-0.7,1-0.7,1-0.7, 1);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, 0.7,0.7,0.7, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.5,0.5,0.5, 1);
    cairo_set_source (cr, pat);
  }
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  
 cairo_arc (cr, name_box.x+name_box.width-0.035*radius, name_box.y+0.035*radius, 0.025 * radius, 0, 2*M_PI);
  pat = cairo_pattern_create_radial (name_box.x+name_box.width-0.035*radius, name_box.y+0.035*radius, 0.07*radius,
                                     name_box.x+name_box.width-0.035*radius, name_box.y+0.035*radius, 0.025*radius);
  cairo_pattern_add_color_stop_rgba (pat,0, 0.6, 0.6, 0.6,0.4);
  cairo_pattern_add_color_stop_rgba (pat,1, 0.2, 0.2, 0.2,0.4);
  cairo_set_source (cr, pat);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
    
  cairo_arc (cr, name_box.x+name_box.width-0.035*radius, name_box.y+0.035*radius, 0.019 * radius, 0, 2*M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb(cr, 0.5,0.5,0.5);
    else
      cairo_pattern_add_color_stop_rgba (pat, 0, 1-0.7,1-0.7,1-0.7, 1);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, 0.7,0.7,0.7, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.5,0.5,0.5, 1);
    cairo_set_source (cr, pat);
  }
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  
  name_box.x = bar_box.x;
  name_box.y = bar_box.y+bar_box.height+0.12*radius;
  name_box.width = bar_box.width;
  name_box.height = 0.2 * radius;
  cairo_stroke(cr);
  
  if (!priv->grayscale_color)
		sprintf (str, "<span foreground=\"black\">%s</span>", name);
  else
		sprintf (str, "<span foreground=\"white\">%s</span>", name);
  gtk_bar_gauge_draw_name (GTK_BAR_GAUGE (bg), (gchar *)&str, &name_box,cr);
    
  switch(index)
  {
	  case 1:
		priv->alert1=alert;
		break;
	  case 2:
		priv->alert2=alert;
		break;
	  case 3:
		priv->alert3=alert;
		break;
	  case 4:
		priv->alert4=alert;
		break;
  }
    
  cairo_pattern_destroy (pat);    
  return;
}

/**
 * @fn static void gtk_bar_gauge_draw_screws (GtkWidget * bg, cairo_t * cr)
 * @brief Private widget's function that draw the widget's screws using cairo.
 */
static void gtk_bar_gauge_draw_screws (GtkWidget * bg, cairo_t * cr)
{
  GtkBarGaugePrivate *priv;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_draw_screws()");
  }
  g_return_if_fail (IS_GTK_BAR_GAUGE (bg));

  priv = GTK_BAR_GAUGE_GET_PRIVATE (bg);
  
  cairo_pattern_t *pat=NULL;
  double x, y, radius;
  radius=priv->radius;
  x=priv->x;
  y=priv->y;
  
  // **** top left screw
  cairo_arc (cr, x-0.82*radius, y-0.82*radius, 0.1 * radius, 0, 2*M_PI);
  pat = cairo_pattern_create_radial (x-0.82*radius, y-0.82*radius, 0.07*radius,
                                     x-0.82*radius, y-0.82*radius, 0.1*radius);
  cairo_pattern_add_color_stop_rgba (pat,0, 0, 0, 0,0.7);
  cairo_pattern_add_color_stop_rgba (pat,1, 0, 0, 0,0.1);
  cairo_set_source (cr, pat);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
    
  cairo_arc (cr, x-0.82*radius, y-0.82*radius, 0.07 * radius, 0, 2*M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_bounderie.red / 65535,
                          (gdouble) priv->bg_color_bounderie.green / 65535,
                            (gdouble) priv->bg_color_bounderie.blue / 65535);
    else
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_bounderie.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15,0.15,0.15, 1);
    cairo_set_source (cr, pat);
  }
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  
  cairo_set_line_width (cr, 0.02 * radius);
  if (!priv->grayscale_color)
       cairo_set_source_rgb (cr, 0., 0., 0.);
  else
       cairo_set_source_rgb (cr, 1., 1., 1.);
  cairo_move_to (cr, x-0.88*radius, y-0.82*radius);
  cairo_line_to (cr, x-0.76*radius, y-0.82*radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);  
  cairo_move_to (cr, x-0.82*radius, y-0.88*radius);
  cairo_line_to (cr, x-0.82*radius, y-0.76*radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_set_line_width (cr, 0.01 * radius);
  if (!priv->grayscale_color)
		cairo_set_source_rgb (cr, 0.1, 0.1, 0.1);
  else
      cairo_set_source_rgb (cr, 0.9, 0.9, 0.9);  
  cairo_move_to (cr, x-0.88*radius, y-0.82*radius);
  cairo_line_to (cr, x-0.76*radius, y-0.82*radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);  
  cairo_move_to (cr, x-0.82*radius, y-0.88*radius);
  cairo_line_to (cr, x-0.82*radius, y-0.76*radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);      
  
   // **** top right screw
  cairo_arc (cr, x+0.82*radius, y-0.82*radius, 0.1 * radius, 0, 2*M_PI);
  pat = cairo_pattern_create_radial (x+0.82*radius, y-0.82*radius, 0.07*radius,
                                     x+0.82*radius, y-0.82*radius, 0.1*radius);
  cairo_pattern_add_color_stop_rgba (pat,0, 0, 0, 0,0.7);
  cairo_pattern_add_color_stop_rgba (pat,1, 0, 0, 0,0.1);
  cairo_set_source (cr, pat);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
    
  cairo_arc (cr, x+0.82*radius, y-0.82*radius, 0.07 * radius, 0, 2*M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_bounderie.red / 65535,
                            (gdouble) priv->bg_color_bounderie.green / 65535,
                            (gdouble) priv->bg_color_bounderie.blue / 65535);
    else
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_bounderie.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15,0.15,0.15, 1);
    cairo_set_source (cr, pat);
  }
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  
  cairo_set_line_width (cr, 0.02 * radius);
  if (!priv->grayscale_color)
       cairo_set_source_rgb (cr, 0., 0., 0.);
  else
       cairo_set_source_rgb (cr, 1., 1., 1.);
  cairo_move_to (cr, x+0.88*radius, y-0.82*radius);
  cairo_line_to (cr, x+0.76*radius, y-0.82*radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);  
  cairo_move_to (cr, x+0.82*radius, y-0.88*radius);
  cairo_line_to (cr, x+0.82*radius, y-0.76*radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_set_line_width (cr, 0.01 * radius);
  if (!priv->grayscale_color)
		cairo_set_source_rgb (cr, 0.1, 0.1, 0.1);
  else
      cairo_set_source_rgb (cr, 0.9, 0.9, 0.9);  
  cairo_move_to (cr, x+0.88*radius, y-0.82*radius);
  cairo_line_to (cr, x+0.76*radius, y-0.82*radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);  
  cairo_move_to (cr, x+0.82*radius, y-0.88*radius);
  cairo_line_to (cr, x+0.82*radius, y-0.76*radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);     
  
   // **** bottom left screw
  cairo_arc (cr, x-0.82*radius, y+0.82*radius, 0.1 * radius, 0, 2*M_PI);
  pat = cairo_pattern_create_radial (x-0.82*radius, y+0.82*radius, 0.07*radius,
                                     x-0.82*radius, y+0.82*radius, 0.1*radius);
  cairo_pattern_add_color_stop_rgba (pat,0, 0, 0, 0,0.7);
  cairo_pattern_add_color_stop_rgba (pat,1, 0, 0, 0,0.1);
  cairo_set_source (cr, pat);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
    
  cairo_arc (cr, x-0.82*radius, y+0.82*radius, 0.07 * radius, 0, 2*M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_bounderie.red / 65535,
                            (gdouble) priv->bg_color_bounderie.green / 65535,
                            (gdouble) priv->bg_color_bounderie.blue / 65535);
    else
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_bounderie.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15,0.15,0.15, 1);
    cairo_set_source (cr, pat);
  }
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  
  cairo_set_line_width (cr, 0.02 * radius);
  if (!priv->grayscale_color)
       cairo_set_source_rgb (cr, 0., 0., 0.);
  else
       cairo_set_source_rgb (cr, 1., 1., 1.);
  cairo_move_to (cr, x-0.88*radius, y+0.82*radius);
  cairo_line_to (cr, x-0.76*radius, y+0.82*radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);  
  cairo_move_to (cr, x-0.82*radius, y+0.88*radius);
  cairo_line_to (cr, x-0.82*radius, y+0.76*radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_set_line_width (cr, 0.01 * radius);
  if (!priv->grayscale_color)
		cairo_set_source_rgb (cr, 0.1, 0.1, 0.1);
  else
      cairo_set_source_rgb (cr, 0.9, 0.9, 0.9);  
  cairo_move_to (cr, x-0.88*radius, y+0.82*radius);
  cairo_line_to (cr, x-0.76*radius, y+0.82*radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);  
  cairo_move_to (cr, x-0.82*radius, y+0.88*radius);
  cairo_line_to (cr, x-0.82*radius, y+0.76*radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);     
  
   // **** bottom right screw
  cairo_arc (cr, x+0.82*radius, y+0.82*radius, 0.1 * radius, 0, 2*M_PI);
  pat = cairo_pattern_create_radial (x+0.82*radius, y+0.82*radius, 0.07*radius,
                                     x+0.82*radius, y+0.82*radius, 0.1*radius);
  cairo_pattern_add_color_stop_rgba (pat,0, 0, 0, 0,0.7);
  cairo_pattern_add_color_stop_rgba (pat,1, 0, 0, 0,0.1);
  cairo_set_source (cr, pat);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
    
  cairo_arc (cr, x+0.82*radius, y+0.82*radius, 0.07 * radius, 0, 2*M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_bounderie.red / 65535,
                            (gdouble) priv->bg_color_bounderie.green / 65535,
                            (gdouble) priv->bg_color_bounderie.blue / 65535);
    else
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_bounderie.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15,0.15,0.15, 1);
    cairo_set_source (cr, pat);
  }
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  
  cairo_set_line_width (cr, 0.02 * radius);
  if (!priv->grayscale_color)
       cairo_set_source_rgb (cr, 0., 0., 0.);
  else
       cairo_set_source_rgb (cr, 1., 1., 1.);
  cairo_move_to (cr, x+0.88*radius, y+0.82*radius);
  cairo_line_to (cr, x+0.76*radius, y+0.82*radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);  
  cairo_move_to (cr, x+0.82*radius, y+0.88*radius);
  cairo_line_to (cr, x+0.82*radius, y+0.76*radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_set_line_width (cr, 0.01 * radius);
  if (!priv->grayscale_color)
		cairo_set_source_rgb (cr, 0.1, 0.1, 0.1);
  else
      cairo_set_source_rgb (cr, 0.9, 0.9, 0.9);  
  cairo_move_to (cr, x+0.88*radius, y+0.82*radius);
  cairo_line_to (cr, x+0.76*radius, y+0.82*radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);  
  cairo_move_to (cr, x+0.82*radius, y+0.88*radius);
  cairo_line_to (cr, x+0.82*radius, y+0.76*radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);     
  cairo_pattern_destroy (pat);  
  return;
}

/**
 * @fn static void gtk_gauge_draw_name (GtkGauge * gauge, gchar * pch_text, GdkRectangle * rect, cairo_t * cr)
 * @brief Private widget's function that draw the gauge name using pango.
 */
static void gtk_bar_gauge_draw_name (GtkBarGauge * bg, gchar * pch_text, GdkRectangle * rect, cairo_t * cr)
{
  GtkBarGaugePrivate *priv;
  PangoLayout *layout = NULL;
  PangoFontDescription *desc = NULL;
  gint x_pos = 0, y_pos = 0, width = 0, height = 0;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_draw_name()");
  }

  g_return_if_fail (IS_GTK_BAR_GAUGE (bg));
  if (pch_text == NULL)
  {
    return;
  }
  g_return_if_fail (rect != NULL);
  priv = GTK_BAR_GAUGE_GET_PRIVATE (bg);
  g_return_if_fail (cr != NULL);

  // **** create pango layout
  layout = pango_cairo_create_layout (cr);
  pango_layout_set_markup (layout, pch_text, -1);
  pango_layout_set_alignment (layout, PANGO_ALIGN_CENTER);

  // **** create a pango description
  desc = pango_font_description_new ();

  // **** search convenable size if needed
  if (priv->gauge_name_font_size_ok)
  {
    pango_font_description_set_size (desc, priv->gauge_name_font_size * PANGO_SCALE);
    pango_layout_set_font_description (layout, desc);
    pango_layout_get_pixel_size (layout, &width, &height);
  }
  else
  {
    priv->gauge_name_font_size = 30;
    pango_font_description_set_size (desc, priv->gauge_name_font_size * PANGO_SCALE);
    pango_layout_set_font_description (layout, desc);

    pango_layout_get_pixel_size (layout, &width, &height);

    // **** find the font size arcording to the rect size
    while ((width > rect->width) || (height > rect->height))
    {
      priv->gauge_name_font_size = priv->gauge_name_font_size - 1;
      if (priv->gauge_name_font_size <= 1)
        break;
      pango_font_description_set_size (desc, priv->gauge_name_font_size * PANGO_SCALE);
      pango_layout_set_font_description (layout, desc);
      pango_layout_get_pixel_size (layout, &width, &height);
    }
    priv->gauge_name_font_size_ok = TRUE;
  }

  if (priv->gauge_name_font_size <= 1)
  {
    g_object_unref (layout);
    pango_font_description_free (desc);
    return;
  }

  x_pos = rect->x + (gint) ((rect->width - width) / 2);
  y_pos = rect->y + (gint) ((rect->height - height) / 2);

  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 1, 1, 1);
  else
    cairo_set_source_rgb (cr, 0., 0., 0.);
  cairo_move_to (cr, x_pos, y_pos);
  pango_cairo_show_layout (cr, layout);
  cairo_stroke (cr);

  g_object_unref (layout);
  pango_font_description_free (desc);
  return;
}

/**
 * @fn static gboolean gtk_bar_gauge_button_press_event (GtkWidget * widget, GdkEventButton * ev)
 * @brief Special Gtk API function. Override the _button_press_event<br>
 * handler. Perform mouse button press events. 
 * 
 * Here, the mouse events are not used for the widget (maybe<br>
 * in future released) but to allow the user to enable/disable<br>
 * the debug messages.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static gboolean gtk_bar_gauge_button_press_event (GtkWidget * widget, GdkEventButton * ev)
{
  GtkBarGaugePrivate *priv;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_button_press_event_cb()");
  }
  g_return_val_if_fail (IS_GTK_BAR_GAUGE (widget), FALSE);

  priv = GTK_BAR_GAUGE_GET_PRIVATE (widget);

  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 2) && priv->b_mouse_onoff)
  {
    gtk_bar_gauge_debug = gtk_bar_gauge_debug ? FALSE : TRUE;
    return TRUE;
  }
  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 1) && priv->b_mouse_onoff)
  {
    gtk_bar_gauge_lock_update = gtk_bar_gauge_lock_update ? FALSE : TRUE;
    return TRUE;
  }
  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 3))
  {
    priv->b_mouse_onoff = priv->b_mouse_onoff ? FALSE : TRUE;
    return TRUE;
  }

  return FALSE;
}

/**
 * @fn static void gtk_bar_gauge_destroy (GtkObject * object)
 * @brief Special Gtk API function. Override the _destroy handler.<br>
 * Allow the destruction of all widget's pointer.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_bar_gauge_destroy (GtkObject * object)
{
  GtkBarGaugePrivate *priv = NULL;
  GtkWidget *widget = NULL;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_destroy(enter)");
  }

  g_return_if_fail (object != NULL);

  widget = GTK_WIDGET (object);

  g_return_if_fail (IS_GTK_BAR_GAUGE (widget));

  priv = GTK_BAR_GAUGE_GET_PRIVATE (widget);
  g_return_if_fail (priv != NULL);

  
  if (priv->static_surface)
  {
    priv->static_surface=NULL;
    priv->dynamic_surface=NULL;
    g_free (priv->widget_name);
    g_free (priv->name_bar1);
    g_free (priv->name_bar2);
    g_free (priv->name_bar3);
    g_free (priv->name_bar4);

    if (GTK_OBJECT_CLASS (gtk_bar_gauge_parent_class)->destroy != NULL)
    {
      (*GTK_OBJECT_CLASS (gtk_bar_gauge_parent_class)->destroy) (object);
    }
  }
  if (gtk_bar_gauge_debug)
  {
    g_debug ("gtk_bar_gauge_destroy(exit)");
  }

  return;
}

/**
 * @fn static void gtk_bar_gauge_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec)
 * @brief Special Gtk API function. Override the _set_property handler <br>
 * in order to set the object parameters.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_bar_gauge_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec)
{
  GtkBarGaugePrivate *priv = NULL;
  GtkBarGauge *bg = NULL;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_set_property()");
  }
  g_return_if_fail (object != NULL);

  bg = GTK_BAR_GAUGE (object);
  g_return_if_fail (IS_GTK_BAR_GAUGE (bg));

  priv = GTK_BAR_GAUGE_GET_PRIVATE (bg);
  g_return_if_fail (priv != NULL);

  switch (prop_id)
  {
	 case PROP_BAR_NUMBER:
      priv->bar_number= g_value_get_int (value);
      break;
   case PROP_NAME:
      priv->widget_name = g_strdup (g_value_get_string (value));
      break;      
	  
    case PROP_NAME1:
      priv->name_bar1 = g_strdup (g_value_get_string (value));
      break;
    case PROP_START_VALUE1:
      priv->start_value1 = g_value_get_int (value);
      break;
    case PROP_END_VALUE1:
      priv->end_value1 = g_value_get_int (value);
      break;
    case PROP_GREEN_STRIP_START1:
      priv->green_strip_start1 = g_value_get_int (value);
      break;
    case PROP_YELLOW_STRIP_START1:
      priv->yellow_strip_start1 = g_value_get_int (value);
      break;                
      
    case PROP_NAME2:
      priv->name_bar2 = g_strdup (g_value_get_string (value));
      break;      
	 case PROP_START_VALUE2:
      priv->start_value2 = g_value_get_int (value);
      break;
    case PROP_END_VALUE2:
      priv->end_value2 = g_value_get_int (value);
      break;
    case PROP_GREEN_STRIP_START2:
      priv->green_strip_start2 = g_value_get_int (value);
      break;
    case PROP_YELLOW_STRIP_START2:
      priv->yellow_strip_start2 = g_value_get_int (value);
      break;                         
            
    case PROP_NAME3:
      priv->name_bar3 = g_strdup (g_value_get_string (value));
      break;
	 case PROP_START_VALUE3:
      priv->start_value3 = g_value_get_int (value);
      break;
    case PROP_END_VALUE3:
      priv->end_value3 = g_value_get_int (value);
      break;
    case PROP_GREEN_STRIP_START3:
      priv->green_strip_start3= g_value_get_int (value);
      break;
    case PROP_YELLOW_STRIP_START3:
      priv->yellow_strip_start3 = g_value_get_int (value);
      break;                 
            
    case PROP_NAME4:
      priv->name_bar4 = g_strdup (g_value_get_string (value));
      break;
    case PROP_START_VALUE4:
      priv->start_value4 = g_value_get_int (value);
      break;
    case PROP_END_VALUE4:
      priv->end_value4 = g_value_get_int (value);
      break;
    case PROP_GREEN_STRIP_START4:
      priv->green_strip_start4 = g_value_get_int (value);
      break;
    case PROP_YELLOW_STRIP_START4:
      priv->yellow_strip_start4 = g_value_get_int (value);
      break;      

    case PROP_GRAYSCALE_COLOR:
      priv->grayscale_color = g_value_get_boolean (value);
      break;
    case PROP_RADIAL_COLOR:
      priv->radial_color = g_value_get_boolean (value);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
  return;
}
