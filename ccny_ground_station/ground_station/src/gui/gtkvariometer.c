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
 * @file gtkvariometer.c
 * @brief Gtk+ based Variometer Widget
 * @author Gautier Dumonteil <gautier.dumonteil@gmail.com>
 * @version 0.1
 * @date 06/06/2010
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
 * <th><IMG SRC="file:///home/gaitt/Bureau/gtkvariometer.png"></th>
 * <th><IMG SRC="file:///home/gaitt/Bureau/gtkvariometer_g.png"></th>
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

#include <ground_station/gui/gtkvariometer.h>

/**
 * @typedef struct GtkVariometerPrivate 
 * @brief Special Gtk API strucure. Allow to add a private data<br>
 * for the widget. Defined in the C file in order to be private.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkVariometerPrivate
{
  /* new cairo design */
  cairo_t *cr;
  GdkRectangle plot_box;

  /* widget data */
  gint unit_value;
  gboolean unit_is_feet;
  gboolean grayscale_color;
  gboolean radial_color;
  gdouble altitude;
  gdouble l_time;
  gdouble l_altitude;

  /* drawing data */
  gdouble x;
  gdouble y;
  gdouble radius;
  GdkColor bg_color_inv;
  GdkColor bg_color_variometer;
  GdkColor bg_color_bounderie;
  GdkColor bg_radial_color_begin_variometer;
  GdkColor bg_radial_color_begin_bounderie;

  /* mouse information */
  gboolean b_mouse_onoff;
  GdkPoint mouse_pos;
  GdkModifierType mouse_state;

} GtkVariometerPrivate;

/**
 * @enum _GTK_VARIOMETER_PROPERTY_ID
 * @brief Special Gtk API enum. Allow to identify widget's properties.
 *
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
enum _GTK_VARIOMETER_PROPERTY_ID
{
  PROP_0,
  PROP_GRAYSCALE_COLOR,
  PROP_UNIT_IS_FEET,
  PROP_UNIT_STEP_VALUE,
  PROP_RADIAL_COLOR,
} GTK_VARIOMETER_PROPERTY_ID;

/**
 * @fn G_DEFINE_TYPE (GtkVariometer, gtk_variometer, GTK_TYPE_DRAWING_AREA);
 * @brief Special Gtk API function. Define a new object type named GtkVariometer <br>
 * and all preface of the widget's functions calls with gtk_variometer.<br>
 * We are inheriting the type of GtkDrawingArea.<br>
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
G_DEFINE_TYPE (GtkVariometer, gtk_variometer, GTK_TYPE_DRAWING_AREA);

/**
 * @def GTK_VARIOMETER_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GTK_VARIOMETER_TYPE, GtkVariometerPrivate))
 * @brief Special Gtk API define. Add a macro for easy access to the private<br>
 * data struct.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
#define GTK_VARIOMETER_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GTK_VARIOMETER_TYPE, GtkVariometerPrivate))

static void gtk_variometer_class_init (GtkVariometerClass * klass);
static void gtk_variometer_init (GtkVariometer * vario);
static void gtk_variometer_destroy (GtkObject * object);
static void gtk_variometer_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec);

static gboolean gtk_variometer_configure_event (GtkWidget * widget, GdkEventConfigure * event);
static gboolean gtk_variometer_expose (GtkWidget * graph, GdkEventExpose * event);
static gboolean gtk_variometer_button_press_event (GtkWidget * widget, GdkEventButton * ev);
static gboolean gtk_variometer_motion_notify_event (GtkWidget * widget, GdkEventMotion * ev);

static void gtk_variometer_draw (GtkWidget * vario);
static void gtk_variometer_draw_screws (GtkWidget * vario);
static void gtk_variometer_draw_hand (GtkWidget * vario);

static gboolean gtk_variometer_debug = FALSE;

/**
 * @fn static void gtk_variometer_class_init (GtkVariometerClass * klass)
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
 * Also register the private struct GtkVariometerPrivate with<br>
 * the class and install widget properties.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_variometer_class_init (GtkVariometerClass * klass)
{
  GObjectClass *obj_class = G_OBJECT_CLASS (klass);
  GtkWidgetClass *widget_class = GTK_WIDGET_CLASS (klass);
  GtkObjectClass *gtkobject_class = GTK_OBJECT_CLASS (klass);

  if (gtk_variometer_debug)
  {
    g_debug ("===> gtk_variometer_class_init()");
  }

  /* GObject signal overrides */
  obj_class->set_property = gtk_variometer_set_property;

  /* GtkObject signal overrides */
  gtkobject_class->destroy = gtk_variometer_destroy;

  /* GtkWidget signals overrides */
  widget_class->configure_event = gtk_variometer_configure_event;
  widget_class->expose_event = gtk_variometer_expose;
  widget_class->motion_notify_event = gtk_variometer_motion_notify_event;
  widget_class->button_press_event = gtk_variometer_button_press_event;

  g_type_class_add_private (obj_class, sizeof (GtkVariometerPrivate));

  g_object_class_install_property (obj_class,
                                   PROP_GRAYSCALE_COLOR,
                                   g_param_spec_boolean ("grayscale-color",
                                                         "use grayscale for the widget color",
                                                         "use grayscale for the widget color", FALSE,
                                                         G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_UNIT_IS_FEET,
                                   g_param_spec_boolean ("unit-is-feet",
                                                         "set the variometer unit to feet or meter",
                                                         "set the variometer unit to feet or meter",
                                                         TRUE, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_UNIT_STEP_VALUE,
                                   g_param_spec_int ("unit-step-value",
                                                     "select the value of the initial step (1, 10 or 100)",
                                                     "select the value of the initial step (1, 10 or 100)",
                                                     100, 1000, 100, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_RADIAL_COLOR,
                                   g_param_spec_boolean ("radial-color",
                                                         "the widget use radial color",
                                                         "the widget use radial color", TRUE, G_PARAM_WRITABLE));
  return;
}

/**
 * @fn static void gtk_variometer_init (GtkVariometer * vario)
 * @brief Special Gtk API function. Function called when the creating a<br>
 * new GtkVariometer. Allow to initialize some private variables of<br>
 * widget.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_variometer_init (GtkVariometer * vario)
{
  GtkVariometerPrivate *priv = NULL;

  if (gtk_variometer_debug)
  {
    g_debug ("===> gtk_variometer_init()");
  }
  g_return_if_fail (IS_GTK_VARIOMETER (vario));

  priv = GTK_VARIOMETER_GET_PRIVATE (vario);

  gtk_widget_add_events (GTK_WIDGET (vario), GDK_BUTTON_PRESS_MASK |
                         GDK_BUTTON_RELEASE_MASK | GDK_POINTER_MOTION_MASK | GDK_POINTER_MOTION_HINT_MASK);
  priv->b_mouse_onoff = FALSE;
  priv->grayscale_color = FALSE;
  priv->radial_color = TRUE;
  priv->unit_value = 1000;
  priv->altitude = 0;
  priv->l_time = 0;
  priv->altitude = 0;
  priv->l_altitude = 0;

  priv->bg_color_bounderie.red = 6553.5;        // 0.1 cairo
  priv->bg_color_bounderie.green = 6553.5;
  priv->bg_color_bounderie.blue = 6553.5;
  priv->bg_color_variometer.red = 3276.75;      // 0.05 cairo
  priv->bg_color_variometer.green = 3276.75;
  priv->bg_color_variometer.blue = 3276.75;
  priv->bg_color_inv.red = 45874.5;     // 0.7 cairo
  priv->bg_color_inv.green = 45874.5;
  priv->bg_color_inv.blue = 45874.5;
  priv->bg_radial_color_begin_bounderie.red = 13107;    // 0.2 cairo
  priv->bg_radial_color_begin_bounderie.green = 13107;
  priv->bg_radial_color_begin_bounderie.blue = 13107;
  priv->bg_radial_color_begin_variometer.red = 45874.5; // 0.7 cairo
  priv->bg_radial_color_begin_variometer.green = 45874.5;
  priv->bg_radial_color_begin_variometer.blue = 45874.5;
  return;
}

/**
 * @fn static gboolean gtk_variometer_configure_event (GtkWidget * widget, GdkEventConfigure * event)
 * @brief Special Gtk API function. Override the _configure_event handler<br>
 * in order to resize the widget when the main window is resized.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static gboolean gtk_variometer_configure_event (GtkWidget * widget, GdkEventConfigure * event)
{
  GtkVariometerPrivate *priv;
  GtkVariometer *vario = GTK_VARIOMETER (widget);

  if (gtk_variometer_debug)
  {
    g_debug ("===> gtk_variometer_configure_event()");
  }
  g_return_val_if_fail (IS_GTK_VARIOMETER (vario), FALSE);

  g_return_val_if_fail (event->type == GDK_CONFIGURE, FALSE);

  priv = GTK_VARIOMETER_GET_PRIVATE (vario);
  g_return_val_if_fail (priv != NULL, FALSE);

  if (gtk_variometer_debug)
  {
    g_debug ("===> gtk_variometer_configure_event(new width=%d, height=%d)", event->width, event->height);
  }

  if ((event->width < GTK_VARIOMETER_MODEL_X) || (event->height < GTK_VARIOMETER_MODEL_Y))
  {
    priv->plot_box.width = GTK_VARIOMETER_MODEL_X;
    priv->plot_box.height = GTK_VARIOMETER_MODEL_Y;
  }
  else
  {
    priv->plot_box.width = event->width;
    priv->plot_box.height = event->height;
  }

  priv->l_time = 0;

  if (gtk_variometer_debug)
  {
    g_debug ("cfg:Max.Avail: plot_box.width=%d, plot_box.height=%d", priv->plot_box.width, priv->plot_box.height);
  }
  return FALSE;
}

/**
 * @fn static gboolean gtk_variometer_expose (GtkWidget * vario, GdkEventExpose * event)
 * @brief Special Gtk API function. Override of the expose handler.<br>
 * An “expose-event” signal is emitted when the widget need to be drawn.<br>
 * A Cairo context is created for the parent's GdkWindow.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static gboolean gtk_variometer_expose (GtkWidget * vario, GdkEventExpose * event)
{
  GtkVariometerPrivate *priv;
  GtkWidget *widget = vario;

  cairo_t *cr = NULL;
  cairo_status_t status;

  if (gtk_variometer_debug)
  {
    g_debug ("===> gtk_variometer_expose()");
  }
  g_return_val_if_fail (IS_GTK_VARIOMETER (vario), FALSE);

  priv = GTK_VARIOMETER_GET_PRIVATE (vario);
  g_return_val_if_fail (priv != NULL, FALSE);

  priv->plot_box.width = widget->allocation.width;
  priv->plot_box.height = widget->allocation.height;

  if (gtk_variometer_debug)
  {
    g_debug ("gtk_variometer_expose(width=%d, height=%d)", widget->allocation.width, widget->allocation.height);
  }

  priv->cr = cr = gdk_cairo_create (widget->window);
  status = cairo_status (cr);
  if (status != CAIRO_STATUS_SUCCESS)
  {
    g_message ("GLG-Expose:cairo_create:status %d=%s", status, cairo_status_to_string (status));
  }

  cairo_rectangle (cr, 0, 0, priv->plot_box.width, priv->plot_box.height);
  cairo_clip (cr);

  gtk_variometer_draw (vario);

  cairo_destroy (cr);
  priv->cr = NULL;

  return FALSE;
}

/**
 * @fn extern void gtk_variometer_redraw (GtkVariometer * vario)
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
extern void gtk_variometer_redraw (GtkVariometer * vario)
{
  GtkWidget *widget;
  GdkRegion *region;

  if (gtk_variometer_debug)
  {
    g_debug ("===> gtk_variometer_redraw()");
  }
  g_return_if_fail (IS_GTK_VARIOMETER (vario));

  widget = GTK_WIDGET (vario);

  if (!widget->window)
    return;

  region = gdk_drawable_get_clip_region (widget->window);
  /* redraw the window completely by exposing it */
  gdk_window_invalidate_region (widget->window, region, TRUE);
  gdk_window_process_updates (widget->window, TRUE);

  gdk_region_destroy (region);
}

/**
 * @fn static inline double getTime() 
 * @brief Private widget's function allow to get the current time.
 * @return (double) current time
 */
static inline double getTime ()
{
  struct timeval tp;
  gettimeofday (&tp, NULL);
  return (double) tp.tv_sec + tp.tv_usec * 1e-6;
}

/**
 * @fn extern void gtk_variometer_set_alti (GtkVariometer * vario, gdouble alti)
 * @brief Public widget's function that allow the main program/user to<br>
 * set the internal value variable of the widget. 
 * 
 * "alti": double, define the altitude you want to display by the widget - the value is<br>
 * from 0 to 999999.
 * Note that this value need to be the same as the altimeter widget.
 */
extern void gtk_variometer_set_alti (GtkVariometer * vario, gdouble alti)
{
  GtkVariometerPrivate *priv;

  if (gtk_variometer_debug)
  {
    g_debug ("===> gtk_variometer_set_alti()");
  }
  g_return_if_fail (IS_GTK_VARIOMETER (vario));

  priv = GTK_VARIOMETER_GET_PRIVATE (vario);

  if ((alti >= 0) && (alti <= 999999))
  {
    if (priv->l_time == 0)
    {
      priv->l_time = getTime ();
      priv->l_altitude = alti;
      return;
    }

    double unit_per_min, now = getTime ();

    unit_per_min = (60 * (alti - priv->l_altitude)) / (now - priv->l_time);
    if (unit_per_min < 6 * priv->unit_value)
    {
      priv->altitude = (60 * (alti - priv->l_altitude)) / (now - priv->l_time);
    }
    else
    {
      priv->l_time = 0;
      return;
    }

    priv->l_time = now;
    priv->l_altitude = alti;
  }
  else
    g_warning ("GtkVariometer : gtk_variometer_set_alti : value out of range");

  return;
}

/**
 * @fn extern GtkWidget *gtk_variometer_new (void)
 * @brief Special Gtk API function. This function is simply a wrapper<br>
 * for convienience.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
extern GtkWidget *gtk_variometer_new (void)
{
  if (gtk_variometer_debug)
  {
    g_debug ("===> gtk_variometer_new()");
  }
  return GTK_WIDGET (gtk_type_new (gtk_variometer_get_type ()));
}

/**
 * @fn static void gtk_variometer_draw (GtkWidget * vario)
 * @brief Special Gtk API function. Override the _draw handler of the<br>
 * parent class GtkDrawingArea. This function use the cairo context<br>
 * created before in order to draw scalable graphics. 
 * 
 * See GObject,Cairo and GTK+ references for more informations: 
 * http://library.gnome.org/devel/references.html.en
 */
static void gtk_variometer_draw (GtkWidget * vario)
{
  GtkVariometerPrivate *priv;

  if (gtk_variometer_debug)
  {
    g_debug ("===> gtk_variometer_draw()");
  }
  g_return_if_fail (IS_GTK_VARIOMETER (vario));

  priv = GTK_VARIOMETER_GET_PRIVATE (vario);

  double x, y, rec_x0, rec_y0, rec_width, rec_height, rec_degrees;
  double rec_aspect, rec_corner_radius, rec_radius, radius;
  int i;

  x = vario->allocation.width / 2;
  y = vario->allocation.height / 2;
  radius = MIN (vario->allocation.width / 2, vario->allocation.height / 2) - 5;
  cairo_pattern_t *pat = NULL;

  rec_x0 = x - radius;
  rec_y0 = y - radius;
  rec_width = radius * 2;
  rec_height = radius * 2;
  rec_aspect = 1.0;
  rec_corner_radius = rec_height / 8.0;

  rec_radius = rec_corner_radius / rec_aspect;
  rec_degrees = M_PI / 180.0;

  // **** variometer base
  cairo_new_sub_path (priv->cr);
  cairo_arc (priv->cr, rec_x0 + rec_width - rec_radius, rec_y0 + rec_radius,
             rec_radius, -90 * rec_degrees, 0 * rec_degrees);
  cairo_arc (priv->cr, rec_x0 + rec_width - rec_radius, rec_y0 + rec_height - rec_radius,
             rec_radius, 0 * rec_degrees, 90 * rec_degrees);
  cairo_arc (priv->cr, rec_x0 + rec_radius, rec_y0 + rec_height - rec_radius,
             rec_radius, 90 * rec_degrees, 180 * rec_degrees);
  cairo_arc (priv->cr, rec_x0 + rec_radius, rec_y0 + rec_radius, rec_radius, 180 * rec_degrees, 270 * rec_degrees);
  cairo_close_path (priv->cr);

  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_bounderie.red / 65535,
                            (gdouble) priv->bg_color_bounderie.green / 65535,
                            (gdouble) priv->bg_color_bounderie.blue / 65535);
    else
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_inv.red / 65535,
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
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  cairo_arc (priv->cr, x, y, radius, 0, 2 * M_PI);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (priv->cr, 0., 0., 0.);
  else
    cairo_set_source_rgb (priv->cr, 1., 1., 1.);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  cairo_arc (priv->cr, x, y, radius - 0.04 * radius, 0, 2 * M_PI);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (priv->cr, 0.6, 0.5, 0.5);
  else
    cairo_set_source_rgb (priv->cr, 1 - 0.6, 1 - 0.5, 1 - 0.5);
  cairo_stroke (priv->cr);

  cairo_set_line_width (priv->cr, 0.01 * radius);
  radius = radius - 0.1 * radius;
  cairo_arc (priv->cr, x, y, radius, 0, 2 * M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_variometer.red / 65535,
                            (gdouble) priv->bg_color_variometer.green / 65535,
                            (gdouble) priv->bg_color_variometer.blue / 65535);
    else
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_variometer.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_variometer.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_variometer.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, (gdouble) priv->bg_color_variometer.red / 65535,
                                       (gdouble) priv->bg_color_variometer.green / 65535,
                                       (gdouble) priv->bg_color_variometer.blue / 65535, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  cairo_arc (priv->cr, x, y, radius, 0, 2 * M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_variometer.red / 65535,
                            (gdouble) priv->bg_color_variometer.green / 65535,
                            (gdouble) priv->bg_color_variometer.blue / 65535);
    else
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_variometer.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_variometer.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_variometer.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, (gdouble) priv->bg_color_variometer.red / 65535,
                                       (gdouble) priv->bg_color_variometer.green / 65535,
                                       (gdouble) priv->bg_color_variometer.blue / 65535, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill (priv->cr);
  cairo_stroke (priv->cr);


  // **** Variometer ticks
  if (!priv->grayscale_color)
    cairo_set_source_rgb (priv->cr, 1, 1, 1);
  else
    cairo_set_source_rgb (priv->cr, 0., 0., 0.);
  cairo_set_line_width (priv->cr, 0.023 * radius);
  for (i = -7; i <= 7; i++)
  {
    int inset = 0.12 * radius;
    cairo_save (priv->cr);

    cairo_move_to (priv->cr, x + (radius - inset) * cos (M_PI + i * (M_PI / 9 + M_PI / 38)),
                   y + (radius - inset) * sin (M_PI + i * (M_PI / 9 + M_PI / 38)));
    cairo_line_to (priv->cr, x + radius * cos (M_PI + i * (M_PI / 9 + M_PI / 38)),
                   y + radius * sin (M_PI + i * (M_PI / 9 + M_PI / 38)));
    cairo_stroke (priv->cr);
    cairo_restore (priv->cr);
  }

  cairo_set_line_width (priv->cr, 0.015 * radius);
  for (i = -10; i <= 10; i++)
  {
    int inset = 0.12 * radius / 3;
    cairo_save (priv->cr);

    cairo_move_to (priv->cr, x + (radius - 2 * inset) * cos (M_PI + i * (M_PI / 9 + M_PI / 38) / 5),
                   y + (radius - 2 * inset) * sin (M_PI + i * (M_PI / 9 + M_PI / 38) / 5));
    cairo_line_to (priv->cr, x + (radius - inset) * cos (M_PI + i * (M_PI / 9 + M_PI / 38) / 5),
                   y + (radius - inset) * sin (M_PI + i * (M_PI / 9 + M_PI / 38) / 5));
    cairo_stroke (priv->cr);
    cairo_restore (priv->cr);
  }
  for (i = 4; i <= 14; i++)
  {
    int inset = 0.12 * radius / 3;
    cairo_save (priv->cr);

    cairo_move_to (priv->cr, x + (radius - 2 * inset) * cos (M_PI + i * (M_PI / 9 + M_PI / 38) / 2),
                   y + (radius - 2 * inset) * sin (M_PI + i * (M_PI / 9 + M_PI / 38) / 2));
    cairo_line_to (priv->cr, x + (radius - inset) * cos (M_PI + i * (M_PI / 9 + M_PI / 38) / 2),
                   y + (radius - inset) * sin (M_PI + i * (M_PI / 9 + M_PI / 38) / 2));
    cairo_stroke (priv->cr);
    cairo_restore (priv->cr);
  }
  for (i = -4; i >= -14; i--)
  {
    int inset = 0.12 * radius / 3;
    cairo_save (priv->cr);

    cairo_move_to (priv->cr, x + (radius - 2 * inset) * cos (M_PI + i * (M_PI / 9 + M_PI / 38) / 2),
                   y + (radius - 2 * inset) * sin (M_PI + i * (M_PI / 9 + M_PI / 38) / 2));
    cairo_line_to (priv->cr, x + (radius - inset) * cos (M_PI + i * (M_PI / 9 + M_PI / 38) / 2),
                   y + (radius - inset) * sin (M_PI + i * (M_PI / 9 + M_PI / 38) / 2));
    cairo_stroke (priv->cr);
    cairo_restore (priv->cr);
  }

  cairo_set_line_width (priv->cr, 0.08 * radius);
  cairo_arc (priv->cr, x, y, radius - 0.05 * radius, M_PI + 7 * (M_PI / 9 + M_PI / 38),
             M_PI - 7 * (M_PI / 9 + M_PI / 38));
  if (!priv->grayscale_color)
    cairo_set_source_rgb (priv->cr, 1., 1., 1.);
  else
    cairo_set_source_rgb (priv->cr, 0., 0., 0.);
  cairo_stroke (priv->cr);

  // **** "up" drawing
  cairo_select_font_face (priv->cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size (priv->cr, 0.08 * radius);
  cairo_move_to (priv->cr, x - 0.55 * radius, y - 0.12 * radius);
  cairo_show_text (priv->cr, "UP");
  cairo_stroke (priv->cr);

  // **** "down" drawing
  cairo_move_to (priv->cr, x - 0.55 * radius, y + 0.17 * radius);
  cairo_show_text (priv->cr, "DOWN");
  cairo_stroke (priv->cr);

  // **** up/down arrow
  cairo_set_line_width (priv->cr, 0.01 * radius);
  cairo_new_sub_path (priv->cr);
  cairo_move_to (priv->cr, x - 0.6 * radius, y + 0.05 * radius);
  cairo_line_to (priv->cr, x - 0.6 * radius, y + 0.23 * radius);
  cairo_line_to (priv->cr, x - 0.62 * radius, y + 0.23 * radius);
  cairo_line_to (priv->cr, x - 0.6 * radius, y + 0.25 * radius);
  cairo_line_to (priv->cr, x - 0.58 * radius, y + 0.23 * radius);
  cairo_line_to (priv->cr, x - 0.6 * radius, y + 0.23 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_close_path (priv->cr);
  cairo_stroke (priv->cr);

  cairo_new_sub_path (priv->cr);
  cairo_move_to (priv->cr, x - 0.6 * radius, y - 0.05 * radius);
  cairo_line_to (priv->cr, x - 0.6 * radius, y - 0.23 * radius);
  cairo_line_to (priv->cr, x - 0.62 * radius, y - 0.23 * radius);
  cairo_line_to (priv->cr, x - 0.6 * radius, y - 0.25 * radius);
  cairo_line_to (priv->cr, x - 0.58 * radius, y - 0.23 * radius);
  cairo_line_to (priv->cr, x - 0.6 * radius, y - 0.23 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_close_path (priv->cr);
  cairo_stroke (priv->cr);

  // **** "Variometer" drawing
  cairo_select_font_face (priv->cr, "Sans", CAIRO_FONT_SLANT_OBLIQUE, CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size (priv->cr, 0.13 * radius);
  cairo_move_to (priv->cr, x - 0.34 * radius, y - 0.3 * radius);
  cairo_show_text (priv->cr, "Variometer");
  cairo_stroke (priv->cr);

  // **** infos drawing
  cairo_select_font_face (priv->cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size (priv->cr, 0.08 * radius);
  switch (priv->unit_value)
  {
    case 100:
      cairo_move_to (priv->cr, x - 0.35 * radius, y + 0.4 * radius);
      if (priv->unit_is_feet)
        cairo_show_text (priv->cr, "100 FEET PER MIN");
      else
        cairo_show_text (priv->cr, "100 METER PER MIN");
      break;
      cairo_stroke (priv->cr);
    case 1000:
      cairo_move_to (priv->cr, x - 0.35 * radius, y + 0.4 * radius);
      if (priv->unit_is_feet)
        cairo_show_text (priv->cr, "1000 FEET PER MIN");
      else
        cairo_show_text (priv->cr, "1000 METER PER MIN");
      cairo_stroke (priv->cr);
      break;
  }

  // Number drawing
  if (!priv->grayscale_color)
    cairo_set_source_rgb (priv->cr, 1, 1, 1);
  else
    cairo_set_source_rgb (priv->cr, 0., 0., 0.);
  int inset = 0.25 * radius;
  cairo_select_font_face (priv->cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size (priv->cr, 0.2 * radius);

  cairo_move_to (priv->cr, x - 0.065 * radius + (radius - inset) * cos (M_PI),
                 y + 0.07 * radius + (radius - inset) * sin (M_PI));
  cairo_show_text (priv->cr, "0");
  cairo_stroke (priv->cr);
  cairo_set_font_size (priv->cr, 0.1 * radius);
  cairo_move_to (priv->cr, x - 0.065 * radius + (radius - inset) * cos (M_PI + M_PI / 9 + M_PI / 38),
                 y + 0.03 * radius + (radius - inset) * sin (M_PI + M_PI / 9 + M_PI / 38));
  cairo_show_text (priv->cr, ".5");
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.065 * radius + (radius - inset) * cos (M_PI - (M_PI / 9 + M_PI / 38)),
                 y + 0.065 * radius + (radius - inset) * sin (M_PI - (M_PI / 9 + M_PI / 38)));
  cairo_show_text (priv->cr, ".5");
  cairo_stroke (priv->cr);
  cairo_set_font_size (priv->cr, 0.2 * radius);
  cairo_move_to (priv->cr, x - 0.065 * radius + (radius - inset) * cos (M_PI + 2 * (M_PI / 9 + M_PI / 38)),
                 y + 0.07 * radius + (radius - inset) * sin (M_PI + 2 * (M_PI / 9 + M_PI / 38)));
  cairo_show_text (priv->cr, "1");
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.065 * radius + (radius - inset) * cos (M_PI - 2 * (M_PI / 9 + M_PI / 38)),
                 y + 0.07 * radius + (radius - inset) * sin (M_PI - 2 * (M_PI / 9 + M_PI / 38)));
  cairo_show_text (priv->cr, "1");
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.065 * radius + (radius - inset) * cos (M_PI + 3 * (M_PI / 9 + M_PI / 38)),
                 y + 0.07 * radius + (radius - inset) * sin (M_PI + 3 * (M_PI / 9 + M_PI / 38)));
  cairo_show_text (priv->cr, "2");
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.065 * radius + (radius - inset) * cos (M_PI - 3 * (M_PI / 9 + M_PI / 38)),
                 y + 0.07 * radius + (radius - inset) * sin (M_PI - 3 * (M_PI / 9 + M_PI / 38)));
  cairo_show_text (priv->cr, "2");
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.065 * radius + (radius - inset) * cos (M_PI + 5 * (M_PI / 9 + M_PI / 38)),
                 y + 0.07 * radius + (radius - inset) * sin (M_PI + 5 * (M_PI / 9 + M_PI / 38)));
  cairo_show_text (priv->cr, "4");
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.065 * radius + (radius - inset) * cos (M_PI - 5 * (M_PI / 9 + M_PI / 38)),
                 y + 0.07 * radius + (radius - inset) * sin (M_PI - 5 * (M_PI / 9 + M_PI / 38)));
  cairo_show_text (priv->cr, "4");
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.065 * radius + (radius - inset) * cos (0),
                 y + 0.07 * radius + (radius - inset) * sin (0));
  cairo_show_text (priv->cr, "6");
  cairo_stroke (priv->cr);

  priv->radius = radius;
  priv->x = x;
  priv->y = y;
  gtk_variometer_draw_screws (vario);

  // draw hands
  gtk_variometer_draw_hand (vario);

  cairo_pattern_destroy (pat);
  return;
}

/**
 * @fn static void gtk_variometer_draw_screws (GtkWidget * vario)
 * @brief Private widget's function that draw the widget's screws using cairo.
 */
static void gtk_variometer_draw_screws (GtkWidget * vario)
{
  GtkVariometerPrivate *priv;

  if (gtk_variometer_debug)
  {
    g_debug ("===> gtk_variometer_draw()");
  }
  g_return_if_fail (IS_GTK_VARIOMETER (vario));

  priv = GTK_VARIOMETER_GET_PRIVATE (vario);

  cairo_pattern_t *pat = NULL;
  double x, y, radius;
  radius = priv->radius;
  x = priv->x;
  y = priv->y;
  radius = radius + 0.12 * radius;

  // **** top left screw
  cairo_arc (priv->cr, x - 0.82 * radius, y - 0.82 * radius, 0.1 * radius, 0, 2 * M_PI);
  pat = cairo_pattern_create_radial (x - 0.82 * radius, y - 0.82 * radius, 0.07 * radius,
                                     x - 0.82 * radius, y - 0.82 * radius, 0.1 * radius);
  cairo_pattern_add_color_stop_rgba (pat, 0, 0, 0, 0, 0.7);
  cairo_pattern_add_color_stop_rgba (pat, 1, 0, 0, 0, 0.1);
  cairo_set_source (priv->cr, pat);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  cairo_arc (priv->cr, x - 0.82 * radius, y - 0.82 * radius, 0.07 * radius, 0, 2 * M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_bounderie.red / 65535,
                            (gdouble) priv->bg_color_bounderie.green / 65535,
                            (gdouble) priv->bg_color_bounderie.blue / 65535);
    else
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_bounderie.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15, 0.15, 0.15, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  cairo_set_line_width (priv->cr, 0.02 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (priv->cr, 0., 0., 0.);
  else
    cairo_set_source_rgb (priv->cr, 1., 1., 1.);
  cairo_move_to (priv->cr, x - 0.88 * radius, y - 0.82 * radius);
  cairo_line_to (priv->cr, x - 0.76 * radius, y - 0.82 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.82 * radius, y - 0.88 * radius);
  cairo_line_to (priv->cr, x - 0.82 * radius, y - 0.76 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_set_line_width (priv->cr, 0.01 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (priv->cr, 0.1, 0.1, 0.1);
  else
    cairo_set_source_rgb (priv->cr, 0.9, 0.9, 0.9);
  cairo_move_to (priv->cr, x - 0.88 * radius, y - 0.82 * radius);
  cairo_line_to (priv->cr, x - 0.76 * radius, y - 0.82 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.82 * radius, y - 0.88 * radius);
  cairo_line_to (priv->cr, x - 0.82 * radius, y - 0.76 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  // **** top right screw
  cairo_arc (priv->cr, x + 0.82 * radius, y - 0.82 * radius, 0.1 * radius, 0, 2 * M_PI);
  pat = cairo_pattern_create_radial (x + 0.82 * radius, y - 0.82 * radius, 0.07 * radius,
                                     x + 0.82 * radius, y - 0.82 * radius, 0.1 * radius);
  cairo_pattern_add_color_stop_rgba (pat, 0, 0, 0, 0, 0.7);
  cairo_pattern_add_color_stop_rgba (pat, 1, 0, 0, 0, 0.1);
  cairo_set_source (priv->cr, pat);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  cairo_arc (priv->cr, x + 0.82 * radius, y - 0.82 * radius, 0.07 * radius, 0, 2 * M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_bounderie.red / 65535,
                            (gdouble) priv->bg_color_bounderie.green / 65535,
                            (gdouble) priv->bg_color_bounderie.blue / 65535);
    else
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_bounderie.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15, 0.15, 0.15, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  cairo_set_line_width (priv->cr, 0.02 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (priv->cr, 0., 0., 0.);
  else
    cairo_set_source_rgb (priv->cr, 1., 1., 1.);
  cairo_move_to (priv->cr, x + 0.88 * radius, y - 0.82 * radius);
  cairo_line_to (priv->cr, x + 0.76 * radius, y - 0.82 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x + 0.82 * radius, y - 0.88 * radius);
  cairo_line_to (priv->cr, x + 0.82 * radius, y - 0.76 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_set_line_width (priv->cr, 0.01 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (priv->cr, 0.1, 0.1, 0.1);
  else
    cairo_set_source_rgb (priv->cr, 0.9, 0.9, 0.9);
  cairo_move_to (priv->cr, x + 0.88 * radius, y - 0.82 * radius);
  cairo_line_to (priv->cr, x + 0.76 * radius, y - 0.82 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x + 0.82 * radius, y - 0.88 * radius);
  cairo_line_to (priv->cr, x + 0.82 * radius, y - 0.76 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  // **** bottom left screw
  cairo_arc (priv->cr, x - 0.82 * radius, y + 0.82 * radius, 0.1 * radius, 0, 2 * M_PI);
  pat = cairo_pattern_create_radial (x - 0.82 * radius, y + 0.82 * radius, 0.07 * radius,
                                     x - 0.82 * radius, y + 0.82 * radius, 0.1 * radius);
  cairo_pattern_add_color_stop_rgba (pat, 0, 0, 0, 0, 0.7);
  cairo_pattern_add_color_stop_rgba (pat, 1, 0, 0, 0, 0.1);
  cairo_set_source (priv->cr, pat);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  cairo_arc (priv->cr, x - 0.82 * radius, y + 0.82 * radius, 0.07 * radius, 0, 2 * M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_bounderie.red / 65535,
                            (gdouble) priv->bg_color_bounderie.green / 65535,
                            (gdouble) priv->bg_color_bounderie.blue / 65535);
    else
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_bounderie.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15, 0.15, 0.15, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  cairo_set_line_width (priv->cr, 0.02 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (priv->cr, 0., 0., 0.);
  else
    cairo_set_source_rgb (priv->cr, 1., 1., 1.);
  cairo_move_to (priv->cr, x - 0.88 * radius, y + 0.82 * radius);
  cairo_line_to (priv->cr, x - 0.76 * radius, y + 0.82 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.82 * radius, y + 0.88 * radius);
  cairo_line_to (priv->cr, x - 0.82 * radius, y + 0.76 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_set_line_width (priv->cr, 0.01 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (priv->cr, 0.1, 0.1, 0.1);
  else
    cairo_set_source_rgb (priv->cr, 0.9, 0.9, 0.9);
  cairo_move_to (priv->cr, x - 0.88 * radius, y + 0.82 * radius);
  cairo_line_to (priv->cr, x - 0.76 * radius, y + 0.82 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.82 * radius, y + 0.88 * radius);
  cairo_line_to (priv->cr, x - 0.82 * radius, y + 0.76 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  // **** bottom right screw
  cairo_arc (priv->cr, x + 0.82 * radius, y + 0.82 * radius, 0.1 * radius, 0, 2 * M_PI);
  pat = cairo_pattern_create_radial (x + 0.82 * radius, y + 0.82 * radius, 0.07 * radius,
                                     x + 0.82 * radius, y + 0.82 * radius, 0.1 * radius);
  cairo_pattern_add_color_stop_rgba (pat, 0, 0, 0, 0, 0.7);
  cairo_pattern_add_color_stop_rgba (pat, 1, 0, 0, 0, 0.1);
  cairo_set_source (priv->cr, pat);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  cairo_arc (priv->cr, x + 0.82 * radius, y + 0.82 * radius, 0.07 * radius, 0, 2 * M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_bounderie.red / 65535,
                            (gdouble) priv->bg_color_bounderie.green / 65535,
                            (gdouble) priv->bg_color_bounderie.blue / 65535);
    else
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_bounderie.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_bounderie.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15, 0.15, 0.15, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  cairo_set_line_width (priv->cr, 0.02 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (priv->cr, 0., 0., 0.);
  else
    cairo_set_source_rgb (priv->cr, 1., 1., 1.);
  cairo_move_to (priv->cr, x + 0.88 * radius, y + 0.82 * radius);
  cairo_line_to (priv->cr, x + 0.76 * radius, y + 0.82 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x + 0.82 * radius, y + 0.88 * radius);
  cairo_line_to (priv->cr, x + 0.82 * radius, y + 0.76 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_set_line_width (priv->cr, 0.01 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (priv->cr, 0.1, 0.1, 0.1);
  else
    cairo_set_source_rgb (priv->cr, 0.9, 0.9, 0.9);
  cairo_move_to (priv->cr, x + 0.88 * radius, y + 0.82 * radius);
  cairo_line_to (priv->cr, x + 0.76 * radius, y + 0.82 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x + 0.82 * radius, y + 0.88 * radius);
  cairo_line_to (priv->cr, x + 0.82 * radius, y + 0.76 * radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_pattern_destroy (pat);
  return;
}

/**
 * @fn static void gtk_variometer_draw_hand (GtkWidget * vario)
 * @brief Private widget's function that draw the variometer hand using cairo.
 */
static void gtk_variometer_draw_hand (GtkWidget * vario)
{
  GtkVariometerPrivate *priv;
  if (gtk_variometer_debug)
  {
    g_debug ("===> gtk_variometer_draw_hands()");
  }
  g_return_if_fail (IS_GTK_VARIOMETER (vario));

  priv = GTK_VARIOMETER_GET_PRIVATE (vario);

  double x = priv->x;
  double y = priv->y;
  double radius = priv->radius;
  int altitude = priv->altitude;

  // **** centre cercle 
  cairo_save (priv->cr);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (priv->cr, 0, 0, 0);
  else
    cairo_set_source_rgb (priv->cr, 1, 1, 1);
  cairo_arc (priv->cr, x, y, radius - 0.9 * radius, 0, 2 * M_PI);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (priv->cr, 0.2, 0.2, 0.2);
  else
    cairo_set_source_rgb (priv->cr, 0.8, 0.8, 0.8);
  cairo_arc (priv->cr, x, y, radius - 0.9 * radius, 0, 2 * M_PI);
  cairo_stroke (priv->cr);
  cairo_restore (priv->cr);

  // **** gauge hand
  if (priv->altitude >= 1000)
    altitude += 1000;
  else if (priv->altitude <= -1000)
    altitude -= 1000;

  if ((priv->altitude >= 1000) || (priv->altitude <= -1000))
  {
    cairo_save (priv->cr);
    if (!priv->grayscale_color)
      cairo_set_source_rgb (priv->cr, 1, 1, 1);
    else
      cairo_set_source_rgb (priv->cr, 0, 0, 0);
    cairo_set_line_width (priv->cr, 0.02 * radius);
    cairo_move_to (priv->cr, x, y);
    cairo_line_to (priv->cr,
                   x + (radius - 0.2 * radius) * cos (M_PI + altitude * ((M_PI / 9 + M_PI / 38) / 1000)),
                   y + (radius - 0.2 * radius) * sin (M_PI + altitude * ((M_PI / 9 + M_PI / 38) / 1000)));
    cairo_stroke (priv->cr);
    cairo_restore (priv->cr);

    cairo_save (priv->cr);
    if (!priv->grayscale_color)
      cairo_set_source_rgb (priv->cr, 1, 1, 1);
    else
      cairo_set_source_rgb (priv->cr, 0, 0, 0);
    cairo_set_line_width (priv->cr, 0.02 * radius);
    cairo_move_to (priv->cr, x, y);
    cairo_line_to (priv->cr,
                   x - (radius - 0.9 * radius) * cos (M_PI + altitude * ((M_PI / 9 + M_PI / 38) / 1000)),
                   y - (radius - 0.9 * radius) * sin (M_PI + altitude * ((M_PI / 9 + M_PI / 38) / 1000)));
    cairo_stroke (priv->cr);
    cairo_restore (priv->cr);
  }
  else
  {
    cairo_save (priv->cr);
    if (!priv->grayscale_color)
      cairo_set_source_rgb (priv->cr, 1, 1, 1);
    else
      cairo_set_source_rgb (priv->cr, 0, 0, 0);
    cairo_set_line_width (priv->cr, 0.02 * radius);
    cairo_move_to (priv->cr, x, y);
    cairo_line_to (priv->cr,
                   x + (radius - 0.2 * radius) * cos (M_PI + altitude * ((M_PI / (9 / 2) + M_PI / 38) / 1000)),
                   y + (radius - 0.2 * radius) * sin (M_PI + altitude * ((M_PI / (9 / 2) + M_PI / 38) / 1000)));
    cairo_stroke (priv->cr);
    cairo_restore (priv->cr);

    cairo_save (priv->cr);
    if (!priv->grayscale_color)
      cairo_set_source_rgb (priv->cr, 1, 1, 1);
    else
      cairo_set_source_rgb (priv->cr, 0, 0, 0);
    cairo_set_line_width (priv->cr, 0.02 * radius);
    cairo_move_to (priv->cr, x, y);
    cairo_line_to (priv->cr,
                   x - (radius - 0.9 * radius) * cos (M_PI + altitude * ((M_PI / (9 / 2) + M_PI / 38) / 1000)),
                   y - (radius - 0.9 * radius) * sin (M_PI + altitude * ((M_PI / (9 / 2) + M_PI / 38) / 1000)));
    cairo_stroke (priv->cr);
    cairo_restore (priv->cr);
  }
  return;
}

/**
 * @fn static gboolean gtk_variometer_button_press_event (GtkWidget * widget, GdkEventButton * ev)
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
static gboolean gtk_variometer_button_press_event (GtkWidget * widget, GdkEventButton * ev)
{
  GtkVariometerPrivate *priv;
  gint x = 0, y = 0;

  if (gtk_variometer_debug)
  {
    g_debug ("===> gtk_variometer_button_press_event_cb()");
  }
  g_return_val_if_fail (IS_GTK_VARIOMETER (widget), FALSE);

  priv = GTK_VARIOMETER_GET_PRIVATE (widget);

  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 1))
  {
    gdk_window_get_pointer (ev->window, &x, &y, &priv->mouse_state);
    priv->mouse_pos.x = x;
    priv->mouse_pos.y = y;
    return TRUE;
  }
  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 2) && priv->b_mouse_onoff)
  {
    gtk_variometer_debug = gtk_variometer_debug ? FALSE : TRUE;
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
 * @fn static gboolean gtk_variometer_motion_notify_event (GtkWidget * widget, GdkEventMotion * ev)
 * @brief Special Gtk API function. Override the _motion_notify_event<br>
 * handler. Perform mouse motion events. 
 * 
 * Here, the mouse events are not used (maybe in future released).
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static gboolean gtk_variometer_motion_notify_event (GtkWidget * widget, GdkEventMotion * ev)
{
  GtkVariometerPrivate *priv;
  GdkModifierType state;
  gint x = 0, y = 0;

  if (gtk_variometer_debug)
  {
    g_debug ("===> gtk_variometer_motion_notify_event_cb()");
  }
  g_return_val_if_fail (IS_GTK_VARIOMETER (widget), FALSE);

  priv = GTK_VARIOMETER_GET_PRIVATE (widget);

  if (ev->is_hint)
  {
    gdk_window_get_pointer (ev->window, &x, &y, &state);
  }
  else
  {
    x = ev->x;
    y = ev->y;
    state = ev->state;
  }

  /* save mousse coordinates */
  priv->mouse_pos.x = x;
  priv->mouse_pos.y = y;
  priv->mouse_state = state;

  if (gtk_variometer_debug)
  {
    g_debug ("===> gtk_variometer_motion_notify_event_cb() : mouse x=%d, y=%d", x, y);
  }

  return TRUE;
}

/**
 * @fn static void gtk_variometer_destroy (GtkObject * object)
 * @brief Special Gtk API function. Override the _destroy handler.<br>
 * Allow the destruction of all widget's pointer.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_variometer_destroy (GtkObject * object)
{
  GtkVariometerPrivate *priv = NULL;
  GtkWidget *widget = NULL;

  if (gtk_variometer_debug)
  {
    g_debug ("===> gtk_variometer_destroy(enter)");
  }

  g_return_if_fail (object != NULL);

  widget = GTK_WIDGET (object);

  g_return_if_fail (IS_GTK_VARIOMETER (widget));

  priv = GTK_VARIOMETER_GET_PRIVATE (widget);
  g_return_if_fail (priv != NULL);

  if (priv->cr)
  {
    g_free (priv->cr);

    if (GTK_OBJECT_CLASS (gtk_variometer_parent_class)->destroy != NULL)
    {
      (*GTK_OBJECT_CLASS (gtk_variometer_parent_class)->destroy) (object);
    }
  }
  if (gtk_variometer_debug)
  {
    g_debug ("gtk_variometer_destroy(exit)");
  }

  return;
}

/**
 * @fn static void gtk_variometer_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec)
 * @brief Special Gtk API function. Override the _set_property handler <br>
 * in order to set the object parameters.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_variometer_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec)
{
  GtkVariometerPrivate *priv = NULL;
  GtkVariometer *vario = NULL;

  if (gtk_variometer_debug)
  {
    g_debug ("===> gtk_variometer_set_property()");
  }
  g_return_if_fail (object != NULL);

  vario = GTK_VARIOMETER (object);
  g_return_if_fail (IS_GTK_VARIOMETER (vario));

  priv = GTK_VARIOMETER_GET_PRIVATE (vario);
  g_return_if_fail (priv != NULL);

  switch (prop_id)
  {
    case PROP_GRAYSCALE_COLOR:
      priv->grayscale_color = g_value_get_boolean (value);
      break;
    case PROP_UNIT_IS_FEET:
      priv->unit_is_feet = g_value_get_boolean (value);
      break;
    case PROP_UNIT_STEP_VALUE:
      priv->unit_value = g_value_get_int (value);
      if ((priv->unit_value != 100) && (priv->unit_value != 1000))
      {
        priv->unit_value = 100;
        g_warning ("GtkVariometer: gtk_variometer_set_property: unit-step-value out of range");
      }
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
