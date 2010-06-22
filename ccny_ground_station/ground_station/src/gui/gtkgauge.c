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

#include <ground_station/gui/gtkgauge.h>

typedef struct _GtkGaugePrivate
{
  /* new cairo design */
  cairo_t *cr;
  GdkRectangle plot_box;

  /* widget data */
  gint start_value;
  gint end_value;
  gint initial_step;
  gdouble sub_step;
  gint drawing_step;
  gboolean color_mode_inv;
  gboolean radial_color;
  gdouble value;
  gchar *gauge_name;

  /* drawing data */
  gdouble x;
  gdouble y;
  gdouble radius;
  GdkColor bg_color_inv;
  GdkColor bg_color_gauge;
  GdkColor bg_color_bounderie;
  GdkColor bg_radial_color_begin_gauge;
  GdkColor bg_radial_color_begin_bounderie;

  gchar *strip_color_order;
  gboolean gauge_name_font_size_ok;
  gdouble gauge_name_font_size;
  gdouble green_strip_start;
  gdouble yellow_strip_start;
  gdouble orange_strip_start;
  gdouble red_strip_start;

  /* mouse information */
  gboolean b_mouse_onoff;
  GdkPoint mouse_pos;
  GdkModifierType mouse_state;

} GtkGaugePrivate;

enum _GLG_PROPERTY_ID
{
  PROP_0,
  PROP_NAME,
  PROP_INVERSED_COLOR,
  PROP_RADIAL_COLOR,
  PROP_START_VALUE,
  PROP_END_VALUE,
  PROP_INITIAL_STEP,
  PROP_SUB_STEP,
  PROP_DRAWING_STEP,
  PROP_STRIP_COLOR_ORDER,
  PROP_GREEN_STRIP_START,
  PROP_YELLOW_STRIP_START,
  PROP_ORANGE_STRIP_START,
  PROP_RED_STRIP_START,
} GLG_PROPERTY_ID;

G_DEFINE_TYPE (GtkGauge, gtk_gauge, GTK_TYPE_DRAWING_AREA);

#define GTK_GAUGE_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GTK_GAUGE_TYPE, GtkGaugePrivate))

static void gtk_gauge_class_init (GtkGaugeClass * klass);
static void gtk_gauge_init (GtkGauge * gauge);
static void gtk_gauge_destroy (GtkObject * object);
static void gtk_gauge_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec);

static gboolean gtk_gauge_configure_event (GtkWidget * widget, GdkEventConfigure * event);
static gboolean gtk_gauge_expose (GtkWidget * graph, GdkEventExpose * event);
static gboolean gtk_gauge_button_press_event (GtkWidget * widget, GdkEventButton * ev);
static gboolean gtk_gauge_motion_notify_event (GtkWidget * widget, GdkEventMotion * ev);

static void gtk_gauge_draw (GtkWidget * gauge);
static void gtk_gauge_draw_name (GtkGauge * gauge, gchar * pch_text, GdkRectangle * rect);
static void gtk_gauge_draw_hand (GtkWidget * gauge);

static gboolean gtk_gauge_debug = FALSE;

static void gtk_gauge_class_init (GtkGaugeClass * klass)
{
  GObjectClass *obj_class = G_OBJECT_CLASS (klass);
  GtkWidgetClass *widget_class = GTK_WIDGET_CLASS (klass);
  GtkObjectClass *gtkobject_class = GTK_OBJECT_CLASS (klass);

  if (gtk_gauge_debug)
  {
    g_debug ("===> gtk_gauge_class_init()");
  }

  /* GObject signal overrides */
  obj_class->set_property = gtk_gauge_set_property;

  /* GtkObject signal overrides */
  gtkobject_class->destroy = gtk_gauge_destroy;

  /* GtkWidget signals overrides */
  widget_class->configure_event = gtk_gauge_configure_event;
  widget_class->expose_event = gtk_gauge_expose;
  widget_class->motion_notify_event = gtk_gauge_motion_notify_event;
  widget_class->button_press_event = gtk_gauge_button_press_event;

  g_type_class_add_private (obj_class, sizeof (GtkGaugePrivate));


  g_object_class_install_property (obj_class,
                                   PROP_NAME,
                                   g_param_spec_string ("name",
                                                        "Gauge name",
                                                        "Gauge name", "...Name not set...", G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_INVERSED_COLOR,
                                   g_param_spec_boolean ("inverse-color",
                                                         "inverse or not the widget color",
                                                         "inverse or not the widget color", FALSE, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_RADIAL_COLOR,
                                   g_param_spec_boolean ("radial_color",
                                                         "the widget use radial color",
                                                         "the widget use radial color", TRUE, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_START_VALUE,
                                   g_param_spec_int ("start-value",
                                                     "set the start value of the gauge",
                                                     "set the start value of the gauge",
                                                     0, 10000, 0, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_END_VALUE,
                                   g_param_spec_int ("end-value",
                                                     "set the end value of the gauge",
                                                     "set the end value of the gauge",
                                                     0, 10000, 100, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_INITIAL_STEP,
                                   g_param_spec_int ("initial-step",
                                                     "set the initial step of the gauge",
                                                     "set the initial step of the gauge",
                                                     0, 10000, 10, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_SUB_STEP,
                                   g_param_spec_double ("sub-step",
                                                        "set the sub step of the gauge",
                                                        "set the sub step of the gauge", 0.0, 10000.0, 2.0,
                                                        G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class, PROP_DRAWING_STEP,
                                   g_param_spec_int ("drawing-step", "set the sub step of the gauge",
                                                     "set the sub step of the gauge", 0, 10000, 10, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class, PROP_STRIP_COLOR_ORDER,
                                   g_param_spec_string ("color-strip-order",
                                                        "color order for gauge strip (YOR,GYR,RYG,ROY)",
                                                        "color order for gauge strip (YOR,GYR,RYG,ROY)", "YOR",
                                                        G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class, PROP_GREEN_STRIP_START,
                                   g_param_spec_int ("green-strip-start", "set the green strip start of the gauge",
                                                     "set the green strip start of the gauge", 0, 10000, 0,
                                                     G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class, PROP_YELLOW_STRIP_START,
                                   g_param_spec_int ("yellow-strip-start", "set the yellow strip start of the gauge",
                                                     "set the yellow strip start of the gauge", 0, 10000, 0,
                                                     G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class, PROP_ORANGE_STRIP_START,
                                   g_param_spec_int ("orange-strip-start", "set the orange strip start of the gauge",
                                                     "set the orange strip start of the gauge", 0, 10000, 0,
                                                     G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class, PROP_RED_STRIP_START,
                                   g_param_spec_int ("red-strip-start", "set the red strip start of the gauge",
                                                     "set the red strip start of the gauge", 0, 10000, 0,
                                                     G_PARAM_WRITABLE));
  return;
}

static void gtk_gauge_init (GtkGauge * gauge)
{
  GtkGaugePrivate *priv = NULL;

  if (gtk_gauge_debug)
  {
    g_debug ("===> gtk_gauge_init()");
  }
  g_return_if_fail (IS_GTK_GAUGE (gauge));

  priv = GTK_GAUGE_GET_PRIVATE (gauge);

  gtk_widget_add_events (GTK_WIDGET (gauge), GDK_BUTTON_PRESS_MASK |
                         GDK_BUTTON_RELEASE_MASK | GDK_POINTER_MOTION_MASK | GDK_POINTER_MOTION_HINT_MASK);
  priv->b_mouse_onoff = FALSE;
  priv->gauge_name_font_size_ok = FALSE;

  priv->gauge_name = g_strdup ("**NO TITLE SET**");
  priv->strip_color_order = g_strdup ("YOR");

  priv->start_value = -1;
  priv->end_value = -1;
  priv->initial_step = -1;
  priv->sub_step = -1;
  priv->drawing_step = -1;
  priv->green_strip_start = -1;
  priv->yellow_strip_start = -1;
  priv->orange_strip_start = -1;
  priv->red_strip_start = -1;

  priv->color_mode_inv = TRUE;
  priv->radial_color = FALSE;
  priv->value = 0;

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

static gboolean gtk_gauge_configure_event (GtkWidget * widget, GdkEventConfigure * event)
{
  GtkGaugePrivate *priv;
  GtkGauge *gauge = GTK_GAUGE (widget);

  if (gtk_gauge_debug)
  {
    g_debug ("===> gtk_gauge_configure_event()");
  }
  g_return_val_if_fail (IS_GTK_GAUGE (gauge), FALSE);

  g_return_val_if_fail (event->type == GDK_CONFIGURE, FALSE);

  priv = GTK_GAUGE_GET_PRIVATE (gauge);
  g_return_val_if_fail (priv != NULL, FALSE);

  if (gtk_gauge_debug)
  {
    g_debug ("===> gtk_gauge_configure_event(new width=%d, height=%d)", event->width, event->height);
  }

  if ((event->width < GTK_GAUGE_MODEL_X) || (event->height < GTK_GAUGE_MODEL_Y))
  {
    priv->plot_box.width = GTK_GAUGE_MODEL_X;
    priv->plot_box.height = GTK_GAUGE_MODEL_Y;
  }
  else
  {
    priv->plot_box.width = event->width;
    priv->plot_box.height = event->height;
  }

  // **** force the font size research
  priv->gauge_name_font_size_ok = FALSE;

  if (gtk_gauge_debug)
  {
    g_debug ("cfg:Max.Avail: plot_box.width=%d, plot_box.height=%d", priv->plot_box.width, priv->plot_box.height);
  }
  return FALSE;
}

static gboolean gtk_gauge_expose (GtkWidget * gauge, GdkEventExpose * event)
{
  GtkGaugePrivate *priv;
  GtkWidget *widget = gauge;

  cairo_t *cr = NULL;
  cairo_status_t status;

  if (gtk_gauge_debug)
  {
    g_debug ("===> gtk_gauge_expose()");
  }
  g_return_val_if_fail (IS_GTK_GAUGE (gauge), FALSE);

  priv = GTK_GAUGE_GET_PRIVATE (gauge);
  g_return_val_if_fail (priv != NULL, FALSE);

  priv->plot_box.width = widget->allocation.width;
  priv->plot_box.height = widget->allocation.height;

  if (gtk_gauge_debug)
  {
    g_debug ("gtk_gauge_expose(width=%d, height=%d)", widget->allocation.width, widget->allocation.height);
  }

  priv->cr = cr = gdk_cairo_create (widget->window);
  status = cairo_status (cr);
  if (status != CAIRO_STATUS_SUCCESS)
  {
    g_message ("GLG-Expose:cairo_create:status %d=%s", status, cairo_status_to_string (status));
  }

  cairo_rectangle (cr, 0, 0, priv->plot_box.width, priv->plot_box.height);
  cairo_clip (cr);

  gtk_gauge_draw (gauge);

  cairo_destroy (cr);
  priv->cr = NULL;

  return FALSE;
}

extern void gtk_gauge_redraw (GtkGauge * gauge)
{
  GtkWidget *widget;
  GdkRegion *region;

  if (gtk_gauge_debug)
  {
    g_debug ("===> gtk_gauge_redraw()");
  }
  g_return_if_fail (IS_GTK_GAUGE (gauge));

  widget = GTK_WIDGET (gauge);

  if (!widget->window)
    return;

  region = gdk_drawable_get_clip_region (widget->window);
  /* redraw the window completely by exposing it */
  gdk_window_invalidate_region (widget->window, region, TRUE);
  gdk_window_process_updates (widget->window, TRUE);

  gdk_region_destroy (region);
}

extern void gtk_gauge_set_value (GtkGauge * gauge, gdouble val)
{
  GtkGaugePrivate *priv;

  if (gtk_gauge_debug)
  {
    g_debug ("===> gtk_gauge_draw()");
  }
  g_return_if_fail (IS_GTK_GAUGE (gauge));

  priv = GTK_GAUGE_GET_PRIVATE (gauge);
  priv->value = val;
}

extern GtkWidget *gtk_gauge_new (void)
{
  if (gtk_gauge_debug)
  {
    g_debug ("===> gtk_gauge_new()");
  }
  return GTK_WIDGET (gtk_type_new (gtk_gauge_get_type ()));
}

static void gtk_gauge_draw (GtkWidget * gauge)
{
  GtkGaugePrivate *priv;

  if (gtk_gauge_debug)
  {
    g_debug ("===> gtk_gauge_draw()");
  }
  g_return_if_fail (IS_GTK_GAUGE (gauge));

  priv = GTK_GAUGE_GET_PRIVATE (gauge);

  // **** Test if the minimum parameters are set
  if ((priv->start_value == -1) || (priv->end_value == -1) || (priv->initial_step == -1)
      || (priv->drawing_step == -1) || (priv->sub_step == -1))
  {
    g_warning ("GTK_GAUGE : minimal parameters not set, unable to paint.");
    return;
  }

  if ((priv->end_value == 0) || (priv->initial_step == 0) || (priv->drawing_step == 0) || (priv->sub_step == 0))
  {
    g_warning ("GTK_GAUGE : one of the minimal parameters is set to 0, unable to paint.");
    return;
  }

  GdkRectangle name_box;
  double x, y, rec_x0, rec_y0, rec_width, rec_height, rec_degrees;
  double rec_aspect, rec_corner_radius, rec_radius, radius;
  double alpha = 0, alpha_inv = 1, alpha_step = 0;
  char str[GTK_GAUGE_MAX_STRING];
  int i, strip_c_order = 0;
  cairo_pattern_t *pat;

  x = gauge->allocation.width / 2;
  y = gauge->allocation.height / 2;
  radius = MIN (gauge->allocation.width / 2, gauge->allocation.height / 2) - 5;

  rec_x0 = x - radius;
  rec_y0 = y - radius;
  rec_width = radius * 2;
  rec_height = radius * 2;
  rec_aspect = 1.0;
  rec_corner_radius = rec_height / 8.0;

  rec_radius = rec_corner_radius / rec_aspect;
  rec_degrees = M_PI / 180.0;

  // **** Gauge base
  cairo_new_sub_path (priv->cr);
  cairo_arc (priv->cr, rec_x0 + rec_width - rec_radius, rec_y0 + rec_radius,
             rec_radius, -90 * rec_degrees, 0 * rec_degrees);
  cairo_arc (priv->cr, rec_x0 + rec_width - rec_radius, rec_y0 + rec_height - rec_radius,
             rec_radius, 0 * rec_degrees, 90 * rec_degrees);
  cairo_arc (priv->cr, rec_x0 + rec_radius, rec_y0 + rec_height - rec_radius,
             rec_radius, 90 * rec_degrees, 180 * rec_degrees);
  cairo_arc (priv->cr, rec_x0 + rec_radius, rec_y0 + rec_radius, rec_radius, 180 * rec_degrees, 270 * rec_degrees);
  cairo_close_path (priv->cr);

  if (((priv->radial_color) && (priv->color_mode_inv)) || ((!priv->radial_color) && (priv->color_mode_inv))
      || ((!priv->radial_color) && (!priv->color_mode_inv)))
  {
    if (!priv->color_mode_inv)
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
  if (!priv->color_mode_inv)
    cairo_set_source_rgb (priv->cr, 0., 0., 0.);
  else
    cairo_set_source_rgb (priv->cr, 1., 1., 1.);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  cairo_arc (priv->cr, x, y, radius - 0.04 * radius, 0, 2 * M_PI);
  if (!priv->color_mode_inv)
    cairo_set_source_rgb (priv->cr, 0.6, 0.5, 0.5);
  else
    cairo_set_source_rgb (priv->cr, 1 - 0.6, 1 - 0.5, 1 - 0.5);
  cairo_stroke (priv->cr);

  cairo_set_line_width (priv->cr, 0.01 * radius);
  radius = radius - 0.1 * radius;
  cairo_arc (priv->cr, x, y, radius, 0, 2 * M_PI);
  if (((priv->radial_color) && (priv->color_mode_inv)) || ((!priv->radial_color) && (priv->color_mode_inv))
      || ((!priv->radial_color) && (!priv->color_mode_inv)))
  {
    if (!priv->color_mode_inv)
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_gauge.red / 65535,
                            (gdouble) priv->bg_color_gauge.green / 65535, (gdouble) priv->bg_color_gauge.blue / 65535);
    else
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_gauge.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_gauge.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_gauge.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, (gdouble) priv->bg_color_gauge.red / 65535,
                                       (gdouble) priv->bg_color_gauge.green / 65535,
                                       (gdouble) priv->bg_color_gauge.blue / 65535, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  // **** Gauge color strips 
  if (!g_strcmp0 (priv->strip_color_order, "YOR"))
  {
    strip_c_order = 0;
    if ((priv->yellow_strip_start == -1) || (priv->orange_strip_start == -1) || (priv->red_strip_start == -1))
    {
      priv->yellow_strip_start = 0;
      priv->orange_strip_start = priv->end_value / 3;
      priv->red_strip_start = 2 * priv->end_value / 3;
    }
  }
  else if (!g_strcmp0 (priv->strip_color_order, "GYR"))
  {
    strip_c_order = 1;
    if ((priv->yellow_strip_start == -1) || (priv->orange_strip_start == -1) || (priv->red_strip_start == -1))
    {
      priv->green_strip_start = 0;
      priv->yellow_strip_start = priv->end_value / 3;
      priv->red_strip_start = 2 * priv->end_value / 3;
    }
  }
  else if (!g_strcmp0 (priv->strip_color_order, "ROY"))
  {
    strip_c_order = 2;
    if ((priv->yellow_strip_start == -1) || (priv->orange_strip_start == -1) || (priv->red_strip_start == -1))
    {
      priv->red_strip_start = 0;
      priv->orange_strip_start = priv->end_value / 3;
      priv->yellow_strip_start = 2 * priv->end_value / 3;
    }
  }
  else if (!g_strcmp0 (priv->strip_color_order, "RYG"))
  {
    strip_c_order = 3;
    if ((priv->yellow_strip_start == -1) || (priv->orange_strip_start == -1) || (priv->red_strip_start == -1))
    {
      priv->red_strip_start = 0;
      priv->yellow_strip_start = priv->end_value / 3;
      priv->green_strip_start = 2 * priv->end_value / 3;
    }
  }

  for (i = 0; i < ((priv->end_value - priv->start_value) / priv->sub_step); i++)
  {
    cairo_save (priv->cr);
    switch (strip_c_order)
    {
      case 1:                  // **** Green - Yellow - Red
        alpha_step = fabs (1 / ((priv->yellow_strip_start - priv->green_strip_start) / priv->sub_step));
        if ((i * priv->sub_step >= priv->green_strip_start) && (i * priv->sub_step < priv->yellow_strip_start))
        {
          alpha = alpha + alpha_step;
          cairo_set_source_rgba (priv->cr, 0, 0.65, 0, alpha);
        }
        else if ((i * priv->sub_step >= priv->yellow_strip_start) && (i * priv->sub_step < priv->red_strip_start))
        {
          cairo_set_source_rgb (priv->cr, 1, 1, 0);
        }
        else if (i * priv->sub_step >= priv->red_strip_start)
        {
          cairo_set_line_width (priv->cr, 0.1 * radius);
          cairo_set_source_rgba (priv->cr, 1., 0., 0., 0.2);
          cairo_arc (priv->cr, x, y, radius - 0.23 * radius,
                     -5 * M_PI / 4 +
                     i * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step)),
                     -5 * M_PI / 4 + (i +
                                      1) * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step)));
          cairo_stroke (priv->cr);
          cairo_set_source_rgb (priv->cr, 1, 0., 0);
        }
        cairo_set_line_width (priv->cr, 0.12 * radius);
        cairo_arc (priv->cr, x, y, radius - 0.06 * radius,
                   -5 * M_PI / 4 +
                   i * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step)),
                   -5 * M_PI / 4 + (i +
                                    1) * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step)));
        cairo_stroke (priv->cr);
        break;
      case 2:                  // **** Red - Orange - Yellow
        alpha_step = fabs (1 / ((priv->end_value - priv->yellow_strip_start) / priv->sub_step));
        if ((i * priv->sub_step >= priv->red_strip_start) && (i * priv->sub_step < priv->orange_strip_start))
        {
          cairo_set_line_width (priv->cr, 0.1 * radius);
          cairo_set_source_rgba (priv->cr, 1., 0., 0., 0.2);
          cairo_arc (priv->cr, x, y, radius - 0.23 * radius,
                     -5 * M_PI / 4 +
                     i * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step)),
                     -5 * M_PI / 4 + (i +
                                      1) * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step)));
          cairo_stroke (priv->cr);
          cairo_set_source_rgb (priv->cr, 1, 0., 0);
        }
        else if ((i * priv->sub_step >= priv->orange_strip_start) && (i * priv->sub_step < priv->yellow_strip_start))
        {
          cairo_set_source_rgb (priv->cr, 1, 0.65, 0);
        }
        else if (i * priv->sub_step >= priv->yellow_strip_start)
        {
          cairo_set_source_rgba (priv->cr, 1, 1, 0, alpha_inv);
          alpha_inv = alpha_inv - alpha_step;
        }
        cairo_set_line_width (priv->cr, 0.12 * radius);
        cairo_arc (priv->cr, x, y, radius - 0.06 * radius,
                   -5 * M_PI / 4 +
                   i * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step)),
                   -5 * M_PI / 4 + (i +
                                    1) * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step)));
        cairo_stroke (priv->cr);
        break;
      case 3:                  // **** Red - Yellow - Green
        alpha_step = fabs (1 / ((priv->end_value - priv->green_strip_start) / priv->sub_step));
        if ((i * priv->sub_step >= priv->red_strip_start) && (i * priv->sub_step < priv->yellow_strip_start))
        {
          cairo_set_line_width (priv->cr, 0.1 * radius);
          cairo_set_source_rgba (priv->cr, 1., 0., 0., 0.2);
          cairo_arc (priv->cr, x, y, radius - 0.23 * radius,
                     -5 * M_PI / 4 +
                     i * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step)),
                     -5 * M_PI / 4 + (i +
                                      1) * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step)));
          cairo_stroke (priv->cr);
          cairo_set_source_rgb (priv->cr, 1, 0., 0);
        }
        else if ((i * priv->sub_step >= priv->yellow_strip_start) && (i * priv->sub_step < priv->green_strip_start))
        {
          cairo_set_source_rgb (priv->cr, 1, 1, 0);
        }
        else if (i * priv->sub_step >= priv->green_strip_start)
        {
          cairo_set_source_rgba (priv->cr, 0, 0.65, 0, alpha_inv);
          alpha_inv = alpha_inv - alpha_step;
        }
        cairo_set_line_width (priv->cr, 0.12 * radius);
        cairo_arc (priv->cr, x, y, radius - 0.06 * radius,
                   -5 * M_PI / 4 +
                   i * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step)),
                   -5 * M_PI / 4 + (i +
                                    1) * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step)));
        cairo_stroke (priv->cr);
        break;
      case 0:
      default:
        alpha_step = fabs (1 / ((priv->yellow_strip_start - priv->orange_strip_start) / priv->sub_step));
        if ((i * priv->sub_step >= priv->yellow_strip_start) && (i * priv->sub_step < priv->orange_strip_start))
        {
          alpha = alpha + alpha_step;
          cairo_set_source_rgba (priv->cr, 1, 1, 0, alpha);
        }
        else if ((i * priv->sub_step >= priv->orange_strip_start) && (i * priv->sub_step < priv->red_strip_start))
        {
          cairo_set_source_rgb (priv->cr, 1, 0.65, 0);
        }
        else if (i * priv->sub_step >= priv->red_strip_start)
        {
          cairo_set_line_width (priv->cr, 0.1 * radius);
          cairo_set_source_rgba (priv->cr, 1., 0., 0., 0.2);
          cairo_arc (priv->cr, x, y, radius - 0.23 * radius,
                     -5 * M_PI / 4 +
                     i * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step)),
                     -5 * M_PI / 4 + (i +
                                      1) * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step)));
          cairo_stroke (priv->cr);
          cairo_set_source_rgb (priv->cr, 1, 0., 0);
        }
        cairo_set_line_width (priv->cr, 0.12 * radius);
        cairo_arc (priv->cr, x, y, radius - 0.06 * radius,
                   -5 * M_PI / 4 +
                   i * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step)),
                   -5 * M_PI / 4 + (i +
                                    1) * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step)));
        cairo_stroke (priv->cr);
    }
    cairo_restore (priv->cr);
  }

  priv->end_value = priv->end_value * 10;
  priv->start_value = priv->start_value * 10;
  priv->sub_step = priv->sub_step * 10;
  priv->initial_step = priv->initial_step * 10;
  for (i = 0; i < ((priv->end_value - priv->start_value) / priv->sub_step) + 1; i++)
  {
    int inset;
    cairo_save (priv->cr);

    if (((i * (int) priv->sub_step) % priv->initial_step) == 0)
    {
      if (!priv->color_mode_inv)
        cairo_set_source_rgb (priv->cr, 1, 1, 1);
      else
        cairo_set_source_rgb (priv->cr, 0., 0., 0.);
      inset = 0.18 * radius;
      cairo_set_line_width (priv->cr, 0.015 * radius);
    }
    else
    {
      cairo_set_source_rgba (priv->cr, 1, 1, 1, 0.5);
      inset = 0.12 * radius;
      cairo_set_line_width (priv->cr, 0.01 * radius);
    }

    cairo_move_to (priv->cr,
                   x + (radius - inset) * cos (-5 * M_PI / 4 +
                                               i * ((5 * M_PI / 4) /
                                                    ((priv->end_value - priv->start_value) / priv->sub_step))),
                   y + (radius - inset) * sin (-5 * M_PI / 4 +
                                               i * ((5 * M_PI / 4) /
                                                    ((priv->end_value - priv->start_value) / priv->sub_step))));
    cairo_line_to (priv->cr,
                   x + radius * cos (-5 * M_PI / 4 +
                                     i * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step))),
                   y + radius * sin (-5 * M_PI / 4 +
                                     i * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) / priv->sub_step))));
    cairo_stroke (priv->cr);
    cairo_restore (priv->cr);

  }

  cairo_move_to (priv->cr, x + (radius - 0.18 * radius) * cos (0), y + (radius - 0.18 * radius) * sin (0));
  cairo_line_to (priv->cr, x + radius * cos (0), y + radius * sin (0));
  priv->end_value = priv->end_value / 10;
  priv->start_value = priv->start_value / 10;
  priv->sub_step = priv->sub_step / 10;
  priv->initial_step = priv->initial_step / 10;


  // **** Number drawing
  for (i = 0; i < ((priv->end_value - priv->start_value) / priv->drawing_step) + 1; i++)
  {
    int inset;
    if (i * priv->drawing_step + priv->start_value < 10)
    {
      cairo_save (priv->cr);
      cairo_set_source_rgb (priv->cr, 1., 1., 1.);
      cairo_select_font_face (priv->cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
      cairo_set_font_size (priv->cr, 0.15 * radius);
      inset = 0.3 * radius;
      cairo_move_to (priv->cr,
                     x - 0.05 * radius + (radius - inset) * cos (-5 * M_PI / 4 +
                                                                 i * ((5 * M_PI / 4) /
                                                                      ((priv->end_value -
                                                                        priv->start_value) / priv->drawing_step))),
                     y + 0.045 * radius + (radius - inset) * sin (-5 * M_PI / 4 +
                                                                  i * ((5 * M_PI / 4) /
                                                                       ((priv->end_value -
                                                                         priv->start_value) / priv->drawing_step))));
      sprintf (str, "%d", i * priv->drawing_step + priv->start_value);
      cairo_show_text (priv->cr, str);
      cairo_stroke (priv->cr);
      cairo_restore (priv->cr);
    }
    else if ((i * priv->drawing_step + priv->start_value >= 10) && (i * priv->drawing_step + priv->start_value < 100))
    {
      cairo_save (priv->cr);
      cairo_set_source_rgb (priv->cr, 1., 1., 1.);
      cairo_select_font_face (priv->cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
      cairo_set_font_size (priv->cr, 0.15 * radius);
      inset = 0.33 * radius;
      cairo_move_to (priv->cr,
                     x - 0.08 * radius + (radius - inset) * cos (-5 * M_PI / 4 +
                                                                 i * ((5 * M_PI / 4) /
                                                                      ((priv->end_value -
                                                                        priv->start_value) / priv->drawing_step))),
                     y + 0.04 * radius + (radius - inset) * sin (-5 * M_PI / 4 +
                                                                 i * ((5 * M_PI / 4) /
                                                                      ((priv->end_value -
                                                                        priv->start_value) / priv->drawing_step))));
      sprintf (str, "%d", i * priv->drawing_step + priv->start_value);
      cairo_show_text (priv->cr, str);
      cairo_stroke (priv->cr);
      cairo_restore (priv->cr);
    }
    else if ((i * priv->drawing_step + priv->start_value >= 100) && (i * priv->drawing_step + priv->start_value < 1000))
    {
      cairo_save (priv->cr);
      cairo_set_source_rgb (priv->cr, 1., 1., 1.);
      cairo_select_font_face (priv->cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
      cairo_set_font_size (priv->cr, 0.14 * radius);
      inset = 0.36 * radius;
      cairo_move_to (priv->cr,
                     x - 0.14 * radius + (radius - inset) * cos (-5 * M_PI / 4 +
                                                                 i * ((5 * M_PI / 4) /
                                                                      ((priv->end_value -
                                                                        priv->start_value) / priv->drawing_step))),
                     y + 0.03 * radius + (radius - 0.34 * radius) * sin (-5 * M_PI / 4 +
                                                                         i * ((5 * M_PI / 4) /
                                                                              ((priv->end_value -
                                                                                priv->start_value) /
                                                                               priv->drawing_step))));
      sprintf (str, "%d", i * priv->drawing_step + priv->start_value);
      cairo_show_text (priv->cr, str);
      cairo_stroke (priv->cr);
      cairo_restore (priv->cr);
    }
    else if ((i * priv->drawing_step + priv->start_value >= 1000)
             && (i * priv->drawing_step + priv->start_value < 10000))
    {
      cairo_save (priv->cr);
      cairo_set_source_rgb (priv->cr, 1., 1., 1.);
      cairo_select_font_face (priv->cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
      cairo_set_font_size (priv->cr, 0.14 * radius);
      inset = 0.4 * radius;
      cairo_move_to (priv->cr,
                     x - 0.18 * radius + (radius - inset) * cos (-5 * M_PI / 4 +
                                                                 i * ((5 * M_PI / 4) /
                                                                      ((priv->end_value -
                                                                        priv->start_value) / priv->drawing_step))),
                     y + 0.04 * radius + (radius - 0.32 * radius) * sin (-5 * M_PI / 4 +
                                                                         i * ((5 * M_PI / 4) /
                                                                              ((priv->end_value -
                                                                                priv->start_value) /
                                                                               priv->drawing_step))));
      sprintf (str, "%d", i * priv->drawing_step + priv->start_value);
      cairo_show_text (priv->cr, str);
      cairo_stroke (priv->cr);
      cairo_restore (priv->cr);
    }

    // **** value over 9999 are not scale to fit the gauge
  }

  // **** draw gauge name
  name_box.x = x - 0.2 * radius;
  name_box.y = y + 0.35 * radius;
  name_box.width = 1 * radius;
  name_box.height = 0.4 * radius;
  gtk_gauge_draw_name (GTK_GAUGE (gauge), priv->gauge_name, &name_box);

  priv->x = x;
  priv->y = y;
  priv->radius = radius;

  // **** draw hand
  gtk_gauge_draw_hand (gauge);

  return;
}

static void gtk_gauge_draw_name (GtkGauge * gauge, gchar * pch_text, GdkRectangle * rect)
{
  GtkGaugePrivate *priv;
  PangoLayout *layout = NULL;
  PangoFontDescription *desc = NULL;
  gint x_pos = 0, y_pos = 0, width = 0, height = 0;

  if (gtk_gauge_debug)
  {
    g_debug ("===> gtk_gauge_draw_name()");
  }

  g_return_if_fail (IS_GTK_GAUGE (gauge));
  if (pch_text == NULL)
  {
    return;
  }
  g_return_if_fail (rect != NULL);
  priv = GTK_GAUGE_GET_PRIVATE (gauge);
  g_return_if_fail (priv->cr != NULL);

  // **** create pango layout
  layout = pango_cairo_create_layout (priv->cr);
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

  if (!priv->color_mode_inv)
    cairo_set_source_rgb (priv->cr, 1, 1, 1);
  else
    cairo_set_source_rgb (priv->cr, 0., 0., 0.);
  cairo_move_to (priv->cr, x_pos, y_pos);
  pango_cairo_show_layout (priv->cr, layout);
  cairo_stroke (priv->cr);

  g_object_unref (layout);
  pango_font_description_free (desc);
  return;
}

static void gtk_gauge_draw_hand (GtkWidget * gauge)
{
  GtkGaugePrivate *priv;
  if (gtk_gauge_debug)
  {
    g_debug ("===> gtk_gauge_draw_hands()");
  }
  g_return_if_fail (IS_GTK_GAUGE (gauge));

  priv = GTK_GAUGE_GET_PRIVATE (gauge);

  double x = priv->x;
  double y = priv->y;
  double radius = priv->radius;
  int value = priv->value * 1000;

  // **** centre cercle 
  cairo_save (priv->cr);
  if (!priv->color_mode_inv)
    cairo_set_source_rgb (priv->cr, 0, 0, 0);
  else
    cairo_set_source_rgb (priv->cr, 1, 1, 1);
  cairo_arc (priv->cr, x, y, radius - 0.9 * radius, 0, 2 * M_PI);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  if (!priv->color_mode_inv)
    cairo_set_source_rgb (priv->cr, 0.2, 0.2, 0.2);
  else
    cairo_set_source_rgb (priv->cr, 0.8, 0.8, 0.8);
  cairo_arc (priv->cr, x, y, radius - 0.9 * radius, 0, 2 * M_PI);
  cairo_stroke (priv->cr);
  cairo_restore (priv->cr);

  // **** gauge hand
  cairo_save (priv->cr);
  if (!priv->color_mode_inv)
    cairo_set_source_rgb (priv->cr, 1, 1, 1);
  else
    cairo_set_source_rgb (priv->cr, 0, 0, 0);
  cairo_set_line_width (priv->cr, 0.02 * radius);
  cairo_move_to (priv->cr, x, y);

  if (value * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) * 1000)) <= 5 * M_PI / 4)
  {
    cairo_line_to (priv->cr,
                   x + (radius - 0.2 * radius) * cos (-5 * M_PI / 4 +
                                                      value * ((5 * M_PI / 4) /
                                                               ((priv->end_value - priv->start_value) * 1000))),
                   y + (radius - 0.2 * radius) * sin (-5 * M_PI / 4 +
                                                      value * ((5 * M_PI / 4) /
                                                               ((priv->end_value - priv->start_value) * 1000))));
  }
  else
    g_warning ("GTK_GAUGE : value out of range");

  cairo_stroke (priv->cr);
  cairo_restore (priv->cr);

  cairo_save (priv->cr);
  if (!priv->color_mode_inv)
    cairo_set_source_rgb (priv->cr, 1, 1, 1);
  else
    cairo_set_source_rgb (priv->cr, 0, 0, 0);
  cairo_set_line_width (priv->cr, 0.02 * radius);
  cairo_move_to (priv->cr, x, y);
  if (value * ((5 * M_PI / 4) / ((priv->end_value - priv->start_value) * 1000)) <= 5 * M_PI / 4)
  {
    cairo_line_to (priv->cr,
                   x - (radius - 0.9 * radius) * cos (-5 * M_PI / 4 +
                                                      value * ((5 * M_PI / 4) /
                                                               ((priv->end_value - priv->start_value) * 1000))),
                   y - (radius - 0.9 * radius) * sin (-5 * M_PI / 4 +
                                                      value * ((5 * M_PI / 4) /
                                                               ((priv->end_value - priv->start_value) * 1000))));
  }
  else
    g_warning ("GTK_GAUGE : value out of range");
  cairo_stroke (priv->cr);
  cairo_restore (priv->cr);
  return;
}

static gboolean gtk_gauge_button_press_event (GtkWidget * widget, GdkEventButton * ev)
{
  GtkGaugePrivate *priv;
  gint x = 0, y = 0;

  if (gtk_gauge_debug)
  {
    g_debug ("===> gtk_gauge_button_press_event_cb()");
  }
  g_return_val_if_fail (IS_GTK_GAUGE (widget), FALSE);

  priv = GTK_GAUGE_GET_PRIVATE (widget);

  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 1))
  {
    gdk_window_get_pointer (ev->window, &x, &y, &priv->mouse_state);
    priv->mouse_pos.x = x;
    priv->mouse_pos.y = y;
    return TRUE;
  }
  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 2) && priv->b_mouse_onoff)
  {
    gtk_gauge_debug = gtk_gauge_debug ? FALSE : TRUE;
    return TRUE;
  }
  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 3))
  {
    priv->b_mouse_onoff = priv->b_mouse_onoff ? FALSE : TRUE;
    return TRUE;
  }

  return FALSE;
}

static gboolean gtk_gauge_motion_notify_event (GtkWidget * widget, GdkEventMotion * ev)
{
  GtkGaugePrivate *priv;
  GdkModifierType state;
  gint x = 0, y = 0;

  if (gtk_gauge_debug)
  {
    g_debug ("===> gtk_gauge_motion_notify_event_cb()");
  }
  g_return_val_if_fail (IS_GTK_GAUGE (widget), FALSE);

  priv = GTK_GAUGE_GET_PRIVATE (widget);

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

  if (gtk_gauge_debug)
  {
    g_debug ("===> gtk_gauge_motion_notify_event_cb() : mouse x=%d, y=%d", x, y);
  }

  return TRUE;
}

static void gtk_gauge_destroy (GtkObject * object)
{
  GtkGaugePrivate *priv = NULL;
  GtkWidget *widget = NULL;

  if (gtk_gauge_debug)
  {
    g_debug ("===> gtk_gauge_destroy(enter)");
  }

  g_return_if_fail (object != NULL);

  widget = GTK_WIDGET (object);

  g_return_if_fail (IS_GTK_GAUGE (widget));

  priv = GTK_GAUGE_GET_PRIVATE (widget);
  g_return_if_fail (priv != NULL);

  if (priv->cr)
  {
    g_free (priv->cr);
    g_free (priv->gauge_name);

    if (GTK_OBJECT_CLASS (gtk_gauge_parent_class)->destroy != NULL)
    {
      (*GTK_OBJECT_CLASS (gtk_gauge_parent_class)->destroy) (object);
    }
  }
  if (gtk_gauge_debug)
  {
    g_debug ("gtk_gauge_destroy(exit)");
  }

  return;
}

static void gtk_gauge_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec)
{
  GtkGaugePrivate *priv = NULL;
  GtkGauge *gauge = NULL;

  if (gtk_gauge_debug)
  {
    g_debug ("===> gtk_gauge_set_property()");
  }
  g_return_if_fail (object != NULL);

  gauge = GTK_GAUGE (object);
  g_return_if_fail (IS_GTK_GAUGE (gauge));

  priv = GTK_GAUGE_GET_PRIVATE (gauge);
  g_return_if_fail (priv != NULL);

  switch (prop_id)
  {
    case PROP_NAME:
      priv->gauge_name = g_strdup (g_value_get_string (value));
      break;
    case PROP_INVERSED_COLOR:
      priv->color_mode_inv = g_value_get_boolean (value);
      break;
    case PROP_RADIAL_COLOR:
      priv->radial_color = g_value_get_boolean (value);
      break;
    case PROP_START_VALUE:
      priv->start_value = g_value_get_int (value);
      break;
    case PROP_END_VALUE:
      priv->end_value = g_value_get_int (value);
      break;
    case PROP_INITIAL_STEP:
      priv->initial_step = g_value_get_int (value);
      break;
    case PROP_SUB_STEP:
      priv->sub_step = g_value_get_double (value);
      break;
    case PROP_DRAWING_STEP:
      priv->drawing_step = g_value_get_int (value);
      break;
    case PROP_STRIP_COLOR_ORDER:
      priv->strip_color_order = g_strdup (g_value_get_string (value));
      break;
    case PROP_GREEN_STRIP_START:
      priv->green_strip_start = g_value_get_int (value);
      break;
    case PROP_YELLOW_STRIP_START:
      priv->yellow_strip_start = g_value_get_int (value);
      break;
    case PROP_ORANGE_STRIP_START:
      priv->orange_strip_start = g_value_get_int (value);
      break;
    case PROP_RED_STRIP_START:
      priv->red_strip_start = g_value_get_int (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
  return;
}
