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
 * @file gtkartificialhorizon.c
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

#include <ground_station/gui/gtkartificialhorizon.h>

typedef struct _GtkArtificialHorizonPrivate
{
  /* new cairo design */
  cairo_t *cr;
  GdkRectangle plot_box;

  /* widget data */
  gint unit_value;
  gboolean unit_is_feet;
  gboolean color_mode_inv;
  gboolean radial_color;
  gdouble altitude;

  /* drawing data */
  gdouble x;
  gdouble y;
  gdouble radius;
  GdkColor bg_color_inv;
  GdkColor bg_color_altimeter;
  GdkColor bg_color_bounderie;
  GdkColor bg_radial_color_begin_altimeter;
  GdkColor bg_radial_color_begin_bounderie;

  /* mouse information */
  gboolean b_mouse_onoff;
  GdkPoint mouse_pos;
  GdkModifierType mouse_state;

} GtkArtificialHorizonPrivate;

enum _GTK_ARTIFICIAL_HORIZON_PROPERTY_ID
{
  PROP_0,
  PROP_INVERSED_COLOR,
  PROP_UNIT_IS_FEET,
  PROP_UNIT_STEP_VALUE,
  PROP_RADIAL_COLOR,
} GTK_ARTIFICIAL_HORIZON_PROPERTY_ID;

G_DEFINE_TYPE (GtkArtificialHorizon, gtk_artificial_horizon, GTK_TYPE_DRAWING_AREA);

#define GTK_ARTIFICIAL_HORIZON_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GTK_ARTIFICIAL_HORIZON_TYPE, GtkArtificialHorizonPrivate))

static void gtk_artificial_horizon_class_init (GtkArtificialHorizonClass * klass);
static void gtk_artificial_horizon_init (GtkArtificialHorizon * arh);
static void gtk_artificial_horizon_destroy (GtkObject * object);
static void gtk_artificial_horizon_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec);

static gboolean gtk_artificial_horizon_configure_event (GtkWidget * widget, GdkEventConfigure * event);
static gboolean gtk_artificial_horizon_expose (GtkWidget * graph, GdkEventExpose * event);
static gboolean gtk_artificial_horizon_button_press_event (GtkWidget * widget, GdkEventButton * ev);
static gboolean gtk_artificial_horizon_motion_notify_event (GtkWidget * widget, GdkEventMotion * ev);

static void gtk_artificial_horizon_draw (GtkWidget * arh);
static void gtk_artificial_horizon_draw_screws (GtkWidget * arh);
static void gtk_artificial_horizon_draw_external_arc (GtkWidget * arh);
static void gtk_artificial_horizon_draw_internal_sphere (GtkWidget * arh);

static gboolean gtk_artificial_horizon_debug = FALSE;

static void gtk_artificial_horizon_class_init (GtkArtificialHorizonClass * klass)
{
  GObjectClass *obj_class = G_OBJECT_CLASS (klass);
  GtkWidgetClass *widget_class = GTK_WIDGET_CLASS (klass);
  GtkObjectClass *gtkobject_class = GTK_OBJECT_CLASS (klass);

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("===> gtk_artificial_horizon_class_init()");
  }

  /* GObject signal overrides */
  obj_class->set_property = gtk_artificial_horizon_set_property;

  /* GtkObject signal overrides */
  gtkobject_class->destroy = gtk_artificial_horizon_destroy;

  /* GtkWidget signals overrides */
  widget_class->configure_event = gtk_artificial_horizon_configure_event;
  widget_class->expose_event = gtk_artificial_horizon_expose;
  widget_class->motion_notify_event = gtk_artificial_horizon_motion_notify_event;
  widget_class->button_press_event = gtk_artificial_horizon_button_press_event;

  g_type_class_add_private (obj_class, sizeof (GtkArtificialHorizonPrivate));

  g_object_class_install_property (obj_class,
                                   PROP_INVERSED_COLOR,
                                   g_param_spec_boolean ("inverse-color",
                                                         "inverse or not the widget color",
                                                         "inverse or not the widget color", FALSE, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_UNIT_IS_FEET,
                                   g_param_spec_boolean ("unit-is-feet",
                                                         "set the altimeter unit to feet or meter",
                                                         "set the altimeter unit to feet or meter",
                                                         TRUE, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_UNIT_STEP_VALUE,
                                   g_param_spec_int ("unit-step-value",
                                                     "select the value of the initial step (1, 10 or 100)",
                                                     "select the value of the initial step (1, 10 or 100)",
                                                     1, 100, 100, G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class,
                                   PROP_RADIAL_COLOR,
                                   g_param_spec_boolean ("radial-color",
                                                         "the widget use radial color",
                                                         "the widget use radial color", TRUE, G_PARAM_WRITABLE));
  return;
}

static void gtk_artificial_horizon_init (GtkArtificialHorizon * arh)
{
  GtkArtificialHorizonPrivate *priv = NULL;

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("===> gtk_artificial_horizon_init()");
  }
  g_return_if_fail (IS_GTK_ARTIFICIAL_HORIZON (arh));

  priv = GTK_ARTIFICIAL_HORIZON_GET_PRIVATE (arh);

  gtk_widget_add_events (GTK_WIDGET (arh), GDK_BUTTON_PRESS_MASK |
                         GDK_BUTTON_RELEASE_MASK | GDK_POINTER_MOTION_MASK | GDK_POINTER_MOTION_HINT_MASK);
  priv->b_mouse_onoff = FALSE;
  priv->color_mode_inv = FALSE;
  priv->radial_color = TRUE;

  priv->bg_color_bounderie.red = 6553.5;        // 0.1 cairo
  priv->bg_color_bounderie.green = 6553.5;
  priv->bg_color_bounderie.blue = 6553.5;
  priv->bg_color_altimeter.red = 3276.75;       // 0.05 cairo
  priv->bg_color_altimeter.green = 3276.75;
  priv->bg_color_altimeter.blue = 3276.75;
  priv->bg_color_inv.red = 45874.5;     // 0.7 cairo
  priv->bg_color_inv.green = 45874.5;
  priv->bg_color_inv.blue = 45874.5;
  priv->bg_radial_color_begin_bounderie.red = 13107;    // 0.2 cairo
  priv->bg_radial_color_begin_bounderie.green = 13107;
  priv->bg_radial_color_begin_bounderie.blue = 13107;
  priv->bg_radial_color_begin_altimeter.red = 45874.5;  // 0.7 cairo
  priv->bg_radial_color_begin_altimeter.green = 45874.5;
  priv->bg_radial_color_begin_altimeter.blue = 45874.5;
  return;
}

static gboolean gtk_artificial_horizon_configure_event (GtkWidget * widget, GdkEventConfigure * event)
{
  GtkArtificialHorizonPrivate *priv;
  GtkArtificialHorizon *arh = GTK_ARTIFICIAL_HORIZON (widget);

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("===> gtk_artificial_horizon_configure_event()");
  }
  g_return_val_if_fail (IS_GTK_ARTIFICIAL_HORIZON (arh), FALSE);

  g_return_val_if_fail (event->type == GDK_CONFIGURE, FALSE);

  priv = GTK_ARTIFICIAL_HORIZON_GET_PRIVATE (arh);
  g_return_val_if_fail (priv != NULL, FALSE);

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("===> gtk_artificial_horizon_configure_event(new width=%d, height=%d)", event->width, event->height);
  }

  if ((event->width < GTK_ARTIFICIAL_HORIZON_MODEL_X) || (event->height < GTK_ARTIFICIAL_HORIZON_MODEL_Y))
  {
    priv->plot_box.width = GTK_ARTIFICIAL_HORIZON_MODEL_X;
    priv->plot_box.height = GTK_ARTIFICIAL_HORIZON_MODEL_Y;
  }
  else
  {
    priv->plot_box.width = event->width;
    priv->plot_box.height = event->height;
  }

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("cfg:Max.Avail: plot_box.width=%d, plot_box.height=%d", priv->plot_box.width, priv->plot_box.height);
  }
  return FALSE;
}

static gboolean gtk_artificial_horizon_expose (GtkWidget * arh, GdkEventExpose * event)
{
  GtkArtificialHorizonPrivate *priv;
  GtkWidget *widget = arh;

  cairo_t *cr = NULL;
  cairo_status_t status;

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("===> gtk_artificial_horizon_expose()");
  }
  g_return_val_if_fail (IS_GTK_ARTIFICIAL_HORIZON (arh), FALSE);

  priv = GTK_ARTIFICIAL_HORIZON_GET_PRIVATE (arh);
  g_return_val_if_fail (priv != NULL, FALSE);

  priv->plot_box.width = widget->allocation.width;
  priv->plot_box.height = widget->allocation.height;

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("gtk_artificial_horizon_expose(width=%d, height=%d)", widget->allocation.width, widget->allocation.height);
  }

  priv->cr = cr = gdk_cairo_create (widget->window);
  status = cairo_status (cr);
  if (status != CAIRO_STATUS_SUCCESS)
  {
    g_message ("GLG-Expose:cairo_create:status %d=%s", status, cairo_status_to_string (status));
  }

  cairo_rectangle (cr, 0, 0, priv->plot_box.width, priv->plot_box.height);
  cairo_clip (cr);

  gtk_artificial_horizon_draw (arh);

  cairo_destroy (cr);
  priv->cr = NULL;

  return FALSE;
}

extern void gtk_artificial_horizon_redraw (GtkArtificialHorizon * arh)
{
  GtkWidget *widget;
  GdkRegion *region;

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("===> gtk_artificial_horizon_redraw()");
  }
  g_return_if_fail (IS_GTK_ARTIFICIAL_HORIZON (arh));

  widget = GTK_WIDGET (arh);

  if (!widget->window)
    return;

  region = gdk_drawable_get_clip_region (widget->window);
  /* redraw the window completely by exposing it */
  gdk_window_invalidate_region (widget->window, region, TRUE);
  gdk_window_process_updates (widget->window, TRUE);

  gdk_region_destroy (region);
}
//~ 
//~ extern void gtk_artificial_horizon_set_arhi (GtkArtificialHorizon * arh, gdouble arhi)
//~ {
  //~ GtkArtificialHorizonPrivate *priv;
//~ 
  //~ if (gtk_artificial_horizon_debug)
  //~ {
    //~ g_debug ("===> gtk_artificial_horizon_draw()");
  //~ }
  //~ g_return_if_fail (IS_GTK_ARTIFICIAL_HORIZON (arh));
//~ 
  //~ priv = GTK_ARTIFICIAL_HORIZON_GET_PRIVATE (arh);
  //~ priv->arhitude = arhi;
//~ }

extern GtkWidget *gtk_artificial_horizon_new (void)
{
  if (gtk_artificial_horizon_debug)
  {
    g_debug ("===> gtk_artificial_horizon_new()");
  }
  return GTK_WIDGET (gtk_type_new (gtk_artificial_horizon_get_type ()));
}

static void gtk_artificial_horizon_draw (GtkWidget * arh)
{
  GtkArtificialHorizonPrivate *priv;

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("===> gtk_artificial_horizon_draw()");
  }
  g_return_if_fail (IS_GTK_ARTIFICIAL_HORIZON (arh));

  priv = GTK_ARTIFICIAL_HORIZON_GET_PRIVATE (arh);

  double x, y, rec_x0, rec_y0, rec_width, rec_height, rec_degrees;
  double rec_aspect, rec_corner_radius, rec_radius, radius;
  cairo_pattern_t *pat=NULL;

  x = arh->allocation.width / 2;
  y = arh->allocation.height / 2;
  radius = MIN (arh->allocation.width / 2, arh->allocation.height / 2) - 5;

  rec_x0 = x - radius;
  rec_y0 = y - radius;
  rec_width = radius * 2;
  rec_height = radius * 2;
  rec_aspect = 1.0;
  rec_corner_radius = rec_height / 8.0;

  rec_radius = rec_corner_radius / rec_aspect;
  rec_degrees = M_PI / 180.0;

  // Altimeter base
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
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_altimeter.red / 65535,
                            (gdouble) priv->bg_color_altimeter.green / 65535,
                            (gdouble) priv->bg_color_altimeter.blue / 65535);
    else
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_altimeter.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_altimeter.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_altimeter.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, (gdouble) priv->bg_color_altimeter.red / 65535,
                                       (gdouble) priv->bg_color_altimeter.green / 65535,
                                       (gdouble) priv->bg_color_altimeter.blue / 65535, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  
  priv->radius = radius;
  priv->x = x;
  priv->y = y;
  
  gtk_artificial_horizon_draw_screws(arh);
  gtk_artificial_horizon_draw_external_arc(arh);
  gtk_artificial_horizon_draw_internal_sphere(arh);

  // **** base arrow
  cairo_new_sub_path (priv->cr);
  cairo_set_line_width (priv->cr, 0.02 * radius);  
  cairo_set_source_rgba (priv->cr, 0.3, 0.3, 0.3,0.15);
  cairo_move_to (priv->cr, x + (radius - 0.205 * radius) * cos (-M_PI / 2),
									y + (radius - 0.205 * radius) * sin (-M_PI / 2));
  cairo_line_to (priv->cr, x + (radius - 0.325 * radius) * cos (-M_PI / 2+M_PI/30), 
									y + (radius - 0.325 * radius) * sin (-M_PI / 2+M_PI/30));
  cairo_line_to (priv->cr, x + (radius - 0.325 * radius) * cos (-M_PI / 2-M_PI/30), 
									y + (radius - 0.325 * radius) * sin (-M_PI / 2-M_PI/30));
  cairo_line_to (priv->cr, x + (radius - 0.205 * radius)  * cos (-M_PI / 2), 
									y + (radius - 0.205 * radius)  * sin (-M_PI / 2));
  cairo_close_path (priv->cr);  									
  cairo_stroke (priv->cr);  
  
  cairo_new_sub_path (priv->cr);
  cairo_set_line_width (priv->cr, 0.02 * radius);  
  cairo_set_source_rgb (priv->cr, 1, 0.65, 0.);
  cairo_move_to (priv->cr, x + (radius - 0.18 * radius) * cos (-M_PI / 2),
									y + (radius - 0.18 * radius) * sin (-M_PI / 2));
  cairo_line_to (priv->cr, x + (radius - 0.3 * radius) * cos (-M_PI / 2+M_PI/30), 
									y + (radius - 0.3 * radius) * sin (-M_PI / 2+M_PI/30));
  cairo_line_to (priv->cr, x + (radius - 0.3 * radius) * cos (-M_PI / 2-M_PI/30), 
									y + (radius - 0.3 * radius) * sin (-M_PI / 2-M_PI/30));
  cairo_line_to (priv->cr, x + (radius - 0.18 * radius)  * cos (-M_PI / 2), 
									y + (radius - 0.18 * radius)  * sin (-M_PI / 2));
  cairo_close_path (priv->cr);  									
  cairo_stroke (priv->cr);  
  
  // **** base quart arc
  cairo_arc (priv->cr, x, y, radius+0.009*radius, M_PI/5,4*M_PI/5);
  if (((priv->radial_color) && (priv->color_mode_inv)) || ((!priv->radial_color) && (priv->color_mode_inv))
      || ((!priv->radial_color) && (!priv->color_mode_inv)))
  {
    if (!priv->color_mode_inv)
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_altimeter.red / 65535,
                            (gdouble) priv->bg_color_altimeter.green / 65535,
                            (gdouble) priv->bg_color_altimeter.blue / 65535);
    else
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_altimeter.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_altimeter.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_altimeter.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, (gdouble) priv->bg_color_altimeter.red / 65535,
                                       (gdouble) priv->bg_color_altimeter.green / 65535,
                                       (gdouble) priv->bg_color_altimeter.blue / 65535, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill (priv->cr);
  cairo_stroke (priv->cr);

  cairo_new_sub_path (priv->cr);
  cairo_set_line_width (priv->cr, 0.02 * radius);  
  if (((priv->radial_color) && (priv->color_mode_inv)) || ((!priv->radial_color) && (priv->color_mode_inv))
      || ((!priv->radial_color) && (!priv->color_mode_inv)))
  {
    if (!priv->color_mode_inv)
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_altimeter.red / 65535,
                            (gdouble) priv->bg_color_altimeter.green / 65535,
                            (gdouble) priv->bg_color_altimeter.blue / 65535);
    else
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_altimeter.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_altimeter.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_altimeter.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, (gdouble) priv->bg_color_altimeter.red / 65535,
                                       (gdouble) priv->bg_color_altimeter.green / 65535,
                                       (gdouble) priv->bg_color_altimeter.blue / 65535, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_move_to (priv->cr, x - 0.3 * radius,y + 0.60 * radius);
  cairo_line_to (priv->cr, x - 0.2 * radius,y + 0.35 * radius);
  cairo_line_to (priv->cr, x - 0.05 * radius,y + 0.35 * radius);
  cairo_line_to (priv->cr, x - 0.05 * radius,y + 0.25 * radius);
  cairo_line_to (priv->cr, x - 0.015 * radius,y + 0.15 * radius);  
  cairo_line_to (priv->cr, x - 0.015 * radius,y);
  cairo_line_to (priv->cr, x + 0.015 * radius,y);
  cairo_line_to (priv->cr, x + 0.015 * radius,y + 0.15 * radius);  
  cairo_line_to (priv->cr, x + 0.05 * radius,y + 0.25 * radius);
  cairo_line_to (priv->cr, x + 0.05 * radius,y + 0.35 * radius);  
  cairo_line_to (priv->cr, x + 0.2 * radius,y + 0.35 * radius);
  cairo_line_to (priv->cr, x + 0.3 * radius,y + 0.60 * radius);
  cairo_fill (priv->cr);  
  cairo_close_path (priv->cr);  									
  cairo_stroke (priv->cr);  
  
  cairo_set_source_rgb (priv->cr,0,0,0);
  cairo_set_line_width (priv->cr, 0.06 * radius);  
  cairo_move_to (priv->cr, x - 0.61 * radius,y);
  cairo_line_to (priv->cr, x - 0.2 * radius,y);
  cairo_line_to (priv->cr, x - 0.1 * radius,y + 0.1 * radius);
  cairo_line_to (priv->cr, x, y);
  cairo_line_to (priv->cr, x + 0.1 * radius,y + 0.1 * radius);
  cairo_line_to (priv->cr, x + 0.2 * radius,y);
  cairo_line_to (priv->cr, x + 0.61 * radius,y);
  cairo_stroke (priv->cr);  
  cairo_set_source_rgb (priv->cr,1,0.65,0);
  cairo_set_line_width (priv->cr, 0.04 * radius);  
  cairo_move_to (priv->cr, x - 0.6 * radius,y);
  cairo_line_to (priv->cr, x - 0.2 * radius,y);
  cairo_line_to (priv->cr, x - 0.1 * radius,y + 0.1 * radius);
  cairo_line_to (priv->cr, x, y);
  cairo_line_to (priv->cr, x + 0.1 * radius,y + 0.1 * radius);
  cairo_line_to (priv->cr, x + 0.2 * radius,y);
  cairo_line_to (priv->cr, x + 0.6 * radius,y);
  cairo_stroke (priv->cr); 
  
  cairo_pattern_destroy (pat);
  return;
}

static void gtk_artificial_horizon_draw_screws (GtkWidget * arh)
{
  GtkArtificialHorizonPrivate *priv;

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("===> gtk_artificial_horizon_draw()");
  }
  g_return_if_fail (IS_GTK_ARTIFICIAL_HORIZON (arh));

  priv = GTK_ARTIFICIAL_HORIZON_GET_PRIVATE (arh);
  
  cairo_pattern_t *pat=NULL;
  double x, y, radius;
  radius=priv->radius;
  x=priv->x;
  y=priv->y;
  radius = radius+0.12*radius;
  
  // **** top left screw
  cairo_arc (priv->cr, x-0.82*radius, y-0.82*radius, 0.1 * radius, 0, 2*M_PI);
  pat = cairo_pattern_create_radial (x-0.82*radius, y-0.82*radius, 0.07*radius,
                                     x-0.82*radius, y-0.82*radius, 0.1*radius);
  cairo_pattern_add_color_stop_rgba (pat,0, 0, 0, 0,0.7);
  cairo_pattern_add_color_stop_rgba (pat,1, 0, 0, 0,0.1);
  cairo_set_source (priv->cr, pat);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
    
  cairo_arc (priv->cr, x-0.82*radius, y-0.82*radius, 0.07 * radius, 0, 2*M_PI);
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
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15,0.15,0.15, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  
  cairo_set_line_width (priv->cr, 0.02 * radius);
  if (!priv->color_mode_inv)
       cairo_set_source_rgb (priv->cr, 0., 0., 0.);
  else
       cairo_set_source_rgb (priv->cr, 1., 1., 1.);
  cairo_move_to (priv->cr, x-0.88*radius, y-0.82*radius);
  cairo_line_to (priv->cr, x-0.76*radius, y-0.82*radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);  
  cairo_move_to (priv->cr, x-0.82*radius, y-0.88*radius);
  cairo_line_to (priv->cr, x-0.82*radius, y-0.76*radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_set_line_width (priv->cr, 0.01 * radius);
  if (!priv->color_mode_inv)
		cairo_set_source_rgb (priv->cr, 0.1, 0.1, 0.1);
  else
      cairo_set_source_rgb (priv->cr, 0.9, 0.9, 0.9);  
  cairo_move_to (priv->cr, x-0.88*radius, y-0.82*radius);
  cairo_line_to (priv->cr, x-0.76*radius, y-0.82*radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);  
  cairo_move_to (priv->cr, x-0.82*radius, y-0.88*radius);
  cairo_line_to (priv->cr, x-0.82*radius, y-0.76*radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);      
  
   // **** top right screw
  cairo_arc (priv->cr, x+0.82*radius, y-0.82*radius, 0.1 * radius, 0, 2*M_PI);
  pat = cairo_pattern_create_radial (x+0.82*radius, y-0.82*radius, 0.07*radius,
                                     x+0.82*radius, y-0.82*radius, 0.1*radius);
  cairo_pattern_add_color_stop_rgba (pat,0, 0, 0, 0,0.7);
  cairo_pattern_add_color_stop_rgba (pat,1, 0, 0, 0,0.1);
  cairo_set_source (priv->cr, pat);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
    
  cairo_arc (priv->cr, x+0.82*radius, y-0.82*radius, 0.07 * radius, 0, 2*M_PI);
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
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15,0.15,0.15, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  
  cairo_set_line_width (priv->cr, 0.02 * radius);
  if (!priv->color_mode_inv)
       cairo_set_source_rgb (priv->cr, 0., 0., 0.);
  else
       cairo_set_source_rgb (priv->cr, 1., 1., 1.);
  cairo_move_to (priv->cr, x+0.88*radius, y-0.82*radius);
  cairo_line_to (priv->cr, x+0.76*radius, y-0.82*radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);  
  cairo_move_to (priv->cr, x+0.82*radius, y-0.88*radius);
  cairo_line_to (priv->cr, x+0.82*radius, y-0.76*radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_set_line_width (priv->cr, 0.01 * radius);
  if (!priv->color_mode_inv)
		cairo_set_source_rgb (priv->cr, 0.1, 0.1, 0.1);
  else
      cairo_set_source_rgb (priv->cr, 0.9, 0.9, 0.9);  
  cairo_move_to (priv->cr, x+0.88*radius, y-0.82*radius);
  cairo_line_to (priv->cr, x+0.76*radius, y-0.82*radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);  
  cairo_move_to (priv->cr, x+0.82*radius, y-0.88*radius);
  cairo_line_to (priv->cr, x+0.82*radius, y-0.76*radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);     
  
   // **** bottom left screw
  cairo_arc (priv->cr, x-0.82*radius, y+0.82*radius, 0.1 * radius, 0, 2*M_PI);
  pat = cairo_pattern_create_radial (x-0.82*radius, y+0.82*radius, 0.07*radius,
                                     x-0.82*radius, y+0.82*radius, 0.1*radius);
  cairo_pattern_add_color_stop_rgba (pat,0, 0, 0, 0,0.7);
  cairo_pattern_add_color_stop_rgba (pat,1, 0, 0, 0,0.1);
  cairo_set_source (priv->cr, pat);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
    
  cairo_arc (priv->cr, x-0.82*radius, y+0.82*radius, 0.07 * radius, 0, 2*M_PI);
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
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15,0.15,0.15, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  
  cairo_set_line_width (priv->cr, 0.02 * radius);
  if (!priv->color_mode_inv)
       cairo_set_source_rgb (priv->cr, 0., 0., 0.);
  else
       cairo_set_source_rgb (priv->cr, 1., 1., 1.);
  cairo_move_to (priv->cr, x-0.88*radius, y+0.82*radius);
  cairo_line_to (priv->cr, x-0.76*radius, y+0.82*radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);  
  cairo_move_to (priv->cr, x-0.82*radius, y+0.88*radius);
  cairo_line_to (priv->cr, x-0.82*radius, y+0.76*radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_set_line_width (priv->cr, 0.01 * radius);
  if (!priv->color_mode_inv)
		cairo_set_source_rgb (priv->cr, 0.1, 0.1, 0.1);
  else
      cairo_set_source_rgb (priv->cr, 0.9, 0.9, 0.9);  
  cairo_move_to (priv->cr, x-0.88*radius, y+0.82*radius);
  cairo_line_to (priv->cr, x-0.76*radius, y+0.82*radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);  
  cairo_move_to (priv->cr, x-0.82*radius, y+0.88*radius);
  cairo_line_to (priv->cr, x-0.82*radius, y+0.76*radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);     
  
   // **** bottom right screw
  cairo_arc (priv->cr, x+0.82*radius, y+0.82*radius, 0.1 * radius, 0, 2*M_PI);
  pat = cairo_pattern_create_radial (x+0.82*radius, y+0.82*radius, 0.07*radius,
                                     x+0.82*radius, y+0.82*radius, 0.1*radius);
  cairo_pattern_add_color_stop_rgba (pat,0, 0, 0, 0,0.7);
  cairo_pattern_add_color_stop_rgba (pat,1, 0, 0, 0,0.1);
  cairo_set_source (priv->cr, pat);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
    
  cairo_arc (priv->cr, x+0.82*radius, y+0.82*radius, 0.07 * radius, 0, 2*M_PI);
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
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15,0.15,0.15, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  
  cairo_set_line_width (priv->cr, 0.02 * radius);
  if (!priv->color_mode_inv)
       cairo_set_source_rgb (priv->cr, 0., 0., 0.);
  else
       cairo_set_source_rgb (priv->cr, 1., 1., 1.);
  cairo_move_to (priv->cr, x+0.88*radius, y+0.82*radius);
  cairo_line_to (priv->cr, x+0.76*radius, y+0.82*radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);  
  cairo_move_to (priv->cr, x+0.82*radius, y+0.88*radius);
  cairo_line_to (priv->cr, x+0.82*radius, y+0.76*radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  cairo_set_line_width (priv->cr, 0.01 * radius);
  if (!priv->color_mode_inv)
		cairo_set_source_rgb (priv->cr, 0.1, 0.1, 0.1);
  else
      cairo_set_source_rgb (priv->cr, 0.9, 0.9, 0.9);  
  cairo_move_to (priv->cr, x+0.88*radius, y+0.82*radius);
  cairo_line_to (priv->cr, x+0.76*radius, y+0.82*radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);  
  cairo_move_to (priv->cr, x+0.82*radius, y+0.88*radius);
  cairo_line_to (priv->cr, x+0.82*radius, y+0.76*radius);
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);     
  cairo_pattern_destroy (pat);  
  return;
}


static void gtk_artificial_horizon_draw_internal_sphere (GtkWidget * arh)
{
  GtkArtificialHorizonPrivate *priv;

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("===> gtk_artificial_horizon_draw()");
  }
  g_return_if_fail (IS_GTK_ARTIFICIAL_HORIZON (arh));

  priv = GTK_ARTIFICIAL_HORIZON_GET_PRIVATE (arh);
  
  cairo_pattern_t *pat = NULL;
  double x, y, radius;
  radius=priv->radius;
  x=priv->x;
  y=priv->y;
  
  // **** internal sphere
  cairo_arc (priv->cr, x, y, radius - 0.15 * radius, M_PI,0);
  if(!priv->radial_color)
  {
    cairo_set_source_rgb (priv->cr, 0.117, 0.564, 1.);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_altimeter.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_altimeter.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_altimeter.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.117, 0.564, 1.,1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill (priv->cr);
  cairo_stroke (priv->cr);

  cairo_stroke (priv->cr);

  
  cairo_arc (priv->cr, x, y, radius - 0.15 * radius, 0, M_PI);
  if(!priv->radial_color)
  {
          cairo_set_source_rgb (priv->cr, 0.651, 0.435, 0.098);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_altimeter.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_altimeter.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_altimeter.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat,1, 0.651, 0.435, 0.098,1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill (priv->cr);
  cairo_stroke (priv->cr);
  
  cairo_set_line_width (priv->cr, 0.02 * radius);
  cairo_move_to (priv->cr, x - radius + 0.15 * radius, y);
  cairo_line_to (priv->cr, x + radius - 0.15 * radius, y);
  cairo_set_source_rgb (priv->cr, 1., 1., 1.);
  cairo_stroke (priv->cr);
  
  // **** horizontal line (pitch)
  cairo_move_to (priv->cr, x - 0.4*radius,y- 0.4*radius);
  cairo_line_to (priv->cr, x + 0.4*radius,y- 0.4*radius);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.3*radius,y- 0.268*radius);
  cairo_line_to (priv->cr, x + 0.3*radius,y- 0.268*radius);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.2*radius,y- 0.134*radius);
  cairo_line_to (priv->cr, x + 0.2*radius,y- 0.134*radius);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.1*radius,y- 0.4*radius + 0.067*radius);
  cairo_line_to (priv->cr, x + 0.1*radius,y- 0.4*radius + 0.067*radius);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.1*radius,y- 0.268*radius+ 0.067*radius);
  cairo_line_to (priv->cr, x + 0.1*radius,y- 0.268*radius+ 0.067*radius);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.1*radius,y- 0.134*radius+ 0.067*radius);
  cairo_line_to (priv->cr, x + 0.1*radius,y- 0.134*radius+ 0.067*radius);
  cairo_stroke (priv->cr);
  
  cairo_move_to (priv->cr, x - 0.4*radius,y+ 0.4*radius);
  cairo_line_to (priv->cr, x + 0.4*radius,y+ 0.4*radius);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.3*radius,y+ 0.268*radius);
  cairo_line_to (priv->cr, x + 0.3*radius,y+ 0.268*radius);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.2*radius,y+ 0.134*radius);
  cairo_line_to (priv->cr, x + 0.2*radius,y+ 0.134*radius);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.1*radius,y+ 0.4*radius- 0.067*radius);
  cairo_line_to (priv->cr, x + 0.1*radius,y+ 0.4*radius- 0.067*radius);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.1*radius,y+ 0.268*radius- 0.067*radius);
  cairo_line_to (priv->cr, x + 0.1*radius,y+ 0.268*radius- 0.067*radius);
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.1*radius,y+ 0.134*radius- 0.067*radius);
  cairo_line_to (priv->cr, x + 0.1*radius,y+ 0.134*radius- 0.067*radius);
  cairo_stroke (priv->cr);
  
  // **** 10 drawing  
  cairo_select_font_face (priv->cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size (priv->cr, 0.1 * radius);
  cairo_move_to (priv->cr, x - 0.35 * radius, y- 0.1*radius);
  cairo_show_text (priv->cr, "10");
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x + 0.21 * radius, y- 0.1*radius);
  cairo_show_text (priv->cr, "10");
  cairo_stroke (priv->cr);
  
  cairo_move_to (priv->cr, x - 0.35 * radius, y+ 0.17*radius);
  cairo_show_text (priv->cr, "10");
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x + 0.21 * radius, y+ 0.17*radius);
  cairo_show_text (priv->cr, "10");
  cairo_stroke (priv->cr);
  
  // **** 20 drawing    
  cairo_move_to (priv->cr, x - 0.45 * radius, y- 0.234*radius);
  cairo_show_text (priv->cr, "20");
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x + 0.31 * radius, y- 0.234*radius);
  cairo_show_text (priv->cr, "20");
  cairo_stroke (priv->cr);
  
  cairo_move_to (priv->cr, x - 0.45 * radius, y+ 0.302*radius);
  cairo_show_text (priv->cr, "20");
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x + 0.31 * radius, y+ 0.302*radius);
  cairo_show_text (priv->cr, "20");
  cairo_stroke (priv->cr);
  
  // **** 30 drawing    
  cairo_move_to (priv->cr, x - 0.55 * radius, y- 0.368*radius);
  cairo_show_text (priv->cr, "30");
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x + 0.41 * radius, y- 0.368*radius);
  cairo_show_text (priv->cr, "30");
  cairo_stroke (priv->cr);
  
  cairo_move_to (priv->cr, x - 0.55 * radius, y+ 0.434*radius);
  cairo_show_text (priv->cr, "30");
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x + 0.41 * radius, y+ 0.434*radius);
  cairo_show_text (priv->cr, "30");
  cairo_stroke (priv->cr);  

  // **** alpha arc
  cairo_arc (priv->cr, x, y, radius - 0.15 * radius, 0, 2*M_PI);
  pat = cairo_pattern_create_radial (x, y, radius - 0.23 * radius,
                                     x, y, radius - 0.15 * radius);
  cairo_pattern_add_color_stop_rgba (pat,0, 0.3, 0.3, 0.3,0.1);
  cairo_pattern_add_color_stop_rgba (pat,1, 0.3, 0.3, 0.3,0.6);
  cairo_set_source (priv->cr, pat);
  cairo_fill (priv->cr);
  cairo_stroke (priv->cr);
  cairo_pattern_destroy (pat);    
  return;
}

static void gtk_artificial_horizon_draw_external_arc (GtkWidget * arh)
{
  GtkArtificialHorizonPrivate *priv;

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("===> gtk_artificial_horizon_draw()");
  }
  g_return_if_fail (IS_GTK_ARTIFICIAL_HORIZON (arh));

  priv = GTK_ARTIFICIAL_HORIZON_GET_PRIVATE (arh);
  
  cairo_pattern_t *pat = NULL;
  double x, y, radius;
  radius=priv->radius;
  x=priv->x;
  y=priv->y;
  
  // **** external demi arc sky
  cairo_arc (priv->cr, x, y, radius, M_PI, 0);
 if(!priv->radial_color)
  {
    cairo_set_source_rgb (priv->cr, 0.117, 0.564, 1.);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_altimeter.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_altimeter.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_altimeter.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.117, 0.564, 1.,1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  
  // **** external demi arc ground
  cairo_arc (priv->cr, x, y, radius, 0, M_PI);
  if(!priv->radial_color)
  {
    cairo_set_source_rgb (priv->cr, 0.651, 0.435, 0.098);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_altimeter.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_altimeter.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_altimeter.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat,1, 0.651, 0.435, 0.098,1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);  
  
  // **** external arc alpha composante
  cairo_arc (priv->cr, x, y, radius, 0,2 *M_PI);
  cairo_set_source_rgba (priv->cr, 0.3, 0.3, 0.3,0.3);
  cairo_fill (priv->cr);
  cairo_stroke (priv->cr);
  
  // **** external arc tips
  cairo_set_line_width (priv->cr, 0.04 * radius);
  cairo_move_to (priv->cr, x - radius, y);
  cairo_line_to (priv->cr, x + radius, y);
  cairo_set_source_rgb (priv->cr, 1., 1., 1.);
  cairo_stroke (priv->cr);
  
  cairo_set_line_width (priv->cr, 0.02 * radius);
  cairo_move_to (priv->cr, x + (radius - 0.15 * radius) * cos (-M_PI / 6),
									y + (radius - 0.15 * radius) * sin (-M_PI / 6));
  cairo_line_to (priv->cr, x + (radius - 0.04 * radius)  * cos (-M_PI / 6), 
									y + (radius - 0.04 * radius)  * sin (-M_PI / 6));
  cairo_stroke (priv->cr);
  
  cairo_move_to (priv->cr, x + (radius - 0.15 * radius) * cos (-2*M_PI / 6),
									y + (radius - 0.15 * radius) * sin (-2*M_PI / 6));
  cairo_line_to (priv->cr, x + (radius - 0.04 * radius)  * cos (-2*M_PI / 6), 
									y + (radius - 0.04 * radius)  * sin (-2*M_PI / 6));
  cairo_stroke (priv->cr);
  
  cairo_move_to (priv->cr, x + (radius - 0.15 * radius) * cos (-4*M_PI / 6),
									y + (radius - 0.15 * radius) * sin (-4*M_PI / 6));
  cairo_line_to (priv->cr, x + (radius - 0.04 * radius)  * cos (-4*M_PI / 6), 
									y + (radius - 0.04 * radius)  * sin (-4*M_PI / 6));
  cairo_stroke (priv->cr);
  
  cairo_move_to (priv->cr, x + (radius - 0.15 * radius) * cos (-5*M_PI / 6),
									y + (radius - 0.15 * radius) * sin (-5*M_PI / 6));
  cairo_line_to (priv->cr, x + (radius - 0.04 * radius)  * cos (-5*M_PI / 6), 
									y + (radius - 0.04 * radius)  * sin (-5*M_PI / 6));
  cairo_stroke (priv->cr);
  
  cairo_set_line_width (priv->cr, 0.015 * radius);
  cairo_move_to (priv->cr, x + (radius - 0.15 * radius) * cos (-7*M_PI / 18),
									y + (radius - 0.15 * radius) * sin (-7*M_PI / 18));
  cairo_line_to (priv->cr, x + (radius - 0.07 * radius)  * cos (-7*M_PI / 18), 
									y + (radius - 0.07 * radius)  * sin (-7*M_PI / 18));
  cairo_stroke (priv->cr);
    cairo_move_to (priv->cr, x + (radius - 0.15 * radius) * cos (-8*M_PI / 18),
									y + (radius - 0.15 * radius) * sin (-8*M_PI / 18));
  cairo_line_to (priv->cr, x + (radius - 0.07 * radius)  * cos (-8*M_PI / 18), 
									y + (radius - 0.07 * radius)  * sin (-8*M_PI / 18));
  cairo_stroke (priv->cr);
    cairo_move_to (priv->cr, x + (radius - 0.15 * radius) * cos (-10*M_PI / 18),
									y + (radius - 0.15 * radius) * sin (-10*M_PI / 18));
  cairo_line_to (priv->cr, x + (radius - 0.07 * radius)  * cos (-10*M_PI / 18), 
									y + (radius - 0.07 * radius)  * sin (-10*M_PI / 18));
  cairo_stroke (priv->cr);
    cairo_move_to (priv->cr, x + (radius - 0.15 * radius) * cos (-11*M_PI / 18),
									y + (radius - 0.15 * radius) * sin (-11*M_PI / 18));
  cairo_line_to (priv->cr, x + (radius - 0.07 * radius)  * cos (-11*M_PI / 18), 
									y + (radius - 0.07 * radius)  * sin (-11*M_PI / 18));
  cairo_stroke (priv->cr);
  
  // **** external arc arrow
  cairo_move_to (priv->cr, x + (radius - 0.15 * radius) * cos (-3*M_PI / 12),
									y + (radius - 0.15 * radius) * sin (-3*M_PI / 12));
  cairo_line_to (priv->cr, x + (radius - 0.07 * radius)  * cos (-3*M_PI / 12+M_PI/45), 
									y + (radius - 0.07 * radius)  * sin (-3*M_PI / 12+M_PI/45));
  cairo_line_to (priv->cr, x + (radius - 0.07 * radius)  * cos (-3*M_PI / 12-M_PI/45), 
									y + (radius - 0.07 * radius)  * sin (-3*M_PI / 12-M_PI/45));
  cairo_line_to (priv->cr, x + (radius - 0.15 * radius)  * cos (-3*M_PI / 12), 
									y + (radius - 0.15 * radius)  * sin (-3*M_PI / 12));
  cairo_fill (priv->cr);
  cairo_stroke (priv->cr);
  
  cairo_move_to (priv->cr, x + (radius - 0.15 * radius) * cos (-9*M_PI / 12),
									y + (radius - 0.15 * radius) * sin (-9*M_PI / 12));
  cairo_line_to (priv->cr, x + (radius - 0.07 * radius)  * cos (-9*M_PI / 12+M_PI/45), 
									y + (radius - 0.07 * radius)  * sin (-9*M_PI / 12+M_PI/45));
  cairo_line_to (priv->cr, x + (radius - 0.07 * radius)  * cos (-9*M_PI / 12-M_PI/45), 
									y + (radius - 0.07 * radius)  * sin (-9*M_PI / 12-M_PI/45));
  cairo_line_to (priv->cr, x + (radius - 0.15 * radius)  * cos (-9*M_PI / 12), 
									y + (radius - 0.15 * radius)  * sin (-9*M_PI / 12));
  cairo_fill (priv->cr);
  cairo_stroke (priv->cr);  
  
  cairo_move_to (priv->cr, x + (radius - 0.15 * radius) * cos (-M_PI / 2),
									y + (radius - 0.15 * radius) * sin (-M_PI / 2));
  cairo_line_to (priv->cr, x + radius * cos (-M_PI / 2+M_PI/30), 
									y + radius * sin (-M_PI / 2+M_PI/30));
  cairo_line_to (priv->cr, x + radius * cos (-M_PI / 2-M_PI/30), 
									y + radius * sin (-M_PI / 2-M_PI/30));
  cairo_line_to (priv->cr, x + (radius - 0.15 * radius)  * cos (-M_PI / 2), 
									y + (radius - 0.15 * radius)  * sin (-M_PI / 2));
  cairo_fill (priv->cr);
  cairo_stroke (priv->cr);  
  cairo_pattern_destroy (pat);
  return;
}

static gboolean gtk_artificial_horizon_button_press_event (GtkWidget * widget, GdkEventButton * ev)
{
  GtkArtificialHorizonPrivate *priv;
  gint x = 0, y = 0;

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("===> gtk_artificial_horizon_button_press_event_cb()");
  }
  g_return_val_if_fail (IS_GTK_ARTIFICIAL_HORIZON (widget), FALSE);

  priv = GTK_ARTIFICIAL_HORIZON_GET_PRIVATE (widget);

  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 1))
  {
    gdk_window_get_pointer (ev->window, &x, &y, &priv->mouse_state);
    priv->mouse_pos.x = x;
    priv->mouse_pos.y = y;
    return TRUE;
  }
  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 2) && priv->b_mouse_onoff)
  {
    gtk_artificial_horizon_debug = gtk_artificial_horizon_debug ? FALSE : TRUE;
    return TRUE;
  }
  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 3))
  {
    priv->b_mouse_onoff = priv->b_mouse_onoff ? FALSE : TRUE;
    return TRUE;
  }

  return FALSE;
}

static gboolean gtk_artificial_horizon_motion_notify_event (GtkWidget * widget, GdkEventMotion * ev)
{
  GtkArtificialHorizonPrivate *priv;
  GdkModifierType state;
  gint x = 0, y = 0;

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("===> gtk_artificial_horizon_motion_notify_event_cb()");
  }
  g_return_val_if_fail (IS_GTK_ARTIFICIAL_HORIZON (widget), FALSE);

  priv = GTK_ARTIFICIAL_HORIZON_GET_PRIVATE (widget);

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

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("===> gtk_artificial_horizon_motion_notify_event_cb() : mouse x=%d, y=%d", x, y);
  }

  return TRUE;
}

static void gtk_artificial_horizon_destroy (GtkObject * object)
{
  GtkArtificialHorizonPrivate *priv = NULL;
  GtkWidget *widget = NULL;

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("===> gtk_artificial_horizon_destroy(enter)");
  }

  g_return_if_fail (object != NULL);

  widget = GTK_WIDGET (object);

  g_return_if_fail (IS_GTK_ARTIFICIAL_HORIZON (widget));

  priv = GTK_ARTIFICIAL_HORIZON_GET_PRIVATE (widget);
  g_return_if_fail (priv != NULL);

  if (priv->cr)
  {
    g_free (priv->cr);

    if (GTK_OBJECT_CLASS (gtk_artificial_horizon_parent_class)->destroy != NULL)
    {
      (*GTK_OBJECT_CLASS (gtk_artificial_horizon_parent_class)->destroy) (object);
    }
  }
  if (gtk_artificial_horizon_debug)
  {
    g_debug ("gtk_artificial_horizon_destroy(exit)");
  }

  return;
}

static void gtk_artificial_horizon_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec)
{
  GtkArtificialHorizonPrivate *priv = NULL;
  GtkArtificialHorizon *arh = NULL;

  if (gtk_artificial_horizon_debug)
  {
    g_debug ("===> gtk_artificial_horizon_set_property()");
  }
  g_return_if_fail (object != NULL);

  arh = GTK_ARTIFICIAL_HORIZON (object);
  g_return_if_fail (IS_GTK_ARTIFICIAL_HORIZON (arh));

  priv = GTK_ARTIFICIAL_HORIZON_GET_PRIVATE (arh);
  g_return_if_fail (priv != NULL);

  switch (prop_id)
  {
    case PROP_INVERSED_COLOR:
      priv->color_mode_inv = g_value_get_boolean (value);
      break;
    case PROP_UNIT_IS_FEET:
      priv->unit_is_feet = g_value_get_boolean (value);
      break;
    case PROP_UNIT_STEP_VALUE:
      priv->unit_value = g_value_get_int (value);
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
