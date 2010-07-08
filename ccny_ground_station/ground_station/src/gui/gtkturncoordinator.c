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
 * @file gtkturncoordinator.c
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
 * vario = gtk_turn_coordinator_new();
 * g_object_set(GTK_TURN_COORDINATOR_ (vario),
 *					"inverse-color", false,
 *					"unit-is-feet", true,
 *					"unit-step-value", 1000,
 *					"radial-color", true, NULL);
 * 
 * gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(vario), TRUE, TRUE, 0);
 * gtk_widget_show_all(window);
 * @endcode
 */

#include <ground_station/gui/gtkturncoordinator.h>

/**
 * @typedef struct GtkTurnCoordinatorPrivate 
 * @brief Special Gtk API strucure which contains the widget's data.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkTurnCoordinatorPrivate
{
  /* new cairo design */
  cairo_t *cr;
  GdkRectangle plot_box;

  /* widget data */
  gint unit_value;
  gboolean unit_is_feet;
  gboolean color_mode_inv;
  gboolean radial_color;
  gdouble value;
  gdouble l_time;
  gdouble l_value;  

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

} GtkTurnCoordinatorPrivate;

/**
 * @enum _GTK_TURN_COORDINATOR_PROPERTY_ID
 * @brief Special Gtk API enum that allow to set widget's properties
 *
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
enum _GTK_TURN_COORDINATOR_PROPERTY_ID
{
  PROP_0,
  PROP_INVERSED_COLOR,
  PROP_UNIT_IS_FEET,
  PROP_UNIT_STEP_VALUE,
  PROP_RADIAL_COLOR,
} GTK_TURN_COORDINATOR_PROPERTY_ID;

/**
 * @fn G_DEFINE_TYPE (GtkTurnCoordinator, gtk_turn_coordinator, GTK_TYPE_DRAWING_AREA);
 * @brief Special Gtk API function that permit to define the current
 * widget as an GTK_TYPE_DRAWING_AREA. 
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
G_DEFINE_TYPE (GtkTurnCoordinator, gtk_turn_coordinator, GTK_TYPE_DRAWING_AREA);

#define GTK_TURN_COORDINATOR_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GTK_TURN_COORDINATOR_TYPE, GtkTurnCoordinatorPrivate))

static void gtk_turn_coordinator_class_init (GtkTurnCoordinatorClass * klass);
static void gtk_turn_coordinator_init (GtkTurnCoordinator * vario);
static void gtk_turn_coordinator_destroy (GtkObject * object);
static void gtk_turn_coordinator_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec);

static gboolean gtk_turn_coordinator_configure_event (GtkWidget * widget, GdkEventConfigure * event);
static gboolean gtk_turn_coordinator_expose (GtkWidget * graph, GdkEventExpose * event);
static gboolean gtk_turn_coordinator_button_press_event (GtkWidget * widget, GdkEventButton * ev);
static gboolean gtk_turn_coordinator_motion_notify_event (GtkWidget * widget, GdkEventMotion * ev);

static void gtk_turn_coordinator_draw (GtkWidget * vario);
static void gtk_turn_coordinator_draw_screws (GtkWidget * vario);
static void gtk_turn_coordinator_draw_hand (GtkWidget * vario);

static gboolean gtk_turn_coordinator_debug = FALSE;

/**
 * @fn static void gtk_turn_coordinator_class_init (GtkTurnCoordinatorClass * klass)
 * @brief Special Gtk API function that perform the initilization step of
 * the widget's class. 
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */ 
static void gtk_turn_coordinator_class_init (GtkTurnCoordinatorClass * klass)
{
  GObjectClass *obj_class = G_OBJECT_CLASS (klass);
  GtkWidgetClass *widget_class = GTK_WIDGET_CLASS (klass);
  GtkObjectClass *gtkobject_class = GTK_OBJECT_CLASS (klass);

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_class_init()");
  }

  /* GObject signal overrides */
  obj_class->set_property = gtk_turn_coordinator_set_property;

  /* GtkObject signal overrides */
  gtkobject_class->destroy = gtk_turn_coordinator_destroy;

  /* GtkWidget signals overrides */
  widget_class->configure_event = gtk_turn_coordinator_configure_event;
  widget_class->expose_event = gtk_turn_coordinator_expose;
  widget_class->motion_notify_event = gtk_turn_coordinator_motion_notify_event;
  widget_class->button_press_event = gtk_turn_coordinator_button_press_event;

  g_type_class_add_private (obj_class, sizeof (GtkTurnCoordinatorPrivate));

  g_object_class_install_property (obj_class,
                                   PROP_INVERSED_COLOR,
                                   g_param_spec_boolean ("inverse-color",
                                                         "inverse or not the widget color",
                                                         "inverse or not the widget color", FALSE, G_PARAM_WRITABLE));
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
 * @fn static void gtk_turn_coordinator_init (GtkTurnCoordinator * vario)
 * @brief Special Gtk API function that initialize the widget and 
 * widget's data. 
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */ 
static void gtk_turn_coordinator_init (GtkTurnCoordinator * vario)
{
  GtkTurnCoordinatorPrivate *priv = NULL;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_init()");
  }
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (vario));

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (vario);

  gtk_widget_add_events (GTK_WIDGET (vario), GDK_BUTTON_PRESS_MASK |
                         GDK_BUTTON_RELEASE_MASK | GDK_POINTER_MOTION_MASK | GDK_POINTER_MOTION_HINT_MASK);
  priv->b_mouse_onoff = FALSE;
  priv->color_mode_inv = FALSE;
  priv->radial_color = TRUE;
  priv->unit_value = 1000;
  priv->value=0;
  priv->l_time=0;

  priv->bg_color_bounderie.red = 6553.5;        // 0.1 cairo
  priv->bg_color_bounderie.green = 6553.5;
  priv->bg_color_bounderie.blue = 6553.5;
  priv->bg_color_variometer.red = 3276.75;       // 0.05 cairo
  priv->bg_color_variometer.green = 3276.75;
  priv->bg_color_variometer.blue = 3276.75;
  priv->bg_color_inv.red = 45874.5;     // 0.7 cairo
  priv->bg_color_inv.green = 45874.5;
  priv->bg_color_inv.blue = 45874.5;
  priv->bg_radial_color_begin_bounderie.red = 13107;    // 0.2 cairo
  priv->bg_radial_color_begin_bounderie.green = 13107;
  priv->bg_radial_color_begin_bounderie.blue = 13107;
  priv->bg_radial_color_begin_variometer.red = 45874.5;  // 0.7 cairo
  priv->bg_radial_color_begin_variometer.green = 45874.5;
  priv->bg_radial_color_begin_variometer.blue = 45874.5;
  return;
}

/**
 * @fn static gboolean gtk_turn_coordinator_configure_event (GtkWidget * widget, GdkEventConfigure * event)
 * @brief Special Gtk API function. 
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */ 
static gboolean gtk_turn_coordinator_configure_event (GtkWidget * widget, GdkEventConfigure * event)
{
  GtkTurnCoordinatorPrivate *priv;
  GtkTurnCoordinator *vario = GTK_TURN_COORDINATOR (widget);

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_configure_event()");
  }
  g_return_val_if_fail (IS_GTK_TURN_COORDINATOR (vario), FALSE);

  g_return_val_if_fail (event->type == GDK_CONFIGURE, FALSE);

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (vario);
  g_return_val_if_fail (priv != NULL, FALSE);

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_configure_event(new width=%d, height=%d)", event->width, event->height);
  }

  if ((event->width < GTK_TURN_COORDINATOR_MODEL_X) || (event->height < GTK_TURN_COORDINATOR_MODEL_Y))
  {
    priv->plot_box.width = GTK_TURN_COORDINATOR_MODEL_X;
    priv->plot_box.height = GTK_TURN_COORDINATOR_MODEL_Y;
  }
  else
  {
    priv->plot_box.width = event->width;
    priv->plot_box.height = event->height;
  }
  
  priv->l_time = 0;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("cfg:Max.Avail: plot_box.width=%d, plot_box.height=%d", priv->plot_box.width, priv->plot_box.height);
  }
  return FALSE;
}

/**
 * @fn static gboolean gtk_turn_coordinator_expose (GtkWidget * vario, GdkEventExpose * event)
 * @brief Special Gtk API function. 
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */ 
static gboolean gtk_turn_coordinator_expose (GtkWidget * vario, GdkEventExpose * event)
{
  GtkTurnCoordinatorPrivate *priv;
  GtkWidget *widget = vario;

  cairo_t *cr = NULL;
  cairo_status_t status;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_expose()");
  }
  g_return_val_if_fail (IS_GTK_TURN_COORDINATOR (vario), FALSE);

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (vario);
  g_return_val_if_fail (priv != NULL, FALSE);

  priv->plot_box.width = widget->allocation.width;
  priv->plot_box.height = widget->allocation.height;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("gtk_turn_coordinator_expose(width=%d, height=%d)", widget->allocation.width, widget->allocation.height);
  }

  priv->cr = cr = gdk_cairo_create (widget->window);
  status = cairo_status (cr);
  if (status != CAIRO_STATUS_SUCCESS)
  {
    g_message ("GLG-Expose:cairo_create:status %d=%s", status, cairo_status_to_string (status));
  }

  cairo_rectangle (cr, 0, 0, priv->plot_box.width, priv->plot_box.height);
  cairo_clip (cr);

  gtk_turn_coordinator_draw (vario);

  cairo_destroy (cr);
  priv->cr = NULL;

  return FALSE;
}

/**
 * @fn extern void gtk_turn_coordinator_redraw (GtkTurnCoordinator * vario)
 * @brief Special Gtk API function. Redraw the widget when called.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */ 
extern void gtk_turn_coordinator_redraw (GtkTurnCoordinator * vario)
{
  GtkWidget *widget;
  GdkRegion *region;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_redraw()");
  }
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (vario));

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
 * @fn static inline double Now() 
 * @brief Time function, return the current time.
 * @return (double) current time
 */ 
static inline double getTime() 
{ 	
	struct timeval tp;	
	gettimeofday(&tp,NULL); 
	return (double)tp.tv_sec+tp.tv_usec*1e-6; 
} 

/**
 * @fn extern void gtk_turn_coordinator_set_value (GtkTurnCoordinator * vario, gdouble val)
 * @brief Gtk Variometer function that permit to set the internal value
 * variable of the widget. 
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */ 
extern void gtk_turn_coordinator_set_value (GtkTurnCoordinator * vario, gdouble val)
{
  GtkTurnCoordinatorPrivate *priv;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_set_value()");
  }
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (vario));

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (vario);
  
  if(priv->l_time==0)
  {
	  priv->l_time = getTime();
	  priv->l_value = val;
	  return;
  }
  
  double unit_per_min,now = getTime();
  
  unit_per_min = (60*(val-priv->l_value))/(now-priv->l_time); 
  if(unit_per_min<6*priv->unit_value)
  {
		priv->value = (60*(val-priv->l_value))/(now-priv->l_time);   
  }
  else 
  {
		priv->l_time = 0;
		return;
  }
  
  priv->l_time = now;
  priv->l_value = val;
  return;
}

/**
 * @fn extern GtkWidget *gtk_turn_coordinator_new (void)
 * @brief Special Gtk API function. 
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */ 
extern GtkWidget *gtk_turn_coordinator_new (void)
{
  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_new()");
  }
  return GTK_WIDGET (gtk_type_new (gtk_turn_coordinator_get_type ()));
}

/**
 * @fn static void gtk_turn_coordinator_draw (GtkWidget * vario)
 * @brief Special Gtk API function that draw the widget by using cairo.
 * 
 * See GObject,Cairo and GTK+ references for more informations: 
 * http://library.gnome.org/devel/references.html.en
 */ 
static void gtk_turn_coordinator_draw (GtkWidget * vario)
{
  GtkTurnCoordinatorPrivate *priv;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_draw()");
  }
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (vario));

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (vario);

  double x, y, rec_x0, rec_y0, rec_width, rec_height, rec_degrees;
  double rec_aspect, rec_corner_radius, rec_radius, radius;
  double x0,y0,x1,y1,x2,y2,x3,y3;  
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
  if (((priv->radial_color) && (priv->color_mode_inv)) || ((!priv->radial_color) && (priv->color_mode_inv))
      || ((!priv->radial_color) && (!priv->color_mode_inv)))
  {
    if (!priv->color_mode_inv)
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
  
  // **** "turn coordinator" drawing
  cairo_select_font_face (priv->cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_source_rgb(priv->cr,1.,1.,1.);
  cairo_set_font_size (priv->cr, 0.11 * radius);
  cairo_move_to (priv->cr, x - 0.15 * radius, y- 0.44*radius);
  cairo_show_text (priv->cr, "TURN");
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.41 * radius, y- 0.32*radius);
  cairo_show_text (priv->cr, "COORDINATOR");
  cairo_stroke (priv->cr);
  
  // **** turn coordinator ticks
  cairo_set_line_width (priv->cr, 0.06 * radius);
  cairo_move_to (priv->cr, x + (radius - 0.1 * radius) * cos (M_PI),
									y + (radius - 0.1 * radius) * sin (M_PI));
  cairo_line_to (priv->cr, x + (radius - 0.25 * radius)  * cos (M_PI), 
									y + (radius - 0.25 * radius)  * sin (M_PI));
  cairo_stroke (priv->cr);
  
  cairo_move_to (priv->cr, x + (radius - 0.1 * radius) * cos (0),
									y + (radius - 0.1 * radius) * sin (0));
  cairo_line_to (priv->cr, x + (radius - 0.25 * radius)  * cos (0), 
									y + (radius - 0.25 * radius)  * sin (0));
  cairo_stroke (priv->cr);
  
  cairo_move_to (priv->cr, x + (radius - 0.1 * radius) * cos (M_PI-M_PI/8),
									y + (radius - 0.1 * radius) * sin (M_PI-M_PI/8));
  cairo_line_to (priv->cr, x + (radius - 0.25 * radius)  * cos (M_PI-M_PI/8), 
									y + (radius - 0.25 * radius)  * sin (M_PI-M_PI/8));
  cairo_stroke (priv->cr);
  
  cairo_move_to (priv->cr, x + (radius - 0.1 * radius) * cos (M_PI/8),
									y + (radius - 0.1 * radius) * sin (M_PI/8));
  cairo_line_to (priv->cr, x + (radius - 0.25 * radius)  * cos (M_PI/8), 
									y + (radius - 0.25 * radius)  * sin (M_PI/8));
  cairo_stroke (priv->cr);
  
  // **** "R" & "L" drawing
  cairo_select_font_face (priv->cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);  
  cairo_set_font_size (priv->cr, 0.16 * radius);  
  cairo_move_to (priv->cr, x + (radius - 0.15 * radius) * cos (M_PI/8+M_PI/12),
									y + (radius - 0.15 * radius) * sin (M_PI/8+M_PI/12));
  cairo_show_text (priv->cr, "R");
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x + (radius - 0.02 * radius) * cos (M_PI-M_PI/8-M_PI/12),
									y + (radius - 0.15 * radius) * sin (M_PI-M_PI/8-M_PI/12));
  cairo_show_text (priv->cr, "L");
  cairo_stroke (priv->cr);
  
  // **** "NO PITCH INFORMATION" drawing
  cairo_select_font_face (priv->cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);  
  cairo_set_font_size (priv->cr, 0.08 * radius);  
  cairo_move_to (priv->cr, x - 0.19 * radius, y + 0.85*radius);
  cairo_show_text (priv->cr, "NO PITCH");
  cairo_stroke (priv->cr);
  cairo_move_to (priv->cr, x - 0.28 * radius, y+ 0.94*radius);
  cairo_show_text (priv->cr, "INFORMATION");
  cairo_stroke (priv->cr);
  
  // **** plane drawing
  cairo_set_line_width (priv->cr, 0.02 * radius);  
  cairo_arc (priv->cr, x, y, radius - 0.9 * radius, 0, 2 * M_PI);
  if (!priv->color_mode_inv)
    cairo_set_source_rgb (priv->cr, 1., 1., 1.);
  else
    cairo_set_source_rgb (priv->cr, 0., 0., 0.);
  cairo_fill_preserve(priv->cr);
  cairo_stroke (priv->cr);
  
  cairo_new_sub_path (priv->cr);
  cairo_move_to(priv->cr,x,y);
  cairo_line_to(priv->cr,x-0.68*radius,y);
  cairo_line_to(priv->cr,x,y+0.05*radius);
  cairo_line_to(priv->cr,x+0.68*radius,y);
  cairo_line_to(priv->cr,x,y);
  cairo_close_path (priv->cr);
  cairo_fill_preserve(priv->cr);
  cairo_stroke (priv->cr);  
  
  cairo_set_line_width (priv->cr, 0.02 * radius);    
  cairo_move_to(priv->cr,x,y);
  cairo_line_to(priv->cr,x,y-0.2*radius);
  cairo_stroke (priv->cr);  
  
  cairo_move_to(priv->cr,x-0.2*radius,y-0.1*radius);
  cairo_line_to(priv->cr,x+0.2*radius,y-0.1*radius);
  cairo_stroke (priv->cr);  
  
  // **** balle spot
  cairo_set_source_rgb (priv->cr, 1, 1,1);  
  cairo_new_sub_path (priv->cr);
  x0 = x;
  y0 = y + 0.35 * radius;
  x1 = x - 0.4 * radius;
  y1 = y + 0.35 * radius;
  x2 = x - 0.65 * radius;
  y2 = y + 0.2 * radius;
  x3 = x - 0.6 * radius;
  y3 = y + 0.35 * radius;
  cairo_move_to (priv->cr, x0, y0);
  cairo_curve_to (priv->cr, x1, y1, x2, y2, x3, y3);
  x1 = x - 0.6 * radius;
  y1 = y + 0.4 * radius;
  x2 = x - 0.5 * radius;
  y2 = y + 0.55 * radius;
  x3 = x;
  y3 = y + 0.55 * radius;
  cairo_curve_to (priv->cr, x1, y1, x2, y2, x3, y3);
  cairo_close_path (priv->cr);
  cairo_fill_preserve (priv->cr);  
  cairo_set_source_rgb (priv->cr, 0.2, 0.2,0.2);    
  cairo_stroke (priv->cr);  
  
  cairo_set_source_rgb (priv->cr, 1, 1,1);    
  cairo_new_sub_path (priv->cr);
  x0 = x;
  y0 = y + 0.35 * radius;
  x1 = x + 0.4 * radius;
  y1 = y + 0.35 * radius;
  x2 = x + 0.65 * radius;
  y2 = y + 0.2 * radius;
  x3 = x + 0.6 * radius;
  y3 = y + 0.35 * radius;
  cairo_move_to (priv->cr, x0, y0);
  cairo_curve_to (priv->cr, x1, y1, x2, y2, x3, y3);
  x1 = x + 0.6 * radius;
  y1 = y + 0.4 * radius;
  x2 = x + 0.5 * radius;
  y2 = y + 0.55 * radius;
  x3 = x;
  y3 = y + 0.55 * radius;
  cairo_curve_to (priv->cr, x1, y1, x2, y2, x3, y3);
  cairo_close_path (priv->cr);
  cairo_fill_preserve (priv->cr);  
  cairo_set_source_rgb (priv->cr, 0.2, 0.2,0.2);  
  cairo_stroke (priv->cr);  

  cairo_set_line_width (priv->cr, 0.04 * radius);    
  cairo_move_to(priv->cr,x,y + 0.36 * radius);
  cairo_line_to(priv->cr,x,y + 0.54 * radius);
  cairo_set_source_rgb (priv->cr, 1, 1, 1);  
  cairo_stroke (priv->cr);
  
   
     
  // alpha effect
  cairo_arc (priv->cr, x, y, radius - 0.25 * radius, 0, 2*M_PI);
  pat = cairo_pattern_create_radial (x, y, radius - 0.35 * radius,
                                     x, y, radius - 0.25 * radius);
  cairo_pattern_add_color_stop_rgba (pat,0, 0.3, 0.3, 0.3,0.2);
  cairo_pattern_add_color_stop_rgba (pat,1, 0.3, 0.3, 0.3,0.6);
  cairo_set_source (priv->cr, pat);
  cairo_fill (priv->cr);
  cairo_stroke (priv->cr);

  priv->radius = radius;
  priv->x = x;
  priv->y = y;
  gtk_turn_coordinator_draw_screws(vario);

  // draw hands
  //gtk_turn_coordinator_draw_hand(vario);
  
  cairo_pattern_destroy (pat);  
  return;
}

/**
 * @fn static void gtk_turn_coordinator_draw_screws (GtkWidget * vario)
 * @brief Widget function that draw the widget screws using cairo.
 * 
 * See GObject,Cairo and GTK+ references for more informations: 
 * http://library.gnome.org/devel/references.html.en
 */ 
static void gtk_turn_coordinator_draw_screws (GtkWidget * vario)
{
  GtkTurnCoordinatorPrivate *priv;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_draw()");
  }
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (vario));

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (vario);
  
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

/**
 * @fn static void gtk_turn_coordinator_draw_hand (GtkWidget * vario)
 * @brief Widget function that draw the variometer hands using cairo.
 * 
 * See GObject,Cairo and GTK+ references for more informations: 
 * http://library.gnome.org/devel/references.html.en
 */
static void gtk_turn_coordinator_draw_hand (GtkWidget * vario)
{
  GtkTurnCoordinatorPrivate *priv;
  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_draw_hands()");
  }
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (vario));

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (vario);

  double x = priv->x;
  double y = priv->y;
  double radius = priv->radius;
  int value = priv->value;

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
  if(priv->value>=1000)
		value+=1000;
  else if(priv->value<=-1000)
		value-=1000;
		
  if((priv->value>=1000)||(priv->value<=-1000)){
		cairo_save (priv->cr);
		if (!priv->color_mode_inv)
			cairo_set_source_rgb (priv->cr, 1, 1, 1);
		else
			cairo_set_source_rgb (priv->cr, 0, 0, 0);
		cairo_set_line_width (priv->cr, 0.02 * radius);
		cairo_move_to (priv->cr, x, y);
		cairo_line_to (priv->cr,
							x + (radius - 0.2 * radius) * cos (M_PI + value * ((M_PI / 9+ M_PI/38)/1000)),
							y + (radius - 0.2 * radius) * sin (M_PI + value * ((M_PI / 9+ M_PI/38)/1000)));
		cairo_stroke (priv->cr);
		cairo_restore (priv->cr);
	
		cairo_save (priv->cr);
		if (!priv->color_mode_inv)
			cairo_set_source_rgb (priv->cr, 1, 1, 1);
		else
			cairo_set_source_rgb (priv->cr, 0, 0, 0);
		cairo_set_line_width (priv->cr, 0.02 * radius);
		cairo_move_to (priv->cr, x, y);
		cairo_line_to (priv->cr,
							x - (radius - 0.9 * radius) * cos (M_PI + value * ((M_PI / 9+ M_PI/38)/1000)),
							y - (radius - 0.9 * radius) * sin (M_PI + value * ((M_PI / 9+ M_PI/38)/1000)));
		cairo_stroke (priv->cr);
		cairo_restore (priv->cr);
	}
	else
	{
		cairo_save (priv->cr);
		if (!priv->color_mode_inv)
			cairo_set_source_rgb (priv->cr, 1, 1, 1);
		else
			cairo_set_source_rgb (priv->cr, 0, 0, 0);
		cairo_set_line_width (priv->cr, 0.02 * radius);
		cairo_move_to (priv->cr, x, y);
		cairo_line_to (priv->cr,
							x + (radius - 0.2 * radius) * cos (M_PI + value * ((M_PI / (9/2)+ M_PI/38)/1000)),
							y + (radius - 0.2 * radius) * sin (M_PI + value * ((M_PI / (9/2)+ M_PI/38)/1000)));
		cairo_stroke (priv->cr);
		cairo_restore (priv->cr);
	
		cairo_save (priv->cr);
		if (!priv->color_mode_inv)
			cairo_set_source_rgb (priv->cr, 1, 1, 1);
		else
			cairo_set_source_rgb (priv->cr, 0, 0, 0);
		cairo_set_line_width (priv->cr, 0.02 * radius);
		cairo_move_to (priv->cr, x, y);
		cairo_line_to (priv->cr,
							x - (radius - 0.9 * radius) * cos (M_PI + value * ((M_PI / (9/2)+ M_PI/38)/1000)),
							y - (radius - 0.9 * radius) * sin (M_PI + value * ((M_PI / (9/2)+ M_PI/38)/1000)));
		cairo_stroke (priv->cr);
		cairo_restore (priv->cr);
	}
  return;
}

/**
 * @fn static gboolean gtk_turn_coordinator_button_press_event (GtkWidget * widget, GdkEventButton * ev)
 * @brief Special Gtk API function. Perform mouse button press events. 
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static gboolean gtk_turn_coordinator_button_press_event (GtkWidget * widget, GdkEventButton * ev)
{
  GtkTurnCoordinatorPrivate *priv;
  gint x = 0, y = 0;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_button_press_event_cb()");
  }
  g_return_val_if_fail (IS_GTK_TURN_COORDINATOR (widget), FALSE);

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (widget);

  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 1))
  {
    gdk_window_get_pointer (ev->window, &x, &y, &priv->mouse_state);
    priv->mouse_pos.x = x;
    priv->mouse_pos.y = y;
    return TRUE;
  }
  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 2) && priv->b_mouse_onoff)
  {
    gtk_turn_coordinator_debug = gtk_turn_coordinator_debug ? FALSE : TRUE;
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
 * @fn static gboolean gtk_turn_coordinator_motion_notify_event (GtkWidget * widget, GdkEventMotion * ev)
 * @brief Special Gtk API function. Perform mouse motion events. 
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static gboolean gtk_turn_coordinator_motion_notify_event (GtkWidget * widget, GdkEventMotion * ev)
{
  GtkTurnCoordinatorPrivate *priv;
  GdkModifierType state;
  gint x = 0, y = 0;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_motion_notify_event_cb()");
  }
  g_return_val_if_fail (IS_GTK_TURN_COORDINATOR (widget), FALSE);

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (widget);

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

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_motion_notify_event_cb() : mouse x=%d, y=%d", x, y);
  }

  return TRUE;
}

/**
 * @fn static void gtk_turn_coordinator_destroy (GtkObject * object)
 * @brief Special Gtk API function. 
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_turn_coordinator_destroy (GtkObject * object)
{
  GtkTurnCoordinatorPrivate *priv = NULL;
  GtkWidget *widget = NULL;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_destroy(enter)");
  }

  g_return_if_fail (object != NULL);

  widget = GTK_WIDGET (object);

  g_return_if_fail (IS_GTK_TURN_COORDINATOR (widget));

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (widget);
  g_return_if_fail (priv != NULL);

  if (priv->cr)
  {
    g_free (priv->cr);

    if (GTK_OBJECT_CLASS (gtk_turn_coordinator_parent_class)->destroy != NULL)
    {
      (*GTK_OBJECT_CLASS (gtk_turn_coordinator_parent_class)->destroy) (object);
    }
  }
  if (gtk_turn_coordinator_debug)
  {
    g_debug ("gtk_turn_coordinator_destroy(exit)");
  }

  return;
}

/**
 * @fn static void gtk_turn_coordinator_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec)
 * @brief Special Gtk API function. 
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_turn_coordinator_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec)
{
  GtkTurnCoordinatorPrivate *priv = NULL;
  GtkTurnCoordinator *vario = NULL;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_set_property()");
  }
  g_return_if_fail (object != NULL);

  vario = GTK_TURN_COORDINATOR (object);
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (vario));

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (vario);
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
