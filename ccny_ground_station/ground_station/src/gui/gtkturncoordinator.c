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
 * @version 0.2
 * @date 02/09/2010
 *
 * Gtk Turn Coordinator Widget <br>
 * Copyright (C) 2010, CCNY Robotics Lab <br>
 * http://robotics.ccny.cuny.edu <br>
 * 
 * This widget provide an easy to read turn coordinator instrument. <br>
 * The design is volontary based on a real turn coordinator flight instrument <br>
 * in order to be familiar for aircraft and helicopter pilots.<br>
 * 
 * @b Pictures:<br>
 * <table><tr>
 * <th><IMG SRC="http://www.ros.org/wiki/ground_station?action=AttachFile&do=get&target=gtkturncoordinator.png"></th>
 * <th><IMG SRC="http://www.ros.org/wiki/ground_station?action=AttachFile&do=get&target=gtkturncoordinator_g.png"></th>
 * </tr></table>
 * 
 * @b Example: <br>
 * Add Turn Coordinator widget to an gtkvbox and set some params <br>
 * @code
 * window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
 * vbox = gtk_vbox_new(TRUE, 1);
 * gtk_container_add(GTK_CONTAINER (window), vbox);
 * 
 * tc = gtk_turn_coordinator_new();
 * g_object_set(GTK_TURN_COORDINATOR (tc),
 *		"grayscale-color", false,
 *		"radial-color", true, NULL);
 * 
 * gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(tc), TRUE, TRUE, 0);
 * gtk_widget_show_all(window);
 * @endcode
 * 
 * The following code show how to change widget's values and redraw it:<br>
 * Note that here tc's type is "GtkWidget *".<br>
 * @code
 * if (IS_GTK_TURN_COORDINATOR (tc))
 * {
 *	gtk_turn_coordinator_set_value (GTK_TURN_COORDINATOR (tc), plane_angle,ball_tr);
 *	gtk_turn_coordinator_redraw(GTK_TURN_COORDINATOR(tc));
 * }		
 * @endcode
 * 
  @b Widget @b Parameters:<br>
 * - "grayscale-colors": boolean, if TRUE, draw the widget with grayscale colors (outdoor use)<br>
 * - "radial-color": boolean, if TRUE, draw a fake light reflexion<br>
 * 
 * @b Widget @b values:<br>
 * - "plane_angle": double, provide the plane rotation - the value is from 0 to 360.<br>
 * - "ball_tr": double, provide the ball translation - the value is from -100 to 100
 * 
 */

#include <ground_station/gui/gtkturncoordinator.h>

/**
 * @typedef struct GtkTurnCoordinatorPrivate 
 * @brief Special Gtk API strucure. Allow to add a private data<br>
 * for the widget. Defined in the C file in order to be private.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkTurnCoordinatorPrivate
{
  /* cairo context */
  gboolean draw_once;
  cairo_surface_t * static_surface;
  cairo_surface_t * dynamic_surface;

  /* widget data */
  gboolean grayscale_color;
  gboolean radial_color;
  gdouble plane_value;
  gdouble ball_value;

  /* drawing data */
  gdouble x;
  gdouble y;
  gdouble radius;
  GdkColor bg_color_inv;
  GdkColor bg_color_tc;
  GdkColor bg_color_bounderie;
  GdkColor bg_radial_color_begin_tc;
  GdkColor bg_radial_color_begin_bounderie;

  /* mouse information */
  gboolean b_mouse_onoff;
  GdkPoint mouse_pos;
  GdkModifierType mouse_state;

} GtkTurnCoordinatorPrivate;

/**
 * @enum _GTK_TURN_COORDINATOR_PROPERTY_ID
 * @brief Special Gtk API enum. Allow to identify widget's properties.
 *
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
enum _GTK_TURN_COORDINATOR_PROPERTY_ID
{
  PROP_0,
  PROP_GRAYSCALE_COLOR,
  PROP_RADIAL_COLOR,
} GTK_TURN_COORDINATOR_PROPERTY_ID;

/**
 * @fn G_DEFINE_TYPE (GtkTurnCoordinator, gtk_turn_coordinator, GTK_TYPE_DRAWING_AREA);
 * @brief Special Gtk API function. Define a new object type named GtkTurnCoordinator <br>
 * and all preface of the widget's functions calls with gtk_turn_coordinator.<br>
 * We are inheriting the type of GtkDrawingArea.<br>
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
G_DEFINE_TYPE (GtkTurnCoordinator, gtk_turn_coordinator, GTK_TYPE_DRAWING_AREA);

/**
 * @def GTK_TURN_COORDINATOR_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GTK_TURN_COORDINATOR_TYPE, GtkTurnCoordinatorPrivate))
 * @brief Special Gtk API define. Add a macro for easy access to the private<br>
 * data struct.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
#define GTK_TURN_COORDINATOR_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GTK_TURN_COORDINATOR_TYPE, GtkTurnCoordinatorPrivate))

static void gtk_turn_coordinator_class_init (GtkTurnCoordinatorClass * klass);
static void gtk_turn_coordinator_init (GtkTurnCoordinator * tc);
static void gtk_turn_coordinator_destroy (GtkObject * object);
static void gtk_turn_coordinator_set_property (GObject * object, guint prop_id, const GValue * value,
                                               GParamSpec * pspec);

static gboolean gtk_turn_coordinator_configure_event (GtkWidget * widget, GdkEventConfigure * event);
static gboolean gtk_turn_coordinator_expose (GtkWidget * graph, GdkEventExpose * event);
static gboolean gtk_turn_coordinator_button_press_event (GtkWidget * widget, GdkEventButton * ev);

static void gtk_turn_coordinator_draw_static (GtkWidget * tc, cairo_t * cr);
static void gtk_turn_coordinator_draw_base (GtkWidget * tc, cairo_t * cr);
static void gtk_turn_coordinator_draw_screws (GtkWidget * tc, cairo_t * cr);

static void gtk_turn_coordinator_draw_dynamic (GtkWidget * tc, cairo_t * cr);
static void gtk_turn_coordinator_draw_ball (GtkWidget * tc, cairo_t * cr);
static void gtk_turn_coordinator_draw_plane (GtkWidget * tc, cairo_t * cr);

static gboolean gtk_turn_coordinator_debug = FALSE;
static gboolean gtk_turn_coordinator_lock_update = FALSE;

/**
 * @fn static void gtk_turn_coordinator_class_init (GtkTurnCoordinatorClass * klass)
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
 * Also register the private struct GtkTurnCoordinatorPrivate with<br>
 * the class and install widget properties.
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
  widget_class->button_press_event = gtk_turn_coordinator_button_press_event;

  g_type_class_add_private (obj_class, sizeof (GtkTurnCoordinatorPrivate));

  g_object_class_install_property (obj_class,
                                   PROP_GRAYSCALE_COLOR,
                                   g_param_spec_boolean ("grayscale-color",
                                                         "use grayscale for the widget color",
                                                         "use grayscale for the widget color", FALSE,
                                                         G_PARAM_WRITABLE));
  g_object_class_install_property (obj_class, PROP_RADIAL_COLOR,
                                   g_param_spec_boolean ("radial-color", "the widget use radial color",
                                                         "the widget use radial color", TRUE, G_PARAM_WRITABLE));
  return;
}

/**
 * @fn static void gtk_turn_coordinator_init (GtkTurnCoordinator * tc)
 * @brief Special Gtk API function. Function called when the creating a<br>
 * new GtkTurnCoordinator. Allow to initialize some private variables of<br>
 * widget.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_turn_coordinator_init (GtkTurnCoordinator * tc)
{
  GtkTurnCoordinatorPrivate *priv = NULL;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_init()");
  }
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (tc));

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (tc);

  gtk_widget_add_events (GTK_WIDGET (tc), GDK_BUTTON_PRESS_MASK |
                         GDK_BUTTON_RELEASE_MASK | GDK_POINTER_MOTION_MASK | GDK_POINTER_MOTION_HINT_MASK);
  priv->b_mouse_onoff = FALSE;
  priv->draw_once = FALSE;
  priv->grayscale_color = FALSE;
  priv->radial_color = TRUE;
  priv->ball_value = 0;
  priv->plane_value = 0;

  priv->bg_color_bounderie.red = 6553.5;        // 0.1 cairo
  priv->bg_color_bounderie.green = 6553.5;
  priv->bg_color_bounderie.blue = 6553.5;
  priv->bg_color_tc.red = 3276.75;      // 0.05 cairo
  priv->bg_color_tc.green = 3276.75;
  priv->bg_color_tc.blue = 3276.75;
  priv->bg_color_inv.red = 45874.5;     // 0.7 cairo
  priv->bg_color_inv.green = 45874.5;
  priv->bg_color_inv.blue = 45874.5;
  priv->bg_radial_color_begin_bounderie.red = 13107;    // 0.2 cairo
  priv->bg_radial_color_begin_bounderie.green = 13107;
  priv->bg_radial_color_begin_bounderie.blue = 13107;
  priv->bg_radial_color_begin_tc.red = 45874.5; // 0.7 cairo
  priv->bg_radial_color_begin_tc.green = 45874.5;
  priv->bg_radial_color_begin_tc.blue = 45874.5;
  return;
}

/**
 * @fn static gboolean gtk_turn_coordinator_configure_event (GtkWidget * widget, GdkEventConfigure * event)
 * @brief Special Gtk API function. Override the _configure_event handler<br>
 * in order to resize the widget when the main window is resized.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static gboolean gtk_turn_coordinator_configure_event (GtkWidget * widget, GdkEventConfigure * event)
{
  GtkTurnCoordinatorPrivate *priv;
  GtkTurnCoordinator *tc = GTK_TURN_COORDINATOR (widget);

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_configure_event()");
  }
  g_return_val_if_fail (IS_GTK_TURN_COORDINATOR (tc), FALSE);
  g_return_val_if_fail (event->type == GDK_CONFIGURE, FALSE);

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (tc);
  g_return_val_if_fail (priv != NULL, FALSE);

  priv->draw_once = FALSE;

  return FALSE;
}

/**
 * @fn static gboolean gtk_turn_coordinator_expose (GtkWidget * tc, GdkEventExpose * event)
 * @brief Special Gtk API function. Override of the expose handler.<br>
 * An “expose-event” signal is emitted when the widget need to be drawn.<br>
 * A Cairo context is created for the parent's GdkWindow.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static gboolean gtk_turn_coordinator_expose (GtkWidget * tc, GdkEventExpose * event)
{
  GtkTurnCoordinatorPrivate *priv;
  GtkWidget *widget = tc;
  cairo_t * cr_final;
  cairo_t * cr_static;
  cairo_t * cr_dynamic;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_expose()");
  }
  g_return_val_if_fail (IS_GTK_TURN_COORDINATOR (tc), FALSE);

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (tc);
  g_return_val_if_fail (priv != NULL, FALSE);

  cr_final = gdk_cairo_create (widget->window);
  
  if(!priv->draw_once)
  {
		priv->static_surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, event->area.width, event->area.height);
		cr_static = cairo_create(priv->static_surface);
  
		cairo_rectangle (cr_static, event->area.x, event->area.y, event->area.width, event->area.height);
		cairo_clip (cr_static);
		gtk_turn_coordinator_draw_static (tc,cr_static);
		cairo_destroy (cr_static);
		priv->draw_once=TRUE;
  }

  priv->dynamic_surface = cairo_surface_create_similar (priv->static_surface,CAIRO_CONTENT_COLOR_ALPHA,event->area.width, event->area.height);
  cr_dynamic = cairo_create(priv->dynamic_surface);
  cairo_rectangle (cr_dynamic, event->area.x, event->area.y, event->area.width, event->area.height);
  cairo_clip (cr_dynamic);
  gtk_turn_coordinator_draw_dynamic (tc, cr_dynamic);
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
 * @fn extern void gtk_turn_coordinator_redraw (GtkTurnCoordinator * tc)
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
extern void gtk_turn_coordinator_redraw (GtkTurnCoordinator * tc)
{
  GtkWidget *widget;
  GdkRegion *region;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_redraw()");
  }
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (tc));

  widget = GTK_WIDGET (tc);

  if (!widget->window)
    return;

  region = gdk_drawable_get_clip_region (widget->window);
  /* redraw the window completely by exposing it */
  gdk_window_invalidate_region (widget->window, region, TRUE);
  gdk_window_process_updates (widget->window, TRUE);

  gdk_region_destroy (region);
}

/**
 * @fn extern void gtk_turn_coordinator_set_value (GtkTurnCoordinator * tc, gdouble plane_angle, gdouble ball_val)
 * @brief Public widget's function that allow the main program/user to<br>
 * set the internal value variable of the widget. 
 * 
 * Here, two values have to be set:<br>
 * "plane_angle": double, provide the plane rotation - the value is from 0 to 360.<br>
 * "ball_tr": double, provide the ball translation - the value is from -100 to 100
 */
extern void gtk_turn_coordinator_set_value (GtkTurnCoordinator * tc, gdouble plane_angle, gdouble ball_val)
{
  GtkTurnCoordinatorPrivate *priv;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_set_value()");
  }
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (tc));

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (tc);

  if(!gtk_turn_coordinator_lock_update)
  {
		if ((plane_angle >= 0) && (plane_angle <= 360))
			priv->plane_value = plane_angle;
		else
			g_warning ("GtkTurnCoordinator : gtk_turn_coordinator_set_value : value out of range");
	
		if ((ball_val >= -100) && (ball_val <= 100))
			priv->ball_value = ball_val;
		else
			g_warning ("GtkTurnCoordinator : gtk_turn_coordinator_set_value : value out of range");
  }
  return;
}

/**
 * @fn extern GtkWidget *gtk_turn_coordinator_new (void)
 * @brief Special Gtk API function. This function is simply a wrapper<br>
 * for convienience.
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
 * @fn static void gtk_turn_coordinator_draw_static (GtkWidget * tc, cairo_t * cr)
 * @brief Special Gtk API function. This function use the cairo context<br>
 * created before in order to draw scalable graphics. 
 * 
 * See GObject,Cairo and GTK+ references for more informations: 
 * http://library.gnome.org/devel/references.html.en
 */
static void gtk_turn_coordinator_draw_static (GtkWidget * tc, cairo_t * cr)
{
  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_draw_static()");
  }
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (tc));

  gtk_turn_coordinator_draw_base (tc,cr);
  gtk_turn_coordinator_draw_screws (tc,cr);
}

/**
 * @fn static void gtk_turn_coordinator_draw_dynamic (GtkWidget * tc, cairo_t * cr)
 * @brief Special Gtk API function. This function use the cairo context<br>
 * created before in order to draw scalable graphics. 
 * 
 * See GObject,Cairo and GTK+ references for more informations: 
 * http://library.gnome.org/devel/references.html.en
 */
static void gtk_turn_coordinator_draw_dynamic (GtkWidget * tc, cairo_t * cr)
{
  GtkTurnCoordinatorPrivate *priv;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_draw_dynamic()");
  }
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (tc));
  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (tc);

  double x = priv->x;
  double y = priv->y;
  double radius = priv->radius;

  gtk_turn_coordinator_draw_ball (tc,cr);
  
  cairo_set_line_width (cr, 0.015 * radius);
  cairo_move_to (cr, x + 0.09 * radius, y + 0.35 * radius);
  cairo_line_to (cr, x + 0.09 * radius, y + 0.54 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0, 0, 0);
  else
    cairo_set_source_rgb (cr, 1, 1, 1);
  cairo_stroke (cr);

  cairo_set_line_width (cr, 0.015 * radius);
  cairo_move_to (cr, x - 0.09 * radius, y + 0.35 * radius);
  cairo_line_to (cr, x - 0.09 * radius, y + 0.54 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0, 0, 0);
  else
    cairo_set_source_rgb (cr, 1, 1, 1);
  cairo_stroke (cr);
  
  gtk_turn_coordinator_draw_plane (tc,cr);
}

/**
 * @fn static void gtk_turn_coordinator_draw (GtkWidget * tc, cairo_t * cr)
 * @brief Special Gtk API function. This function use the cairo context<br>
 * created before in order to draw scalable graphics. 
 * 
 * See GObject,Cairo and GTK+ references for more informations: 
 * http://library.gnome.org/devel/references.html.en
 */
static void gtk_turn_coordinator_draw_base (GtkWidget * tc, cairo_t * cr)
{
  GtkTurnCoordinatorPrivate *priv;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_draw()");
  }
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (tc));

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (tc);

  double x, y, rec_x0, rec_y0, rec_width, rec_height, rec_degrees;
  double rec_aspect, rec_corner_radius, rec_radius, radius;
  double x0, y0, x1, y1, x2, y2, x3, y3;

  x = tc->allocation.width / 2;
  y = tc->allocation.height / 2;
  radius = MIN (tc->allocation.width / 2, tc->allocation.height / 2) - 5;
  cairo_pattern_t *pat = NULL;

  rec_x0 = x - radius;
  rec_y0 = y - radius;
  rec_width = radius * 2;
  rec_height = radius * 2;
  rec_aspect = 1.0;
  rec_corner_radius = rec_height / 8.0;

  rec_radius = rec_corner_radius / rec_aspect;
  rec_degrees = M_PI / 180.0;

  // **** turn coordinator base
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

  cairo_arc (cr, x, y, radius, 0, 2 * M_PI);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0., 0., 0.);
  else
    cairo_set_source_rgb (cr, 1., 1., 1.);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  cairo_arc (cr, x, y, radius - 0.04 * radius, 0, 2 * M_PI);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0.6, 0.5, 0.5);
  else
    cairo_set_source_rgb (cr, 1 - 0.6, 1 - 0.5, 1 - 0.5);
  cairo_stroke (cr);

  cairo_set_line_width (cr, 0.01 * radius);
  radius = radius - 0.1 * radius;
  cairo_arc (cr, x, y, radius, 0, 2 * M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_tc.red / 65535,
                            (gdouble) priv->bg_color_tc.green / 65535, (gdouble) priv->bg_color_tc.blue / 65535);
    else
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_tc.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_tc.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_tc.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, (gdouble) priv->bg_color_tc.red / 65535,
                                       (gdouble) priv->bg_color_tc.green / 65535,
                                       (gdouble) priv->bg_color_tc.blue / 65535, 1);
    cairo_set_source (cr, pat);
  }
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  cairo_arc (cr, x, y, radius, 0, 2 * M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_tc.red / 65535,
                            (gdouble) priv->bg_color_tc.green / 65535, (gdouble) priv->bg_color_tc.blue / 65535);
    else
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_tc.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_tc.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_tc.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, (gdouble) priv->bg_color_tc.red / 65535,
                                       (gdouble) priv->bg_color_tc.green / 65535,
                                       (gdouble) priv->bg_color_tc.blue / 65535, 1);
    cairo_set_source (cr, pat);
  }
  cairo_fill (cr);
  cairo_stroke (cr);

  // **** "turn coordinator" drawing
  cairo_select_font_face (cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 1, 1, 1);
  else
    cairo_set_source_rgb (cr, 0, 0, 0);
  cairo_set_font_size (cr, 0.11 * radius);
  cairo_move_to (cr, x - 0.15 * radius, y - 0.44 * radius);
  cairo_show_text (cr, "TURN");
  cairo_stroke (cr);
  cairo_move_to (cr, x - 0.41 * radius, y - 0.32 * radius);
  cairo_show_text (cr, "COORDINATOR");
  cairo_stroke (cr);

  // **** turn coordinator ticks
  cairo_set_line_width (cr, 0.06 * radius);
  cairo_move_to (cr, x + (radius - 0.1 * radius) * cos (M_PI), y + (radius - 0.1 * radius) * sin (M_PI));
  cairo_line_to (cr, x + (radius - 0.25 * radius) * cos (M_PI), y + (radius - 0.25 * radius) * sin (M_PI));
  cairo_stroke (cr);

  cairo_move_to (cr, x + (radius - 0.1 * radius) * cos (0), y + (radius - 0.1 * radius) * sin (0));
  cairo_line_to (cr, x + (radius - 0.25 * radius) * cos (0), y + (radius - 0.25 * radius) * sin (0));
  cairo_stroke (cr);

  cairo_move_to (cr, x + (radius - 0.1 * radius) * cos (M_PI - M_PI / 8),
                 y + (radius - 0.1 * radius) * sin (M_PI - M_PI / 8));
  cairo_line_to (cr, x + (radius - 0.25 * radius) * cos (M_PI - M_PI / 8),
                 y + (radius - 0.25 * radius) * sin (M_PI - M_PI / 8));
  cairo_stroke (cr);

  cairo_move_to (cr, x + (radius - 0.1 * radius) * cos (M_PI / 8), y + (radius - 0.1 * radius) * sin (M_PI / 8));
  cairo_line_to (cr, x + (radius - 0.25 * radius) * cos (M_PI / 8),
                 y + (radius - 0.25 * radius) * sin (M_PI / 8));
  cairo_stroke (cr);

  // **** "R" & "L" drawing
  cairo_select_font_face (cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
  cairo_set_font_size (cr, 0.16 * radius);
  cairo_move_to (cr, x + (radius - 0.15 * radius) * cos (M_PI / 8 + M_PI / 12),
                 y + (radius - 0.15 * radius) * sin (M_PI / 8 + M_PI / 12));
  cairo_show_text (cr, "R");
  cairo_stroke (cr);
  cairo_move_to (cr, x + (radius - 0.02 * radius) * cos (M_PI - M_PI / 8 - M_PI / 12),
                 y + (radius - 0.15 * radius) * sin (M_PI - M_PI / 8 - M_PI / 12));
  cairo_show_text (cr, "L");
  cairo_stroke (cr);

  // **** "NO PITCH INFORMATION" drawing
  cairo_select_font_face (cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size (cr, 0.08 * radius);
  cairo_move_to (cr, x - 0.19 * radius, y + 0.85 * radius);
  cairo_show_text (cr, "NO PITCH");
  cairo_stroke (cr);
  cairo_move_to (cr, x - 0.28 * radius, y + 0.94 * radius);
  cairo_show_text (cr, "INFORMATION");
  cairo_stroke (cr);

  // **** balle spot
  cairo_set_line_width (cr, 0.02 * radius);
  cairo_new_sub_path (cr);
  x0 = x;
  y0 = y + 0.35 * radius;
  x1 = x - 0.4 * radius;
  y1 = y + 0.35 * radius;
  x2 = x - 0.65 * radius;
  y2 = y + 0.2 * radius;
  x3 = x - 0.6 * radius;
  y3 = y + 0.35 * radius;
  cairo_move_to (cr, x0, y0);
  cairo_curve_to (cr, x1, y1, x2, y2, x3, y3);
  x1 = x - 0.6 * radius;
  y1 = y + 0.4 * radius;
  x2 = x - 0.5 * radius;
  y2 = y + 0.55 * radius;
  x3 = x;
  y3 = y + 0.55 * radius;
  cairo_curve_to (cr, x1, y1, x2, y2, x3, y3);
  cairo_close_path (cr);
  cairo_fill_preserve (cr);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0.2, 0.2, 0.2);
  else
    cairo_set_source_rgb (cr, 1 - 0.2, 1 - 0.2, 1 - 0.2);
  cairo_stroke (cr);

  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 1, 1, 1);
  else
    cairo_set_source_rgb (cr, 0, 0, 0);
  cairo_new_sub_path (cr);
  x0 = x;
  y0 = y + 0.35 * radius;
  x1 = x + 0.4 * radius;
  y1 = y + 0.35 * radius;
  x2 = x + 0.65 * radius;
  y2 = y + 0.2 * radius;
  x3 = x + 0.6 * radius;
  y3 = y + 0.35 * radius;
  cairo_move_to (cr, x0, y0);
  cairo_curve_to (cr, x1, y1, x2, y2, x3, y3);
  x1 = x + 0.6 * radius;
  y1 = y + 0.4 * radius;
  x2 = x + 0.5 * radius;
  y2 = y + 0.55 * radius;
  x3 = x;
  y3 = y + 0.55 * radius;
  cairo_curve_to (cr, x1, y1, x2, y2, x3, y3);
  cairo_close_path (cr);
  cairo_fill_preserve (cr);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0.2, 0.2, 0.2);
  else
    cairo_set_source_rgb (cr, 1 - 0.2, 1 - 0.2, 1 - 0.2);
  cairo_stroke (cr);

  cairo_set_line_width (cr, 0.04 * radius);
  cairo_move_to (cr, x, y + 0.36 * radius);
  cairo_line_to (cr, x, y + 0.54 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 1, 1, 1);
  else
    cairo_set_source_rgb (cr, 0, 0, 0);
  cairo_stroke (cr);

  // **** alpha effect
  cairo_arc (cr, x, y, radius - 0.25 * radius, 0, 2 * M_PI);
  pat = cairo_pattern_create_radial (x, y, radius - 0.35 * radius, x, y, radius - 0.25 * radius);
  cairo_pattern_add_color_stop_rgba (pat, 0, 0.3, 0.3, 0.3, 0.2);
  cairo_pattern_add_color_stop_rgba (pat, 1, 0.3, 0.3, 0.3, 0.6);
  cairo_set_source (cr, pat);
  cairo_fill (cr);
  cairo_stroke (cr);

  priv->radius = radius;
  priv->x = x;
  priv->y = y;
  cairo_pattern_destroy (pat);
  return;
}

/**
 * @fn static void gtk_turn_coordinator_draw_screws (GtkWidget * tc,cairo_t * cr)
 * @brief Private widget's function that draw the widget's screws using cairo.
 */
static void gtk_turn_coordinator_draw_screws (GtkWidget * tc,cairo_t * cr)
{
  GtkTurnCoordinatorPrivate *priv;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_draw_screws()");
  }
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (tc));

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (tc);

  cairo_pattern_t *pat = NULL;
  double x, y, radius;
  radius = priv->radius;
  x = priv->x;
  y = priv->y;
  radius = radius + 0.12 * radius;

  // **** top left screw
  cairo_arc (cr, x - 0.82 * radius, y - 0.82 * radius, 0.1 * radius, 0, 2 * M_PI);
  pat = cairo_pattern_create_radial (x - 0.82 * radius, y - 0.82 * radius, 0.07 * radius,
                                     x - 0.82 * radius, y - 0.82 * radius, 0.1 * radius);
  cairo_pattern_add_color_stop_rgba (pat, 0, 0, 0, 0, 0.7);
  cairo_pattern_add_color_stop_rgba (pat, 1, 0, 0, 0, 0.1);
  cairo_set_source (cr, pat);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  cairo_arc (cr, x - 0.82 * radius, y - 0.82 * radius, 0.07 * radius, 0, 2 * M_PI);
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
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15, 0.15, 0.15, 1);
    cairo_set_source (cr, pat);
  }
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  cairo_set_line_width (cr, 0.02 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0., 0., 0.);
  else
    cairo_set_source_rgb (cr, 1., 1., 1.);
  cairo_move_to (cr, x - 0.88 * radius, y - 0.82 * radius);
  cairo_line_to (cr, x - 0.76 * radius, y - 0.82 * radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_move_to (cr, x - 0.82 * radius, y - 0.88 * radius);
  cairo_line_to (cr, x - 0.82 * radius, y - 0.76 * radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_set_line_width (cr, 0.01 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0.1, 0.1, 0.1);
  else
    cairo_set_source_rgb (cr, 0.9, 0.9, 0.9);
  cairo_move_to (cr, x - 0.88 * radius, y - 0.82 * radius);
  cairo_line_to (cr, x - 0.76 * radius, y - 0.82 * radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_move_to (cr, x - 0.82 * radius, y - 0.88 * radius);
  cairo_line_to (cr, x - 0.82 * radius, y - 0.76 * radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  // **** top right screw
  cairo_arc (cr, x + 0.82 * radius, y - 0.82 * radius, 0.1 * radius, 0, 2 * M_PI);
  pat = cairo_pattern_create_radial (x + 0.82 * radius, y - 0.82 * radius, 0.07 * radius,
                                     x + 0.82 * radius, y - 0.82 * radius, 0.1 * radius);
  cairo_pattern_add_color_stop_rgba (pat, 0, 0, 0, 0, 0.7);
  cairo_pattern_add_color_stop_rgba (pat, 1, 0, 0, 0, 0.1);
  cairo_set_source (cr, pat);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  cairo_arc (cr, x + 0.82 * radius, y - 0.82 * radius, 0.07 * radius, 0, 2 * M_PI);
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
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15, 0.15, 0.15, 1);
    cairo_set_source (cr, pat);
  }
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  cairo_set_line_width (cr, 0.02 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0., 0., 0.);
  else
    cairo_set_source_rgb (cr, 1., 1., 1.);
  cairo_move_to (cr, x + 0.88 * radius, y - 0.82 * radius);
  cairo_line_to (cr, x + 0.76 * radius, y - 0.82 * radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_move_to (cr, x + 0.82 * radius, y - 0.88 * radius);
  cairo_line_to (cr, x + 0.82 * radius, y - 0.76 * radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_set_line_width (cr, 0.01 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0.1, 0.1, 0.1);
  else
    cairo_set_source_rgb (cr, 0.9, 0.9, 0.9);
  cairo_move_to (cr, x + 0.88 * radius, y - 0.82 * radius);
  cairo_line_to (cr, x + 0.76 * radius, y - 0.82 * radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_move_to (cr, x + 0.82 * radius, y - 0.88 * radius);
  cairo_line_to (cr, x + 0.82 * radius, y - 0.76 * radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  // **** bottom left screw
  cairo_arc (cr, x - 0.82 * radius, y + 0.82 * radius, 0.1 * radius, 0, 2 * M_PI);
  pat = cairo_pattern_create_radial (x - 0.82 * radius, y + 0.82 * radius, 0.07 * radius,
                                     x - 0.82 * radius, y + 0.82 * radius, 0.1 * radius);
  cairo_pattern_add_color_stop_rgba (pat, 0, 0, 0, 0, 0.7);
  cairo_pattern_add_color_stop_rgba (pat, 1, 0, 0, 0, 0.1);
  cairo_set_source (cr, pat);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  cairo_arc (cr, x - 0.82 * radius, y + 0.82 * radius, 0.07 * radius, 0, 2 * M_PI);
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
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15, 0.15, 0.15, 1);
    cairo_set_source (cr, pat);
  }
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  cairo_set_line_width (cr, 0.02 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0., 0., 0.);
  else
    cairo_set_source_rgb (cr, 1., 1., 1.);
  cairo_move_to (cr, x - 0.88 * radius, y + 0.82 * radius);
  cairo_line_to (cr, x - 0.76 * radius, y + 0.82 * radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_move_to (cr, x - 0.82 * radius, y + 0.88 * radius);
  cairo_line_to (cr, x - 0.82 * radius, y + 0.76 * radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_set_line_width (cr, 0.01 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0.1, 0.1, 0.1);
  else
    cairo_set_source_rgb (cr, 0.9, 0.9, 0.9);
  cairo_move_to (cr, x - 0.88 * radius, y + 0.82 * radius);
  cairo_line_to (cr, x - 0.76 * radius, y + 0.82 * radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_move_to (cr, x - 0.82 * radius, y + 0.88 * radius);
  cairo_line_to (cr, x - 0.82 * radius, y + 0.76 * radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  // **** bottom right screw
  cairo_arc (cr, x + 0.82 * radius, y + 0.82 * radius, 0.1 * radius, 0, 2 * M_PI);
  pat = cairo_pattern_create_radial (x + 0.82 * radius, y + 0.82 * radius, 0.07 * radius,
                                     x + 0.82 * radius, y + 0.82 * radius, 0.1 * radius);
  cairo_pattern_add_color_stop_rgba (pat, 0, 0, 0, 0, 0.7);
  cairo_pattern_add_color_stop_rgba (pat, 1, 0, 0, 0, 0.1);
  cairo_set_source (cr, pat);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  cairo_arc (cr, x + 0.82 * radius, y + 0.82 * radius, 0.07 * radius, 0, 2 * M_PI);
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
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15, 0.15, 0.15, 1);
    cairo_set_source (cr, pat);
  }
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  cairo_set_line_width (cr, 0.02 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0., 0., 0.);
  else
    cairo_set_source_rgb (cr, 1., 1., 1.);
  cairo_move_to (cr, x + 0.88 * radius, y + 0.82 * radius);
  cairo_line_to (cr, x + 0.76 * radius, y + 0.82 * radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_move_to (cr, x + 0.82 * radius, y + 0.88 * radius);
  cairo_line_to (cr, x + 0.82 * radius, y + 0.76 * radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_set_line_width (cr, 0.01 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0.1, 0.1, 0.1);
  else
    cairo_set_source_rgb (cr, 0.9, 0.9, 0.9);
  cairo_move_to (cr, x + 0.88 * radius, y + 0.82 * radius);
  cairo_line_to (cr, x + 0.76 * radius, y + 0.82 * radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_move_to (cr, x + 0.82 * radius, y + 0.88 * radius);
  cairo_line_to (cr, x + 0.82 * radius, y + 0.76 * radius);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_pattern_destroy (pat);
  return;
}

/**
 * @fn static void gtk_turn_coordinator_draw_ball (GtkWidget * tc,cairo_t * cr)
 * @brief Private widget's function that draw the turn_coordinator ball using cairo.
 */
static void gtk_turn_coordinator_draw_ball (GtkWidget * tc,cairo_t * cr)
{
  GtkTurnCoordinatorPrivate *priv;
  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_draw_ball()");
  }
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (tc));

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (tc);

  double incx, incy;
  double x = priv->x;
  double y = priv->y;
  double radius = priv->radius;
  cairo_pattern_t *pat = NULL;

  if (((priv->ball_value >= 80) && (priv->ball_value <= 100))
      || ((priv->ball_value <= -80) && (priv->ball_value >= -100)))
  {
    incx = (priv->ball_value * 0.4) / 100;
    incy = 0.028 + ((fabs (priv->ball_value) - 80) * 0.017 / 20);
  }
  else if (((priv->ball_value >= 60) && (priv->ball_value < 80))
           || ((priv->ball_value <= -60) && (priv->ball_value > -80)))
  {
    incx = (priv->ball_value * 0.4) / 100;
    incy = 0.016 + ((fabs (priv->ball_value) - 60) * 0.012 / 20);
  }
  else if (((priv->ball_value >= 40) && (priv->ball_value < 60))
           || ((priv->ball_value <= -40) && (priv->ball_value > -60)))
  {
    incx = (priv->ball_value * 0.4) / 100;
    incy = 0.006 + ((fabs (priv->ball_value) - 40) * 0.006 / 20);
  }
  else if (((priv->ball_value > 0) && (priv->ball_value < 40)) || ((priv->ball_value < 0) && (priv->ball_value > -40)))
  {
    incx = (priv->ball_value * 0.4) / 100;
    incy = (fabs (priv->ball_value) * 0.006 / 40);
  }
  else
  {
    incx = 0;
    incy = 0;
  }

  x = x + incx * radius;
  y = y - incy * radius;

  cairo_arc (cr, x, y + 0.36 * radius + 0.09 * radius, 0.09 * radius, 0, 2 * M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (cr, 0, 0, 0);
    else
      cairo_set_source_rgb (cr, 1, 1, 1);
  }
  else
  {
    pat =
      cairo_pattern_create_radial (x - 0.02 * radius, y + 0.36 * radius + 0.09 * radius - 0.03 * radius, 0.01 * radius,
                                   x - 0.015 * radius, y + 0.36 * radius + 0.09 * radius - 0.025 * radius,
                                   0.09 * radius);
    cairo_pattern_add_color_stop_rgb (pat, 0, 0.9, 0.9, 0.9);
    cairo_pattern_add_color_stop_rgb (pat, 0.1, 0.7, 0.7, 0.7);
    cairo_pattern_add_color_stop_rgb (pat, 0.5, 0.2, 0.2, 0.2);
    cairo_pattern_add_color_stop_rgb (pat, 1, 0., 0., 0.);
    cairo_set_source (cr, pat);
  }

  cairo_fill (cr);
  cairo_stroke (cr);
  cairo_pattern_destroy (pat);
  return;
}


/**
 * @fn static void gtk_turn_coordinator_draw_plane (GtkWidget * tc,cairo_t * cr)
 * @brief Private widget's function that draw the turn coordinator plane using cairo.
 */
static void gtk_turn_coordinator_draw_plane (GtkWidget * tc,cairo_t * cr)
{
  GtkTurnCoordinatorPrivate *priv;
  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_draw_plane()");
  }
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (tc));

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (tc);

  double x = priv->x;
  double y = priv->y;
  double radius = priv->radius;

  // **** plane drawing
  cairo_set_line_width (cr, 0.02 * radius);
  cairo_arc (cr, x, y, radius - 0.9 * radius, 0, 2 * M_PI);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 1., 1., 1.);
  else
    cairo_set_source_rgb (cr, 0., 0., 0.);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  cairo_move_to (cr, x, y);

  cairo_save (cr);
  cairo_translate (cr, x, y);
  x = 0;
  y = 0;
  cairo_rotate (cr, DEG2RAD (priv->plane_value));

  cairo_new_sub_path (cr);
  cairo_move_to (cr, x, y);
  cairo_line_to (cr, x - 0.68 * radius, y);
  cairo_line_to (cr, x, y + 0.05 * radius);
  cairo_line_to (cr, x + 0.68 * radius, y);
  cairo_line_to (cr, x, y);
  cairo_close_path (cr);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  cairo_set_line_width (cr, 0.02 * radius);
  cairo_move_to (cr, x, y);
  cairo_line_to (cr, x, y - 0.2 * radius);
  cairo_stroke (cr);

  cairo_move_to (cr, x - 0.2 * radius, y - 0.1 * radius);
  cairo_line_to (cr, x + 0.2 * radius, y - 0.1 * radius);
  cairo_stroke (cr);
  cairo_restore (cr);
  return;
}

/**
 * @fn static gboolean gtk_turn_coordinator_button_press_event (GtkWidget * widget, GdkEventButton * ev)
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
static gboolean gtk_turn_coordinator_button_press_event (GtkWidget * widget, GdkEventButton * ev)
{
  GtkTurnCoordinatorPrivate *priv;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_button_press_event_cb()");
  }
  g_return_val_if_fail (IS_GTK_TURN_COORDINATOR (widget), FALSE);

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (widget);

   if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 2) && priv->b_mouse_onoff)
  {
    gtk_turn_coordinator_debug = gtk_turn_coordinator_debug ? FALSE : TRUE;
    return TRUE;
  }
  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 1) && priv->b_mouse_onoff)
  {
    gtk_turn_coordinator_lock_update = gtk_turn_coordinator_lock_update ? FALSE : TRUE;
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
 * @fn static void gtk_turn_coordinator_destroy (GtkObject * object)
 * @brief Special Gtk API function. Override the _destroy handler.<br>
 * Allow the destruction of all widget's pointer.
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

  if (priv->static_surface)
  {
    priv->static_surface=NULL;
    priv->dynamic_surface=NULL;

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
 * @brief Special Gtk API function. Override the _set_property handler <br>
 * in order to set the object parameters.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_turn_coordinator_set_property (GObject * object, guint prop_id, const GValue * value,
                                               GParamSpec * pspec)
{
  GtkTurnCoordinatorPrivate *priv = NULL;
  GtkTurnCoordinator *tc = NULL;

  if (gtk_turn_coordinator_debug)
  {
    g_debug ("===> gtk_turn_coordinator_set_property()");
  }
  g_return_if_fail (object != NULL);

  tc = GTK_TURN_COORDINATOR (object);
  g_return_if_fail (IS_GTK_TURN_COORDINATOR (tc));

  priv = GTK_TURN_COORDINATOR_GET_PRIVATE (tc);
  g_return_if_fail (priv != NULL);

  switch (prop_id)
  {
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
