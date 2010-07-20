/*
*  Gtk Compass Widget
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
 * @file gtkcompass.c
 * @brief Gtk+ based Compass Widget
 * @author Gautier Dumonteil <gautier.dumonteil@gmail.com>
 * @version 0.1
 * @date 06/06/2010
 *
 * Gtk Compass Widget <br>
 * Copyright (C) 2010, CCNY Robotics Lab <br>
 * http://robotics.ccny.cuny.edu <br>
 * 
 * This widget provide an easy to read compass instrument. <br>
 * The design is volontary based on a real compass flight instrument <br>
 * in order to be familiar to aircraft and helicopter pilots.<br>
 * 
 * @b Pictures:<br>
 * <table><tr>
 * <th><IMG SRC="file:///home/gaitt/Bureau/gtkcompass.png"></th>
 * <th><IMG SRC="file:///home/gaitt/Bureau/gtkcompass_g.png"></th>
 * </tr></table>
 * 
 * @b Example: <br>
 * Add Compass widget to an gtkvbox and set some params <br>
 * @code
 * window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
 * vbox = gtk_vbox_new(TRUE, 1);
 * gtk_container_add(GTK_CONTAINER (window), vbox);
 * 
 * comp = gtk_compass_new();
 * g_object_set(GTK_COMPASS (comp),
 *		"grayscale-color", false,
 *		"radial-color", true, NULL);
 * 
 * gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(comp), TRUE, TRUE, 0);
 * gtk_widget_show_all(window);
 * @endcode
 * 
 * The following code show how to change widget's values and redraw it:<br>
 * Note that here tc's type is "GtkWidget *".<br>
 * @code
 * if (IS_GTK_COMPASS (comp))
 * {
 *	gtk_compass_set_angle (GTK_COMPASS (comp), angle);
 *	gtk_compass_redraw(GTK_COMPASS(comp));
 * }		
 * @endcode
 * 
  @b Widget @b Parameters:<br>
 * - "grayscale-color": boolean, if TRUE, draw the widget with grayscale colors (outdoor use)<br>
 * - "radial-color": boolean, if TRUE, draw a fake light reflexion<br>
 * 
 * @b Widget @b values:<br>
 * - "angle": double, provide the compass's rotation - the value is from 0 to 360.<br>
 * 
 */

#include <ground_station/gui/gtkcompass.h>

/**
 * @typedef struct GtkCompassPrivate 
 * @brief Special Gtk API strucure. Allow to add a private data<br>
 * for the widget. Defined in the C file in order to be private.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkCompassPrivate
{
  /* new cairo design */
  cairo_t *cr;
  GdkRectangle plot_box;

  /* widget data */
  gboolean grayscale_color;
  gboolean radial_color;
  gdouble angle;

  /* drawing data */
  gdouble x;
  gdouble y;
  gdouble radius;
  gdouble plane_scale;
  GdkColor bg_color_inv;
  GdkColor bg_color_compass;
  GdkColor bg_color_bounderie;
  GdkColor bg_radial_color_begin_compass;
  GdkColor bg_radial_color_begin_bounderie;

  /* mouse information */
  gboolean b_mouse_onoff;
  GdkPoint mouse_pos;
  GdkModifierType mouse_state;

} GtkCompassPrivate;

/**
 * @enum _GTK_COMPASS_PROPERTY_ID
 * @brief Special Gtk API enum. Allow to identify widget's properties.
 *
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
enum _GTK_COMPASS_PROPERTY_ID
{
  PROP_0,
  PROP_GRAYSCALE_COLOR,
  PROP_RADIAL_COLOR,
} GTK_COMPASS_PROPERTY_ID;

/**
 * @fn G_DEFINE_TYPE (GtkCompass, gtk_compass, GTK_TYPE_DRAWING_AREA);
 * @brief Special Gtk API function. Define a new object type named GtkCompass <br>
 * and all preface of the widget's functions calls with gtk_compass.<br>
 * We are inheriting the type of GtkDrawingArea.<br>
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
G_DEFINE_TYPE (GtkCompass, gtk_compass, GTK_TYPE_DRAWING_AREA);

/**
 * @def GTK_COMPASS_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GTK_COMPASS_TYPE, GtkCompassPrivate))
 * @brief Special Gtk API define. Add a macro for easy access to the private<br>
 * data struct.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
#define GTK_COMPASS_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GTK_COMPASS_TYPE, GtkCompassPrivate))

static void gtk_compass_class_init (GtkCompassClass * klass);
static void gtk_compass_init (GtkCompass * comp);
static void gtk_compass_destroy (GtkObject * object);
static void gtk_compass_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec);

static gboolean gtk_compass_configure_event (GtkWidget * widget, GdkEventConfigure * event);
static gboolean gtk_compass_expose (GtkWidget * graph, GdkEventExpose * event);
static gboolean gtk_compass_button_press_event (GtkWidget * widget, GdkEventButton * ev);
static gboolean gtk_compass_motion_notify_event (GtkWidget * widget, GdkEventMotion * ev);

static void gtk_compass_draw (GtkWidget * comp);
static void gtk_compass_draw_screws (GtkWidget * comp);
static void gtk_compass_draw_plane (GtkWidget * comp);
//static void gtk_compass_draw_svg_file (GtkWidget * comp);  // **** not yet implemented
static void gtk_compass_draw_tips_and_numbers (GtkWidget * comp);

static gboolean gtk_compass_debug = FALSE;

/**
 * @fn static void gtk_compass_class_init (GtkCompassClass * klass)
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
 * Also register the private struct GtkCompassPrivate with<br>
 * the class and install widget properties.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_compass_class_init (GtkCompassClass * klass)
{
  GObjectClass *obj_class = G_OBJECT_CLASS (klass);
  GtkWidgetClass *widget_class = GTK_WIDGET_CLASS (klass);
  GtkObjectClass *gtkobject_class = GTK_OBJECT_CLASS (klass);

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_class_init()");
  }

  /* GObject signal overrides */
  obj_class->set_property = gtk_compass_set_property;

  /* GtkObject signal overrides */
  gtkobject_class->destroy = gtk_compass_destroy;

  /* GtkWidget signals overrides */
  widget_class->configure_event = gtk_compass_configure_event;
  widget_class->expose_event = gtk_compass_expose;
  widget_class->motion_notify_event = gtk_compass_motion_notify_event;
  widget_class->button_press_event = gtk_compass_button_press_event;

  g_type_class_add_private (obj_class, sizeof (GtkCompassPrivate));

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
  return;
}

/**
 * @fn static void gtk_compass_init (GtkCompass * comp)
 * @brief Special Gtk API function. Function called when the creating a<br>
 * new GtkCompass. Allow to initialize some private variables of<br>
 * widget.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_compass_init (GtkCompass * comp)
{
  GtkCompassPrivate *priv = NULL;

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_init()");
  }
  g_return_if_fail (IS_GTK_COMPASS (comp));

  priv = GTK_COMPASS_GET_PRIVATE (comp);

  gtk_widget_add_events (GTK_WIDGET (comp), GDK_BUTTON_PRESS_MASK |
                         GDK_BUTTON_RELEASE_MASK | GDK_POINTER_MOTION_MASK | GDK_POINTER_MOTION_HINT_MASK);
  priv->b_mouse_onoff = FALSE;
  priv->grayscale_color = FALSE;
  priv->radial_color = TRUE;
  priv->angle=0;

  priv->bg_color_bounderie.red = 6553.5;        // 0.1 cairo
  priv->bg_color_bounderie.green = 6553.5;
  priv->bg_color_bounderie.blue = 6553.5;
  priv->bg_color_compass.red = 3276.75; // 0.05 cairo
  priv->bg_color_compass.green = 3276.75;
  priv->bg_color_compass.blue = 3276.75;
  priv->bg_color_inv.red = 45874.5;     // 0.7 cairo
  priv->bg_color_inv.green = 45874.5;
  priv->bg_color_inv.blue = 45874.5;
  priv->bg_radial_color_begin_bounderie.red = 13107;    // 0.2 cairo
  priv->bg_radial_color_begin_bounderie.green = 13107;
  priv->bg_radial_color_begin_bounderie.blue = 13107;
  priv->bg_radial_color_begin_compass.red = 45874.5;    // 0.7 cairo
  priv->bg_radial_color_begin_compass.green = 45874.5;
  priv->bg_radial_color_begin_compass.blue = 45874.5;

  return;
}

/**
 * @fn static gboolean gtk_compass_configure_event (GtkWidget * widget, GdkEventConfigure * event)
 * @brief Special Gtk API function. Override the _configure_event handler<br>
 * in order to resize the widget when the main window is resized.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static gboolean gtk_compass_configure_event (GtkWidget * widget, GdkEventConfigure * event)
{
  GtkCompassPrivate *priv;
  GtkCompass *comp = GTK_COMPASS (widget);

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_configure_event()");
  }
  g_return_val_if_fail (IS_GTK_COMPASS (comp), FALSE);

  g_return_val_if_fail (event->type == GDK_CONFIGURE, FALSE);

  priv = GTK_COMPASS_GET_PRIVATE (comp);
  g_return_val_if_fail (priv != NULL, FALSE);

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_configure_event(new width=%d, height=%d)", event->width, event->height);
  }

  if ((event->width < GTK_COMPASS_MODEL_X) || (event->height < GTK_COMPASS_MODEL_Y))
  {
    priv->plot_box.width = GTK_COMPASS_MODEL_X;
    priv->plot_box.height = GTK_COMPASS_MODEL_Y;
  }
  else
  {
    priv->plot_box.width = event->width;
    priv->plot_box.height = event->height;
  }

  if (gtk_compass_debug)
  {
    g_debug ("cfg:Max.Avail: plot_box.width=%d, plot_box.height=%d", priv->plot_box.width, priv->plot_box.height);
  }
  return FALSE;
}

/**
 * @fn static gboolean gtk_compass_expose (GtkWidget * comp, GdkEventExpose * event)
 * @brief Special Gtk API function. Override of the expose handler.<br>
 * An “expose-event” signal is emitted when the widget need to be drawn.<br>
 * A Cairo context is created for the parent's GdkWindow.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static gboolean gtk_compass_expose (GtkWidget * comp, GdkEventExpose * event)
{
  GtkCompassPrivate *priv;
  GtkWidget *widget = comp;

  cairo_t *cr = NULL;
  cairo_status_t status;

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_expose()");
  }
  g_return_val_if_fail (IS_GTK_COMPASS (comp), FALSE);

  priv = GTK_COMPASS_GET_PRIVATE (comp);
  g_return_val_if_fail (priv != NULL, FALSE);

  priv->plot_box.width = widget->allocation.width;
  priv->plot_box.height = widget->allocation.height;

  if (gtk_compass_debug)
  {
    g_debug ("gtk_compass_expose(width=%d, height=%d)", widget->allocation.width, widget->allocation.height);
  }

  priv->cr = cr = gdk_cairo_create (widget->window);
  status = cairo_status (cr);
  if (status != CAIRO_STATUS_SUCCESS)
  {
    g_message ("GLG-Expose:cairo_create:status %d=%s", status, cairo_status_to_string (status));
  }

  cairo_rectangle (cr, 0, 0, priv->plot_box.width, priv->plot_box.height);
  cairo_clip (cr);

  gtk_compass_draw (comp);

  cairo_destroy (cr);
  priv->cr = NULL;

  return FALSE;
}

/**
 * @fn extern void gtk_compass_redraw (GtkCompass * comp)
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
extern void gtk_compass_redraw (GtkCompass * comp)
{
  GtkWidget *widget;
  GdkRegion *region;

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_redraw()");
  }
  g_return_if_fail (IS_GTK_COMPASS (comp));

  widget = GTK_WIDGET (comp);

  if (!widget->window)
    return;

  region = gdk_drawable_get_clip_region (widget->window);
  /* redraw the window completely by exposing it */
  gdk_window_invalidate_region (widget->window, region, TRUE);
  gdk_window_process_updates (widget->window, TRUE);

  gdk_region_destroy (region);
}


/**
 * @fn extern void gtk_compass_set_angle (GtkCompass * comp, gdouble ang)
 * @brief Public widget's function that allow the main program/user to<br>
 * set the internal value variable of the widget. 
 * 
 * Here, two values have to be set:<br>
 * "angle": double, provide the compass's rotation - the value is from 0 to 360.<br>
 */
extern void gtk_compass_set_angle (GtkCompass * comp, gdouble ang)
{
  GtkCompassPrivate *priv;

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_draw()");
  }
  g_return_if_fail (IS_GTK_COMPASS (comp));

  priv = GTK_COMPASS_GET_PRIVATE (comp);
  
   if ((ang >= 0) && (ang <= 360))
  priv->angle = ang;
  else
    g_warning ("GtkCompass : gtk_compass_set_angle : value out of range");
}

/**
 * @fn extern GtkWidget *gtk_compass_new (void)
 * @brief Special Gtk API function. This function is simply a wrapper<br>
 * for convienience.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
extern GtkWidget *gtk_compass_new (void)
{
  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_new()");
  }
  return GTK_WIDGET (gtk_type_new (gtk_compass_get_type ()));
}

/**
 * @fn static void gtk_compass_draw (GtkWidget * comp)
 * @brief Special Gtk API function. Override the _draw handler of the<br>
 * parent class GtkDrawingArea. This function use the cairo context<br>
 * created before in order to draw scalable graphics. 
 * 
 * See GObject,Cairo and GTK+ references for more informations: 
 * http://library.gnome.org/devel/references.html.en
 */
static void gtk_compass_draw (GtkWidget * comp)
{
  GtkCompassPrivate *priv;

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_draw()");
  }
  g_return_if_fail (IS_GTK_COMPASS (comp));

  priv = GTK_COMPASS_GET_PRIVATE (comp);

  double x, y, rec_x0, rec_y0, rec_width, rec_height, rec_degrees;
  double rec_aspect, rec_corner_radius, rec_radius, radius;
  cairo_pattern_t *pat = NULL;

  x = comp->allocation.width / 2;
  y = comp->allocation.height / 2;
  radius = MIN (comp->allocation.width / 2, comp->allocation.height / 2) - 5;

  rec_x0 = x - radius;
  rec_y0 = y - radius;
  rec_width = radius * 2;
  rec_height = radius * 2;
  rec_aspect = 1.0;
  rec_corner_radius = rec_height / 8.0;

  rec_radius = rec_corner_radius / rec_aspect;
  rec_degrees = M_PI / 180.0;

  // Compass base
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
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_compass.red / 65535,
                            (gdouble) priv->bg_color_compass.green / 65535,
                            (gdouble) priv->bg_color_compass.blue / 65535);
    else
      cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_inv.red / 65535,
                            (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, (gdouble) priv->bg_radial_color_begin_compass.red / 65535,
                                       (gdouble) priv->bg_radial_color_begin_compass.green / 65535,
                                       (gdouble) priv->bg_radial_color_begin_compass.blue / 65535, 1);
    cairo_pattern_add_color_stop_rgba (pat, 1, (gdouble) priv->bg_color_compass.red / 65535,
                                       (gdouble) priv->bg_color_compass.green / 65535,
                                       (gdouble) priv->bg_color_compass.blue / 65535, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);

  priv->radius = radius;
  priv->x = x;
  priv->y = y;

  gtk_compass_draw_screws (comp);

  // draw plane
  gtk_compass_draw_plane (comp);
  //gtk_compass_draw_svg_file(comp);

  // draw tips & numbers 
  gtk_compass_draw_tips_and_numbers (comp);
  cairo_pattern_destroy (pat);
  return;
}

/**
 * @fn static void gtk_compass_draw_screws (GtkWidget * comp)
 * @brief Private widget's function that draw the widget's screws using cairo.
 */
static void gtk_compass_draw_screws (GtkWidget * comp)
{
  GtkCompassPrivate *priv;

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_draw()");
  }
  g_return_if_fail (IS_GTK_COMPASS (comp));

  priv = GTK_COMPASS_GET_PRIVATE (comp);
  
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
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15,0.15,0.15, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  
  cairo_set_line_width (priv->cr, 0.02 * radius);
  if (!priv->grayscale_color)
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
  if (!priv->grayscale_color)
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
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15,0.15,0.15, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  
  cairo_set_line_width (priv->cr, 0.02 * radius);
  if (!priv->grayscale_color)
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
  if (!priv->grayscale_color)
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
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15,0.15,0.15, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  
  cairo_set_line_width (priv->cr, 0.02 * radius);
  if (!priv->grayscale_color)
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
  if (!priv->grayscale_color)
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
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.15,0.15,0.15, 1);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  
  cairo_set_line_width (priv->cr, 0.02 * radius);
  if (!priv->grayscale_color)
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
  if (!priv->grayscale_color)
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

/*
static void gtk_compass_draw_svg_file (GtkWidget * comp)
{
  GtkCompassPrivate *priv;
  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_draw_svg_file()");
  }
  g_return_if_fail (IS_GTK_COMPASS (comp));

  priv = GTK_COMPASS_GET_PRIVATE (comp);

  // next release
}
*/

/**
 * @fn static void gtk_compass_draw_plane (GtkWidget * comp)
 * @brief Private widget's function that draw the compass plane using cairo.
 */
static void gtk_compass_draw_plane (GtkWidget * comp)
{
  GtkCompassPrivate *priv;
  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_draw_plane()");
  }
  g_return_if_fail (IS_GTK_COMPASS (comp));

  priv = GTK_COMPASS_GET_PRIVATE (comp);

  double x = priv->x;
  double y = priv->y;
  double radius = priv->radius;
  double plane_scale = priv->plane_scale;
  double x0, y0, x1, y1, x2, y2, x3, y3;

  if (!priv->grayscale_color)
    cairo_set_source_rgb (priv->cr, 1., 0.7, 0.);
  else
    cairo_set_source_rgb (priv->cr, 1., 1., 1.);

  plane_scale = 1;
  // plane drawing
  // nez
  cairo_new_sub_path (priv->cr);
  x0 = x;
  y0 = y - 0.35 * radius * plane_scale;
  x1 = x - 0.05 * radius * plane_scale;
  y1 = y - 0.25 * radius * plane_scale;
  x2 = x - 0.05 * radius * plane_scale;
  y2 = y - 0.25 * radius * plane_scale;
  x3 = x - 0.05 * radius * plane_scale;
  y3 = y - 0.09 * radius * plane_scale;
  cairo_move_to (priv->cr, x0, y0);
  cairo_curve_to (priv->cr, x1, y1, x2, y2, x3, y3);

  x0 = x;
  y0 = y - 0.35 * radius * plane_scale;
  x1 = x + 0.05 * radius * plane_scale;
  y1 = y - 0.25 * radius * plane_scale;
  x2 = x + 0.05 * radius * plane_scale;
  y2 = y - 0.25 * radius * plane_scale;
  x3 = x + 0.05 * radius * plane_scale;
  y3 = y - 0.09 * radius * plane_scale;
  cairo_move_to (priv->cr, x0, y0);
  cairo_curve_to (priv->cr, x1, y1, x2, y2, x3, y3);
  cairo_line_to (priv->cr, x - 0.05 * radius * plane_scale, y - 0.09 * radius * plane_scale);
  cairo_close_path (priv->cr);
  cairo_set_line_width (priv->cr, 0.02 * radius);
  cairo_fill (priv->cr);
  cairo_stroke (priv->cr);

  // aile
  cairo_new_sub_path (priv->cr);
  x0 = x - 0.05 * radius * plane_scale;
  y0 = y - 0.1 * radius * plane_scale;
  x1 = x - 0.3 * radius * plane_scale;
  y1 = y;
  x2 = x - 0.3 * radius * plane_scale;
  y2 = y;
  x3 = x - 0.3 * radius * plane_scale;
  y3 = y + 0.05 * radius * plane_scale;
  cairo_move_to (priv->cr, x + 0.05 * radius * plane_scale, y0);
  cairo_line_to (priv->cr, x0, y0);
  cairo_curve_to (priv->cr, x1, y1, x2, y2, x3, y3);
  cairo_line_to (priv->cr, x - 0.05 * radius * plane_scale, y);
  cairo_line_to (priv->cr, x + 0.05 * radius * plane_scale, y);
  cairo_line_to (priv->cr, x + 0.3 * radius * plane_scale, y + 0.05 * radius * plane_scale);

  x0 = x + 0.05 * radius * plane_scale;
  y0 = y - 0.1 * radius * plane_scale;
  x1 = x + 0.3 * radius * plane_scale;
  y1 = y;
  x2 = x + 0.3 * radius * plane_scale;
  y2 = y;
  x3 = x + 0.3 * radius * plane_scale;
  y3 = y + 0.05 * radius * plane_scale;
  cairo_move_to (priv->cr, x0, y0);
  cairo_curve_to (priv->cr, x1, y1, x2, y2, x3, y3);
  cairo_close_path (priv->cr);
  cairo_set_line_width (priv->cr, 0.02 * radius);
  cairo_fill (priv->cr);
  cairo_stroke (priv->cr);

  // queue
  cairo_new_sub_path (priv->cr);
  x0 = x - 0.05 * radius * plane_scale;
  y0 = y;
  x1 = x - 0.05 * radius * plane_scale;
  y1 = y + 0.1 * radius * plane_scale;
  x2 = x - 0.05 * radius * plane_scale;
  y2 = y + 0.17 * radius * plane_scale;
  x3 = x;
  y3 = y + 0.35 * radius * plane_scale;
  cairo_move_to (priv->cr, x + 0.05 * radius * plane_scale, y0);
  cairo_line_to (priv->cr, x0, y0);
  cairo_curve_to (priv->cr, x1, y1, x2, y2, x3, y3);

  x0 = x + 0.05 * radius * plane_scale;
  y0 = y;
  x1 = x + 0.05 * radius * plane_scale;
  y1 = y + 0.1 * radius * plane_scale;
  x2 = x + 0.05 * radius * plane_scale;
  y2 = y + 0.17 * radius * plane_scale;
  x3 = x;
  y3 = y + 0.35 * radius * plane_scale;
  cairo_move_to (priv->cr, x0, y0);
  cairo_curve_to (priv->cr, x1, y1, x2, y2, x3, y3);
  cairo_move_to (priv->cr, x0, y0);
  cairo_line_to (priv->cr, x - 0.05 * radius * plane_scale, y);
  cairo_close_path (priv->cr);
  cairo_set_line_width (priv->cr, 0.02 * radius);
  cairo_fill (priv->cr);
  cairo_stroke (priv->cr);

  // aile queue
  cairo_new_sub_path (priv->cr);
  x0 = x - 0.01 * radius * plane_scale;
  y0 = y + 0.24 * radius * plane_scale;
  x1 = x - 0.1 * radius * plane_scale;
  y1 = y + 0.29 * radius * plane_scale;
  x2 = x - 0.1 * radius * plane_scale;
  y2 = y + 0.29 * radius * plane_scale;
  x3 = x - 0.1 * radius * plane_scale;
  y3 = y + 0.32 * radius * plane_scale;
  cairo_move_to (priv->cr, x + 0.01 * radius * plane_scale, y0);
  cairo_line_to (priv->cr, x0, y0);
  cairo_curve_to (priv->cr, x1, y1, x2, y2, x3, y3);
  cairo_line_to (priv->cr, x - 0.005 * radius * plane_scale, y1);
  cairo_line_to (priv->cr, x + 0.005 * radius * plane_scale, y1);
  cairo_line_to (priv->cr, x + 0.1 * radius * plane_scale, y3);

  x0 = x + 0.01 * radius * plane_scale;
  y0 = y + 0.24 * radius * plane_scale;
  x1 = x + 0.1 * radius * plane_scale;
  y1 = y + 0.29 * radius * plane_scale;
  x2 = x + 0.1 * radius * plane_scale;
  y2 = y + 0.29 * radius * plane_scale;
  x3 = x + 0.1 * radius * plane_scale;
  y3 = y + 0.32 * radius * plane_scale;
  cairo_move_to (priv->cr, x0, y0);
  cairo_curve_to (priv->cr, x1, y1, x2, y2, x3, y3);
  cairo_close_path (priv->cr);
  cairo_set_line_width (priv->cr, 0.02 * radius);
  cairo_fill (priv->cr);
  cairo_stroke (priv->cr);

  // lines
  x0 = x;
  y0 = y - 0.35 * radius * plane_scale;
  cairo_move_to (priv->cr, x0, y0 - 0.05 * radius);
  cairo_line_to (priv->cr, x0, y0 - 0.28 * radius);
  cairo_set_line_width (priv->cr, 0.015 * radius);
  cairo_stroke (priv->cr);

  x0 = x;
  y0 = y + 0.35 * radius * plane_scale;
  cairo_move_to (priv->cr, x0, y0 + 0.05 * radius);
  cairo_line_to (priv->cr, x0, y0 + 0.28 * radius);
  cairo_set_line_width (priv->cr, 0.015 * radius);
  cairo_stroke (priv->cr);

  x0 = x - 0.3 * radius * plane_scale;
  y0 = y;
  cairo_move_to (priv->cr, x0 - 0.05 * radius, y0);
  cairo_line_to (priv->cr, x0 - 0.32 * radius, y0);
  cairo_set_line_width (priv->cr, 0.015 * radius);
  cairo_stroke (priv->cr);

  x0 = x + 0.3 * radius * plane_scale;
  y0 = y;
  cairo_move_to (priv->cr, x0 + 0.05 * radius, y0);
  cairo_line_to (priv->cr, x0 + 0.32 * radius, y0);
  cairo_set_line_width (priv->cr, 0.015 * radius);
  cairo_stroke (priv->cr);
}

/**
 * @fn static void gtk_compass_draw_tips_and_numbers (GtkWidget * comp)
 * @brief Private widget's function that draw compass's tips & numbers using cairo.
 */
static void gtk_compass_draw_tips_and_numbers (GtkWidget * comp)
{
  GtkCompassPrivate *priv;
  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_draw_tips_and_numbers()");
  }
  g_return_if_fail (IS_GTK_COMPASS (comp));

  priv = GTK_COMPASS_GET_PRIVATE (comp);

  double x = priv->x;
  double y = priv->y;
  double radius = priv->radius;
  char str[GTK_COMPASS_MAX_STRING];
  int i;

  // Number drawing
  for (i = 0; i < 12; i++)
  {
    int inset;
    cairo_select_font_face (priv->cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
    cairo_set_font_size (priv->cr, 0.20 * radius);
    inset = 0.25 * radius;
    switch (i)
    {
      case 0:
        // N
        cairo_move_to (priv->cr,
                       x + (radius - inset) * cos (i * M_PI / 6 - M_PI / 32 - 3 * M_PI / 6 - DEG2RAD (priv->angle)),
                       y + (radius - inset) * sin (i * M_PI / 6 - M_PI / 32 - 3 * M_PI / 6 - DEG2RAD (priv->angle)));
        break;
      case 3:
        // E
        cairo_move_to (priv->cr,
                       x + (radius - inset) * cos (i * M_PI / 6 - M_PI / 34 - 3 * M_PI / 6 - DEG2RAD (priv->angle)),
                       y + (radius - inset) * sin (i * M_PI / 6 - M_PI / 34 - 3 * M_PI / 6 - DEG2RAD (priv->angle)));
        break;
      case 6:
        // S
        cairo_move_to (priv->cr,
                       x + (radius - inset) * cos (i * M_PI / 6 - M_PI / 35 - 3 * M_PI / 6 - DEG2RAD (priv->angle)),
                       y + (radius - inset) * sin (i * M_PI / 6 - M_PI / 35 - 3 * M_PI / 6 - DEG2RAD (priv->angle)));
        break;
      case 9:
        // W
        cairo_move_to (priv->cr,
                       x + (radius - inset) * cos (i * M_PI / 6 - M_PI / 24 - 3 * M_PI / 6 - DEG2RAD (priv->angle)),
                       y + (radius - inset) * sin (i * M_PI / 6 - M_PI / 24 - 3 * M_PI / 6 - DEG2RAD (priv->angle)));
        break;
      default:
        if (i * 3 / 10 % 10 == 0)
        {
          cairo_move_to (priv->cr,
                         x + (radius - inset) * cos (i * M_PI / 6 - M_PI / 34 - 3 * M_PI / 6 - DEG2RAD (priv->angle)),
                         y + (radius - inset) * sin (i * M_PI / 6 - M_PI / 34 - 3 * M_PI / 6 - DEG2RAD (priv->angle)));
        }
        else
        {
          cairo_move_to (priv->cr,
                         x + (radius - inset) * cos (i * M_PI / 6 - M_PI / 18 - 3 * M_PI / 6 - DEG2RAD (priv->angle)),
                         y + (radius - inset) * sin (i * M_PI / 6 - M_PI / 18 - 3 * M_PI / 6 - DEG2RAD (priv->angle)));
        }
    }

    if (!priv->grayscale_color)
      cairo_set_source_rgb (priv->cr, 1., 1., 0);
    else
      cairo_set_source_rgb (priv->cr, 0, 0, 0);

    cairo_save (priv->cr);
    cairo_rotate (priv->cr, -DEG2RAD (priv->angle) + i * M_PI / 6);
    switch (i)
    {
      case 0:
        cairo_show_text (priv->cr, "N");
        break;
      case 3:
        cairo_show_text (priv->cr, "E");
        break;
      case 6:
        cairo_show_text (priv->cr, "S");
        break;
      case 9:
        cairo_show_text (priv->cr, "W");
        break;
      default:

        if (!priv->grayscale_color)
          cairo_set_source_rgb (priv->cr, 1, 1, 1);
        else
          cairo_set_source_rgb (priv->cr, 0., 0., 0.);
        sprintf (str, "%d", i * 3);
        cairo_show_text (priv->cr, str);
    }
    cairo_stroke (priv->cr);
    cairo_restore (priv->cr);
  }

  cairo_save (priv->cr);
  cairo_move_to (priv->cr,
                 x + (radius - 0.02 * radius) * cos (-3 * M_PI / 6 - DEG2RAD (priv->angle)),
                 y + (radius - 0.02 * radius) * sin (-3 * M_PI / 6 - DEG2RAD (priv->angle)));
  cairo_line_to (priv->cr,
                 x + (radius - 0.08 * radius) * cos (M_PI / 50 - 3 * M_PI / 6 - DEG2RAD (priv->angle)),
                 y + (radius - 0.08 * radius) * sin (M_PI / 50 - 3 * M_PI / 6 - DEG2RAD (priv->angle)));
  cairo_line_to (priv->cr,
                 x + (radius - 0.08 * radius) * cos (-M_PI / 50 - 3 * M_PI / 6 - DEG2RAD (priv->angle)),
                 y + (radius - 0.08 * radius) * sin (-M_PI / 50 - 3 * M_PI / 6 - DEG2RAD (priv->angle)));
  cairo_move_to (priv->cr,
                 x + (radius - 0.02 * radius) * cos (-3 * M_PI / 6 - DEG2RAD (priv->angle)),
                 y + (radius - 0.02 * radius) * sin (-3 * M_PI / 6 - DEG2RAD (priv->angle)));
  cairo_fill (priv->cr);
  cairo_stroke (priv->cr);
  cairo_rotate (priv->cr, -DEG2RAD (priv->angle));
  cairo_restore (priv->cr);

  cairo_set_line_width (priv->cr, 0.03 * radius);
  radius = radius - 0.3 * radius;

  // Compass ticks 
  for (i = 0; i < 36; i++)
  {
    int inset;
    cairo_save (priv->cr);

    if (i % 9 == 0)
    {
      inset = 0.12 * radius;
      if (!priv->grayscale_color)
        cairo_set_source_rgb (priv->cr, 0.88, 0.88, 0);
      else
        cairo_set_source_rgb (priv->cr, 0, 0, 0);
    }
    else
    {
      inset = 0.06 * radius;
      cairo_set_line_width (priv->cr, 0.5 * cairo_get_line_width (priv->cr));
      if (!priv->grayscale_color)
        cairo_set_source_rgb (priv->cr, 1, 1, 1);
      else
        cairo_set_source_rgb (priv->cr, 0., 0., 0.);
    }

    cairo_move_to (priv->cr, x + (radius - inset) * cos (i * M_PI / 18 - DEG2RAD (priv->angle)),
                   y + (radius - inset) * sin (i * M_PI / 18 - DEG2RAD (priv->angle)));
    cairo_line_to (priv->cr, x + radius * cos (i * M_PI / 18 - DEG2RAD (priv->angle)),
                   y + radius * sin (i * M_PI / 18 - DEG2RAD (priv->angle)));
    cairo_stroke (priv->cr);
    cairo_restore (priv->cr);
  }
  return;
}

/**
 * @fn static gboolean gtk_compass_button_press_event (GtkWidget * widget, GdkEventButton * ev)
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
static gboolean gtk_compass_button_press_event (GtkWidget * widget, GdkEventButton * ev)
{
  GtkCompassPrivate *priv;
  gint x = 0, y = 0;

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_button_press_event_cb()");
  }
  g_return_val_if_fail (IS_GTK_COMPASS (widget), FALSE);

  priv = GTK_COMPASS_GET_PRIVATE (widget);

  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 1))
  {
    gdk_window_get_pointer (ev->window, &x, &y, &priv->mouse_state);
    priv->mouse_pos.x = x;
    priv->mouse_pos.y = y;
    return TRUE;
  }
  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 2) && priv->b_mouse_onoff)
  {
    gtk_compass_debug = gtk_compass_debug ? FALSE : TRUE;
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
 * @fn static gboolean gtk_compass_motion_notify_event (GtkWidget * widget, GdkEventMotion * ev)
 * @brief Special Gtk API function. Override the _motion_notify_event<br>
 * handler. Perform mouse motion events. 
 * 
 * Here, the mouse events are not used (maybe in future released).
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static gboolean gtk_compass_motion_notify_event (GtkWidget * widget, GdkEventMotion * ev)
{
  GtkCompassPrivate *priv;
  GdkModifierType state;
  gint x = 0, y = 0;

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_motion_notify_event_cb()");
  }
  g_return_val_if_fail (IS_GTK_COMPASS (widget), FALSE);

  priv = GTK_COMPASS_GET_PRIVATE (widget);

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

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_motion_notify_event_cb() : mouse x=%d, y=%d", x, y);
  }

  return TRUE;
}

/**
 * @fn static void gtk_compass_destroy (GtkObject * object)
 * @brief Special Gtk API function. Override the _destroy handler.<br>
 * Allow the destruction of all widget's pointer.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_compass_destroy (GtkObject * object)
{
  GtkCompassPrivate *priv = NULL;
  GtkWidget *widget = NULL;

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_destroy(enter)");
  }

  g_return_if_fail (object != NULL);

  widget = GTK_WIDGET (object);

  g_return_if_fail (IS_GTK_COMPASS (widget));

  priv = GTK_COMPASS_GET_PRIVATE (widget);
  g_return_if_fail (priv != NULL);

  if (priv->cr)
  {
    g_free (priv->cr);

    if (GTK_OBJECT_CLASS (gtk_compass_parent_class)->destroy != NULL)
    {
      (*GTK_OBJECT_CLASS (gtk_compass_parent_class)->destroy) (object);
    }
  }
  if (gtk_compass_debug)
  {
    g_debug ("gtk_compass_destroy(exit)");
  }

  return;
}

/**
 * @fn static void gtk_compass_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec)
 * @brief Special Gtk API function. Override the _set_property handler <br>
 * in order to set the object parameters.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_compass_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec)
{
  GtkCompassPrivate *priv = NULL;
  GtkCompass *comp = NULL;

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_set_property()");
  }
  g_return_if_fail (object != NULL);

  comp = GTK_COMPASS (object);
  g_return_if_fail (IS_GTK_COMPASS (comp));

  priv = GTK_COMPASS_GET_PRIVATE (comp);
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
