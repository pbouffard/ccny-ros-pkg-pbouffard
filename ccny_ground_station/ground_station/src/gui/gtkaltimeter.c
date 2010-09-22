/*
*  Gtk Altimeter Widget
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
 * @file gtkaltimeter.c
 * @brief Gtk+ based Altimeter Widget
 * @author Gautier Dumonteil <gautier.dumonteil@gmail.com>
 * @version 0.2
 * @date 02/09/2010
 *
 * Gtk Altimeter Widget <br>
 * Copyright (C) 2010, CCNY Robotics Lab <br>
 * http://robotics.ccny.cuny.edu <br>
 * 
 * This widget provide an easy to read altimeter instrument. <br>
 * The design is based on a real altimeter flight instrument <br>
 * in order to be familiar to aircraft and helicopter pilots.<br>
 * 
 * @b Pictures:<br>
 * <table><tr>
 * <th><IMG SRC="http://www.ros.org/wiki/ground_station?action=AttachFile&do=get&target=gtkaltimeter.png"></th>
 * <th><IMG SRC="http://www.ros.org/wiki/ground_station?action=AttachFile&do=get&target=gtkaltimeter_g.png"></th>
 * </tr></table>
 * 
 * @b Example: <br>
 * Add Altimeter widget to an gtkvbox and set some params <br>
 * @code
 * window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
 * vbox = gtk_vbox_new(TRUE, 1);
 * gtk_container_add(GTK_CONTAINER (window), vbox);
 * 
 * alt = gtk_altimeter_new();
 * g_object_set(GTK_ALTIMETER (alt),
 *		"grayscale-colors", false,
 *		"unit-is-feet", true,
 *		"unit-step-value", 100,
 *		"radial-color", true, NULL);
 * 
 * gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(alt), TRUE, TRUE, 0);
 * gtk_widget_show_all(window);
 * @endcode
 * 
 * The following code show how to change widget's values and redraw it:<br>
 * Note that here tc's type is "GtkWidget *".<br>
 * @code
 * if (IS_GTK_ALTIMETER (alt))
 * {
 *	gtk_altimeter_set_alti (GTK_ALTIMETER (alt), altitude);
 *	gtk_altimeter_redraw(GTK_ALTIMETER(alt));
 * }		
 * @endcode
 * 
  @b Widget @b Parameters:<br>
 * - "grayscale-colors": boolean, if TRUE, draw the widget with grayscale colors (outdoor use)<br>
 * - "unit-is-feet": boolean, if TRUE, the widget display FEET values else display METER<br>
 * - "unit-step-value", int, define the step value of the altimeter can be 1,10,100<br>
 * - "radial-color": boolean, if TRUE, draw a fake light reflexion<br>
 * 
 * @b Widget @b values:<br>
 * - "altitude": double, define the altitude you want to display by the widget - the value is<br>
 * from 0 to 999999.
 */

#include <ground_station/gui/gtkaltimeter.h>

/**
 * @typedef struct GtkAltimeterPrivate 
 * @brief Special Gtk API strucure. Allow to add a private data<br>
 * for the widget. Defined in the C file in order to be private.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GtkAltimeterPrivate
{
  /* cairo data */
  gboolean draw_once;
  cairo_surface_t * static_surface;
  cairo_surface_t * dynamic_surface;

  /* widget data */
  gint unit_value;
  gboolean unit_is_feet;
  gboolean grayscale_color;
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

} GtkAltimeterPrivate;

/**
 * @enum _GTK_ALTIMETER_PROPERTY_ID
 * @brief Special Gtk API enum. Allow to identify widget's properties.
 *
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
enum _GTK_ALTIMETER_PROPERTY_ID
{
  PROP_0,
  PROP_GRAYSCALE_COLOR,
  PROP_UNIT_IS_FEET,
  PROP_UNIT_STEP_VALUE,
  PROP_RADIAL_COLOR,
} GTK_ALTIMETER_PROPERTY_ID;

/**
 * @fn G_DEFINE_TYPE (GtkAltimeter, gtk_altimeter, GTK_TYPE_DRAWING_AREA);
 * @brief Special Gtk API function. Define a new object type named GtkAltimeter <br>
 * and all preface of the widget's functions calls with gtk_altimeter.<br>
 * We are inheriting the type of GtkDrawingArea.<br>
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
G_DEFINE_TYPE (GtkAltimeter, gtk_altimeter, GTK_TYPE_DRAWING_AREA);

/**
 * @def GTK_ALTIMETER_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GTK_ALTIMETER_TYPE, GtkAltimeterPrivate))
 * @brief Special Gtk API define. Add a macro for easy access to the private<br>
 * data struct.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
#define GTK_ALTIMETER_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GTK_ALTIMETER_TYPE, GtkAltimeterPrivate))

static void gtk_altimeter_class_init (GtkAltimeterClass * klass);
static void gtk_altimeter_init (GtkAltimeter * alt);
static void gtk_altimeter_destroy (GtkObject * object);
static void gtk_altimeter_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec);

static gboolean gtk_altimeter_configure_event (GtkWidget * widget, GdkEventConfigure * event);
static gboolean gtk_altimeter_expose (GtkWidget * graph, GdkEventExpose * event);
static gboolean gtk_altimeter_button_press_event (GtkWidget * widget, GdkEventButton * ev);

static void gtk_altimeter_draw_static (GtkWidget * alt, cairo_t * cr);
static void gtk_altimeter_draw_base (GtkWidget * alt, cairo_t * cr);
static void gtk_altimeter_draw_screws (GtkWidget * alt, cairo_t * cr);

static void gtk_altimeter_draw_dynamic (GtkWidget * alt, cairo_t * cr);
static void gtk_altimeter_draw_digital (GtkWidget * alt, cairo_t * cr);
static void gtk_altimeter_draw_hands (GtkWidget * alt, cairo_t * cr);

static gboolean gtk_altimeter_debug = FALSE;
static gboolean gtk_altimeter_lock_update = FALSE;

/**
 * @fn static void gtk_altimeter_class_init (GtkAltimeterClass * klass)
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
 * Also register the private struct GtkAltimeterPrivate with<br>
 * the class and install widget properties.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_altimeter_class_init (GtkAltimeterClass * klass)
{
  GObjectClass *obj_class = G_OBJECT_CLASS (klass);
  GtkWidgetClass *widget_class = GTK_WIDGET_CLASS (klass);
  GtkObjectClass *gtkobject_class = GTK_OBJECT_CLASS (klass);

  if (gtk_altimeter_debug)
  {
    g_debug ("===> gtk_altimeter_class_init()");
  }

  /* GObject signal overrides */
  obj_class->set_property = gtk_altimeter_set_property;

  /* GtkObject signal overrides */
  gtkobject_class->destroy = gtk_altimeter_destroy;

  /* GtkWidget signals overrides */
  widget_class->configure_event = gtk_altimeter_configure_event;
  widget_class->expose_event = gtk_altimeter_expose;
  widget_class->button_press_event = gtk_altimeter_button_press_event;

  g_type_class_add_private (obj_class, sizeof (GtkAltimeterPrivate));

  g_object_class_install_property (obj_class,
                                   PROP_GRAYSCALE_COLOR,
                                   g_param_spec_boolean ("grayscale-color",
                                                         "use grayscale for the widget color",
                                                         "use grayscale for the widget color", FALSE,
                                                         G_PARAM_WRITABLE));
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

/**
 * @fn static void gtk_altimeter_init (GtkAltimeter * alt)
 * @brief Special Gtk API function. Function called when the creating a<br>
 * new GtkAltimeter. Allow to initialize some private variables of<br>
 * widget.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_altimeter_init (GtkAltimeter * alt)
{
  GtkAltimeterPrivate *priv = NULL;

  if (gtk_altimeter_debug)
  {
    g_debug ("===> gtk_altimeter_init()");
  }
  g_return_if_fail (IS_GTK_ALTIMETER (alt));

  priv = GTK_ALTIMETER_GET_PRIVATE (alt);

  gtk_widget_add_events (GTK_WIDGET (alt), GDK_BUTTON_PRESS_MASK |
                         GDK_BUTTON_RELEASE_MASK | GDK_POINTER_MOTION_MASK | GDK_POINTER_MOTION_HINT_MASK);
                         
  priv->b_mouse_onoff = FALSE;
  priv->draw_once = FALSE;
  priv->grayscale_color = FALSE;
  priv->radial_color = TRUE;
  priv->unit_is_feet = TRUE;
  priv->altitude = 0;
  priv->unit_value = 100;

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

/**
 * @fn static gboolean gtk_altimeter_configure_event (GtkWidget * widget, GdkEventConfigure * event)
 * @brief Special Gtk API function. Override the _configure_event handler<br>
 * in order to resize the widget when the main window is resized.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static gboolean gtk_altimeter_configure_event (GtkWidget * widget, GdkEventConfigure * event)
{
  GtkAltimeterPrivate *priv;
  GtkAltimeter *alt = GTK_ALTIMETER (widget);

  if (gtk_altimeter_debug)
  {
    g_debug ("===> gtk_altimeter_configure_event()");
  }
  
  g_return_val_if_fail (IS_GTK_ALTIMETER (alt), FALSE);
  g_return_val_if_fail (event->type == GDK_CONFIGURE, FALSE);

  priv = GTK_ALTIMETER_GET_PRIVATE (alt);
  g_return_val_if_fail (priv != NULL, FALSE);

  priv->draw_once = FALSE;

  return FALSE;
}

/**
 * @fn static gboolean gtk_altimeter_expose (GtkWidget * alt, GdkEventExpose * event)
 * @brief Special Gtk API function. Override of the _expose handler.<br>
 * An “expose-event” signal is emitted when the widget need to be drawn.<br>
 * A Cairo context is created for the parent's GdkWindow.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static gboolean gtk_altimeter_expose (GtkWidget * alt, GdkEventExpose * event)
{
  GtkAltimeterPrivate *priv;
  GtkWidget *widget = alt;
  cairo_t * cr_final;
  cairo_t * cr_static;
  cairo_t * cr_dynamic;

  if (gtk_altimeter_debug)
  {
    g_debug ("===> gtk_altimeter_expose()");
  }
  g_return_val_if_fail (IS_GTK_ALTIMETER (alt), FALSE);

  priv = GTK_ALTIMETER_GET_PRIVATE (alt);
  g_return_val_if_fail (priv != NULL, FALSE);

  cr_final = gdk_cairo_create (widget->window);
  
  if(!priv->draw_once)
  {
		priv->static_surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, event->area.width, event->area.height);
		cr_static = cairo_create(priv->static_surface);
  
		cairo_rectangle (cr_static, event->area.x, event->area.y, event->area.width, event->area.height);
		cairo_clip (cr_static);
		gtk_altimeter_draw_static (alt,cr_static);
		cairo_destroy (cr_static);
		priv->draw_once=TRUE;
  }

  priv->dynamic_surface = cairo_surface_create_similar (priv->static_surface,CAIRO_CONTENT_COLOR_ALPHA,event->area.width, event->area.height);
  cr_dynamic = cairo_create(priv->dynamic_surface);
  cairo_rectangle (cr_dynamic, event->area.x, event->area.y, event->area.width, event->area.height);
  cairo_clip (cr_dynamic);
  gtk_altimeter_draw_dynamic (alt, cr_dynamic);
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
 * @fn extern void gtk_altimeter_redraw (GtkAltimeter * alt)
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
extern void gtk_altimeter_redraw (GtkAltimeter * alt)
{
  GtkWidget *widget;
  GdkRegion *region;

  if (gtk_altimeter_debug)
  {
    g_debug ("===> gtk_altimeter_redraw()");
  }
  g_return_if_fail (IS_GTK_ALTIMETER (alt));

  widget = GTK_WIDGET (alt);

  if (!widget->window)
    return;

  region = gdk_drawable_get_clip_region (widget->window);
  /* redraw the window completely by exposing it */
  gdk_window_invalidate_region (widget->window, region, TRUE);
  gdk_window_process_updates (widget->window, TRUE);

  gdk_region_destroy (region);
}

/**
 * @fn extern void gtk_altimeter_set_alti (GtkAltimeter * alt, gdouble alti)
 * @brief Public widget's function that allow the main program/user to<br>
 * set the internal altitude variable of the widget. 
 * 
 * "altitude": double, define the altitude you want to display by the widget - the value is<br>
 * from 0 to 999999.
 */
extern void gtk_altimeter_set_alti (GtkAltimeter * alt, gdouble alti)
{
  GtkAltimeterPrivate *priv;

  if (gtk_altimeter_debug)
  {
    g_debug ("===> gtk_altimeter_set_alti()");
  }
  g_return_if_fail (IS_GTK_ALTIMETER (alt));

  priv = GTK_ALTIMETER_GET_PRIVATE (alt);

  if(!gtk_altimeter_lock_update)
  {
		if ((alti >= 0) && (alti <= 999999))
		{
			priv->altitude = alti;
		}
		else
			g_warning ("GtkAltimeter : gtk_altimeter_set_alti : value out of range");
  }
}

/**
 * @fn extern GtkWidget *gtk_altimeter_new (void)
 * @brief Special Gtk API function. This function is simply a wrapper<br>
 * for convienience.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
extern GtkWidget *gtk_altimeter_new (void)
{
  if (gtk_altimeter_debug)
  {
    g_debug ("===> gtk_altimeter_new()");
  }
  return GTK_WIDGET (gtk_type_new (gtk_altimeter_get_type ()));
}


/**
 * @fn static void gtk_altimeter_draw_static (GtkWidget * alt, cairo_t * cr)
 * @brief Special Gtk API function. This function use the cairo context<br>
 * created before in order to draw scalable graphics. 
 * 
 * See GObject,Cairo and GTK+ references for more informations: 
 * http://library.gnome.org/devel/references.html.en
 */
static void gtk_altimeter_draw_static (GtkWidget * alt, cairo_t * cr)
{
  if (gtk_altimeter_debug)
  {
    g_debug ("===> gtk_altimeter_draw_static()");
  }
  g_return_if_fail (IS_GTK_ALTIMETER (alt));

  gtk_altimeter_draw_base (alt,cr);
  gtk_altimeter_draw_screws (alt,cr);
}

/**
 * @fn static void gtk_altimeter_draw_dynamic (GtkWidget * alt, cairo_t * cr)
 * @brief Special Gtk API function. This function use the cairo context<br>
 * created before in order to draw scalable graphics. 
 * 
 * See GObject,Cairo and GTK+ references for more informations: 
 * http://library.gnome.org/devel/references.html.en
 */
static void gtk_altimeter_draw_dynamic (GtkWidget * alt, cairo_t * cr)
{
  if (gtk_altimeter_debug)
  {
    g_debug ("===> gtk_altimeter_draw_dynamic()");
  }
  g_return_if_fail (IS_GTK_ALTIMETER (alt));

  gtk_altimeter_draw_digital (alt, cr);
  gtk_altimeter_draw_hands (alt, cr);
}

/**
 * @fn static void gtk_altimeter_draw_base (GtkWidget * alt, cairo_t * cr)
 * @brief Special Gtk API function. This function use the cairo context<br>
 * created before in order to draw scalable graphics. 
 * 
 * See GObject,Cairo and GTK+ references for more informations: 
 * http://library.gnome.org/devel/references.html.en
 */
static void gtk_altimeter_draw_base (GtkWidget * alt, cairo_t * cr)
{
  GtkAltimeterPrivate *priv;

  if (gtk_altimeter_debug)
  {
    g_debug ("===> gtk_altimeter_draw_base()");
  }
  g_return_if_fail (IS_GTK_ALTIMETER (alt));

  priv = GTK_ALTIMETER_GET_PRIVATE (alt);

  double x, y, rec_x0, rec_y0, rec_width, rec_height, rec_degrees;
  double rec_aspect, rec_corner_radius, rec_radius, radius;
  char str[GTK_ALTIMETER_MAX_STRING];
  int i, factor;

  x = alt->allocation.width / 2;
  y = alt->allocation.height / 2;
  radius = MIN (alt->allocation.width / 2, alt->allocation.height / 2) - 5;
  cairo_pattern_t *pat = NULL;

  rec_x0 = x - radius;
  rec_y0 = y - radius;
  rec_width = radius * 2;
  rec_height = radius * 2;
  rec_aspect = 1.0;
  rec_corner_radius = rec_height / 8.0;

  rec_radius = rec_corner_radius / rec_aspect;
  rec_degrees = M_PI / 180.0;

  // **** Altimeter base
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
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_altimeter.red / 65535,
                            (gdouble) priv->bg_color_altimeter.green / 65535,
                            (gdouble) priv->bg_color_altimeter.blue / 65535);
    else
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_inv.red / 65535,
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
    cairo_set_source (cr, pat);
  }
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  cairo_arc (cr, x, y, radius, 0, 2 * M_PI);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0.6, 0.6, 0.6);
  else
    cairo_set_source_rgb (cr, 1 - 0.6, 1 - 0.6, 1 - 0.6);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  cairo_arc (cr, x, y, radius - 0.07 * radius, 0, 2 * M_PI);
  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_altimeter.red / 65535,
                            (gdouble) priv->bg_color_altimeter.green / 65535,
                            (gdouble) priv->bg_color_altimeter.blue / 65535);
    else
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_inv.red / 65535,
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
    cairo_set_source (cr, pat);
  }
  cairo_fill (cr);
  cairo_stroke (cr);

  // fun
  cairo_arc (cr, x, y, radius - 0.45 * radius, M_PI / 3, 2 * M_PI / 3);
  cairo_move_to (cr, x + 0.27 * radius, y + 0.4764 * radius);
  cairo_arc (cr, x, y, radius - 0.8 * radius, M_PI / 3, 2 * M_PI / 3);
  cairo_line_to (cr, x - 0.27 * radius, y + 0.4764 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0.9, 0.9, 0.9);
  else
    cairo_set_source_rgb (cr, 1 - 0.9, 1 - 0.9, 1 - 0.9);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  cairo_set_line_width (cr, 0.03 * radius);
  cairo_move_to (cr, x - 0.7 * radius, y);
  cairo_line_to (cr, x - 0.1 * radius, y + 0.6 * radius);
  cairo_move_to (cr, x - 0.6 * radius, y);
  cairo_line_to (cr, x + 0 * radius, y + 0.6 * radius);
  cairo_move_to (cr, x - 0.5 * radius, y);
  cairo_line_to (cr, x + 0.1 * radius, y + 0.6 * radius);
  cairo_move_to (cr, x - 0.4 * radius, y);
  cairo_line_to (cr, x + 0.2 * radius, y + 0.6 * radius);
  cairo_move_to (cr, x - 0.3 * radius, y);
  cairo_line_to (cr, x + 0.3 * radius, y + 0.6 * radius);
  cairo_move_to (cr, x - 0.2 * radius, y);
  cairo_line_to (cr, x + 0.4 * radius, y + 0.6 * radius);
  cairo_move_to (cr, x - 0.1 * radius, y);
  cairo_line_to (cr, x + 0.5 * radius, y + 0.6 * radius);

  if (((priv->radial_color) && (priv->grayscale_color)) || ((!priv->radial_color) && (priv->grayscale_color))
      || ((!priv->radial_color) && (!priv->grayscale_color)))
  {
    if (!priv->grayscale_color)
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_altimeter.red / 65535,
                            (gdouble) priv->bg_color_altimeter.green / 65535,
                            (gdouble) priv->bg_color_altimeter.blue / 65535);
    else
      cairo_set_source_rgb (cr, (gdouble) priv->bg_color_inv.red / 65535,
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
    cairo_set_source (cr, pat);
  }
  cairo_stroke (cr);

  // Altimeter ticks 
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 1, 1, 1);
  else
    cairo_set_source_rgb (cr, 0., 0., 0.);
  for (i = 0; i < 50; i++)
  {
    int inset;
    cairo_save (cr);

    if (i % 5 == 0)
      inset = 0.12 * radius;
    else
    {
      inset = 0.06 * radius;
      cairo_set_line_width (cr, 0.5 * cairo_get_line_width (cr));
    }

    cairo_move_to (cr, x + (radius - inset) * cos (M_PI / 2 + i * M_PI / 25),
                   y + (radius - inset) * sin (M_PI / 2 + i * M_PI / 25));
    cairo_line_to (cr, x + radius * cos (M_PI / 2 + i * M_PI / 25), y + radius * sin (M_PI / 2 + i * M_PI / 25));
    cairo_stroke (cr);
    cairo_restore (cr);
  }

  // "Altimeter" drawing
  cairo_select_font_face (cr, "Sans", CAIRO_FONT_SLANT_OBLIQUE, CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size (cr, 0.1 * radius);
  cairo_move_to (cr, x - 0.23 * radius, y - 0.12 * radius);
  cairo_show_text (cr, "Altimeter");
  cairo_stroke (cr);


  cairo_select_font_face (cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
  if (priv->unit_is_feet)
  {
    // drawing unit : FEET
    cairo_save (cr);
    cairo_set_font_size (cr, 0.07 * radius);
    cairo_move_to (cr, x + 0.18 * radius, y - 0.83 * radius);
    cairo_rotate (cr, M_PI / 10);
    cairo_show_text (cr, "FEET");
    cairo_stroke (cr);
    cairo_restore (cr);
  }
  else
  {
    // drawing unit : METER
    cairo_save (cr);
    cairo_set_font_size (cr, 0.07 * radius);
    cairo_move_to (cr, x + 0.145 * radius, y - 0.85 * radius);
    cairo_rotate (cr, M_PI / 10);
    cairo_show_text (cr, "METER");
    cairo_stroke (cr);
    cairo_restore (cr);
  }
  switch (priv->unit_value)
  {
    case 1:
      cairo_save (cr);
      cairo_set_font_size (cr, 0.07 * radius);
      cairo_move_to (cr, x - 0.29 * radius, y - 0.81 * radius);
      cairo_rotate (cr, -M_PI / 10);
      sprintf (str, "%d", priv->unit_value);
      cairo_show_text (cr, str);
      cairo_stroke (cr);
      cairo_restore (cr);
      factor = 100;
      break;
    case 10:
      cairo_save (cr);
      cairo_set_font_size (cr, 0.07 * radius);
      cairo_move_to (cr, x - 0.31 * radius, y - 0.8 * radius);
      cairo_rotate (cr, -M_PI / 10);
      sprintf (str, "%d", priv->unit_value);
      cairo_show_text (cr, str);
      cairo_stroke (cr);
      cairo_restore (cr);
      factor = 10;
      break;
    case 100:
    default:
      cairo_save (cr);
      cairo_set_font_size (cr, 0.07 * radius);
      cairo_move_to (cr, x - 0.33 * radius, y - 0.78 * radius);
      cairo_rotate (cr, -M_PI / 10);
      sprintf (str, "%d", priv->unit_value);
      cairo_show_text (cr, str);
      cairo_stroke (cr);
      cairo_restore (cr);
      factor = 1;
      break;
  }

  // Number drawing
  for (i = 0; i < 10; i++)
  {
    int inset;
    cairo_select_font_face (cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
    cairo_set_font_size (cr, 0.20 * radius);
    inset = 0.225 * radius;
    cairo_move_to (cr, x - 0.065 * radius + (radius - inset) * cos (M_PI / 2 + i * M_PI / 5 + M_PI),
                   y + 0.07 * radius + (radius - inset) * sin (M_PI / 2 + i * M_PI / 5 + M_PI));
    sprintf (str, "%d", i);
    cairo_show_text (cr, str);
    cairo_stroke (cr);
  }

  priv->radius = radius;
  priv->x = x;
  priv->y = y;
  cairo_pattern_destroy (pat);
  return;
}

/**
 * @fn static void gtk_altimeter_draw_screws (GtkWidget * alt, cairo_t * cr)
 * @brief Private widget's function that draw the widget's screws using cairo.
 */
static void gtk_altimeter_draw_screws (GtkWidget * alt, cairo_t * cr)
{
  GtkAltimeterPrivate *priv;

  if (gtk_altimeter_debug)
  {
    g_debug ("===> gtk_altimeter_draw_screws()");
  }
  g_return_if_fail (IS_GTK_ALTIMETER (alt));

  priv = GTK_ALTIMETER_GET_PRIVATE (alt);

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
 * @fn static void gtk_altimeter_draw_digital (GtkWidget * alt)
 * @brief Widget function that draw the digital afficher of the widget
 * using cairo. 
 * 
 * See GObject,Cairo and GTK+ references for more informations: 
 * http://library.gnome.org/devel/references.html.en
 */
static void gtk_altimeter_draw_digital (GtkWidget * alt, cairo_t * cr)
{
  GtkAltimeterPrivate *priv;
  if (gtk_altimeter_debug)
  {
    g_debug ("===> gtk_altimeter_draw_digital()");
  }
  g_return_if_fail (IS_GTK_ALTIMETER (alt));

  priv = GTK_ALTIMETER_GET_PRIVATE (alt);

  double x = priv->x;
  double y = priv->y;
  double radius = priv->radius;
  char str[GTK_ALTIMETER_MAX_STRING];
  int altitu = priv->altitude * 1000;

  cairo_select_font_face (cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size (cr, 0.20 * radius);
    
  //digital aff
  cairo_set_line_width (cr, 1);
  cairo_rectangle (cr, x - 0.145 * radius, y - 0.29 * radius - 0.165 * radius, 0.145 * radius, 0.18 * radius);
  cairo_rectangle (cr, x - 0.29 * radius, y - 0.29 * radius - 0.165 * radius, 0.145 * radius, 0.18 * radius);
  cairo_rectangle (cr, x - 0.435 * radius, y - 0.29 * radius - 0.165 * radius, 0.145 * radius, 0.18 * radius);
  cairo_rectangle (cr, x + 0 * radius, y - 0.29 * radius - 0.165 * radius, 0.145 * radius, 0.18 * radius);
  cairo_rectangle (cr, x + 0.145 * radius, y - 0.29 * radius - 0.165 * radius, 0.145 * radius, 0.18 * radius);
  cairo_rectangle (cr, x + 0.29 * radius, y - 0.29 * radius - 0.165 * radius, 0.145 * radius, 0.18 * radius);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0.8, 0.8, 0.8);
  else
    cairo_set_source_rgb (cr, 1 - 0.8, 1 - 0.8, 1 - 0.8);
  cairo_stroke (cr);

  cairo_move_to (cr, x - 0.427 * radius, y - 0.29 * radius);      // X00000
  sprintf (str, "%d", altitu / 100000000 % 10);
  cairo_show_text (cr, str);
  cairo_move_to (cr, x - 0.282 * radius, y - 0.29 * radius);      // 0X0000
  sprintf (str, "%d", altitu / 10000000 % 10);
  cairo_show_text (cr, str);
  cairo_move_to (cr, x - 0.137 * radius, y - 0.29 * radius);      // 00X000
  sprintf (str, "%d", altitu / 1000000 % 10);
  cairo_show_text (cr, str);
  cairo_move_to (cr, x + 0.008 * radius, y - 0.29 * radius);      // 000X00
  sprintf (str, "%d", altitu / 100000 % 10);
  cairo_show_text (cr, str);
  cairo_move_to (cr, x + 0.153 * radius, y - 0.29 * radius);      // 0000X0
  sprintf (str, "%d", altitu / 10000 % 10);
  cairo_show_text (cr, str);
  cairo_move_to (cr, x + 0.298 * radius, y - 0.29 * radius);      // 00000X
  sprintf (str, "%d", altitu / 1000 % 10);
  cairo_show_text (cr, str);
  cairo_stroke (cr);
  return;
}

 /**
 * @fn static void gtk_altimeter_draw_hands (GtkWidget * alt)
 * @brief Private widget's function that draw the altimeter hands using cairo.
 */
static void gtk_altimeter_draw_hands (GtkWidget * alt, cairo_t * cr)
{
  GtkAltimeterPrivate *priv;
  if (gtk_altimeter_debug)
  {
    g_debug ("===> gtk_altimeter_draw_hands()");
  }
  g_return_if_fail (IS_GTK_ALTIMETER (alt));

  priv = GTK_ALTIMETER_GET_PRIVATE (alt);

  double x = priv->x;
  double y = priv->y;
  double radius = priv->radius;
  int factor = 1;
  int altitu = priv->altitude * 1000;

  // 10 thousand hand
  cairo_save (cr);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 1, 1, 1);
  else
    cairo_set_source_rgb (cr, 0, 0, 0);
  cairo_set_line_width (cr, 2);
  cairo_move_to (cr, x, y);
  cairo_line_to (cr, x + radius * sin (5 * M_PI / 25
                                             * (altitu / (10000000 / factor) % 10) + M_PI / 50
                                             * (altitu / (1000000 / factor) % 10) + M_PI / 500
                                             * (altitu / (100000 / factor) % 10) + M_PI / 5000
                                             * (altitu / (10000 / factor) % 10) + M_PI / 50000
                                             * (altitu / (1000 / factor) % 10)),
                 y + radius * -cos (5 * M_PI / 25
                                    * (altitu / (10000000 / factor) % 10) + M_PI / 50
                                    * (altitu / (1000000 / factor) % 10) + M_PI / 500
                                    * (altitu / (100000 / factor) % 10) + M_PI / 5000
                                    * (altitu / (10000 / factor) % 10) + M_PI / 50000
                                    * (altitu / (1000 / factor) % 10)));

  cairo_move_to (cr, x + (radius - 0.1 * radius) * sin (5 * M_PI / 25
                                                              * (altitu / (10000000 / factor) % 10) + M_PI / 50
                                                              * (altitu / (1000000 / factor) % 10) + M_PI / 500
                                                              * (altitu / (100000 / factor) % 10) + M_PI / 5000
                                                              * (altitu / (10000 / factor) % 10) + M_PI / 50000
                                                              * (altitu / (1000 / factor) % 10)),
                 y + (radius - 0.1 * radius) * -cos (5 * M_PI / 25
                                                     * (altitu / (10000000 / factor) % 10) + M_PI / 50
                                                     * (altitu / (1000000 / factor) % 10) + M_PI / 500
                                                     * (altitu / (100000 / factor) % 10) + M_PI / 5000
                                                     * (altitu / (10000 / factor) % 10) + M_PI / 50000
                                                     * (altitu / (1000 / factor) % 10)));
  cairo_line_to (cr, x + radius * sin (M_PI / 50 + 5 * M_PI / 25
                                             * (altitu / (10000000 / factor) % 10) + M_PI / 50
                                             * (altitu / (1000000 / factor) % 10) + M_PI / 500
                                             * (altitu / (100000 / factor) % 10) + M_PI / 5000
                                             * (altitu / (10000 / factor) % 10) + M_PI / 50000
                                             * (altitu / (1000 / factor) % 10)),
                 y + radius * -cos (M_PI / 50 + 5 * M_PI / 25
                                    * (altitu / (10000000 / factor) % 10) + M_PI / 50
                                    * (altitu / (1000000 / factor) % 10) + M_PI / 500
                                    * (altitu / (100000 / factor) % 10) + M_PI / 5000
                                    * (altitu / (10000 / factor) % 10) + M_PI / 50000
                                    * (altitu / (1000 / factor) % 10)));
  cairo_line_to (cr, x + radius * sin (-M_PI / 50 + 5 * M_PI / 25
                                             * (altitu / (10000000 / factor) % 10) + M_PI / 50
                                             * (altitu / (1000000 / factor) % 10) + M_PI / 500
                                             * (altitu / (100000 / factor) % 10) + M_PI / 5000
                                             * (altitu / (10000 / factor) % 10) + M_PI / 50000
                                             * (altitu / (1000 / factor) % 10)),
                 y + radius * -cos (-M_PI / 50 + 5 * M_PI / 25
                                    * (altitu / (10000000 / factor) % 10) + M_PI / 50
                                    * (altitu / (1000000 / factor) % 10) + M_PI / 500
                                    * (altitu / (100000 / factor) % 10) + M_PI / 5000
                                    * (altitu / (10000 / factor) % 10) + M_PI / 50000
                                    * (altitu / (1000 / factor) % 10)));
  cairo_move_to (cr, x + (radius - 0.1 * radius) * sin (5 * M_PI / 25
                                                              * (altitu / (10000000 / factor) % 10) + M_PI / 50
                                                              * (altitu / (1000000 / factor) % 10) + M_PI / 500
                                                              * (altitu / (100000 / factor) % 10) + M_PI / 5000
                                                              * (altitu / (10000 / factor) % 10) + M_PI / 50000
                                                              * (altitu / (1000 / factor) % 10)),
                 y + (radius - 0.1 * radius) * -cos (5 * M_PI / 25
                                                     * (altitu / (10000000 / factor) % 10) + M_PI / 50
                                                     * (altitu / (1000000 / factor) % 10) + M_PI / 500
                                                     * (altitu / (100000 / factor) % 10) + M_PI / 5000
                                                     * (altitu / (10000 / factor) % 10) + M_PI / 50000
                                                     * (altitu / (1000 / factor) % 10)));
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_restore (cr);

  // thousand hand
  cairo_save (cr);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 1, 1, 1);
  else
    cairo_set_source_rgb (cr, 0, 0, 0);
  cairo_set_line_width (cr, 0.03 * radius);
  cairo_move_to (cr, x, y);

  cairo_line_to (cr, x + (radius - 0.7 * radius) * sin (5 * M_PI / 25
                                                              * (altitu / (1000000 / factor) % 10) + M_PI / 50
                                                              * (altitu / (100000 / factor) % 10) + M_PI / 500
                                                              * (altitu / (10000 / factor) % 10) + M_PI / 5000
                                                              * (altitu / (1000 / factor) % 10) + M_PI / 50000
                                                              * (altitu / (100 / factor) % 10)),
                 y + (radius - 0.7 * radius) * -cos (5 * M_PI / 25
                                                     * (altitu / (1000000 / factor) % 10) + M_PI / 50
                                                     * (altitu / (100000 / factor) % 10) + M_PI / 500
                                                     * (altitu / (10000 / factor) % 10) + M_PI / 5000
                                                     * (altitu / (1000 / factor) % 10) + M_PI / 50000
                                                     * (altitu / (100 / factor) % 10)));
  cairo_stroke (cr);
  cairo_restore (cr);

  cairo_save (cr);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 1, 1, 1);
  else
    cairo_set_source_rgb (cr, 0, 0, 0);
  cairo_set_line_width (cr, 1);
  cairo_move_to (cr, x, y);
  cairo_line_to (cr, x + radius / 3 * sin (M_PI / 15 + 5 * M_PI / 25
                                                 * (altitu / (1000000 / factor) % 10) + M_PI / 50
                                                 * (altitu / (100000 / factor) % 10) + M_PI / 500
                                                 * (altitu / (10000 / factor) % 10) + M_PI / 5000
                                                 * (altitu / (1000 / factor) % 10) + M_PI / 50000
                                                 * (altitu / (100 / factor) % 10)),
                 y + radius / 3 * -cos (M_PI / 15 + 5 * M_PI / 25
                                        * (altitu / (1000000 / factor) % 10) + M_PI / 50
                                        * (altitu / (100000 / factor) % 10) + M_PI / 500
                                        * (altitu / (10000 / factor) % 10) + M_PI / 5000
                                        * (altitu / (1000 / factor) % 10) + M_PI / 50000
                                        * (altitu / (100 / factor) % 10)));
  cairo_line_to (cr, x + radius / 2 * sin (5 * M_PI / 25
                                                 * (altitu / (1000000 / factor) % 10) + M_PI / 50
                                                 * (altitu / (100000 / factor) % 10) + M_PI / 500
                                                 * (altitu / (10000 / factor) % 10) + M_PI / 5000
                                                 * (altitu / (1000 / factor) % 10) + M_PI / 50000
                                                 * (altitu / (100 / factor) % 10)),
                 y + radius / 2 * -cos (5 * M_PI / 25
                                        * (altitu / (1000000 / factor) % 10) + M_PI / 50
                                        * (altitu / (100000 / factor) % 10) + M_PI / 500
                                        * (altitu / (10000 / factor) % 10) + M_PI / 5000
                                        * (altitu / (1000 / factor) % 10) + M_PI / 50000
                                        * (altitu / (100 / factor) % 10)));
  cairo_line_to (cr, x + radius / 3 * sin (-M_PI / 15 + 5 * M_PI / 25
                                                 * (altitu / (1000000 / factor) % 10) + M_PI / 50
                                                 * (altitu / (100000 / factor) % 10) + M_PI / 500
                                                 * (altitu / (10000 / factor) % 10) + M_PI / 5000
                                                 * (altitu / (1000 / factor) % 10) + M_PI / 50000
                                                 * (altitu / (100 / factor) % 10)),
                 y + radius / 3 * -cos (-M_PI / 15 + 5 * M_PI / 25
                                        * (altitu / (1000000 / factor) % 10) + M_PI / 50
                                        * (altitu / (100000 / factor) % 10) + M_PI / 500
                                        * (altitu / (10000 / factor) % 10) + M_PI / 5000
                                        * (altitu / (1000 / factor) % 10) + M_PI / 50000
                                        * (altitu / (100 / factor) % 10)));
  cairo_line_to (cr, x, y);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_restore (cr);

  cairo_save (cr);
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 0, 0, 0);
  else
    cairo_set_source_rgb (cr, 1, 1, 1);
  cairo_arc (cr, x, y, radius - 0.86 * radius, M_PI / 3 + 5 * M_PI / 25
             * (altitu / (1000000 / factor) % 10) + M_PI / 50
             * (altitu / (100000 / factor) % 10) + M_PI / 500
             * (altitu / (10000 / factor) % 10) + M_PI / 5000
             * (altitu / (1000 / factor) % 10) + M_PI / 50000
             * (altitu / (100 / factor) % 10),
             2 * M_PI / 3 + 5 * M_PI / 25
             * (altitu / (1000000 / factor) % 10) + M_PI / 50
             * (altitu / (100000 / factor) % 10) + M_PI / 500
             * (altitu / (10000 / factor) % 10) + M_PI / 5000
             * (altitu / (1000 / factor) % 10) + M_PI / 50000 * (altitu / (100 / factor) % 10));
  cairo_line_to (cr, x, y);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);
  cairo_restore (cr);

  // hundred hand
  if (factor == 100)
  {
    cairo_save (cr);
    if (!priv->grayscale_color)
      cairo_set_source_rgb (cr, 1, 1, 1);
    else
      cairo_set_source_rgb (cr, 0, 0, 0);
    cairo_set_line_width (cr, 0.03 * radius);
    cairo_move_to (cr, x, y);
    cairo_line_to (cr, x + (radius - 0.2 * radius) * sin (5 * M_PI / 25
                                                                * (altitu / (100000 / factor) % 10) + M_PI / 50
                                                                * (altitu / (10000 / factor) % 10) + M_PI / 500
                                                                * (altitu / (1000 / factor) % 10) + M_PI / 5000
                                                                * (altitu / (100 / factor) % 10)),
                   y + (radius - 0.2 * radius) * -cos (5 * M_PI / 25
                                                       * (altitu / (100000 / factor) % 10) + M_PI / 50
                                                       * (altitu / (10000 / factor) % 10) + M_PI / 500
                                                       * (altitu / (1000 / factor) % 10) + M_PI / 5000
                                                       * (altitu / (100 / factor) % 10)));
    cairo_stroke (cr);
    cairo_restore (cr);

    cairo_save (cr);
    if (!priv->grayscale_color)
      cairo_set_source_rgb (cr, 0, 0, 0);
    else
      cairo_set_source_rgb (cr, 1, 1, 1);
    cairo_set_line_width (cr, 0.03 * radius);
    cairo_move_to (cr, x - (radius - 0.8 * radius) * sin (5 * M_PI / 25
                                                                * (altitu / (100000 / factor) % 10) + M_PI / 50
                                                                * (altitu / (10000 / factor) % 10) + M_PI / 500
                                                                * (altitu / (1000 / factor) % 10) + M_PI / 5000
                                                                * (altitu / (100 / factor) % 10)),
                   y - (radius - 0.8 * radius) * -cos (5 * M_PI / 25
                                                       * (altitu / (100000 / factor) % 10) + M_PI / 50
                                                       * (altitu / (10000 / factor) % 10) + M_PI / 500
                                                       * (altitu / (1000 / factor) % 10) + M_PI / 5000
                                                       * (altitu / (100 / factor) % 10)));
    cairo_arc (cr, x - (radius - 0.8 * radius) * sin (5 * M_PI / 25
                                                            * (altitu / (100000 / factor) % 10) + M_PI / 50
                                                            * (altitu / (10000 / factor) % 10) + M_PI / 500
                                                            * (altitu / (1000 / factor) % 10) + M_PI / 5000
                                                            * (altitu / (100 / factor) % 10)),
               y - (radius - 0.8 * radius) * -cos (5 * M_PI / 25
                                                   * (altitu / (100000 / factor) % 10) + M_PI / 50
                                                   * (altitu / (10000 / factor) % 10) + M_PI / 500
                                                   * (altitu / (1000 / factor) % 10) + M_PI / 5000
                                                   * (altitu / (100 / factor) % 10)),
               radius - 0.98 * radius, 0, 2 * M_PI);
    cairo_move_to (cr, x - (radius - 0.8 * radius) * sin (5 * M_PI / 25
                                                                * (altitu / (100000 / factor) % 10) + M_PI / 50
                                                                * (altitu / (10000 / factor) % 10) + M_PI / 500
                                                                * (altitu / (1000 / factor) % 10) + M_PI / 5000
                                                                * (altitu / (100 / factor) % 10)),
                   y - (radius - 0.8 * radius) * -cos (5 * M_PI / 25
                                                       * (altitu / (100000 / factor) % 10) + M_PI / 50
                                                       * (altitu / (10000 / factor) % 10) + M_PI / 500
                                                       * (altitu / (1000 / factor) % 10) + M_PI / 5000
                                                       * (altitu / (100 / factor) % 10)));
    cairo_line_to (cr, x, y);
    cairo_stroke (cr);
    cairo_restore (cr);
  }
  else
  {
    cairo_save (cr);
    if (!priv->grayscale_color)
      cairo_set_source_rgb (cr, 1, 1, 1);
    else
      cairo_set_source_rgb (cr, 0, 0, 0);
    cairo_set_line_width (cr, 0.03 * radius);
    cairo_move_to (cr, x, y);
    cairo_line_to (cr, x + (radius - 0.2 * radius) * sin (5 * M_PI / 25
                                                                * (altitu / (100000 / factor) % 10) + M_PI / 50
                                                                * (altitu / (10000 / factor) % 10) + M_PI / 500
                                                                * (altitu / (1000 / factor) % 10) + M_PI / 5000
                                                                * (altitu / (100 / factor) % 10) + M_PI / 50000
                                                                * (altitu / (10 / factor) % 10)),
                   y + (radius - 0.2 * radius) * -cos (5 * M_PI / 25
                                                       * (altitu / (100000 / factor) % 10) + M_PI / 50
                                                       * (altitu / (10000 / factor) % 10) + M_PI / 500
                                                       * (altitu / (1000 / factor) % 10) + M_PI / 5000
                                                       * (altitu / (100 / factor) % 10) + M_PI / 50000
                                                       * (altitu / (10 / factor) % 10)));
    cairo_stroke (cr);
    cairo_restore (cr);

    cairo_save (cr);
    if (!priv->grayscale_color)
      cairo_set_source_rgb (cr, 0, 0, 0);
    else
      cairo_set_source_rgb (cr, 1, 1, 1);
    cairo_set_line_width (cr, 0.03 * radius);
    cairo_move_to (cr, x - (radius - 0.8 * radius) * sin (5 * M_PI / 25
                                                                * (altitu / (100000 / factor) % 10) + M_PI / 50
                                                                * (altitu / (10000 / factor) % 10) + M_PI / 500
                                                                * (altitu / (1000 / factor) % 10) + M_PI / 5000
                                                                * (altitu / (100 / factor) % 10) + M_PI / 50000
                                                                * (altitu / (10 / factor) % 10)),
                   y - (radius - 0.8 * radius) * -cos (5 * M_PI / 25
                                                       * (altitu / (100000 / factor) % 10) + M_PI / 50
                                                       * (altitu / (10000 / factor) % 10) + M_PI / 500
                                                       * (altitu / (1000 / factor) % 10) + M_PI / 5000
                                                       * (altitu / (100 / factor) % 10) + M_PI / 50000
                                                       * (altitu / (10 / factor) % 10)));
    cairo_arc (cr, x - (radius - 0.8 * radius) * sin (5 * M_PI / 25
                                                            * (altitu / (100000 / factor) % 10) + M_PI / 50
                                                            * (altitu / (10000 / factor) % 10) + M_PI / 500
                                                            * (altitu / (1000 / factor) % 10) + M_PI / 5000
                                                            * (altitu / (100 / factor) % 10) + M_PI / 50000
                                                            * (altitu / (10 / factor) % 10)),
               y - (radius - 0.8 * radius) * -cos (5 * M_PI / 25
                                                   * (altitu / (100000 / factor) % 10) + M_PI / 50
                                                   * (altitu / (10000 / factor) % 10) + M_PI / 500
                                                   * (altitu / (1000 / factor) % 10) + M_PI / 5000
                                                   * (altitu / (100 / factor) % 10) + M_PI / 50000
                                                   * (altitu / (10 / factor) % 10)),
               radius - 0.98 * radius, 0, 2 * M_PI);
    cairo_move_to (cr, x - (radius - 0.8 * radius) * sin (5 * M_PI / 25
                                                                * (altitu / (100000 / factor) % 10) + M_PI / 50
                                                                * (altitu / (10000 / factor) % 10) + M_PI / 500
                                                                * (altitu / (1000 / factor) % 10) + M_PI / 5000
                                                                * (altitu / (100 / factor) % 10) + M_PI / 50000
                                                                * (altitu / (10 / factor) % 10)),
                   y - (radius - 0.8 * radius) * -cos (5 * M_PI / 25
                                                       * (altitu / (100000 / factor) % 10) + M_PI / 50
                                                       * (altitu / (10000 / factor) % 10) + M_PI / 500
                                                       * (altitu / (1000 / factor) % 10) + M_PI / 5000
                                                       * (altitu / (100 / factor) % 10) + M_PI / 50000
                                                       * (altitu / (10 / factor) % 10)));
    cairo_line_to (cr, x, y);
    cairo_stroke (cr);
    cairo_restore (cr);
  }

  // centre cercle 
  if (!priv->grayscale_color)
    cairo_set_source_rgb (cr, 1, 1, 1);
  else
    cairo_set_source_rgb (cr, 0, 0, 0);
  cairo_arc (cr, x, y, radius - 0.98 * radius, 0, 2 * M_PI);
  cairo_fill_preserve (cr);
  cairo_stroke (cr);

  return;
}

/**
 * @fn static gboolean gtk_altimeter_button_press_event (GtkWidget * widget, GdkEventButton * ev)
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
static gboolean gtk_altimeter_button_press_event (GtkWidget * widget, GdkEventButton * ev)
{
  GtkAltimeterPrivate *priv;

  if (gtk_altimeter_debug)
  {
    g_debug ("===> gtk_altimeter_button_press_event_cb()");
  }
  g_return_val_if_fail (IS_GTK_ALTIMETER (widget), FALSE);

  priv = GTK_ALTIMETER_GET_PRIVATE (widget);

  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 2) && priv->b_mouse_onoff)
  {
    gtk_altimeter_debug = gtk_altimeter_debug ? FALSE : TRUE;
    return TRUE;
  }
  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 1) && priv->b_mouse_onoff)
  {
    gtk_altimeter_lock_update = gtk_altimeter_lock_update ? FALSE : TRUE;
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
 * @fn static void gtk_altimeter_destroy (GtkObject * object)
 * @brief Special Gtk API function. Override the _destroy handler.<br>
 * Allow the destruction of all widget's pointer.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_altimeter_destroy (GtkObject * object)
{
  GtkAltimeterPrivate *priv = NULL;
  GtkWidget *widget = NULL;

  if (gtk_altimeter_debug)
  {
    g_debug ("===> gtk_altimeter_destroy(enter)");
  }

  g_return_if_fail (object != NULL);

  widget = GTK_WIDGET (object);

  g_return_if_fail (IS_GTK_ALTIMETER (widget));

  priv = GTK_ALTIMETER_GET_PRIVATE (widget);
  g_return_if_fail (priv != NULL);

  if (priv->static_surface)
  {
    priv->static_surface=NULL;
    priv->dynamic_surface=NULL;

    if (GTK_OBJECT_CLASS (gtk_altimeter_parent_class)->destroy != NULL)
    {
      (*GTK_OBJECT_CLASS (gtk_altimeter_parent_class)->destroy) (object);
    }
  }
  if (gtk_altimeter_debug)
  {
    g_debug ("gtk_altimeter_destroy(exit)");
  }

  return;
}

/**
 * @fn static void gtk_altimeter_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec)
 * @brief Special Gtk API function. Override the _set_property handler <br>
 * in order to set the object parameters.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gtk_altimeter_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec)
{
  GtkAltimeterPrivate *priv = NULL;
  GtkAltimeter *alt = NULL;

  if (gtk_altimeter_debug)
  {
    g_debug ("===> gtk_altimeter_set_property()");
  }
  g_return_if_fail (object != NULL);

  alt = GTK_ALTIMETER (object);
  g_return_if_fail (IS_GTK_ALTIMETER (alt));

  priv = GTK_ALTIMETER_GET_PRIVATE (alt);
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
      if ((priv->unit_value != 100) && (priv->unit_value != 10) && (priv->unit_value != 1))
      {
        priv->unit_value = 100;
        g_warning ("GtkAltimeter: gtk_altimeter_set_property: unit-step-value out of range");
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
