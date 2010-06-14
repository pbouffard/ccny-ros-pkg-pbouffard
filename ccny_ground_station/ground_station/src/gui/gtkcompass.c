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

#include <ground_station/gui/gtkcompass.h>

typedef struct _GtkCompassPrivate
{
  /* new cairo design */
  cairo_t *cr;
  GdkRectangle plot_box;

  /* widget data */
  gboolean color_mode_inv;

  /* drawing data */
  gdouble x;
  gdouble y;
  gdouble radius;
  GdkColor bg_color;
  GdkColor bg_color_inv;

  /* mouse information */
  gboolean b_mouse_onoff;
  GdkPoint mouse_pos;
  GdkModifierType mouse_state;

} GtkCompassPrivate;

enum _GLG_PROPERTY_ID
{
  PROP_0,
  PROP_INVERSED_COLOR,
} GLG_PROPERTY_ID;

G_DEFINE_TYPE (GtkCompass, gtk_compass, GTK_TYPE_DRAWING_AREA);

#define GTK_COMPASS_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GTK_COMPASS_TYPE, GtkCompassPrivate))

static void gtk_compass_class_init (GtkCompassClass * klass);
static void gtk_compass_init (GtkCompass * alt);
static void gtk_compass_destroy (GtkObject * object);
static void gtk_compass_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec);

static gboolean gtk_compass_configure_event (GtkWidget * widget, GdkEventConfigure * event);
static gboolean gtk_compass_expose (GtkWidget * graph, GdkEventExpose * event);
static gboolean gtk_compass_button_press_event (GtkWidget * widget, GdkEventButton * ev);
static gboolean gtk_compass_motion_notify_event (GtkWidget * widget, GdkEventMotion * ev);

static void gtk_compass_draw (GtkWidget * alt);
static void gtk_compass_draw_digital (GtkWidget * alt);

static gboolean gtk_compass_debug = FALSE;

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
                                   PROP_INVERSED_COLOR,
                                   g_param_spec_int ("inverse-color",
                                                     "inverse or not the widget color",
                                                     "inverse or not the widget color", 0, 1, 0, G_PARAM_WRITABLE));
  return;
}

static void gtk_compass_init (GtkCompass * alt)
{
  GtkCompassPrivate *priv = NULL;

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_init()");
  }
  g_return_if_fail (IS_GTK_COMPASS (alt));

  priv = GTK_COMPASS_GET_PRIVATE (alt);

  gtk_widget_add_events (GTK_WIDGET (alt), GDK_BUTTON_PRESS_MASK |
                         GDK_BUTTON_RELEASE_MASK | GDK_POINTER_MOTION_MASK | GDK_POINTER_MOTION_HINT_MASK);
  priv->b_mouse_onoff = FALSE;

  priv->bg_color.red = 6553, 5;
  priv->bg_color.green = 6553, 5;
  priv->bg_color.blue = 6553, 5;
  priv->bg_color_inv.red = 45874, 5;
  priv->bg_color_inv.green = 45874, 5;
  priv->bg_color_inv.blue = 45874, 5;

  return;
}

static gboolean gtk_compass_configure_event (GtkWidget * widget, GdkEventConfigure * event)
{
  GtkCompassPrivate *priv;
  GtkCompass *alt = GTK_COMPASS (widget);

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_configure_event()");
  }
  g_return_val_if_fail (IS_GTK_COMPASS (alt), FALSE);

  g_return_val_if_fail (event->type == GDK_CONFIGURE, FALSE);

  priv = GTK_COMPASS_GET_PRIVATE (alt);
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

static gboolean gtk_compass_expose (GtkWidget * alt, GdkEventExpose * event)
{
  GtkCompassPrivate *priv;
  GtkWidget *widget = alt;

  cairo_t *cr = NULL;
  cairo_status_t status;

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_expose()");
  }
  g_return_val_if_fail (IS_GTK_COMPASS (alt), FALSE);

  priv = GTK_COMPASS_GET_PRIVATE (alt);
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

  gtk_compass_draw (alt);

  cairo_destroy (cr);
  priv->cr = NULL;

  return FALSE;
}

extern void gtk_compass_redraw (GtkCompass * alt)
{
  GtkWidget *widget;
  GdkRegion *region;

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_redraw()");
  }
  g_return_if_fail (IS_GTK_COMPASS (alt));

  widget = GTK_WIDGET (alt);

  if (!widget->window)
    return;

  region = gdk_drawable_get_clip_region (widget->window);
  /* redraw the window completely by exposing it */
  gdk_window_invalidate_region (widget->window, region, TRUE);
  gdk_window_process_updates (widget->window, TRUE);

  gdk_region_destroy (region);
}

extern GtkWidget *gtk_compass_new (void)
{
  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_new()");
  }
  return GTK_WIDGET(gtk_type_new(gtk_compass_get_type()));
}

static void gtk_compass_draw (GtkWidget * alt)
{
  GtkCompassPrivate *priv;

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_draw()");
  }
  g_return_if_fail (IS_GTK_COMPASS (alt));

  priv = GTK_COMPASS_GET_PRIVATE (alt);

  double x, y, rec_x0, rec_y0, rec_width, rec_height, rec_degrees;
  double rec_aspect, rec_corner_radius, rec_radius, radius;
  char str[GTK_COMPASS_MAX_STRING];
  int i, factor;

  x = alt->allocation.width / 2;
  y = alt->allocation.height / 2;
  radius = MIN (alt->allocation.width / 2, alt->allocation.height / 2) - 5;

	rec_x0=x-radius;    
   rec_y0=y-radius;
   rec_width=radius*2;
   rec_height=radius*2;
   rec_aspect=1.0;
   rec_corner_radius=rec_height/8.0;

	rec_radius=rec_corner_radius/rec_aspect;
	rec_degrees=M_PI/180.0;

	// Compass base
	cairo_new_sub_path (priv->cr);
	cairo_arc (priv->cr, rec_x0+rec_width-rec_radius, rec_y0+rec_radius,
					rec_radius, -90*rec_degrees, 0*rec_degrees);
	cairo_arc (priv->cr, rec_x0+rec_width-rec_radius, rec_y0+rec_height-rec_radius,
					rec_radius, 0*rec_degrees, 90*rec_degrees);
	cairo_arc (priv->cr, rec_x0+rec_radius, rec_y0+rec_height-rec_radius,
					rec_radius, 90*rec_degrees, 180*rec_degrees);
	cairo_arc (priv->cr, rec_x0+rec_radius, rec_y0+rec_radius, rec_radius,
					180*rec_degrees, 270*rec_degrees);
	cairo_close_path (priv->cr);

	if(!priv->color_mode_inv) cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color.red / 65535,
                          (gdouble) priv->bg_color.green / 65535, (gdouble) priv->bg_color.blue / 65535);
	else cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_inv.red / 65535,
                          (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
	cairo_fill_preserve (priv->cr);
	cairo_stroke (priv->cr);

	cairo_arc (priv->cr, x, y, radius, 0, 2 * M_PI);
	if(!priv->color_mode_inv) cairo_set_source_rgb (priv->cr, 0., 0., 0.);
	else cairo_set_source_rgb (priv->cr, 1., 1., 1.);
	cairo_fill_preserve (priv->cr);
	cairo_stroke (priv->cr);
	
	cairo_arc (priv->cr, x, y, radius-0.04*radius, 0, 2 * M_PI);
	if(!priv->color_mode_inv) cairo_set_source_rgb (priv->cr, 0.6, 0.5, 0.5);
	else cairo_set_source_rgb (priv->cr, 1-0.6, 1-0.5, 1-0.5);
	cairo_stroke (priv->cr);

	cairo_set_line_width (priv->cr,0.01*radius);
	radius=radius-0.1*radius;
	cairo_arc (priv->cr, x, y, radius, 0, 2 * M_PI);
	if(!priv->color_mode_inv) cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color.red / 65535,
                          (gdouble) priv->bg_color.green / 65535, (gdouble) priv->bg_color.blue / 65535);
	else cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_inv.red / 65535,
                          (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
	cairo_fill_preserve (priv->cr);
	cairo_stroke (priv->cr);


  priv->radius = radius;
  priv->x = x;
  priv->y = y;

  // draw digital 
  gtk_compass_draw_digital (alt);

  return;
}

static void gtk_compass_draw_digital (GtkWidget * alt)
{
  GtkCompassPrivate *priv;
  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_draw_digital()");
  }
  g_return_if_fail (IS_GTK_COMPASS (alt));

  priv = GTK_COMPASS_GET_PRIVATE (alt);

  double x = priv->x;
  double y = priv->y;
  double radius = priv->radius;
  char str[GTK_COMPASS_MAX_STRING];
  int i;
  	
	// Number drawing
	for (i = 0; i < 12; i++)
	{
		int inset;
		cairo_select_font_face (priv->cr, "Sans", CAIRO_FONT_SLANT_NORMAL,
                               CAIRO_FONT_WEIGHT_NORMAL);
      cairo_set_font_size (priv->cr, 0.20*radius);
		inset = 0.15*radius;
		cairo_move_to (priv->cr,x-0.069*radius+(radius-inset)*cos(i*M_PI/6-3*M_PI/6),
								y+0.07*radius+(radius-inset)*sin(i*M_PI/6-3*M_PI/6));
		if(!priv->color_mode_inv) cairo_set_source_rgb(priv->cr, 0.88, 0.88, 0);
		else cairo_set_source_rgb (priv->cr, 1-0.88, 1-0.88, 1-0.);
		switch(i)
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
				if(!priv->color_mode_inv) cairo_set_source_rgb(priv->cr, 1, 1, 1);
				else cairo_set_source_rgb (priv->cr, 0., 0., 0.);
				sprintf(str,"%d",i*3);
				cairo_show_text (priv->cr, str);	
				cairo_stroke (priv->cr);
			}
		cairo_stroke (priv->cr);
	}
   
   cairo_set_line_width (priv->cr, 0.03 * radius);
   radius=radius-0.3*radius;
   
	/*cairo_arc (priv->cr, x, y, radius, 0, 2 * M_PI);
	if(!priv->color_mode_inv) cairo_set_source_rgb (priv->cr, 0.6, 0.6, 0.6);
	else cairo_set_source_rgb (priv->cr, 1-0.6, 1-0.6, 1-0.6);
	cairo_fill_preserve (priv->cr);
	cairo_stroke (priv->cr);
	
	cairo_arc (priv->cr, x, y, radius-0.07*radius, 0, 2 * M_PI);
	if(!priv->color_mode_inv) cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color.red / 65535,
                          (gdouble) priv->bg_color.green / 65535, (gdouble) priv->bg_color.blue / 65535);
	else cairo_set_source_rgb (priv->cr, (gdouble) priv->bg_color_inv.red / 65535,
                          (gdouble) priv->bg_color_inv.green / 65535, (gdouble) priv->bg_color_inv.blue / 65535);
	cairo_fill_preserve (priv->cr);
	cairo_stroke (priv->cr);*/
	
	// Compass ticks 
	for (i = 0; i < 36; i++)
	{
		int inset;
		cairo_save (priv->cr);
		
		if (i % 9 == 0){
			 inset = 0.12 * radius;
			 if(!priv->color_mode_inv) cairo_set_source_rgb(priv->cr, 0.88, 0.88, 0);
			 else cairo_set_source_rgb (priv->cr, 1-0.88, 1-0.88, 1-0.);
		}
		else
		{
			inset = 0.06*radius;
			cairo_set_line_width (priv->cr, 0.5 *cairo_get_line_width (priv->cr));
			if(!priv->color_mode_inv) cairo_set_source_rgb(priv->cr, 1, 1, 1);
			else cairo_set_source_rgb (priv->cr, 0., 0., 0.);
		}
		
		cairo_move_to (priv->cr,x+(radius-inset)*cos(i*M_PI/18),
								y+(radius-inset)*sin(i*M_PI/18));
		cairo_line_to (priv->cr,x+radius*cos(i*M_PI/18),
								y+radius*sin(i*M_PI/18));
		cairo_stroke (priv->cr);
		cairo_restore (priv->cr);
	}
	
  
  return;
}

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

static void gtk_compass_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec)
{
  GtkCompassPrivate *priv = NULL;
  GtkCompass *alt = NULL;

  if (gtk_compass_debug)
  {
    g_debug ("===> gtk_compass_set_property()");
  }
  g_return_if_fail (object != NULL);

  alt = GTK_COMPASS (object);
  g_return_if_fail (IS_GTK_COMPASS (alt));

  priv = GTK_COMPASS_GET_PRIVATE (alt);
  g_return_if_fail (priv != NULL);

  switch (prop_id)
  {
    case PROP_INVERSED_COLOR:
      if (g_value_get_int (value) == 0)
        priv->color_mode_inv = FALSE;
      else
        priv->color_mode_inv = TRUE;
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
  return;
}
