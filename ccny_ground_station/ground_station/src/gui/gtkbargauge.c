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

#include <ground_station/gui/gtkbargauge.h>

typedef struct _GtkBarGaugePrivate
{
  /* new cairo design */
  cairo_t *cr;
  GdkRectangle plot_box;

  /* widget data */
  gboolean color_mode_inv;
  gboolean radial_color;
    
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

enum _GLG_PROPERTY_ID
{
  PROP_0,
  PROP_BAR_NUMBER,
  PROP_INVERSED_COLOR,
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
} GLG_PROPERTY_ID;

G_DEFINE_TYPE (GtkBarGauge, gtk_bar_gauge, GTK_TYPE_DRAWING_AREA);

#define GTK_BAR_GAUGE_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GTK_BAR_GAUGE_TYPE, GtkBarGaugePrivate))

static void gtk_bar_gauge_class_init (GtkBarGaugeClass * klass);
static void gtk_bar_gauge_init (GtkBarGauge * bg);
static void gtk_bar_gauge_destroy (GtkObject * object);
static void gtk_bar_gauge_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec);

static gboolean gtk_bar_gauge_configure_event (GtkWidget * widget, GdkEventConfigure * event);
static gboolean gtk_bar_gauge_expose (GtkWidget * graph, GdkEventExpose * event);
static gboolean gtk_bar_gauge_button_press_event (GtkWidget * widget, GdkEventButton * ev);
static gboolean gtk_bar_gauge_motion_notify_event (GtkWidget * widget, GdkEventMotion * ev);

static void gtk_bar_gauge_draw (GtkWidget * bg);
static void gtk_bar_gauge_draw_screws (GtkWidget * arh);
static void gtk_bar_gauge_draw_simple (GtkBarGauge * bg, gint index, gdouble x_bar, gdouble y_bar);
static void gtk_bar_gauge_draw_extend (GtkBarGauge * bg, gint index, gdouble x_bar, gdouble y_bar);
static void gtk_bar_gauge_draw_name (GtkBarGauge * bg, gchar * pch_text, GdkRectangle * rect);


static gboolean gtk_bar_gauge_debug = FALSE;

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
  widget_class->motion_notify_event = gtk_bar_gauge_motion_notify_event;
  widget_class->button_press_event = gtk_bar_gauge_button_press_event;

  g_type_class_add_private (obj_class, sizeof (GtkBarGaugePrivate));
  
  g_object_class_install_property (obj_class,
                                   PROP_INVERSED_COLOR,
                                   g_param_spec_boolean ("inverse-color",
                                                         "inverse or not the widget color",
                                                         "inverse or not the widget color", FALSE, G_PARAM_WRITABLE));
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
  priv->gauge_name_font_size_ok = FALSE;

  priv->name_bar1 = g_strdup ("NO NAME");
  priv->start_value1 = -1;
  priv->end_value1 = -1;
  priv->green_strip_start1 = -1;
  priv->yellow_strip_start1 = -1;  
  priv->value_bar1=0;
    
  priv->name_bar2 = g_strdup ("NO NAME");
  priv->start_value2 = -1;
  priv->end_value2 = -1;
  priv->green_strip_start2 = -1;
  priv->yellow_strip_start2 = -1;  
  priv->value_bar2=0;
    
  priv->name_bar3 = g_strdup ("NO NAME");
  priv->start_value3 = -1;
  priv->end_value3 = -1;
  priv->green_strip_start3 = -1;
  priv->yellow_strip_start3 = -1;  
  priv->value_bar3=0;
  
  priv->name_bar4 = g_strdup ("NO NAME");
  priv->start_value4 = -1;
  priv->end_value4 = -1;
  priv->green_strip_start4 = -1;
  priv->yellow_strip_start4 = -1;  
  priv->value_bar4=0;  
  
  priv->color_mode_inv = TRUE;
  priv->radial_color = FALSE;
  
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

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_configure_event(new width=%d, height=%d)", event->width, event->height);
  }

  if ((event->width < GTK_BAR_GAUGE_MODEL_X) || (event->height < GTK_BAR_GAUGE_MODEL_Y))
  {
    priv->plot_box.width = GTK_BAR_GAUGE_MODEL_X;
    priv->plot_box.height = GTK_BAR_GAUGE_MODEL_Y;
  }
  else
  {
    priv->plot_box.width = event->width;
    priv->plot_box.height = event->height;
  }

  // **** force the font size research
  priv->gauge_name_font_size_ok = FALSE;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("cfg:Max.Avail: plot_box.width=%d, plot_box.height=%d", priv->plot_box.width, priv->plot_box.height);
  }
  return FALSE;
}

static gboolean gtk_bar_gauge_expose (GtkWidget * bg, GdkEventExpose * event)
{
  GtkBarGaugePrivate *priv;
  GtkWidget *widget = bg;

  cairo_t *cr = NULL;
  cairo_status_t status;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_expose()");
  }
  g_return_val_if_fail (IS_GTK_BAR_GAUGE (bg), FALSE);

  priv = GTK_BAR_GAUGE_GET_PRIVATE (bg);
  g_return_val_if_fail (priv != NULL, FALSE);

  priv->plot_box.width = widget->allocation.width;
  priv->plot_box.height = widget->allocation.height;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("gtk_bar_gauge_expose(width=%d, height=%d)", widget->allocation.width, widget->allocation.height);
  }

  priv->cr = cr = gdk_cairo_create (widget->window);
  status = cairo_status (cr);
  if (status != CAIRO_STATUS_SUCCESS)
  {
    g_message ("GLG-Expose:cairo_create:status %d=%s", status, cairo_status_to_string (status));
  }

  cairo_rectangle (cr, 0, 0, priv->plot_box.width, priv->plot_box.height);
  cairo_clip (cr);

  gtk_bar_gauge_draw (bg);

  cairo_destroy (cr);
  priv->cr = NULL;

  return FALSE;
}

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

extern void gtk_bar_gauge_set_value (GtkBarGauge * bg, gint bar_index, gdouble val)
{
  GtkBarGaugePrivate *priv;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_draw()");
  }
  g_return_if_fail (IS_GTK_BAR_GAUGE (bg));

  priv = GTK_BAR_GAUGE_GET_PRIVATE (bg);
  
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

extern GtkWidget *gtk_bar_gauge_new (void)
{
  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_new()");
  }
  return GTK_WIDGET (gtk_type_new (gtk_bar_gauge_get_type ()));
}

static void gtk_bar_gauge_draw (GtkWidget * bg)
{
  GtkBarGaugePrivate *priv;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_draw()");
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
  cairo_pattern_t *pat;

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

  priv->x = x;
  priv->y = y;
  priv->radius = radius;
  //gtk_bar_gauge_draw_screws (GTK_BAR_GAUGE(bg));
  switch(priv->bar_number)
  {
	  case 1:
	  	gtk_bar_gauge_draw_extend(GTK_BAR_GAUGE(bg),1,x - 0.35 * radius ,y-0.86*radius);
	  break;
	  case 2:
		gtk_bar_gauge_draw_extend(GTK_BAR_GAUGE(bg),1,x - 0.80 * radius ,y-0.86*radius);
		gtk_bar_gauge_draw_extend(GTK_BAR_GAUGE(bg),2,x + 0.10 * radius ,y-0.86*radius);
		break;
	  case 3:
		gtk_bar_gauge_draw_simple(GTK_BAR_GAUGE(bg),1,x - 0.65 * radius ,y-0.86*radius);
		gtk_bar_gauge_draw_simple(GTK_BAR_GAUGE(bg),2,x - 0.16 * radius ,y-0.86*radius);
		gtk_bar_gauge_draw_simple(GTK_BAR_GAUGE(bg),3,x + 0.33 * radius ,y-0.86*radius);
		break;
	  case 4:
		gtk_bar_gauge_draw_simple(GTK_BAR_GAUGE(bg),1, x - 0.80 * radius ,y-0.86*radius);
		gtk_bar_gauge_draw_simple(GTK_BAR_GAUGE(bg),2, x - 0.36 * radius ,y-0.86*radius);
		gtk_bar_gauge_draw_simple(GTK_BAR_GAUGE(bg),3, x + 0.08 * radius ,y-0.86*radius);
		gtk_bar_gauge_draw_simple(GTK_BAR_GAUGE(bg),4, x + 0.52 * radius ,y-0.86*radius);
	   break;
  }
  
  return;
}

static void gtk_bar_gauge_draw_extend (GtkBarGauge * bg, gint index, gdouble x_bar, gdouble y_bar)
{
  GtkBarGaugePrivate *priv;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_draw()");
  }
  g_return_if_fail (IS_GTK_BAR_GAUGE (bg));

  priv = GTK_BAR_GAUGE_GET_PRIVATE (bg);
  
  double x, y, rec_x0, rec_y0, rec_width, rec_height, rec_degrees;
  double rec_aspect, rec_corner_radius, rec_radius, radius;
  char str[GTK_BAR_GAUGE_MAX_STRING];
  char * name = NULL;
  double value_bar=0, bar_y=0, bar_height=0;
  int start_value_bar=0,end_value_bar=0,green_strip_start=0,yellow_strip_start=0;
  cairo_pattern_t *pat;
  GdkRectangle bar_box,name_box;
 
  x=priv->x;
  y=priv->y;
  radius=priv->radius;
    
  bar_box.x = x_bar;
  bar_box.y = y_bar;
  bar_box.width = 0.3 * radius;
  bar_box.height = 1.5 * radius;
  
  rec_x0 = bar_box.x-0.05*radius;
  rec_y0 = bar_box.y-0.05*radius;
  rec_width = bar_box.width+0.5*radius;
  rec_height = bar_box.height+0.35*radius;
  rec_aspect = 1.0;
  rec_corner_radius = rec_height / 20.0;
  rec_radius = rec_corner_radius / rec_aspect;
  rec_degrees = M_PI / 180.0;

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
        cairo_set_source_rgb(priv->cr,0.55,0.55,0.55);
    else
        cairo_set_source_rgb(priv->cr,1-0.55,1-0.55,1-0.55);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, 0.9,0.9,0.9, 0.8);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.45,0.45,0.45, 0.8);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  
  rec_x0 = bar_box.x;
  rec_y0 = bar_box.y;
  rec_width = bar_box.width+0.4*radius;
  rec_height = bar_box.height;
  rec_aspect = 1.0;
  rec_corner_radius = rec_height / 20.0;
  rec_radius = rec_corner_radius / rec_aspect;
  rec_degrees = M_PI / 180.0;

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
        cairo_set_source_rgb(priv->cr,0.4,0.4,0.4);
    else
        cairo_set_source_rgb(priv->cr,1-0.4,1-0.4,1-0.4);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, 0.9,0.9,0.9, 0.8);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.65,0.65,0.65, 0.8);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
  
  
  switch(index)
  {
	  case 1:
		value_bar=priv->value_bar1;
		start_value_bar=priv->start_value1;
		end_value_bar=priv->end_value1;
		name=priv->name_bar1;
	
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
   
  if(!priv->color_mode_inv)
  {
	  if(value_bar>=green_strip_start)
	  {
			cairo_rectangle(priv->cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (priv->cr,0,1,0, 1);
			cairo_fill_preserve (priv->cr);
			cairo_stroke(priv->cr);
	  }
	  else  if((value_bar<green_strip_start)&&(value_bar>=yellow_strip_start))
	  {
			cairo_rectangle(priv->cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (priv->cr,1,1,0, 1);
			cairo_fill_preserve (priv->cr);
			cairo_stroke(priv->cr);
	  }
	  else  if(value_bar<=yellow_strip_start)
	  {
			cairo_rectangle(priv->cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (priv->cr,1,0,0, 1);
			cairo_fill_preserve (priv->cr);
			cairo_stroke(priv->cr);
	  }
  }
  else
  {
	  if(value_bar>=green_strip_start)
	  {
			cairo_rectangle(priv->cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (priv->cr,0.2,0.2,0.2, 1);
			cairo_fill_preserve (priv->cr);
			cairo_stroke(priv->cr);
	  }
	  else  if((value_bar<green_strip_start)&&(value_bar>=yellow_strip_start))
	  {
			cairo_rectangle(priv->cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (priv->cr,0.8,0.8,0.8, 1);
			
			cairo_fill_preserve (priv->cr);
			cairo_stroke(priv->cr);
	  }
	  else  if(value_bar<=yellow_strip_start)
	  {
			cairo_rectangle(priv->cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (priv->cr,1,1,1, 1);			
			cairo_fill_preserve (priv->cr);
			cairo_stroke(priv->cr);
	  }
  }
     
  if((bar_y>bar_box.y-20)&&(bar_y<bar_box.y+0.05*radius))
		bar_y	= bar_box.y + 0.06*radius;
  else if((bar_y>bar_box.y+bar_box.height-0.05*radius)&&(bar_y<bar_box.y+bar_box.height+20))
			bar_y = bar_box.y+bar_box.height-0.06*radius;
 
	cairo_move_to(priv->cr,bar_box.x+bar_box.width+0.01*radius,bar_y);
	cairo_line_to(priv->cr,bar_box.x+bar_box.width+0.06*radius,bar_y-0.05*radius);
	cairo_line_to(priv->cr,bar_box.x+bar_box.width+0.06*radius,bar_y+0.05*radius);
	cairo_line_to(priv->cr,bar_box.x+bar_box.width,bar_y);
	cairo_set_source_rgb (priv->cr, 1,1,1);
	cairo_fill_preserve(priv->cr);
	cairo_stroke(priv->cr);
 
	// **** draw value
	name_box.x = bar_box.x+bar_box.width+0.08*radius;
	name_box.y = bar_y-0.052*radius;
	name_box.width = 0.32*radius;
	name_box.height = 0.12*radius;
	int val = value_bar *100;
	if (!priv->color_mode_inv)
		sprintf (str, "<span foreground=\"black\"><i>%d,%d</i></span>", (int)(value_bar),(int)(val/10%10)*10+(val%10));
	else
		sprintf (str, "<span foreground=\"white\"><i>%d,%d</i></span>", (int)(value_bar),(int)(val/10%10)*10+(val%10));
  gtk_bar_gauge_draw_name (GTK_BAR_GAUGE (bg), (gchar *)&str, &name_box); 
  cairo_rectangle(priv->cr,  name_box.x, name_box.y, name_box.width, name_box.height);
  cairo_set_source_rgba(priv->cr,0.2,0.2,0.2,0.2);
  cairo_fill_preserve(priv->cr);  
  cairo_stroke(priv->cr);
   
  cairo_set_line_width (priv->cr, 0.015 * radius);
  cairo_rectangle(priv->cr,  bar_box.x, bar_box.y, bar_box.width, bar_box.height);
  cairo_set_source_rgb(priv->cr,0.2,0.2,0.2);
  cairo_stroke(priv->cr);
  
  // **** draw gauge name
  name_box.x = bar_box.x-0.03*radius;
  name_box.y = bar_box.y+bar_box.height+0.02*radius;
  name_box.width = bar_box.width+0.5*radius;
  name_box.height = 0.26 * radius;
  gtk_bar_gauge_draw_name (GTK_BAR_GAUGE (bg), (gchar *)name, &name_box);
  return;
}

static void gtk_bar_gauge_draw_simple (GtkBarGauge * bg, gint index, gdouble x_bar, gdouble y_bar)
{
  GtkBarGaugePrivate *priv;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_draw()");
  }
  g_return_if_fail (IS_GTK_BAR_GAUGE (bg));

  priv = GTK_BAR_GAUGE_GET_PRIVATE (bg);
  
  double x, y, rec_x0, rec_y0, rec_width, rec_height, rec_degrees;
  double rec_aspect, rec_corner_radius, rec_radius, radius;
  char * name =NULL;
  double value_bar=0, bar_y=0, bar_height=0;
  int start_value_bar=0,end_value_bar=0,green_strip_start=0,yellow_strip_start=0;
  cairo_pattern_t *pat;
  GdkRectangle bar_box,name_box;
 
  x=priv->x;
  y=priv->y;
  radius=priv->radius;
    
  bar_box.x = x_bar;
  bar_box.y = y_bar;
  bar_box.width = 0.3 * radius;
  bar_box.height = 1.5 * radius;
  
  rec_x0 = bar_box.x-0.05*radius;
  rec_y0 = bar_box.y-0.05*radius;
  rec_width = bar_box.width+0.1*radius;
  rec_height = bar_box.height+0.35*radius;
  rec_aspect = 1.0;
  rec_corner_radius = rec_height / 20.0;

  rec_radius = rec_corner_radius / rec_aspect;
  rec_degrees = M_PI / 180.0;

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
        cairo_set_source_rgb(priv->cr,0.55,0.55,0.55);
    else
        cairo_set_source_rgb(priv->cr,1-0.55,1-0.55,1-0.55);
  }
  else
  {
    pat = cairo_pattern_create_radial (x - 0.392 * radius, y - 0.967 * radius, 0.167 * radius,
                                       x - 0.477 * radius, y - 0.967 * radius, 0.836 * radius);
    cairo_pattern_add_color_stop_rgba (pat, 0, 0.9,0.9,0.9, 0.8);
    cairo_pattern_add_color_stop_rgba (pat, 1, 0.45,0.45,0.45, 0.8);
    cairo_set_source (priv->cr, pat);
  }
  cairo_fill_preserve (priv->cr);
  cairo_stroke (priv->cr);
 
  switch(index)
  {
	  case 1:
		value_bar=priv->value_bar1;
		start_value_bar=priv->start_value1;
		end_value_bar=priv->end_value1;
		name=priv->name_bar1;
	
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
   
  if(!priv->color_mode_inv)
  {
	  if(value_bar>=green_strip_start)
	  {
			cairo_rectangle(priv->cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (priv->cr,0,1,0, 1);
			cairo_fill_preserve (priv->cr);
			cairo_stroke(priv->cr);
	  }
	  else  if((value_bar<green_strip_start)&&(value_bar>=yellow_strip_start))
	  {
			cairo_rectangle(priv->cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (priv->cr,1,1,0, 1);
			cairo_fill_preserve (priv->cr);
			cairo_stroke(priv->cr);
	  }
	  else  if(value_bar<=yellow_strip_start)
	  {
			cairo_rectangle(priv->cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (priv->cr,1,0,0, 1);
			cairo_fill_preserve (priv->cr);
			cairo_stroke(priv->cr);
	  }
  }
  else
  {
	  if(value_bar>=green_strip_start)
	  {
			cairo_rectangle(priv->cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (priv->cr,0.2,0.2,0.2, 1);
			cairo_fill_preserve (priv->cr);
			cairo_stroke(priv->cr);
	  }
	  else  if((value_bar<green_strip_start)&&(value_bar>=yellow_strip_start))
	  {
			cairo_rectangle(priv->cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (priv->cr,0.7,0.7,0.7, 1);
			
			cairo_fill_preserve (priv->cr);
			cairo_stroke(priv->cr);
	  }
	  else  if(value_bar<=yellow_strip_start)
	  {
			cairo_rectangle(priv->cr,bar_box.x,bar_y,bar_box.width,bar_height);
			cairo_set_source_rgba (priv->cr,1,1,1, 1);			
			cairo_fill_preserve (priv->cr);
			cairo_stroke(priv->cr);
	  }
  }
   
  cairo_set_line_width (priv->cr, 0.015 * radius);
  cairo_rectangle(priv->cr,  bar_box.x, bar_box.y, bar_box.width, bar_box.height);
  cairo_set_source_rgb(priv->cr,0.2,0.2,0.2);
  cairo_stroke(priv->cr);
  
  // **** draw gauge name
  name_box.x = bar_box.x-0.03*radius;
  name_box.y = bar_box.y+bar_box.height+0.02*radius;
  name_box.width = bar_box.width+0.06*radius;
  name_box.height = 0.26 * radius;
  gtk_bar_gauge_draw_name (GTK_BAR_GAUGE (bg), name, &name_box);
  return;
}


static void gtk_bar_gauge_draw_screws (GtkWidget * bg)
{
  GtkBarGaugePrivate *priv;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_draw()");
  }
  g_return_if_fail (IS_GTK_BAR_GAUGE (bg));

  priv = GTK_BAR_GAUGE_GET_PRIVATE (bg);
  
  cairo_pattern_t *pat;
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

static void gtk_bar_gauge_draw_name (GtkBarGauge * bg, gchar * pch_text, GdkRectangle * rect)
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

static gboolean gtk_bar_gauge_button_press_event (GtkWidget * widget, GdkEventButton * ev)
{
  GtkBarGaugePrivate *priv;
  gint x = 0, y = 0;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_button_press_event_cb()");
  }
  g_return_val_if_fail (IS_GTK_BAR_GAUGE (widget), FALSE);

  priv = GTK_BAR_GAUGE_GET_PRIVATE (widget);

  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 1))
  {
    gdk_window_get_pointer (ev->window, &x, &y, &priv->mouse_state);
    priv->mouse_pos.x = x;
    priv->mouse_pos.y = y;
    return TRUE;
  }
  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 2) && priv->b_mouse_onoff)
  {
    gtk_bar_gauge_debug = gtk_bar_gauge_debug ? FALSE : TRUE;
    return TRUE;
  }
  if ((ev->type & GDK_BUTTON_PRESS) && (ev->button == 3))
  {
    priv->b_mouse_onoff = priv->b_mouse_onoff ? FALSE : TRUE;
    return TRUE;
  }

  return FALSE;
}

static gboolean gtk_bar_gauge_motion_notify_event (GtkWidget * widget, GdkEventMotion * ev)
{
  GtkBarGaugePrivate *priv;
  GdkModifierType state;
  gint x = 0, y = 0;

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_motion_notify_event_cb()");
  }
  g_return_val_if_fail (IS_GTK_BAR_GAUGE (widget), FALSE);

  priv = GTK_BAR_GAUGE_GET_PRIVATE (widget);

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

  if (gtk_bar_gauge_debug)
  {
    g_debug ("===> gtk_bar_gauge_motion_notify_event_cb() : mouse x=%d, y=%d", x, y);
  }

  return TRUE;
}

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

  if (priv->cr)
  {
    g_free (priv->cr);
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

    case PROP_INVERSED_COLOR:
      priv->color_mode_inv = g_value_get_boolean (value);
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
