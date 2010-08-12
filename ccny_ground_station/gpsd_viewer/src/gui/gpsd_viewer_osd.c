/*
 * Gpsd_viewer osd for osm-gps-map
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

#include <gpsd_viewer/gui/gpsd_viewer_osd.h>

#define GPSD_VIEWER_OSD_MAX_SHADOW (4)

#define GPSD_VIEWER_OSD_SCALE_FONT_SIZE (12.0)
#define GPSD_VIEWER_OSD_SCALE_W   (10*GPSD_VIEWER_OSD_SCALE_FONT_SIZE)
#define GPSD_VIEWER_OSD_SCALE_H   (5*GPSD_VIEWER_OSD_SCALE_FONT_SIZE/2)

#define GPSD_VIEWER_OSD_SCALE_H2   (GPSD_VIEWER_OSD_SCALE_H/2)
#define GPSD_VIEWER_OSD_SCALE_TICK (2*GPSD_VIEWER_OSD_SCALE_FONT_SIZE/3)
#define GPSD_VIEWER_OSD_SCALE_M    (GPSD_VIEWER_OSD_SCALE_H2 - GPSD_VIEWER_OSD_SCALE_TICK)
#define GPSD_VIEWER_OSD_SCALE_I    (GPSD_VIEWER_OSD_SCALE_H2 + GPSD_VIEWER_OSD_SCALE_TICK)
#define GPSD_VIEWER_OSD_SCALE_FD   (GPSD_VIEWER_OSD_SCALE_FONT_SIZE/4)

#define GPSD_VIEWER_OSD_COORDINATES_FONT_SIZE (12.0)
#define GPSD_VIEWER_OSD_COORDINATES_OFFSET (GPSD_VIEWER_OSD_COORDINATES_FONT_SIZE/6)
#define GPSD_VIEWER_OSD_COORDINATES_W  (8*GPSD_VIEWER_OSD_COORDINATES_FONT_SIZE+2*GPSD_VIEWER_OSD_COORDINATES_OFFSET)
#define GPSD_VIEWER_OSD_COORDINATES_H  (2*GPSD_VIEWER_OSD_COORDINATES_FONT_SIZE+2*GPSD_VIEWER_OSD_COORDINATES_OFFSET+GPSD_VIEWER_OSD_COORDINATES_FONT_SIZE/4)

#define GPSD_VIEWER_OSD_UAV_RADIUS 8
#define GPSD_VIEWER_OSD_UAV_TICK  (GPSD_VIEWER_OSD_UAV_RADIUS/2)
#define GPSD_VIEWER_OSD_UAV_BORDER (GPSD_VIEWER_OSD_UAV_TICK + GPSD_VIEWER_OSD_UAV_RADIUS/4)
#define GPSD_VIEWER_OSD_UAV_W  ((GPSD_VIEWER_OSD_UAV_RADIUS+GPSD_VIEWER_OSD_UAV_BORDER)*2)
#define GPSD_VIEWER_OSD_UAV_H  ((GPSD_VIEWER_OSD_UAV_RADIUS+GPSD_VIEWER_OSD_UAV_BORDER)*2)

typedef struct _GpsdViewerOsdScale {
    cairo_surface_t *surface;
    int zoom;
    float lat;
} GpsdViewerOsdScale;

typedef struct _GpsdViewerOsdCoordinates {
    cairo_surface_t *surface;
    float lat, lon;
} GpsdViewerOsdCoordinates;

typedef struct _GpsdViewerOsdUAV {
    cairo_surface_t *surface;
    gint x_drone,y_drone;
    gboolean need_render;
    gboolean center_on_map;
} GpsdViewerOsdUAV;

typedef struct _GpsdViewerOsdControls {
    cairo_surface_t *surface;
    gboolean rendered;
    gint gps_enabled;
} GpsdViewerOsdControls;

/**
 * @struct GpsdViewerOsdPrivate 
 * @brief Special Gtk API strucure. Allow to add a private data<br>
 * for the widget. Defined in the C file in order to be private.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
struct _GpsdViewerOsdPrivate
{
	 GpsdViewerOsdScale *scale;
    GpsdViewerOsdCoordinates *coordinates;
    GpsdViewerOsdUAV *drone;
    GpsdViewerOsdControls *controls;
    guint gpsd_viewer_osd_w;
    guint gpsd_viewer_osd_h;
    guint gpsd_viewer_osd_shadow;
    guint gpsd_viewer_osd_pad;
    guint zoom_w;
    guint zoom_h;

    /* properties */
    gint gpsd_viewer_osd_x;
    gint gpsd_viewer_osd_y;
	 guint dpad_radius;
	 gboolean show_scale;
	 gboolean show_coordinates;
	 gboolean show_dpad;
	 gboolean show_zoom;
	 gboolean show_gps_in_dpad;
	 gboolean show_gps_in_zoom;
};

/**
 * @enum _GPSD_VIEWER_OSD_PROPERTY_ID
 * @brief Special Gtk API enum. Allow to identify widget's properties.
 *
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
enum _GPSD_VIEWER_OSD_PROPERTY_ID
{
	PROP_0,
   PROP_OSD_X,
   PROP_OSD_Y,
	PROP_DPAD_RADIUS,
	PROP_SHOW_SCALE,
	PROP_SHOW_COORDINATES,
	PROP_SHOW_DPAD,
	PROP_SHOW_ZOOM,
	PROP_SHOW_GPS_IN_DPAD,
	PROP_SHOW_GPS_IN_ZOOM
} GPSD_VIEWER_OSD_PROPERTY_ID;

static void gpsd_viewer_osd_interface_init (OsmGpsMapLayerIface *iface);

G_DEFINE_TYPE_WITH_CODE (GpsdViewerOsd, gpsd_viewer_osd, G_TYPE_OBJECT,G_IMPLEMENT_INTERFACE (OSM_TYPE_GPS_MAP_LAYER,gpsd_viewer_osd_interface_init));

static void gpsd_viewer_osd_render (OsmGpsMapLayer *osd, OsmGpsMap *map);
static void gpsd_viewer_osd_draw (OsmGpsMapLayer *osd, OsmGpsMap *map, GdkDrawable *drawable);
static gboolean gpsd_viewer_osd_busy (OsmGpsMapLayer *osd);
static gboolean gpsd_viewer_osd_button_press (OsmGpsMapLayer *osd, OsmGpsMap *map, GdkEventButton *event);

static void gpsd_viewer_osd_set_property (GObject *object, guint property_id, const GValue *value, GParamSpec *pspec);
static void gpsd_viewer_osd_get_property (GObject *object, guint property_id, GValue *value, GParamSpec *pspec);
static GObject * gpsd_viewer_osd_constructor (GType gtype, guint n_properties, GObjectConstructParam *properties);
static void gpsd_viewer_osd_finalize (GObject *object);
static void scale_render (GpsdViewerOsd *self, OsmGpsMap *map);
static void scale_draw (GpsdViewerOsd *self, GtkAllocation *allocation, cairo_t *cr);
static void coordinates_render (GpsdViewerOsd *self, OsmGpsMap *map);
static void coordinates_draw (GpsdViewerOsd *self, GtkAllocation *allocation, cairo_t *cr);
static void controls_render (GpsdViewerOsd *self, OsmGpsMap *map);
static void controls_draw (GpsdViewerOsd *self, GtkAllocation *allocation, cairo_t *cr);
static void drone_render(GpsdViewerOsd *self, OsmGpsMap *map);
static void drone_draw (GpsdViewerOsd *self, GtkAllocation *allocation, cairo_t *cr);

static void gpsd_viewer_osd_interface_init (OsmGpsMapLayerIface *iface)
{
    iface->render = gpsd_viewer_osd_render;
    iface->draw = gpsd_viewer_osd_draw;
    iface->busy = gpsd_viewer_osd_busy;
    iface->button_press = gpsd_viewer_osd_button_press;
}

/**
 * @fn static void gpsd_viewer_osd_class_init (GpsdViewerOsdClass *klass)
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
 * Also register the private struct GpsdViewerOsdPrivate with<br>
 * the class and install widget properties.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gpsd_viewer_osd_class_init (GpsdViewerOsdClass *klass)
{
	GObjectClass *object_class = G_OBJECT_CLASS (klass);
	
	object_class->get_property = gpsd_viewer_osd_get_property;
	object_class->set_property = gpsd_viewer_osd_set_property;
	object_class->constructor = gpsd_viewer_osd_constructor;
	object_class->finalize = gpsd_viewer_osd_finalize;
	
	g_type_class_add_private (object_class, sizeof (GpsdViewerOsdPrivate));
	
	g_object_class_install_property (object_class,
	                                 PROP_OSD_X,
	                                 g_param_spec_int ("osd-x",
	                                                     "osd-x",
	                                                     "osd-x",
	                                                     G_MININT,
	                                                     G_MAXINT,
	                                                     10,
	                                                     G_PARAM_READWRITE | G_PARAM_CONSTRUCT));
	g_object_class_install_property (object_class,
	                                 PROP_OSD_Y,
	                                 g_param_spec_int ("osd-y",
	                                                     "osd-y",
	                                                     "osd-y",
	                                                     G_MININT,
	                                                     G_MAXINT,
	                                                     10,
	                                                     G_PARAM_READWRITE | G_PARAM_CONSTRUCT));
	g_object_class_install_property (object_class,
	                                 PROP_DPAD_RADIUS,
	                                 g_param_spec_uint ("dpad-radius",
	                                                     "dpad-radius",
	                                                     "dpad radius",
	                                                     0,
	                                                     G_MAXUINT,
	                                                     30,
	                                                     G_PARAM_READWRITE | G_PARAM_CONSTRUCT));
	g_object_class_install_property (object_class,
	                                 PROP_SHOW_SCALE,
	                                 g_param_spec_boolean ("show-scale",
	                                                       "show-scale",
	                                                       "show scale on the map",
	                                                       TRUE,
	                                                       G_PARAM_READWRITE | G_PARAM_CONSTRUCT));
	g_object_class_install_property (object_class,
	                                 PROP_SHOW_COORDINATES,
	                                 g_param_spec_boolean ("show-coordinates",
	                                                       "show-coordinates",
	                                                       "show coordinates of map centre",
	                                                       TRUE,
	                                                       G_PARAM_READWRITE | G_PARAM_CONSTRUCT));
	g_object_class_install_property (object_class,
	                                 PROP_SHOW_DPAD,
	                                 g_param_spec_boolean ("show-dpad",
	                                                       "show-dpad",
	                                                       "show dpad for map navigation",
	                                                       FALSE,
	                                                       G_PARAM_READWRITE | G_PARAM_CONSTRUCT));
	g_object_class_install_property (object_class,
	                                 PROP_SHOW_ZOOM,
	                                 g_param_spec_boolean ("show-zoom",
	                                                       "show-zoom",
	                                                       "show zoom control for map navigation",
	                                                       FALSE,
	                                                       G_PARAM_READWRITE | G_PARAM_CONSTRUCT));
	g_object_class_install_property (object_class,
	                                 PROP_SHOW_GPS_IN_DPAD,
	                                 g_param_spec_boolean ("show-gps-in-dpad",
	                                                       "show-gps-in-dpad",
	                                                       "show gps indicator in middle of dpad",
	                                                       FALSE,
	                                                       G_PARAM_READWRITE | G_PARAM_CONSTRUCT));
	g_object_class_install_property (object_class,
	                                 PROP_SHOW_GPS_IN_ZOOM,
	                                 g_param_spec_boolean ("show-gps-in-zoom",
	                                                       "show-gps-in-zoom",
	                                                       "show gps indicator in middle of zoom control",
	                                                       FALSE,
	                                                       G_PARAM_READWRITE | G_PARAM_CONSTRUCT));
}


static GObject * gpsd_viewer_osd_constructor (GType gtype, guint n_properties, GObjectConstructParam *properties)
{
    GObject *object;
    GpsdViewerOsdPrivate *priv;

    // **** Always chain up to the parent constructor
    object = G_OBJECT_CLASS(gpsd_viewer_osd_parent_class)->constructor(gtype, n_properties, properties);
    priv = GPSD_VIEWER_OSD(object)->priv;

    // **** shadow also depends on control size 
    priv->gpsd_viewer_osd_shadow = MAX(priv->dpad_radius/8, GPSD_VIEWER_OSD_MAX_SHADOW);

    // **** distance between dpad and zoom
    priv->gpsd_viewer_osd_pad = priv->dpad_radius/4;

    // **** size of zoom pad is wrt. the dpad size
    priv->zoom_w = 2*priv->dpad_radius;
    priv->zoom_h = priv->dpad_radius;

    // **** total width and height of controls incl. shadow
    priv->gpsd_viewer_osd_w = 2*priv->dpad_radius + priv->gpsd_viewer_osd_shadow + priv->zoom_w;
    priv->gpsd_viewer_osd_h = 2*priv->dpad_radius + priv->gpsd_viewer_osd_pad + priv->zoom_h + 2*priv->gpsd_viewer_osd_shadow;

    priv->scale = g_new0(GpsdViewerOsdScale, 1);
    priv->scale->surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, GPSD_VIEWER_OSD_SCALE_W, GPSD_VIEWER_OSD_SCALE_H);
    priv->scale->zoom = -1;
    priv->scale->lat = 360.0; // **** init to an invalid lat so we get re-rendered 

    priv->coordinates = g_new0(GpsdViewerOsdCoordinates, 1);
    priv->coordinates->surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, GPSD_VIEWER_OSD_COORDINATES_W, GPSD_VIEWER_OSD_COORDINATES_H);
    priv->coordinates->lat = priv->coordinates->lon = OSM_GPS_MAP_INVALID;

    priv->drone = g_new0(GpsdViewerOsdUAV, 1);
    priv->drone->surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, GPSD_VIEWER_OSD_UAV_W, GPSD_VIEWER_OSD_UAV_H);
    priv->drone->x_drone = priv->drone->y_drone = -1;
    priv->drone->need_render = FALSE;
    priv->drone->center_on_map = FALSE;

    priv->controls = g_new0(GpsdViewerOsdControls, 1);
    // **** FIXME: SIZE DEPENDS ON IF DPAD AND ZOOM IS THERE OR NOT
    priv->controls->surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, priv->gpsd_viewer_osd_w+2, priv->gpsd_viewer_osd_h+2);
    priv->controls->rendered = FALSE;
    priv->controls->gps_enabled = -1;

    return object;
}

#define GPSD_VIEWER_OSD_STRUCT_DESTROY(_x)                                  \
    if ((_x)) {                                                 \
        if ((_x)->surface)                                      \
            cairo_surface_destroy((_x)->surface);               \
        g_free((_x));                                           \
    }

static void gpsd_viewer_osd_finalize (GObject *object)
{
    GpsdViewerOsdPrivate *priv = GPSD_VIEWER_OSD(object)->priv;

    GPSD_VIEWER_OSD_STRUCT_DESTROY(priv->scale)
    GPSD_VIEWER_OSD_STRUCT_DESTROY(priv->coordinates)
    GPSD_VIEWER_OSD_STRUCT_DESTROY(priv->drone)
    GPSD_VIEWER_OSD_STRUCT_DESTROY(priv->controls)

	G_OBJECT_CLASS (gpsd_viewer_osd_parent_class)->finalize (object);
}

/**
 * @fn static void gpsd_viewer_osd_init (GpsdViewerOsd *self)
 * @brief Special Gtk API function. Function called when the creating a<br>
 * new GtkGauge. Allow to initialize some private variables of<br>
 * widget.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gpsd_viewer_osd_init (GpsdViewerOsd *self)
{
	self->priv = G_TYPE_INSTANCE_GET_PRIVATE (self,GPSD_VIEWER_OSD_TYPE,GpsdViewerOsdPrivate);
}

/**
 * @fn static void gpsd_viewer_osd_render (OsmGpsMapLayer *osd, OsmGpsMap *map)
 * @brief Overwrite the original _render handler of the OsmGpsMap layer.
 */
static void gpsd_viewer_osd_render (OsmGpsMapLayer *osd, OsmGpsMap *map)
{
    GpsdViewerOsd *self;
    GpsdViewerOsdPrivate *priv;

    g_return_if_fail(IS_GPSD_VIEWER_OSD(osd));

    self = GPSD_VIEWER_OSD(osd);
    priv = self->priv;

    drone_render(self, map);

    if (priv->show_scale)
        scale_render(self, map);
    if (priv->show_coordinates)
        coordinates_render(self, map);
    if (priv->show_zoom || priv->show_dpad)
        controls_render(self, map);
}

/**
 * @fn static void gpsd_viewer_osd_draw (OsmGpsMapLayer *osd, OsmGpsMap *map, GdkDrawable *drawable)
 * @brief Overwrite the original _draw handler of the OsmGpsMap layer.
 */
static void gpsd_viewer_osd_draw (OsmGpsMapLayer *osd, OsmGpsMap *map, GdkDrawable *drawable)
{
    cairo_t *cr;
    GpsdViewerOsd *self;
    GpsdViewerOsdPrivate *priv;
    GtkAllocation allocation;

    g_return_if_fail(IS_GPSD_VIEWER_OSD(osd));

    self = GPSD_VIEWER_OSD(osd);
    priv = self->priv;

    gtk_widget_get_allocation(GTK_WIDGET(map), &allocation);
    cr = gdk_cairo_create(drawable);

    drone_draw(self, &allocation, cr);

    if (priv->show_scale)
        scale_draw(self, &allocation, cr);
    if (priv->show_coordinates)
        coordinates_draw(self, &allocation, cr);
    if (priv->show_zoom || priv->show_dpad)
        controls_draw(self, &allocation, cr);

    cairo_destroy(cr);
}

/**
 * @fn static gboolean gpsd_viewer_osd_busy (OsmGpsMapLayer *osd)
 * @brief Overwrite the original _busy handler of the OsmGpsMap layer.
 */
static gboolean gpsd_viewer_osd_busy (OsmGpsMapLayer *osd)
{
	return FALSE;
}

/**
 * @fn static gboolean gpsd_viewer_osd_button_press (OsmGpsMapLayer *osd, OsmGpsMap *map, GdkEventButton *event)
 * @brief Overwrite the original _press handler of the OsmGpsMap layer.
 */
static gboolean gpsd_viewer_osd_button_press (OsmGpsMapLayer *osd, OsmGpsMap *map, GdkEventButton *event)
{
    gboolean handled = FALSE;
    OsdControlPress_t but = OSD_NONE;
    GpsdViewerOsd *self;
    GpsdViewerOsdPrivate *priv;
    GtkAllocation allocation;

    g_return_val_if_fail(IS_GPSD_VIEWER_OSD(osd), FALSE);

    self = GPSD_VIEWER_OSD(osd);
    priv = self->priv;
    gtk_widget_get_allocation(GTK_WIDGET(map), &allocation);

    if ((event->button == 1) && (event->type == GDK_BUTTON_PRESS)) {
        gint mx = event->x - priv->gpsd_viewer_osd_x;
        gint my = event->y - priv->gpsd_viewer_osd_y;

        if(priv->gpsd_viewer_osd_x < 0)
            mx -= (allocation.width - priv->gpsd_viewer_osd_w);
    
        if(priv->gpsd_viewer_osd_y < 0)
            my -= (allocation.height - priv->gpsd_viewer_osd_h);

        /* first do a rough test for the OSD area. */
        /* this is just to avoid an unnecessary detailed test */
        if(mx > 0 && mx < priv->gpsd_viewer_osd_w && my > 0 && my < priv->gpsd_viewer_osd_h) {
            if (priv->show_dpad) {
                but = osd_check_dpad(mx, my, priv->dpad_radius, priv->show_gps_in_dpad);
                my -= (2*priv->dpad_radius);
                my -= priv->gpsd_viewer_osd_pad;
            }
            if (but == OSD_NONE && priv->show_zoom)
                but = osd_check_zoom(mx, my, priv->zoom_w, priv->zoom_h, 0 /*show gps*/);
        }
    }

    switch (but) 
    {
        case OSD_LEFT:
            osm_gps_map_scroll(map, -5, 0);
            handled = TRUE;
            break;
        case OSD_RIGHT:
            osm_gps_map_scroll(map, 5, 0);
            handled = TRUE;
            break;
        case OSD_UP:
            osm_gps_map_scroll(map, 0, -5);
            handled = TRUE;
            break;
        case OSD_DOWN:
            osm_gps_map_scroll(map, 0, 5);
            handled = TRUE;
            break;
        case OSD_OUT:
            osm_gps_map_zoom_out(map);
            handled = TRUE;
            break;
        case OSD_IN:
            osm_gps_map_zoom_in(map);
            handled = TRUE;
            break;
        case OSD_NONE:
        case OSD_GPS:
        default:
            handled = FALSE;
            break;
    }

    return handled;
}

/**
 * @fn GpsdViewerOsd* gpsd_viewer_osd_new (void)
 * @brief Special Gtk API function. This function is simply a wrapper<br>
 * for convienience.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
GpsdViewerOsd* gpsd_viewer_osd_new (void)
{
	return g_object_new (GPSD_VIEWER_OSD_TYPE, NULL);
}

static void scale_render(GpsdViewerOsd *self, OsmGpsMap *map)
{
    GpsdViewerOsdScale *scale = self->priv->scale;

    if(!scale->surface)
        return;

    /* this only needs to be rendered if the zoom or latitude has changed */
    gint zoom;
    gfloat lat;
    g_object_get(G_OBJECT(map), "zoom", &zoom, "latitude", &lat, NULL);
    if(zoom == scale->zoom && lat == scale->lat)
        return;

    scale->zoom = zoom;
    scale->lat = lat;

    float m_per_pix = osm_gps_map_get_scale(map);

    /* first fill with transparency */
    g_assert(scale->surface);
    cairo_t *cr = cairo_create(scale->surface);
    cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
    cairo_set_source_rgba(cr, 1.0, 0.0, 0.0, 0.0);
    // pink for testing:    cairo_set_source_rgba(cr, 1.0, 0.0, 0.0, 0.2);
    cairo_paint(cr);
    cairo_set_operator(cr, CAIRO_OPERATOR_OVER);

    /* determine the size of the scale width in meters */
    float width = (GPSD_VIEWER_OSD_SCALE_W-GPSD_VIEWER_OSD_SCALE_FONT_SIZE/6) * m_per_pix;

    /* scale this to useful values */
    int exp = logf(width)*M_LOG10E;
    int mant = width/pow(10,exp);
    int width_metric = mant * pow(10,exp);
    char *dist_str = NULL;
    if(width_metric<1000)
        dist_str = g_strdup_printf("%u m", width_metric);
    else
        dist_str = g_strdup_printf("%u km", width_metric/1000);
    width_metric /= m_per_pix;

    /* and now the hard part: scale for useful imperial values :-( */
    /* try to convert to feet, 1ft == 0.3048 m */
    width /= 0.3048;
    float imp_scale = 0.3048;
    char *dist_imp_unit = "ft";

    if(width >= 100) {
        /* 1yd == 3 feet */
        width /= 3.0;
        imp_scale *= 3.0;
        dist_imp_unit = "yd";

        if(width >= 1760.0) {
            /* 1mi == 1760 yd */
            width /= 1760.0;
            imp_scale *= 1760.0;
            dist_imp_unit = "mi";
        }
    }

    /* also convert this to full tens/hundreds */
    exp = logf(width)*M_LOG10E;
    mant = width/pow(10,exp);
    int width_imp = mant * pow(10,exp);
    char *dist_str_imp = g_strdup_printf("%u %s", width_imp, dist_imp_unit);

    /* convert back to pixels */
    width_imp *= imp_scale;
    width_imp /= m_per_pix;

    cairo_select_font_face (cr, "Sans",
                            CAIRO_FONT_SLANT_NORMAL,
                            CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size (cr, GPSD_VIEWER_OSD_SCALE_FONT_SIZE);
    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 1.0);

    cairo_text_extents_t extents;
    cairo_text_extents (cr, dist_str, &extents);

    cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
    cairo_set_line_width (cr, GPSD_VIEWER_OSD_SCALE_FONT_SIZE/6);
    cairo_move_to (cr, 2*GPSD_VIEWER_OSD_SCALE_FD, GPSD_VIEWER_OSD_SCALE_H2-GPSD_VIEWER_OSD_SCALE_FD);
    cairo_text_path (cr, dist_str);
    cairo_stroke (cr);
    cairo_move_to (cr, 2*GPSD_VIEWER_OSD_SCALE_FD,
                   GPSD_VIEWER_OSD_SCALE_H2+GPSD_VIEWER_OSD_SCALE_FD + extents.height);
    cairo_text_path (cr, dist_str_imp);
    cairo_stroke (cr);

    cairo_set_source_rgb(cr, 0.0, 0.0, 0.0);
    cairo_move_to (cr, 2*GPSD_VIEWER_OSD_SCALE_FD, GPSD_VIEWER_OSD_SCALE_H2-GPSD_VIEWER_OSD_SCALE_FD);
    cairo_show_text (cr, dist_str);
    cairo_move_to (cr, 2*GPSD_VIEWER_OSD_SCALE_FD,
                   GPSD_VIEWER_OSD_SCALE_H2+GPSD_VIEWER_OSD_SCALE_FD + extents.height);
    cairo_show_text (cr, dist_str_imp);

    g_free(dist_str);
    g_free(dist_str_imp);

    /* draw white line */
    cairo_set_line_cap  (cr, CAIRO_LINE_CAP_ROUND);
    cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 1.0);
    cairo_set_line_width (cr, GPSD_VIEWER_OSD_SCALE_FONT_SIZE/3);
    cairo_move_to (cr, GPSD_VIEWER_OSD_SCALE_FONT_SIZE/6, GPSD_VIEWER_OSD_SCALE_M);
    cairo_rel_line_to (cr, 0,  GPSD_VIEWER_OSD_SCALE_TICK);
    cairo_rel_line_to (cr, width_metric, 0);
    cairo_rel_line_to (cr, 0, -GPSD_VIEWER_OSD_SCALE_TICK);
    cairo_stroke(cr);
    cairo_move_to (cr, GPSD_VIEWER_OSD_SCALE_FONT_SIZE/6, GPSD_VIEWER_OSD_SCALE_I);
    cairo_rel_line_to (cr, 0, -GPSD_VIEWER_OSD_SCALE_TICK);
    cairo_rel_line_to (cr, width_imp, 0);
    cairo_rel_line_to (cr, 0, +GPSD_VIEWER_OSD_SCALE_TICK);
    cairo_stroke(cr);

    /* draw black line */
    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 1.0);
    cairo_set_line_width (cr, GPSD_VIEWER_OSD_SCALE_FONT_SIZE/6);
    cairo_move_to (cr, GPSD_VIEWER_OSD_SCALE_FONT_SIZE/6, GPSD_VIEWER_OSD_SCALE_M);
    cairo_rel_line_to (cr, 0,  GPSD_VIEWER_OSD_SCALE_TICK);
    cairo_rel_line_to (cr, width_metric, 0);
    cairo_rel_line_to (cr, 0, -GPSD_VIEWER_OSD_SCALE_TICK);
    cairo_stroke(cr);
    cairo_move_to (cr, GPSD_VIEWER_OSD_SCALE_FONT_SIZE/6, GPSD_VIEWER_OSD_SCALE_I);
    cairo_rel_line_to (cr, 0, -GPSD_VIEWER_OSD_SCALE_TICK);
    cairo_rel_line_to (cr, width_imp, 0);
    cairo_rel_line_to (cr, 0, +GPSD_VIEWER_OSD_SCALE_TICK);
    cairo_stroke(cr);

    cairo_destroy(cr);
}

static void scale_draw(GpsdViewerOsd *self, GtkAllocation *allocation, cairo_t *cr)
{
    GpsdViewerOsdPrivate *priv = self->priv;
    GpsdViewerOsdScale *scale = self->priv->scale;

    gint x, y;

    x =  priv->gpsd_viewer_osd_x;
    y = -priv->gpsd_viewer_osd_y;
    if(x < 0) x += allocation->width - GPSD_VIEWER_OSD_SCALE_W;
    if(y < 0) y += allocation->height - GPSD_VIEWER_OSD_SCALE_H;

    cairo_set_source_surface(cr, scale->surface, x, y);
    cairo_paint(cr);
}

/***********************************************************************
 * UAV
 **********************************************************************/
extern void update_uav_pose_osd(GpsdViewerOsd *self, gboolean center_on_map,gint x, gint y)
{
	    GpsdViewerOsdUAV *drone = self->priv->drone;
		 drone->x_drone=x;
		 drone->y_drone=y;
	    drone->need_render = TRUE;
	    drone->center_on_map = center_on_map;
}

static void drone_draw(GpsdViewerOsd *self, GtkAllocation *allocation, cairo_t *cr)
{
    GpsdViewerOsdUAV *drone = self->priv->drone;
	 gint x,y;
	 
	 if(!drone->center_on_map)
	 {
		x=drone->x_drone-GPSD_VIEWER_OSD_UAV_W/2;
		y=drone->y_drone-GPSD_VIEWER_OSD_UAV_H/2;
	 }
	 else
	 {
		x = (allocation->width - GPSD_VIEWER_OSD_UAV_W)/2;
		y = (allocation->height - GPSD_VIEWER_OSD_UAV_H)/2;
	 }

    cairo_set_source_surface(cr, drone->surface, x,y);
    cairo_paint(cr);
}

static void drone_render(GpsdViewerOsd *self, OsmGpsMap *map)
{
    GpsdViewerOsdUAV *drone = self->priv->drone;
    cairo_pattern_t *pat=NULL;
    cairo_t *cr = NULL;

    if(!drone->surface || !drone->need_render)
    {
      g_assert(drone->surface);
		cr = cairo_create(drone->surface);
		cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
		cairo_set_source_rgba(cr, 1.0, 1.0, 0.0, 0.0);
		cairo_paint(cr);
		cairo_set_operator(cr, CAIRO_OPERATOR_OVER);
		drone->need_render = FALSE;
    }
    else
    {
		// **** fill with transparency
		g_assert(drone->surface);
		cr = cairo_create(drone->surface);
		cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
		cairo_set_source_rgba(cr, 1.0, 1.0, 0.0, 0.0);
		cairo_paint(cr);
		cairo_set_operator(cr, CAIRO_OPERATOR_OVER);
    
		cairo_set_line_width (cr, 1);
		cairo_arc (cr, GPSD_VIEWER_OSD_UAV_W/2, GPSD_VIEWER_OSD_COORDINATES_H/2, GPSD_VIEWER_OSD_UAV_RADIUS, 0, 2*M_PI);
		pat = cairo_pattern_create_radial (GPSD_VIEWER_OSD_UAV_W/2-1, GPSD_VIEWER_OSD_COORDINATES_H/2-1, GPSD_VIEWER_OSD_UAV_RADIUS*0.2,
														GPSD_VIEWER_OSD_UAV_W/2, GPSD_VIEWER_OSD_COORDINATES_H/2, GPSD_VIEWER_OSD_UAV_RADIUS);
		cairo_pattern_add_color_stop_rgba (pat,0, 0.8, 0.8, 1.0,0.8);
		cairo_pattern_add_color_stop_rgba (pat,1, 0.0, 0.0, 0.65,0.8);
		cairo_set_source (cr, pat);
		cairo_fill_preserve(cr);
		cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 1.0);
		cairo_stroke (cr);
		
		cairo_pattern_destroy (pat);    
		drone->need_render = FALSE;
	}
	cairo_destroy(cr);
}

static void coordinates_render(GpsdViewerOsd *self, OsmGpsMap *map)
{
    GpsdViewerOsdCoordinates *coordinates = self->priv->coordinates;

    if(!coordinates->surface)
        return;

    // **** get current map position 
    gfloat lat, lon;
    g_object_get(G_OBJECT(map), "latitude", &lat, "longitude", &lon, NULL);

    // **** check if position has changed enough to require redraw
    if(!isnan(coordinates->lat) && !isnan(coordinates->lon))
        // **** 1/60000 == 1/1000 minute 
        if((fabsf(lat - coordinates->lat) < 1/60000) &&
           (fabsf(lon - coordinates->lon) < 1/60000))
            return;

    coordinates->lat = lat;
    coordinates->lon = lon;

    // **** first fill with transparency

    g_assert(coordinates->surface);
    cairo_t *cr = cairo_create(coordinates->surface);
    cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
        //cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.5);
    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 0.0);
    cairo_paint(cr);
    cairo_set_operator(cr, CAIRO_OPERATOR_OVER);

    cairo_select_font_face (cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size (cr, GPSD_VIEWER_OSD_COORDINATES_FONT_SIZE);

    char *latitude = osd_latitude_str(lat);
    char *longitude = osd_longitude_str(lon);
    
    int y = GPSD_VIEWER_OSD_COORDINATES_OFFSET;
    y = osd_render_centered_text(cr, y, GPSD_VIEWER_OSD_COORDINATES_W, GPSD_VIEWER_OSD_COORDINATES_FONT_SIZE, latitude);
    y = osd_render_centered_text(cr, y, GPSD_VIEWER_OSD_COORDINATES_W, GPSD_VIEWER_OSD_COORDINATES_FONT_SIZE, longitude);
    
    g_free(latitude);
    g_free(longitude);

    cairo_destroy(cr);
}

static void coordinates_draw(GpsdViewerOsd *self, GtkAllocation *allocation, cairo_t *cr)
{
    GpsdViewerOsdPrivate *priv = self->priv;
    GpsdViewerOsdCoordinates *coordinates = self->priv->coordinates;
    gint x,y;

    x = -priv->gpsd_viewer_osd_x;
    y = -priv->gpsd_viewer_osd_y;
    if(x < 0) x += allocation->width - GPSD_VIEWER_OSD_COORDINATES_W;
    if(y < 0) y += allocation->height - GPSD_VIEWER_OSD_COORDINATES_H;

    cairo_set_source_surface(cr, coordinates->surface, x, y);
    cairo_paint(cr);
}

static void controls_render(GpsdViewerOsd *self, OsmGpsMap *map)
{
    GpsdViewerOsdPrivate *priv = self->priv;
    GpsdViewerOsdControls *controls = self->priv->controls;

    if(!controls->surface || controls->rendered)
        return;

    controls->rendered = TRUE;

    GtkStyle *style = gtk_widget_get_style(GTK_WIDGET(map));
    GdkColor bg = style->bg[GTK_STATE_NORMAL];
    GdkColor fg = style->fg[GTK_STATE_NORMAL];
    //GdkColor da = GTK_WIDGET(map)->style->fg[GTK_STATE_INSENSITIVE];

    // **** first fill with transparency 
    g_assert(controls->surface);
    cairo_t *cr = cairo_create(controls->surface);
    cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
    cairo_set_source_rgba(cr, 1.0, 0.0, 0.0, 0.0);
    cairo_paint(cr);
    cairo_set_operator(cr, CAIRO_OPERATOR_OVER);

    gint x = 1;
    gint y = 1;

    /* --------- draw dpad ----------- */
    if (priv->show_dpad) {
        gint gps_w = (priv->show_gps_in_dpad ? priv->dpad_radius/2 : 0);
        osd_render_dpad(cr, x, y, priv->dpad_radius, gps_w, priv->gpsd_viewer_osd_shadow, &bg, &fg);
        if (priv->show_gps_in_dpad) {
            gint gps_x = x+priv->dpad_radius-(gps_w/2);
            gint gps_y = y+priv->dpad_radius-(gps_w/2);
            osd_render_gps(cr, gps_x, gps_y, gps_w, &bg, &fg);
        }
        y += (2*priv->dpad_radius);
        y += priv->gpsd_viewer_osd_pad;
    }

    /* --------- draw zoom ----------- */
    if (priv->show_zoom) {
        gint gps_w = (priv->show_gps_in_zoom ? priv->dpad_radius/2 : 0);
        osd_render_zoom(cr, x, y, priv->zoom_w, priv->zoom_h, gps_w, priv->gpsd_viewer_osd_shadow, &bg, &fg);
        if (priv->show_gps_in_zoom) {
            gint gps_x = x+(priv->zoom_w/2);
            gint gps_y = y+(priv->zoom_h/2)-(gps_w/2);
            osd_render_gps(cr, gps_x, gps_y, gps_w, &bg, &fg);
        }
        y += priv->zoom_h;
    }

}

static void controls_draw(GpsdViewerOsd *self, GtkAllocation *allocation, cairo_t *cr)
{
    GpsdViewerOsdPrivate *priv = self->priv;
    GpsdViewerOsdControls *controls = self->priv->controls;

    gint x,y;

    x = priv->gpsd_viewer_osd_x;
    if(x < 0)
        x += allocation->width - priv->gpsd_viewer_osd_w;

    y = priv->gpsd_viewer_osd_y;
    if(y < 0)
        y += allocation->height - priv->gpsd_viewer_osd_h;

    cairo_set_source_surface(cr, controls->surface, x, y);
    cairo_paint(cr);
}

/**
 * @fn static void gpsd_viewer_osd_get_property (GObject *object, guint property_id, GValue *value, GParamSpec *pspec)
 * @brief Special Gtk API function. Override the _set_property handler <br>
 * in order to set the object parameters.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gpsd_viewer_osd_get_property (GObject *object, guint property_id, GValue *value, GParamSpec *pspec)
{
	switch (property_id) 
	{
	case PROP_OSD_X:
		g_value_set_int (value, GPSD_VIEWER_OSD (object)->priv->gpsd_viewer_osd_x);
		break;
	case PROP_OSD_Y:
		g_value_set_int (value, GPSD_VIEWER_OSD (object)->priv->gpsd_viewer_osd_y);
		break;
	case PROP_DPAD_RADIUS:
		g_value_set_uint (value, GPSD_VIEWER_OSD (object)->priv->dpad_radius);
		break;
	case PROP_SHOW_SCALE:
		g_value_set_boolean (value, GPSD_VIEWER_OSD (object)->priv->show_scale);
		break;
	case PROP_SHOW_COORDINATES:
		g_value_set_boolean (value, GPSD_VIEWER_OSD (object)->priv->show_coordinates);
		break;
	case PROP_SHOW_DPAD:
		g_value_set_boolean (value, GPSD_VIEWER_OSD (object)->priv->show_dpad);
		break;
	case PROP_SHOW_ZOOM:
		g_value_set_boolean (value, GPSD_VIEWER_OSD (object)->priv->show_zoom);
		break;
	case PROP_SHOW_GPS_IN_DPAD:
		g_value_set_boolean (value, GPSD_VIEWER_OSD (object)->priv->show_gps_in_dpad);
		break;
	case PROP_SHOW_GPS_IN_ZOOM:
		g_value_set_boolean (value, GPSD_VIEWER_OSD (object)->priv->show_gps_in_zoom);
		break;
	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
	}
}

/**
 * @fn static void gpsd_viewer_osd_set_property (GObject *object, guint property_id, const GValue *value, GParamSpec *pspec)
 * @brief Special Gtk API function. Override the _set_property handler <br>
 * in order to set the object parameters.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
static void gpsd_viewer_osd_set_property (GObject *object, guint property_id, const GValue *value, GParamSpec *pspec)
{
	switch (property_id) 
	{
		case PROP_OSD_X:
		GPSD_VIEWER_OSD (object)->priv->gpsd_viewer_osd_x = g_value_get_int (value);
		break;
	case PROP_OSD_Y:
		GPSD_VIEWER_OSD (object)->priv->gpsd_viewer_osd_y = g_value_get_int (value);
		break;
	case PROP_DPAD_RADIUS:
		GPSD_VIEWER_OSD (object)->priv->dpad_radius = g_value_get_uint (value);
		break;
	case PROP_SHOW_SCALE:
		GPSD_VIEWER_OSD (object)->priv->show_scale = g_value_get_boolean (value);
		break;
	case PROP_SHOW_COORDINATES:
		GPSD_VIEWER_OSD (object)->priv->show_coordinates = g_value_get_boolean (value);
		break;
	case PROP_SHOW_DPAD:
		GPSD_VIEWER_OSD (object)->priv->show_dpad = g_value_get_boolean (value);
		break;
	case PROP_SHOW_ZOOM:
		GPSD_VIEWER_OSD (object)->priv->show_zoom = g_value_get_boolean (value);
		break;
	case PROP_SHOW_GPS_IN_DPAD:
		GPSD_VIEWER_OSD (object)->priv->show_gps_in_dpad = g_value_get_boolean (value);
		break;
	case PROP_SHOW_GPS_IN_ZOOM:
		GPSD_VIEWER_OSD (object)->priv->show_gps_in_zoom = g_value_get_boolean (value);
		break;
	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
	}
}
