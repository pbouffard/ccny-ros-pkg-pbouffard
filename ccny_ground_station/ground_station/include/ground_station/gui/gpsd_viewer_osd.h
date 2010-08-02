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

#ifndef __GPSD_VIEWER_OSD_H__
#define __GPSD_VIEWER_OSD_H__

#include <glib-object.h>
#include <math.h>

// **** osm-gps-map headers
#include "osd-utils.h"
#include "private.h"
#include "osm-gps-map-layer.h"

G_BEGIN_DECLS

/**
 * @typedef struct GpsdViewerOsdPrivate 
 * @brief Special Gtk API strucure. Allow to add a private data<br>
 * for the widget. Defined in the C file in order to be private.
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GpsdViewerOsdPrivate GpsdViewerOsdPrivate;

/**
 * @typedef struct GpsdViewerOsdClass 
 * @brief Special Gtk API strucure. Define an instance the widget class
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GpsdViewerOsdClass
{
	GObjectClass parent_class;

} GpsdViewerOsdClass;

/**
 * @typedef struct GpsdViewerOsd
 * @brief Special Gtk API strucure. Define widget's class
 * 
 * See GObject and GTK+ references for
 * more informations: http://library.gnome.org/devel/references.html.en
 */
typedef struct _GpsdViewerOsd
{
  GObject parent;
  
  /*< private >*/
	GpsdViewerOsdPrivate *priv;

} GpsdViewerOsd;

#define GPSD_VIEWER_OSD_TYPE			(gpsd_viewer_osd_get_type ())
#define GPSD_VIEWER_OSD(obj)			(G_TYPE_CHECK_INSTANCE_CAST ((obj), GPSD_VIEWER_OSD_TYPE, GpsdViewerOsd))
#define GPSD_VIEWER_OSD_CLASS(obj)	(G_TYPE_CHECK_CLASS_CAST ((obj), GPSD_VIEWER_OSD, GpsdViewerOsdClass))
#define IS_GPSD_VIEWER_OSD(obj)		(G_TYPE_CHECK_INSTANCE_TYPE ((obj), GPSD_VIEWER_OSD_TYPE))
#define IS_GPSD_VIEWER_OSD_CLASS(obj)	(G_TYPE_CHECK_CLASS_TYPE ((obj), GPSD_VIEWER_OSD_TYPE))
#define GPSD_VIEWER_OSD_GET_CLASS	(G_TYPE_INSTANCE_GET_CLASS ((obj), GPSD_VIEWER_OSD_TYPE, GpsdViewerOsdClass))

GType gpsd_viewer_osd_get_type (void);
GpsdViewerOsd* gpsd_viewer_osd_new (void);
extern void update_uav_pose_osd(GpsdViewerOsd *, gboolean ,gint , gint);

G_END_DECLS
#endif
