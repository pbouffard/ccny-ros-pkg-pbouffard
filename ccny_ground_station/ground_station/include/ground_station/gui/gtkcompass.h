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

#ifndef __GTK_COMPASS_H
#define __GTK_COMPASS_H

#include <gtk/gtk.h>
#include <glib-object.h>
#include <math.h>
#include <time.h>

#define GTK_COMPASS_MAX_STRING  256   /* Size of a text string */
#define GTK_COMPASS_MODEL_X 300
#define GTK_COMPASS_MODEL_Y 300

G_BEGIN_DECLS 
typedef struct _GtkCompassClass
{
  GtkDrawingAreaClass parent_class;

} GtkCompassClass;

typedef struct _GtkCompass
{
  GtkDrawingArea parent;

  /* < private > */
} GtkCompass;

#define GTK_COMPASS_TYPE			(gtk_compass_get_type ())
#define GTK_COMPASS(obj)			(G_TYPE_CHECK_INSTANCE_CAST ((obj), GTK_COMPASS_TYPE, GtkCompass))
#define GTK_COMPASS_CLASS(obj)	(G_TYPE_CHECK_CLASS_CAST ((obj), GTK_COMPASS, GtkCompassClass))
#define IS_GTK_COMPASS(obj)		(G_TYPE_CHECK_INSTANCE_TYPE ((obj), GTK_COMPASS_TYPE))
#define IS_GTK_COMPASS_CLASS(obj)	(G_TYPE_CHECK_CLASS_TYPE ((obj), GTK_COMPASS_TYPE))
#define GTK_COMPASS_GET_CLASS	(G_TYPE_INSTANCE_GET_CLASS ((obj), GTK_COMPASS_TYPE, GtkCompassClass))


extern GType gtk_compass_get_type (void) G_GNUC_CONST;
extern GtkWidget *gtk_compass_new (void);
extern void gtk_compass_redraw (GtkCompass * alt);

G_END_DECLS
#endif
