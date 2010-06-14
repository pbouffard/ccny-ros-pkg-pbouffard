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
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __GTK_ALTIMETER_H__
#define __GTK_ALTIMETER_H__

#include <gtk/gtk.h>
#include <glib-object.h>
#include <math.h>
#include <time.h>

#define GTK_ALTIMETER_MAX_STRING  256   /* Size of a text string */
#define GTK_ALTIMETER_MODEL_X 300
#define GTK_ALTIMETER_MODEL_Y 300

G_BEGIN_DECLS 
typedef struct _GtkAltimeterClass
{
  GtkDrawingAreaClass parent_class;

} GtkAltimeterClass;

typedef struct _GtkAltimeter
{
  GtkDrawingArea parent;

  /* < private > */
} GtkAltimeter;

#define GTK_ALTIMETER_TYPE			(gtk_altimeter_get_type ())
#define GTK_ALTIMETER(obj)			(G_TYPE_CHECK_INSTANCE_CAST ((obj), GTK_ALTIMETER_TYPE, GtkAltimeter))
#define GTK_ALTIMETER_CLASS(obj)	(G_TYPE_CHECK_CLASS_CAST ((obj), GTK_ALTIMETER, GtkAltimeterClass))
#define IS_GTK_ALTIMETER(obj)		(G_TYPE_CHECK_INSTANCE_TYPE ((obj), GTK_ALTIMETER_TYPE))
#define IS_GTK_ALTIMETER_CLASS(obj)	(G_TYPE_CHECK_CLASS_TYPE ((obj), GTK_ALTIMETER_TYPE))
#define GTK_ALTIMETER_GET_CLASS	(G_TYPE_INSTANCE_GET_CLASS ((obj), GTK_ALTIMETER_TYPE, GtkAltimeterClass))


extern GType gtk_altimeter_get_type (void) G_GNUC_CONST;
extern GtkWidget *gtk_altimeter_new (void);
extern void gtk_altimeter_set_alti (GtkAltimeter * alt, gdouble alti);
extern void gtk_altimeter_redraw (GtkAltimeter * alt);

G_END_DECLS
#endif
