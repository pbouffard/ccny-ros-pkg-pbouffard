#ifndef __GTK_ALTIMETER_H
#define __GTK_ALTIMETER_H

#include <gtk/gtk.h>
#include <math.h>

G_BEGIN_DECLS

#define GTK_ALTIMETER_TYPE				(gtk_altimeter_get_type ())
#define GTK_ALTIMETER(obj)				(G_TYPE_CHECK_INSTANCE_CAST ((obj), GTK_ALTIMETER_TYPE, GtkAltimeter))
#define GTK_ALTIMETER_CLASS(obj)		(G_TYPE_CHECK_CLASS_CAST ((obj), GTK_ALTIMETER, GtkAltimeterClass))
#define IS_GTK_ALTIMETER(obj)			(G_TYPE_CHECK_INSTANCE_TYPE ((obj), GTK_ALTIMETER_TYPE))
#define IS_GTK_ALTIMETER_CLASS(obj)	(G_TYPE_CHECK_CLASS_TYPE ((obj), GTK_ALTIMETER_TYPE))
#define GTK_ALTIMETER_GET_CLASS		(G_TYPE_INSTANCE_GET_CLASS ((obj), GTK_ALTIMETER_TYPE, GtkAltimeterClass))

typedef struct _GtkAltimeter		GtkAltimeter;
typedef struct _GtkAltimeterClass	GtkAltimeterClass;

struct _GtkAltimeter
{
	GtkDrawingArea parent;
	/* < private > */
};

struct _GtkAltimeterClass
{
	GtkDrawingAreaClass parent_class;
};

GtkWidget *gtk_altimeter_new (void);
void gtk_altimeter_set_unit(GtkWidget *widget, gboolean unit_type);
void gtk_altimeter_set_unit_value(GtkWidget *widget, gint val);
void gtk_altimeter_set_color_mode(GtkWidget *widget, gboolean col);
void gtk_altimeter_set_alti(GtkWidget *widget, gdouble altitude);

G_END_DECLS

#endif /* __ALTIMETER_H */
