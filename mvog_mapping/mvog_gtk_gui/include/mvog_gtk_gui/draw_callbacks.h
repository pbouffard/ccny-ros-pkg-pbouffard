#ifndef MVOG_GTK_GUI_DRAW_CALLBACKS_H
#define MVOG_GTK_GUI_DRAW_CALLBACKS_H

#include <iostream>

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include <gtk/gtkgl.h>
#include <glade/glade.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include <mvog_gtk_gui/gtk_gui.h>
#include <mvog_gtk_gui/callbacks.h>

namespace MVOG
{

class GTKGui;

extern "C" G_MODULE_EXPORT 
gboolean on_drawArea_idle(void * data);


extern "C" G_MODULE_EXPORT 
void on_drawArea_realize (GtkWidget * widget, 
					 	            	GTKGui 	  * gui);


extern "C" G_MODULE_EXPORT 
gboolean on_drawArea_configure_event(GtkWidget         * widget,
                                 	   GdkEventConfigure * event,
                                 	   GTKGui 			     * gui);

extern "C" G_MODULE_EXPORT 
gboolean on_drawArea_expose_event(GtkWidget      * widget,
                             	    GdkEventExpose * event,
                                  GTKGui 			   * gui);

extern "C" G_MODULE_EXPORT 
gboolean on_drawArea_button_press_event(GtkWidget      * widget,
                                        GdkEventButton * event,
                                        GTKGui 			   * gui);

extern "C" G_MODULE_EXPORT 
gboolean on_drawArea_button_release_event(GtkWidget      * widget,
                                          GdkEventButton * event,
                                          GTKGui 	       * gui);

extern "C" G_MODULE_EXPORT 
gboolean on_drawArea_motion_notify_event(GtkWidget      * widget,
                                         GdkEventMotion * event,
                                         GTKGui 		    * gui);


extern "C" G_MODULE_EXPORT 
gboolean on_drawArea_key_press_event(GtkWidget   * widget,
                                     GdkEventKey * event,
                                     GTKGui 	   * gui);


//void set2DView(AppData* data);

}

#endif //MVOG_GTK_GUI_DRAW_CALLBACKS_H

