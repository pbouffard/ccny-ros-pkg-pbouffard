#include <mvog_gtk_gui/draw_callbacks.h>


double mx;
double my;
bool flatView = false;
bool mouseLeftIsDown = false;
bool mouseRightIsDown = false;

namespace MVOG
{

extern "C" G_MODULE_EXPORT 
gboolean on_drawArea_idle(void * data)
{
	GTKGui * gui = static_cast<GTKGui*>(data);

  if (!GTK_IS_WIDGET(gui->getControls()->drawArea)) return TRUE;

  gtk_widget_draw(gui->getControls()->drawArea, NULL);

  return TRUE;
}


extern "C" G_MODULE_EXPORT 
void on_drawArea_realize (GtkWidget *widget, 
					 	              GTKGui 	*gui)
{
	GdkGLContext  *glcontext  = gtk_widget_get_gl_context (widget);
	GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable (widget);

	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext)) return;

	// choose background color
	glClearColor(1.0, 1.0, 1.0, 1.0);	// background color
   
	// shading type
	glShadeModel(GL_SMOOTH);

	// hidden surface
	glEnable(GL_DEPTH_TEST);

	gdk_gl_drawable_gl_end (gldrawable);
}

extern "C" G_MODULE_EXPORT 
gboolean on_drawArea_configure_event (GtkWidget         *widget,
                                 	    GdkEventConfigure *event,
                                 	    GTKGui 			*gui)
{
  gui->setCanvasWidth (event->width);
  gui->setCanvasHeight(event->height);
	gui->setView();
	return FALSE;
}

extern "C" G_MODULE_EXPORT 
gboolean on_drawArea_expose_event (GtkWidget       *widget,
                             	     GdkEventExpose  *event,
                                   GTKGui 	 *gui)
{
	GdkGLContext  *glcontext  = gtk_widget_get_gl_context  (widget);
	GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable (widget);

	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext))
	{
    printf ("error in gdk_gl_drawable_gl_begin\n");
		return FALSE;
	}

	// clear the screen
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	

	// draw map

	gui->draw();

	if (gdk_gl_drawable_is_double_buffered (gldrawable))
		gdk_gl_drawable_swap_buffers (gldrawable);
	else
		glFlush ();

	gdk_gl_drawable_gl_end (gldrawable);

	return TRUE;
}


extern "C" G_MODULE_EXPORT 
gboolean on_drawArea_button_press_event(GtkWidget      * widget,
                                        GdkEventButton * event,
                                        GTKGui  	     * gui) 
{
  gui->mouseDown(event->x, event->y, event->button);

	return TRUE;
}

extern "C" G_MODULE_EXPORT 
gboolean on_drawArea_button_release_event(GtkWidget      * widget,
                                          GdkEventButton * event,
                                          GTKGui  	     * gui) 
{
  gui->mouseUp(event->x, event->y, event->button);

	return TRUE;
}

extern "C" G_MODULE_EXPORT 
gboolean on_drawArea_motion_notify_event(GtkWidget      * widget,
                                         GdkEventMotion * event,
                                         GTKGui  	      * gui)
{
  gui->mouseMove(event->x, event->y);

	return TRUE;
}



extern "C" G_MODULE_EXPORT 
gboolean on_drawArea_key_press_event(GtkWidget   * widget,
                                     GdkEventKey * event,
                                     GTKGui 	   * gui)
{
  //if (event->keyval == GDK_a)

	return TRUE;
}
/*
void set2DView(AppData* data)
{
	glMatrixMode(GL_PROJECTION);		// set clipping window
	glLoadIdentity();

	double visCenterX = data->map->getVisCenterX();
	double visCenterY = data->map->getVisCenterY();
	double visSize    = data->map->getVisSize();

	gluOrtho2D(visCenterX - (visSize/2.0), 
			   visCenterX + (visSize/2.0),
	           visCenterY - visSize/2.0, 
			   visCenterY + visSize/2.0);
}
*/

} // namespace MVOG
