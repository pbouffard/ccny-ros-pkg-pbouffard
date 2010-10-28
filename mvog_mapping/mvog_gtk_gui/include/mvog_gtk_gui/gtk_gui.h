#ifndef MVOG_GTK_GUI_GTK_GUI_H
#define MVOG_GTK_GUI_GTK_GUI_H

#include <string>
#include <ros/ros.h>
#include <ros/package.h>

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <glade/glade.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include <mvog_gtk_gui/map_drawer_2d.h>
#include <mvog_gtk_gui/map_drawer_3d.h>

#include <mvog_gtk_gui/draw_callbacks.h>

namespace MVOG 
{

class MapDrawer3D;

const double PAN_SPEED  = 0.01;
const double TILT_SPEED = 0.01;
const double ZOOM_SPEED = 0.01;
const double MOVE_SPEED = 0.825;

class GTKGui
{
  private:

    struct Controls 
    {
      GtkWidget * winMain; 
	    GtkWidget * btn1;
	    GtkWidget * drawArea;

	    GtkNotebook * ntbkViewOptions;

	    GtkEntry * txtCamPosX;
	    GtkEntry * txtCamPosY;
	    GtkEntry * txtCamPosZ;

	    GtkEntry * txtCamLookX;
	    GtkEntry * txtCamLookY;
	    GtkEntry * txtCamLookZ;

	    GtkSpinButton * txtZPlane;
	    GtkSpinButton * txtVerticalScale;
	    GtkSpinButton * txtVerticalOffset;
	    GtkSpinButton * txtObstacleCutoff;

	    GtkEntry * txtVisCenterX;
	    GtkEntry * txtVisCenterY;
	    GtkEntry * txtVisSize;

	    GtkToggleButton * btnDrawPVolumes;
	    GtkToggleButton * btnDrawNVolumes;
	    GtkToggleButton * btnDrawIVolumes;
	    GtkToggleButton * btnDrawCloud;
	    GtkToggleButton * btnDrawZPlane;
	    GtkToggleButton * btnCutOffZPlane;

	    GtkToggleButton * btnHighlightUnknown;
    };

    struct Options
    {
      bool draw3D;

      bool drawPVolumes;
      bool drawNVolumes;    

    };

    Controls controls_;
    Options options_;

    // **** mouse events

    bool mouseLeftIsDown_;
    bool mouseMidIsDown_;
    bool mouseRightIsDown_;
    double mouseX_;
    double mouseY_;

    // **** draw window size

    double canvasWidth_;
    double canvasHeight_;

    // **** OpenGL drawers

    MapDrawer2D drawer2D_;
    MapDrawer3D * drawer3D_;

    void setUpGTK();

  public:

    GTKGui();
    virtual ~GTKGui();

    Controls * getControls() { return &controls_; }

    void setMap(MVOG::Map * map);

    void start();

    void setDraw3D(bool draw3D) { options_.draw3D = draw3D; }

    void setDrawPVolumes(bool drawPVolumes);
    void setDrawNVolumes(bool drawNVolumes);

    bool getDrawPVolumes() const;
    bool getDrawNVolumes() const;

    void setCanvasWidth (double canvasWidth ) { canvasWidth_  = canvasWidth;  }
    void setCanvasHeight(double canvasHeight) { canvasHeight_ = canvasHeight; }

    double getCanvasWidth()  const { return canvasWidth_;  }
    double getCanvasHeight() const { return canvasHeight_; }

    void setView();
    void updateControls();

    void mouseDown(double x, double y, int button);
    void mouseUp  (double x, double y, int button);
    void mouseMove(double x, double y);

    void draw();
};

void* startGTK(void *);

} // namespace MVOG

#endif //MVOG_GTK_GUI_GTK_GUI_H

