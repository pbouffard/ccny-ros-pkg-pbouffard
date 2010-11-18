#ifndef MVOG_GTK_GUI_MAP_DRAWER_3D_H
#define MVOG_GTK_GUI_MAP_DRAWER_3D_H

#include <GL/glut.h>

#include <mvog_model/map.h>

#include <mvog_gtk_gui/camera.h>
#include <mvog_gtk_gui/gtk_gui.h>

namespace MVOG
{

class GTKGui;

const double COLOR_P_VOLUMES[3]  = {0.75, 0.00, 0.00};
const double COLOR_N_VOLUMES[3]  = {0.00, 0.00, 0.75};
const double COLOR_ML_VOLUMES[3] = {0.00, 0.75, 0.00};
const double GRID_COLOR[3]      = {0.70, 0.70, 0.70};

class MapDrawer3D
{
	private:

    Map    * map_;
    GTKGui * gui_;

    Camera camera_;
    
    void drawMLolumes();
		void drawPVolumes();
		void drawNVolumes();

    void drawAxes();
    void drawGrid();

    void drawSolidColorVolume(double bottom, double top, const double color[3]);
    void drawHeightColorVolume(double bottom, double top);

	public:

		MapDrawer3D(GTKGui * gui);
		~MapDrawer3D();

    void draw();
    
    double getCameraPosX() const { return camera_.getPosX(); }
    double getCameraPosY() const { return camera_.getPosY(); }    
    double getCameraPosZ() const { return camera_.getPosZ(); }   

    double getCameraLookX() const { return camera_.getLookX(); }
    double getCameraLookY() const { return camera_.getLookY(); }    
    double getCameraLookZ() const { return camera_.getLookZ(); }     

    void pan(double angle);
    void tilt(double angle);
    void zoom(double r);
    void move(double x, double y);

    void setView();
    void setMap(MVOG::Map * map) { map_ = map; }
    Map * getMap() {return map_;}
};

} // namespace MVOG

#endif // MVOG_GTK_GUI_MAP_DRAWER_H
