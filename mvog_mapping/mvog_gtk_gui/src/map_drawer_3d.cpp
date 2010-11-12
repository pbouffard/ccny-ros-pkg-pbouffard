#include <mvog_gtk_gui/map_drawer_3d.h>

namespace MVOG
{

MapDrawer3D::MapDrawer3D(GTKGui * gui)
{
  gui_ = gui;
}

MapDrawer3D::~MapDrawer3D()
{
	
}

void MapDrawer3D::draw()
{  
  setView();

  drawGrid();
  drawAxes();

  if (gui_->getDrawRawData())
  {
    if (gui_->getDrawPVolumes()) drawPVolumes();
    if (gui_->getDrawNVolumes()) drawNVolumes();
  }
  else
  {
    drawMLolumes();
  }
}

void MapDrawer3D::drawMLolumes()
{

  for (int i = 0; i < map_->sizeX_; ++i)
  for (int j = 0; j < map_->sizeY_; ++j)
  {
    MLVolumeVector vect;
    map_->grid_[i][j].createMLVolumes(vect);

    double x = i - map_->offsetX_;
    double y = j - map_->offsetY_;

		glPushMatrix();

    glScalef(map_->resolution_, map_->resolution_, map_->resolution_);
		glTranslatef(x, y, 0.0);

    for (size_t c = 0; c < vect.size(); ++c)
    {
      drawSolidColorVolume(vect[c].top, vect[c].bot, COLOR_ML_VOLUMES);  
    }

    glPopMatrix();
  }

}

void MapDrawer3D::drawGrid()
{
  glColor3dv(GRID_COLOR);

  glPushMatrix();
  glScaled(map_->resolution_, map_->resolution_, 1.0);
  glTranslated(-map_->offsetX_, -map_->offsetY_, 0.0);

  for (int i = 0; i <= map_->sizeX_; ++i)
  {
    glBegin(GL_LINES);
      glVertex3d(i, 0           , 0.0);
      glVertex3d(i, map_->sizeY_, 0.0);
    glEnd();
  }

  for (int j = 0; j <= map_->sizeY_; ++j)
  {
    glBegin(GL_LINES);
      glVertex3d(0,            j, 0.0);
      glVertex3d(map_->sizeX_, j, 0.0);
    glEnd();
  }

  glPopMatrix();
}

void MapDrawer3D::drawAxes()
{
  glLineWidth(2.0);

  glColor3d(1.0, 0.0, 0.0);

  glBegin(GL_LINES);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(1.0, 0.0, 0.0);
  glEnd();

  glColor3d(0.0, 1.0, 0.0);
  glBegin(GL_LINES);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(0.0, 1.0, 0.0);
  glEnd();

  glColor3d(0.0, 0.0, 1.0);
  glBegin(GL_LINES);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(0.0, 0.0, 1.0);
  glEnd();

  glLineWidth(1.0);
}

void MapDrawer3D::drawPVolumes()
{
  for (int i = 0; i < map_->sizeX_; ++i)
  for (int j = 0; j < map_->sizeY_; ++j)
  {
    Volume * volumes = map_->grid_[i][j].getPVolumes();
    int count = map_->grid_[i][j].getPVolumesCount();

    for (int v = 0; v < count; ++v)
    {
      double x = i - map_->offsetX_;
      double y = j - map_->offsetY_;

  		glPushMatrix();

      glScalef(map_->resolution_, map_->resolution_, map_->resolution_);
			glTranslatef(x, y, 0.0);


      //if (useHeightColor_) 
      //  drawHeightColorVolume(volume->getBot(), volume->getTop());
      //else
        drawSolidColorVolume(getBot(volumes[v]), getTop(volumes[v]), COLOR_P_VOLUMES);

			glPopMatrix();
    } 
  }
}

void MapDrawer3D::drawNVolumes()
{
  for (int i = 0; i < map_->sizeX_; ++i)
  for (int j = 0; j < map_->sizeY_; ++j)
  {
    Volume * volumes = map_->grid_[i][j].getNVolumes();
    int count = map_->grid_[i][j].getNVolumesCount();

    for (int v = 0; v < count; ++v)
    {
      double x = i - map_->offsetX_;
      double y = j - map_->offsetY_;

  		glPushMatrix();

      glScalef(map_->resolution_, map_->resolution_, map_->resolution_);
			glTranslatef(x, y, 0.0);


      //if (useHeightColor_) 
      //  drawHeightColorVolume(volume->getBot(), volume->getTop());
      //else
        drawSolidColorVolume(getBot(volumes[v]), getTop(volumes[v]), COLOR_N_VOLUMES);

			glPopMatrix();
    } 
  }
}

void MapDrawer3D::drawSolidColorVolume(double bottom, double top, const double color[3])
{
	glEnable(GL_POLYGON_OFFSET_FILL); // Avoid Stitching!
	glPolygonOffset(1.0, 1.0);

	double eps  =  0.5;
	double eps1 =  0.0;
	double eps2 =  1.0;

	// top and bottom

	glColor3dv(color);

	glBegin(GL_POLYGON);
		glVertex3f(eps1, eps1, bottom);
		glVertex3f(eps1, eps2, bottom);
		glVertex3f(eps2, eps2, bottom);
		glVertex3f(eps2, eps1, bottom);
	glEnd();

	glBegin(GL_POLYGON);
		glVertex3f(eps1, eps1, top);
		glVertex3f(eps1, eps2, top);
		glVertex3f(eps2, eps2, top);
		glVertex3f(eps2, eps1, top);
	glEnd();

	// left and right

	glColor3dv(color);

	glBegin(GL_POLYGON);
		glVertex3f(eps1, eps1, bottom);
		glVertex3f(eps1, eps2, bottom);
		glVertex3f(eps1, eps2, top);
		glVertex3f(eps1, eps1, top);
	glEnd();

	glBegin(GL_POLYGON);
		glVertex3f(eps2, eps1, bottom);
		glVertex3f(eps2, eps2, bottom);
		glVertex3f(eps2, eps2, top);
		glVertex3f(eps2, eps1, top);
	glEnd();

	// front and back

	glBegin(GL_POLYGON);
		glVertex3f(eps1, eps1, bottom);
		glVertex3f(eps2, eps1, bottom);
		glVertex3f(eps2, eps1, top);
		glVertex3f(eps1, eps1, top);
	glEnd();

	glBegin(GL_POLYGON);
		glVertex3f(eps1, eps2, bottom);
		glVertex3f(eps2, eps2, bottom);
		glVertex3f(eps2, eps2, top);
		glVertex3f(eps1, eps2, top);
	glEnd();

	glDisable(GL_POLYGON_OFFSET_FILL);

	// ******* LINES

	glColor3f(0.0, 0.0, 0.0);

	glBegin(GL_LINE_LOOP);
		glVertex3f(eps1, eps1, bottom);
		glVertex3f(eps1, eps2, bottom);
		glVertex3f(eps2, eps2, bottom);
		glVertex3f(eps2, eps1, bottom);
	glEnd();

	glBegin(GL_LINE_LOOP);
		glVertex3f(eps1, eps1, top);
		glVertex3f(eps1, eps2, top);
		glVertex3f(eps2, eps2, top);
		glVertex3f(eps2, eps1, top);
	glEnd();

	glBegin(GL_LINES);
		glVertex3f(eps1, eps1, bottom);
		glVertex3f(eps1, eps1, top);
		glVertex3f(eps1, eps2, bottom);
		glVertex3f(eps1, eps2, top);
		glVertex3f(eps2, eps2, bottom);
		glVertex3f(eps2, eps2, top);
		glVertex3f(eps2, eps1, bottom);
		glVertex3f(eps2, eps1, top);
	glEnd();

}

void MapDrawer3D::drawHeightColorVolume(double bottom, double top)
{

}

void MapDrawer3D::setView()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();    

  glViewport(0,0, gui_->getCanvasWidth(), gui_->getCanvasHeight());

	gluPerspective(45.0, gui_->getCanvasWidth()/gui_->getCanvasHeight(), 0.1, 100.0);

	// display camera view
	gluLookAt(camera_.getPosX(),  camera_.getPosY(),  camera_.getPosZ(),
				    camera_.getLookX(), camera_.getLookY(), camera_.getLookZ(),
				    0.0,               0.0,               1.0);
}

void MapDrawer3D::pan(double angle)
{
  camera_.pan(angle);
  setView();
}

void MapDrawer3D::tilt(double angle)
{
  camera_.tilt(angle);
  setView();
}

void MapDrawer3D::zoom(double r)
{
  camera_.zoom(r);
  setView();
}

void MapDrawer3D::move(double x, double y)
{
  camera_.move(x, y);
  setView();
}

} // namespace MVOG
