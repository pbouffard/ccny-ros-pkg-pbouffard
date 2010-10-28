#ifndef MVOG_MODEL_MAP_H
#define MVOG_MODEL_MAP_H

#include <mvog_model/cell.h>
#include <sensor_msgs/LaserScan.h>
#include <btBulletDynamicsCommon.h>

namespace MVOG 
{

class Map
{
  friend class MapDrawer2D;
  friend class MapDrawer3D;

  private:

    // **** map size and resolution variables

    double resolution_;
    int sizeX_;
    int sizeY_;
    int offsetX_;
    int offsetY_;

    Cell** grid_;

    Cell** initializeGrid(int sizeX, int sizeY);
    void deleteGrid();

  public:

    Map(double resolution, double sizeXmeters, double sizeYmeters);
    virtual ~Map();

    Cell* getCell(double x, double y);
    double getResolution() const { return resolution_; }

    // test

    void addVolume(int x, int y, double top, double bottom);
    void printVolumes(int x, int y);

};

}; // namespace MVOG


#endif // MVOG_MODEL_MAP_H
