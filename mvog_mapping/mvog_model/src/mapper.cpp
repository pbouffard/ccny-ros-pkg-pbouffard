#include "mvog_model/mapper.h"

namespace MVOG 
{

Mapper::Mapper(double resolution, double sizeXmeters, double sizeYmeters):
        map_(resolution, sizeXmeters, sizeYmeters)
{
  modelNegativeSpace_  = true;

}

Mapper::~Mapper ()
{


}

void Mapper::setModelNegativeSpace(bool modelNegativeSpace)
{
  modelNegativeSpace_ = modelNegativeSpace;
}

bool Mapper::getModelNegativeSpace() const
{
  return modelNegativeSpace_;
}

Map * Mapper::getMap()
{
  return &map_;
}

void Mapper::addLaserData (const sensor_msgs::LaserScanConstPtr& scan, const btTransform& w2l)
{
  boost::mutex::scoped_lock(map_.mutex_);

  // position of the laser in the world coordinates
	btVector3 origin = w2l * btVector3(0.0, 0.0, 0.0);

  double scanAngle = scan->angle_min;
  for (size_t i = 0; i < scan->ranges.size(); i++)
  {
    if (scan->ranges[i] > scan->range_min && scan->ranges[i] < scan->range_max)
    {
      // valid, in range reading
		  btVector3 obstacle = w2l * btVector3(cos(scanAngle)*scan->ranges[i], sin(scanAngle)*scan->ranges[i], 0.0);
      addBeamReading(origin, obstacle);
    }
		else if (scan->ranges[i] > scan->range_max || scan->ranges[i] == 0)
		{
      // out of range reading
    }
    else
    {
      // invalid reading - too close, or error

    }
    
    // increment scan angle
    scanAngle += scan->angle_increment;
  }
}

void Mapper::addBeamReading(btVector3 origin, btVector3 obstacle)
{
  // **** convert to grid scale
  origin   /= map_.getResolution();
  obstacle /= map_.getResolution();

  addPositiveBeamSpace(obstacle);
  if (modelNegativeSpace_) addNegativeBeamSpace(origin, obstacle);
}

void Mapper::addPositiveBeamSpace(btVector3 obstacle)
{
  // **** add a positive volume

  map_.getCell(obstacle.getX(), obstacle.getY())->
    addPVolume(obstacle.getZ() - 0.5, obstacle.getZ() + 0.5);
}

void Mapper::addNegativeBeamSpace(btVector3 origin, btVector3 obstacle)
{
  // **** precalculate some variables

  btVector3 beam = obstacle - origin;

  double cx    = origin.getX();
  double cy    = origin.getY();
  double cz    = origin.getZ();
  double cd    = 0.0;

  double beamLength = beam.length();

  double slopeYX  = beam.getY() / beam.getX(); 
  double slopeZX  = beam.getZ() / beam.getX(); 
  double slopeZY  = beam.getZ() / beam.getY(); 
  double slopeXY  = beam.getX() / beam.getY(); 
  double slopeZYX = beam.getZ() / sqrt(beam.getX()*beam.getX() + beam.getY()*beam.getY());
  double slopeDX  = beamLength / beam.getX();
  double slopeDY  = beamLength / beam.getY();

  // **** rasterize

  Cell * currentCell;
  double pz;

  while(1)
  {
    pz = cz;

    // **** rasterize in the appropriate quadrant

	  if      (beam.getX() >= 0 && beam.getY() >= 0)	  // **** Q1
    {
      double dx = floor(cx) + 1.0 - cx; // distance to right cell wall
      double dy = floor(cy) + 1.0 - cy; // distance to top cell wall

      currentCell = map_.getCell(cx, cy);

      if (dy > dx * slopeYX)
      {
        cx += dx;
        cy += dx * slopeYX;
        cz += dx * slopeZX;
        cd += dx * slopeDX;
      }
      else
      {
        cx += dy * slopeXY;
        cy += dy;
        cz += dy * slopeZY;
        cd += dy * slopeDY;
      }
    }
	  else if (beam.getX() <  0 && beam.getY() >= 0)		// **** Q2
    {
      double dx = floor(cx) - cx;       // distance to left cell wall
      double dy = floor(cy) + 1.0 - cy; // distance to top cell wall

      if (dx == 0.0) dx = -1.0;

      currentCell = map_.getCell(cx + dx, cy);

      if (dy > dx * slopeYX)
      {
        cx += dx;
        cy += dx * slopeYX;
        cz += dx * slopeZX;
        cd += dx * slopeDX;
      }
      else
      {
        cx += dy * slopeXY;
        cy += dy;
        cz += dy * slopeZY;
        cd += dy * slopeDY;
      }
    }
    else if (beam.getX() <= 0 && beam.getY() <  0)		// **** Q3
    {
      double dx = floor(cx) - cx; // distance to right cell wall
      double dy = floor(cy) - cy; // distance to top cell wall

      if (dx == 0) dx = -1.0; 
      if (dy == 0) dy = -1.0;

      currentCell = map_.getCell(cx + dx, cy+dy);

      if (dy < dx * slopeYX)
      {
        cx += dx;
        cy += dx * slopeYX;
        cz += dx * slopeZX;
        cd += dx * slopeDX;
      }
      else
      {
        cx += dy * slopeXY;
        cy += dy;
        cz += dy * slopeZY;
        cd += dy * slopeDY;
      }
    }
    else                                           	// **** Q4
    {
      double dx = floor(cx) + 1.0 - cx; // distance to right cell wall
      double dy = floor(cy) - cy; // distance to top cell wall

      if (dy == 0.0) dy = -1.0;

      currentCell = map_.getCell(cx, cy + dy);

      if (dy < dx * slopeYX)
      {
        cx += dx;
        cy += dx * slopeYX;
        cz += dx * slopeZX;
        cd += dx * slopeDX;
      }
      else
      {
        cx += dy * slopeXY;
        cy += dy;
        cz += dy * slopeZY;
        cd += dy * slopeDY;
      }  
    }

    // check if we reached the end
    if (cd < beamLength)
    {
      currentCell->addNVolume(pz, cz);
    }
    else
    {  
      if(slopeZYX > 1.0)
      {
        double ez = obstacle.getZ() - 0.5;
        if (ez > pz) currentCell->addNVolume(pz, ez);
      }
      else if(slopeZYX < -1.0)
      {
        double ez = obstacle.getZ() + 0.5;
        if (ez < pz) currentCell->addNVolume(ez, pz);
      }
      break;
    }
  }    
}

} // namespace MVOG
