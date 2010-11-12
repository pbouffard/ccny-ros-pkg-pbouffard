#ifndef MVOG_MODEL_MAPPER_H
#define MVOG_MODEL_MAPPER_H

#include <boost/thread.hpp>

#include <mvog_model/map.h>

namespace MVOG 
{

class Mapper
{
  private:

    Map map_;

    // **** map building options
    
    bool modelNegativeSpace_;
    bool modelOutOfRange_;

    void addPositiveBeamSpace(btVector3 obstacle);
    void addNegativeBeamSpace(btVector3 origin, btVector3 obstacle);

  public:

    Mapper(double resolution, double sizeXmeters, double sizeYmeters);
    virtual ~Mapper();

    void addLaserData (const sensor_msgs::LaserScanConstPtr& scan, const btTransform& w2l);

    void addBeamReading(btVector3 origin, btVector3 obstacle);

    void setModelNegativeSpace(bool modelNegativeSpace);
    bool getModelNegativeSpace() const;

    Map * getMap();

};

} // namespace MVOG

#endif // MVOG_MODEL_MAPPER_H
