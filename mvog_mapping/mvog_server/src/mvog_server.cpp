#include "mvog_server/mvog_server.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "mvog_server");
  MVOGServer mvogServer;

  while(ros::ok())
  {
    mvogServer.spinOnce();
  }

  return 0;
}

MVOGServer::MVOGServer ()
{
  ROS_INFO ("Starting MVOGServer");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private ("~");

  // **** get parameters

  double mapResolution;
  double initMapSizeX;
  double initMapSizeY;
  bool   modelNegativeSpace;
  double tfTolerance;

  if (!nh_private.getParam ("map_resolution", mapResolution))
    mapResolution = 0.10;
  if (!nh_private.getParam ("init_map_size_x", initMapSizeX))
    initMapSizeX = 10;
  if (!nh_private.getParam ("init_map_size_y", initMapSizeY))
    initMapSizeY = 10;
  if (!nh_private.getParam ("model_negative_space", modelNegativeSpace))
    modelNegativeSpace = true;
  if (!nh_private.getParam ("tf_tolerance", tfTolerance))
    tfTolerance = 0.01;
  if (!nh_private.getParam ("world_frame", worldFrame_))
    worldFrame_ = "map";

  // **** create mapper

  mapper_ = new MVOG::Mapper(mapResolution, initMapSizeX, initMapSizeY);
  mapper_->setModelNegativeSpace(modelNegativeSpace);

  // **** create gui

  gui_ = new MVOG::GTKGui();
  gui_->setMap(mapper_->getMap());

  // **** testing

/*
  mapper_->addBeamReading(btVector3( 0.05,  0.05, 0.0),
                          btVector3(-15,  1, 2.0));
*/

  mapper_->getMap()->test();

  // **** subscribe to laser scan messages
  scanFilterSub_ = new message_filters::Subscriber < sensor_msgs::LaserScan > (nh, scanTopic_, 10);
  scanFilter_ = new tf::MessageFilter < sensor_msgs::LaserScan > (*scanFilterSub_, tfListener_, worldFrame_, 10);
  scanFilter_->registerCallback (boost::bind (&MVOGServer::scanCallback, this, _1));
  scanFilter_->setTolerance (ros::Duration (tfTolerance));
}

MVOGServer::~MVOGServer ()
{
  printf("Final Size: %f\n", mapper_->getMap()->getMemorySize());
  ROS_INFO ("Destroying MVOGServer");
}

void MVOGServer::spinOnce()
{
  gui_->spinOnce();
  ros::spinOnce();
}

void MVOGServer::scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  // **** obtain transform between world and laser frame
  tf::StampedTransform worldToLaser;
  try
  {
    tfListener_.lookupTransform (worldFrame_, scan->header.frame_id, scan->header.stamp, worldToLaser);
  }
  catch (tf::TransformException ex)
  {
    // transform unavailable - skip scan
    ROS_WARN ("Skipping scan (%s)", ex.what ());
    return;
  }

  timeval start_insert, stop_insert; 
  gettimeofday(&start_insert, NULL);

  mapper_->addLaserData(scan, worldToLaser);

  gettimeofday(&stop_insert, NULL);

  double dur_insert = ( (stop_insert.tv_sec  - start_insert.tv_sec ) * 1000000 + 
                        (stop_insert.tv_usec - start_insert.tv_usec) ) /1000.0;

  printf("dur_insert: %f ms\n", dur_insert);
}

