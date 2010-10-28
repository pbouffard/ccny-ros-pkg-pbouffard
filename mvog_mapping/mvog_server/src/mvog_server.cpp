#include "mvog_server/mvog_server.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "mvog_server");
  MVOGServer mvogServer;
  ros::spin ();
  return 0;
}

MVOGServer::MVOGServer ()
{
  ROS_INFO ("Starting MVOGServer");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private ("~");

  // **** get parameters

  double mapResolution;
  double initMapWidth;
  double initMapHeight;
  bool modelNegativeSpace;

  if (!nh_private.getParam ("map_resolution", mapResolution))
    mapResolution = 0.10;
  if (!nh_private.getParam ("init_map_width", initMapWidth))
    initMapWidth = 10;
  if (!nh_private.getParam ("init_map_height", initMapHeight))
    initMapHeight = 10;
  if (!nh_private.getParam ("model_negative_space", modelNegativeSpace))
    modelNegativeSpace = true;

  // **** create mapper

  mapper = new MVOG::Mapper(mapResolution, initMapWidth, initMapHeight);
  mapper->setModelNegativeSpace(modelNegativeSpace);

  // **** create gui

  gui = new MVOG::GTKGui();
  gui->setMap(mapper->getMap());

  // **** testing

/*
  map->addVolume(0, 0, -8.0, -6.0);
  map->addVolume(0, 0, -4.0, -2.0);
  map->addVolume(0, 0,  0.0,  2.0);
  map->addVolume(0, 0,  4.0,  6.0);
  map->addVolume(0, 0,  8.0, 10.0);
  map->addVolume(0, 0, 12.0, 14.0);
  map->addVolume(0, 0, 16.0, 18.0);
  map->addVolume(0, 0, 20.0, 22.0);
  map->printVolumes(0, 0);

  map->addVolume(0, 0, 6.0, 8.0);
  map->printVolumes(0, 0);
*/

  mapper->addBeamReading(btVector3( 0.05,  0.05, 0.0),
                       btVector3(0.10,  1.5, 0.5));

  mapper->addBeamReading(btVector3(-0.05,  0.05, 0.0),
                       btVector3(-10.00,  0.50, 3.5));

  mapper->addBeamReading(btVector3(-0.05, -0.05, 0.0),
                       btVector3(-1.00, -0.50, 3.5));

  mapper->addBeamReading(btVector3( 0.05, -0.05, 0.0),
                       btVector3( 1.00, -10.0, 3.5));

  // **** subscribe to laser scan messages
  scanSubscriber_ = nh.subscribe (scanTopic_, 100, &MVOGServer::scanCallback, this);

  // **** start gui
  gui->start();
}

MVOGServer::~MVOGServer ()
{
  ROS_INFO ("Destroying MVOGServer");
}

void MVOGServer::scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  ROS_INFO ("Received scan");

}

