#ifndef MVOG_SERVER_MVOG_SERVER_H
#define MVOG_SERVER_MVOG_SERVER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <mvog_model/mapper.h>
#include <mvog_gtk_gui/gtk_gui.h>

const std::string scanTopic_  = "scan";
const std::string cloudTopic_ = "cloud";

class MVOGServer
{
  private:

    ros::Subscriber scanSubscriber_;

    void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);

    MVOG::Mapper * mapper;
    MVOG::GTKGui * gui;

  public:

    MVOGServer ();
    virtual ~MVOGServer();

};

#endif
