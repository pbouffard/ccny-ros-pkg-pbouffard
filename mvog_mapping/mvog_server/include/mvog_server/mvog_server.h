#ifndef MVOG_SERVER_MVOG_SERVER_H
#define MVOG_SERVER_MVOG_SERVER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <mvog_model/mapper.h>
#include <mvog_gtk_gui/gtk_gui.h>

const std::string scanTopic_  = "scan";
const std::string cloudTopic_ = "cloud";

class MVOGServer
{
  private:

    MVOG::Mapper * mapper_;
    MVOG::GTKGui * gui_;

    // **** scan subscribers
    message_filters::Subscriber < sensor_msgs::LaserScan > *scanFilterSub_;
    tf::MessageFilter < sensor_msgs::LaserScan > *scanFilter_;

    // **** transforms
    tf::TransformListener tfListener_;
    std::string worldFrame_;

    void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);

  public:

    MVOGServer ();
    virtual ~MVOGServer();

    void spinOnce();
};

#endif
