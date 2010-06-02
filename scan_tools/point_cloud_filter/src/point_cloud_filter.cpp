#include "point_cloud_filter/point_cloud_filter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PointCloudFilter");

  PointCloudFilter pointCloudFilter;

  ros::spin();

	return 0;
}

PointCloudFilter::PointCloudFilter()
{
	// **** set up ROS

	ROS_INFO("Starting PointCloudFilter"); 
  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh;

  // **** paramters

  if(!nh_private.getParam("copy_channels", copyChannels_))
    copyChannels_ = true;
  if(!nh_private.getParam("channel", channel_))
    channel_ = 1;
  if(!nh_private.getParam("min_value", minValue_))
    minValue_ = 180;
  if(!nh_private.getParam("max_value", maxValue_))
    minValue_ = 255;

  // **** subscribe to point cloud messages
  pointCloudSubscriber_ = nh.subscribe(subscribeTopicName_, 10,  &PointCloudFilter::pointCloudCallback, this);

  // **** advertise published filtered pointcloud
  pointCloudPublisher_ = nh.advertise<sensor_msgs::PointCloud>(publishTopicName_, 10);
}

PointCloudFilter::~PointCloudFilter()
{

}

void PointCloudFilter::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& cloud)
{
  ROS_DEBUG("Received point cloud");

  if (channel_ >= cloud->channels.size())
  {
    ROS_WARN("PointCloudFilter: Received point cloud with too few channels. Skipping");
    return;
  }

  sensor_msgs::PointCloud filteredCloud;
  filteredCloud.header = cloud->header;

  // copy the channel names
  if(copyChannels_)
  {
    filteredCloud.channels.resize(cloud->channels.size());
    for (unsigned int i = 0; i < cloud->channels.size(); i++)
      filteredCloud.channels[i].name = cloud->channels[i].name;
  }

  for (unsigned int i = 0; i < cloud->points.size(); i++)
  {
    double value = cloud->channels[channel_].values[i];

    if (value >= minValue_ && value <=maxValue_)
    {
      // copy the point
      filteredCloud.points.push_back(cloud->points[i]);
  
      // copy the channel data
      if(copyChannels_)
      {
        for (unsigned int c = 0; c < cloud->channels.size(); c++)
        {
          double v = cloud->channels[c].values[i];
          filteredCloud.channels[c].values.push_back(v);
        }
      }
    }
  }

  pointCloudPublisher_.publish(filteredCloud);
}
