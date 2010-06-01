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

  double min_confidence_d;

  // **** paramters

  if(!nh_private.getParam("copy_channels", copyChannels_))
    copyChannels_ = true;
  if(!nh_private.getParam("min_confidence", min_confidence_d))
    min_confidence_d = 0.60;

  min_confidence_ = (int)(min_confidence_d  * 255.0);


  ROS_INFO("PointCloudFilter: min_confidence = %d", min_confidence_);

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

  sensor_msgs::PointCloud filteredCloud;
  filteredCloud.header = cloud->header;

  if(copyChannels_)
  {
    filteredCloud.channels.resize(cloud->channels.size());
    for (int i = 0; i < cloud->channels.size(); i++)
      filteredCloud.channels[i].name = cloud->channels[i].name;
  }

  for (int i = 0; i < cloud->points.size(); i++)
  {
    double value = cloud->channels[1].values[i];

    if (value >= min_confidence_)
    {
      // copy the point

      filteredCloud.points.push_back(cloud->points[i]);
  
      // copy the channel data
      if(copyChannels_)
      {
        for (int c = 0; c < cloud->channels.size(); c++)
        {
          double v = cloud->channels[c].values[i];
          filteredCloud.channels[c].values.push_back(v);
        }
      }
    }
  }

  pointCloudPublisher_.publish(filteredCloud);
}
