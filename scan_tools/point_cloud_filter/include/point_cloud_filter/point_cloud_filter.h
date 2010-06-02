#ifndef POINT_CLOUD_FILTER_POINT_CLOUD_FILTER
#define POINT_CLOUD_FILTER_POINT_CLOUD_FILTER

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>

static const char* subscribeTopicName_ = "swissranger_camera/pointcloud_raw";
static const char* publishTopicName_   = "swissranger_filtered";

class PointCloudFilter
{
	private:
  
    int channel_;
    double minValue_;
    double maxValue_;
    bool copyChannels_;

    ros::Publisher  pointCloudPublisher_;
    ros::Subscriber pointCloudSubscriber_;

  public:

    PointCloudFilter();
    virtual ~PointCloudFilter();

    void pointCloudCallback(const sensor_msgs::PointCloudConstPtr& cloud);
};


#endif
