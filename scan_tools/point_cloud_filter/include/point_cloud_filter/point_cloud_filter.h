/*
*  Point Cloud Filter
*  Copyright (C) 2010, CCNY Robotics Lab
*  Ivan Dryanovski <ivan.dryanovski@gmail.com>
*  William Morris <morris@ee.ccny.cuny.edu>
*  http://robotics.ccny.cuny.edu
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

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
