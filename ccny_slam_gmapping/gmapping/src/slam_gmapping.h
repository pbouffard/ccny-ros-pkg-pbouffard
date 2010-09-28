/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

#include <iostream>
#include <time.h>
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"
#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/GetMap.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"
#include "gmapping/utils/point.h"

#include "laser_ortho_projector/LaserScanWithAngles.h"

#include <boost/thread.hpp>

static const char* scanOrthoTopic_ = "/laser_ortho_projector/scan_ortho";

class SlamGMapping
{
  public:
    SlamGMapping();
    ~SlamGMapping();

    void publishTransform();
    void publishPointCloud();
  
    void laserCallback(const laser_ortho_projector::LaserScanWithAngles::ConstPtr& scan);
    void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& cloud);

    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res);
    void publishLoop(double transform_publish_period);

  private:
    ros::NodeHandle node_;
    ros::Publisher entropy_publisher_;
    ros::Publisher sst_;
    ros::Publisher sstm_;

    ros::Publisher pose2Dpub_;
    ros::ServiceServer ss_;
    tf::TransformListener tf_;
    message_filters::Subscriber<laser_ortho_projector::LaserScanWithAngles>* scan_filter_sub_;
    tf::MessageFilter<laser_ortho_projector::LaserScanWithAngles>* scan_filter_;
    tf::TransformBroadcaster* tfB_;

    GMapping::GridSlamProcessor* gsp_;
    GMapping::RangeSensor* gsp_laser_;
    GMapping::OdometrySensor* gsp_odom_;

    bool inverted_laser_;
    bool got_first_scan_;

    bool got_map_;
    nav_msgs::GetMap::Response map_;

    ros::Duration map_update_interval_;
    tf::Transform map_to_odom_;
    boost::mutex map_to_odom_mutex_;
    boost::mutex map_mutex_;

    int laser_count_;
    int throttle_scans_;

    boost::thread* transform_thread_;

    std::string base_frame_;
    std::string laser_frame_;
    std::string map_frame_;
    std::string odom_frame_;

    void updateMap(const laser_ortho_projector::LaserScanWithAngles& scan);
    bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t);
    bool initMapper(const laser_ortho_projector::LaserScanWithAngles& scan);
    bool addScan(const laser_ortho_projector::LaserScanWithAngles& scan, GMapping::OrientedPoint& gmap_pose);
    double computePoseEntropy();
    
    // ivan

    sensor_msgs::PointCloud lastCloud_;
    boost::mutex cloud_mutex_;
    ros::Publisher pointCloudPublisher_;
    ros::Subscriber pointCloudSubscriber_;
    tf::Transform last_odom_pose;

    // Parameters used by GMapping
    double maxRange_;
    double maxUrange_;
    double maxrange_;
    double sigma_;
    int kernelSize_;
    double lstep_;
    double astep_;
    int iterations_;
    double lsigma_;
    double ogain_;
    int lskip_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    int particles_;
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_;
    double occ_thresh_;
    double llsamplerange_;
    double llsamplestep_;
    double lasamplerange_;
    double lasamplestep_;
};
