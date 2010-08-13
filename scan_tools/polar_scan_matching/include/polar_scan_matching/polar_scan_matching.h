#ifndef POLAR_SCAN_MATCHING_POLAR_SCAN_MATCHING_H
#define POLAR_SCAN_MATCHING_POLAR_SCAN_MATCHING_H

#include <sys/time.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "polar_scan_matching/polar_match.h"

const std::string scanTopic_ = "scan";
const std::string poseTopic_ = "pose2D";

const double ROS_TO_PM = 100.0;   // convert from cm to m

class PolarScanMatching
{
  private:

    PolarMatcher matcher_;

    ros::Subscriber scanSubscriber_;
    ros::Publisher  posePublisher_;
    tf::TransformBroadcaster tfBroadcaster_;
    tf::TransformListener    tfListener_;

    bool initialized_;
    double totalDuration_;
    int scansCount_;
    PMScan * prevPMScan_;
    btTransform prevWorldToBase_;

    btTransform baseToLaser_;
    btTransform laserToBase_;

    // **** parameters

    int    minValidPoints_;
    int    searchWindow_;
    double maxError_;
    int    maxIterations_;
    double stopCondition_;
    bool publishTf_;
    bool publishPose_;

    std::string worldFrame_;
    std::string baseFrame_;
    std::string laserFrame_;

    bool initialize(const sensor_msgs::LaserScan& scan);
    void scanCallback (const sensor_msgs::LaserScan& scan);
    void getCurrentEstimatedPose(btTransform& worldToBase, 
                                 const sensor_msgs::LaserScan& scanMsg);
    void publishTf(const btTransform& transform, 
                   const ros::Time& time);
    void publishPose(const btTransform& transform);

    void rosToPMScan(const sensor_msgs::LaserScan& scan, 
                     const btTransform& change,
                           PMScan* pmScan);
    void pose2DToTf(const geometry_msgs::Pose2D& pose, btTransform& t);
    void tfToPose2D(const btTransform& t, geometry_msgs::Pose2D& pose);

  public:

    PolarScanMatching ();
    virtual ~ PolarScanMatching ();
};

#endif // POLAR_SCAN_MATCHING_POLAR_SCAN_MATCHING_H
