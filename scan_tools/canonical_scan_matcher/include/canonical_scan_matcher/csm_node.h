#ifndef CANONICAL_SCAN_MATCHER_CSM_NODE_H
#define CANONICAL_SCAN_MATCHER_CSM_NODE_H

#include <sys/time.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>

#include <csm/csm_all.h>

const std::string imuTopic_  = "imu";
const std::string scanTopic_ = "scan";
const std::string poseTopic_ = "pose2D";

class CSMNode
{
  private:

    ros::Subscriber scanSubscriber_;
    ros::Subscriber imuSubscriber_;
    ros::Publisher  posePublisher_;

    tf::TransformBroadcaster tfBroadcaster_;
    tf::TransformListener    tfListener_;
    btTransform prevWorldToBase_;
    btTransform baseToLaser_;
    btTransform laserToBase_;

    bool initialized_;
    double totalDuration_;
    int scansCount_;

    sm_params input_;
    sm_result output_;

    LDP prevLDPScan_;

    boost::mutex imuMutex_;
    double prevImuAngle_;
    double currImuAngle_;

    // **** parameters
 
    bool   publishTf_;
    bool   publishPose_;
    bool   useTfOdometry_;
    bool   useImuOdometry_;

    std::string worldFrame_;
    std::string baseFrame_;
    std::string laserFrame_;

    void getParams();
    bool initialize(const sensor_msgs::LaserScan& scan);

    void scanCallback (const sensor_msgs::LaserScan& scan);

    LDP rosToLDPScan(const sensor_msgs::LaserScan& scan,
                     const geometry_msgs::Pose2D& laserPose);

    void publishTf(const btTransform& transform, 
                   const ros::Time& time);
    void publishPose(const btTransform& transform);

    void tfToPose2D(const btTransform& t, geometry_msgs::Pose2D& pose);
    void pose2DToTf(const geometry_msgs::Pose2D& pose, btTransform& t);
    void getCurrentEstimatedPose(btTransform& worldToBase, 
                                 const sensor_msgs::LaserScan& scanMsg);
    void imuCallback (const sensor_msgs::Imu& imuMsg);

  public:

    CSMNode();
    virtual ~CSMNode();
};

#endif // CANONICAL_SCAN_MATCHER_CSM_NODE_H
