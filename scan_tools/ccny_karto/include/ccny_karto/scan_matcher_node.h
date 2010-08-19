#ifndef CCNY_KARTO_SCAN_MATCHER_NODE_H
#define CCNY_KARTO_SCAN_MATCHER_NODE_H

#include <string>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <karto/Mapper.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

const double DEFAULT_RANGE_THRESHOLD = 6.0;

typedef boost::shared_ptr<karto::ScanMatcher> MatcherPtr;
typedef boost::shared_ptr<karto::Mapper> MapperPtr;

class ScanMatcherNode
{
  private:

    // **** ros 

    tf::TransformBroadcaster tfBroadcaster_;
    tf::TransformListener    tfListener_;
    ros::Subscriber scan_sub_;
    ros::Publisher  posePublisher_;

    // **** parameters

    unsigned int historyLength_;
    int historySkip_;
    std::vector<double> searchSizes_;
    std::vector<double> resolutions_;
    std::vector<double> smearDeviations_;
    double distanceVariancePenalty_;
    bool publishPose_;
    bool publishTf_;

    std::string odomFrame_;
    std::string worldFrame_;
    std::string baseFrame_;

    // **** counters & flags

    bool initialized_;
    unsigned int historyIndex_;
    int scansReceived_;
    boost::mutex historyMutex_;
    geometry_msgs::Pose2D initPose_; 
    geometry_msgs::Pose2D lastEstimate_; 
   
    // **** karto

    boost::shared_ptr<karto::Dataset> dataset_;
    std::vector<MatcherPtr> matchers_;
    std::vector<MapperPtr>  mappers_;
    karto::LaserRangeFinder* laser_;
    karto::LocalizedRangeScanVector localizedReferenceScans_;

    // **** private methods

    bool initialize (const sensor_msgs::LaserScan& scanMsg);
    void scanCallback(const sensor_msgs::LaserScan& scan);
    void addToHistory(karto::LocalizedRangeScan* localizedScan);
    void publishTf(const geometry_msgs::Pose2D& estimatedPose, const ros::Time& time);
    void publishPose(const geometry_msgs::Pose2D& estimatedPose);
    void getCurrentEstimatedPose(geometry_msgs::Pose2D& currentEstimate,
                                 const sensor_msgs::LaserScan& scanMsg);
    void tokenize (const std::string& str, std::vector<std::string>& tokens);

    geometry_msgs::Pose2D subtractLaserOffset (const karto::Pose2& pose, 
                                               const karto::Pose2& offset);

  public:

    ScanMatcherNode();
    virtual ~ScanMatcherNode();
};

#endif // CCNY_KARTO_SCAN_MATCHER_NODE_H
