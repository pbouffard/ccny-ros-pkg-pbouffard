//#include <karto_scan_matcher/karto_scan_matcher.h>

#include <string>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <karto/Mapper.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace karto_scan_matcher
{

const double DEFAULT_SMEAR_DEVIATION = 0.005;
const double DEFAULT_RANGE_THRESHOLD = 6.0;

typedef boost::mutex::scoped_lock Lock;

class ScanMatcherNode
{
  private:

    ros::NodeHandle nh_;
    tf::TransformBroadcaster tfBroadcaster_;
    tf::TransformListener    tfListener_;
    ros::Subscriber scan_sub_;
    ros::Publisher  posePublisher_;

    boost::mutex mutex_;
    boost::mutex historyMutex_;

    int historyLength_;
    std::vector<double> searchSizes_;
    std::vector<double> resolutions_;
    int historySkip_;
    std::string mapFrame_;
    std::string odomFrame_;
    std::string worldFrame_;
    std::string baseFrame_;

    double distanceVariancePenalty_;
    double smearDeviation_;

    int historyIndex_;
    bool historyReady_;
    bool initialized_;

    int scansReceived_;
    geometry_msgs::Pose2D initPose_; 
    geometry_msgs::Pose2D lastEstimate_; 
   
    void scanCallback(const sensor_msgs::LaserScan& scan);
    void addToHistory(const sensor_msgs::LaserScan& scanMsg, const geometry_msgs::Pose2D& scanPose);
    void publishMapToOdomTf(const geometry_msgs::Pose2D& estimatedPose, const ros::Time& time);
    void tokenize (const std::string& str, std::vector<std::string>& tokens);

    bool initialize (const sensor_msgs::LaserScan& scanMsg);

    geometry_msgs::Pose2D subtractLaserOffset (const karto::Pose2& pose, 
                                               const karto::Pose2& offset);

    typedef boost::shared_ptr<karto::ScanMatcher> MatcherPtr;
    typedef boost::shared_ptr<karto::Mapper> MapperPtr;

    boost::shared_ptr<karto::Dataset> dataset_;
    std::vector<MatcherPtr> matchers_;
    std::vector<MapperPtr> mappers_;
    karto::LaserRangeFinder* laser_;

    karto::LocalizedRangeScanVector localizedReferenceScans_;

  public:

    ScanMatcherNode();
};

ScanMatcherNode::ScanMatcherNode()
{
  ROS_INFO("Starting ScanMatcherNode");

  initialized_ = false;

  ros::NodeHandle nh_private("~");

  std::string searchSizesString;
  std::string resolutionsString;
  std::vector<std::string> searchSizesTokens;
  std::vector<std::string> resolutionsTokens;

  if (!nh_private.getParam ("history_length", historyLength_))
    historyLength_ = 15;
  if (!nh_private.getParam ("smear_deviation", smearDeviation_))
    smearDeviation_ = 0.01;
  if (!nh_private.getParam ("distance_variance_penalty", distanceVariancePenalty_))
    distanceVariancePenalty_ = 1.0;
  if (!nh_private.getParam ("search_sizes", searchSizesString))
    searchSizesString = "1.0";
  if (!nh_private.getParam ("resolutions", resolutionsString))
    resolutionsString = "0.03";
  if (!nh_private.getParam ("history_skip", historySkip_))
    historySkip_ = 0;
  if (!nh_private.getParam ("map_frame", mapFrame_))
    mapFrame_ = "map";
  if (!nh_private.getParam ("odom_frame", odomFrame_))
    odomFrame_ = "odom";
  if (!nh_private.getParam ("world_frame", worldFrame_))
    worldFrame_ = "world";
  if (!nh_private.getParam ("base_frame", baseFrame_))
    baseFrame_ = "base_link";

  tokenize(searchSizesString, searchSizesTokens);
  tokenize(resolutionsString, resolutionsTokens);

  for(unsigned int i = 0; i < searchSizesTokens.size(); ++i)
  {
    double size = std::strtod(searchSizesTokens[i].c_str(), NULL);
    searchSizes_.push_back(size);
  }

  for(unsigned int i = 0; i < resolutionsTokens.size(); ++i)
  {
    double res = std::strtod(resolutionsTokens[i].c_str(), NULL);
    resolutions_.push_back(res);
  }

  ROS_ASSERT (resolutions_.size() == searchSizes_.size());
  ROS_ASSERT (historyLength_ > 1);

  scansReceived_ = 0;

  initPose_.x     = 0.0;
  initPose_.y     = 0.0;
  initPose_.theta = 0.0;

  historyIndex_ = 0;

  scan_sub_ = nh_.subscribe("scan", 10, &ScanMatcherNode::scanCallback, this);

  posePublisher_ = nh_.advertise<geometry_msgs::Pose2D>("pose2D", 10);
}

void ScanMatcherNode::scanCallback (const sensor_msgs::LaserScan& scanMsg)
{
  struct timeval start, end;
  gettimeofday(&start, NULL);

  scansReceived_++;

  // **** If this is the first scan, initialize the matcher

  if (!initialized_) 
  {
    initialized_ = initialize(scanMsg);
    if (initialized_) ROS_INFO("ScanMatcherNode: Initialized successfully.");
    else ROS_INFO("ScanMatcherNode: Failed to initialize.");
  }

  // **** Fill up the history

  if (localizedReferenceScans_.size() < historyLength_)
  {
    addToHistory(scanMsg, initPose_);
    return;
  }

  boost::mutex::scoped_lock lock(mutex_);

  // **** get estimate for current pose
  
  geometry_msgs::Pose2D currentEstimate = lastEstimate_;

  tf::StampedTransform odomToBaseTf;
  try
  {
     tfListener_.lookupTransform (worldFrame_, baseFrame_, scanMsg.header.stamp, odomToBaseTf);
  }
  catch (tf::TransformException ex)
  {
    // transform unavailable - skip scan
    ROS_WARN("Transform unavailable, skipping scan (%s)", ex.what());
    return;
  }
  btTransform odomToBase = odomToBaseTf;

  btMatrix3x3 m(odomToBase.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  currentEstimate.x = odomToBase.getOrigin().getX();
  currentEstimate.y = odomToBase.getOrigin().getY();
  currentEstimate.theta = yaw;

  // **** scan match

  for (unsigned int i = 0; i < matchers_.size(); ++i)
  {
    karto::Pose2 kartoPose(currentEstimate.x, currentEstimate.y, currentEstimate.theta);
    karto::RangeReadingsVector kartoReadings(scanMsg.ranges.begin(), scanMsg.ranges.end());

    karto::LocalizedRangeScan* localizedScan = new karto::LocalizedRangeScan(laser_->GetName(), kartoReadings);
    localizedScan->SetOdometricPose(kartoPose);
    localizedScan->SetCorrectedPose(kartoPose);

    //ScanPtr scan_ptr(localized_scan); 

    karto::Pose2 mean;
    karto::Matrix3 cov;

    double lastResponse = matchers_[i]->MatchScan(localizedScan, localizedReferenceScans_, mean, cov);

    printf ("lastresponse= %f\n", lastResponse);

    currentEstimate = subtractLaserOffset(mean, laser_->GetOffsetPose());
  }

  lastEstimate_ = currentEstimate;
  addToHistory(scanMsg, currentEstimate);

  publishMapToOdomTf(currentEstimate, scanMsg.header.stamp);

  gettimeofday(&end, NULL);
  double dur = (end.tv_sec * 1000000 + end.tv_usec) - (start.tv_sec * 1000000 + start.tv_usec);
  printf("dur:\t %d ms\n", (int)(dur/1000.0));
}

geometry_msgs::Pose2D ScanMatcherNode::subtractLaserOffset (const karto::Pose2& pose, const karto::Pose2& offset)
{
  btTransform laser_to_base(tf::createQuaternionFromYaw(offset.GetHeading()),
                            btVector3(offset.GetX(), offset.GetY(), 0.0));

  btTransform laser_to_map(tf::createQuaternionFromYaw(pose.GetHeading()),
                           btVector3(pose.GetX(), pose.GetY(), 0.0));

  btTransform base_to_map = laser_to_map*laser_to_base.inverse();
  geometry_msgs::Pose2D result;
  result.x = base_to_map.getOrigin().x();
  result.y = base_to_map.getOrigin().y();
  result.theta = tf::getYaw(base_to_map.getRotation());
  return result;
}

void ScanMatcherNode::addToHistory(const sensor_msgs::LaserScan& scanMsg, 
                                   const geometry_msgs::Pose2D& scanPose)
{
  karto::Pose2 kartoPose(scanPose.x, scanPose.y, scanPose.theta);
  karto::RangeReadingsVector kartoReadings(scanMsg.ranges.begin(), scanMsg.ranges.end());

  karto::LocalizedRangeScan* localizedScan = new karto::LocalizedRangeScan(laser_->GetName(), kartoReadings);
  localizedScan->SetOdometricPose(kartoPose);
  localizedScan->SetCorrectedPose(kartoPose);
  
  historyMutex_.lock();

  if (localizedReferenceScans_.size() < historyLength_)
      localizedReferenceScans_.push_back(localizedScan);
  else
  {
    delete localizedReferenceScans_[historyIndex_]; 
    localizedReferenceScans_[historyIndex_] = localizedScan;
  }

  historyIndex_++;    
  if (historyIndex_ == historyLength_) historyIndex_ = 0;

  historyMutex_.unlock();
}

void ScanMatcherNode::publishMapToOdomTf(const geometry_msgs::Pose2D& estimatedPose, const ros::Time& time)
{

  btTransform transform;

  btQuaternion rotation;
  rotation.setRPY (0.0, 0.0, 0.0);
  transform.setRotation (rotation);

  btVector3 origin;
  origin.setValue (estimatedPose.x, estimatedPose.y, 0.0);
  transform.setOrigin (origin);

  //transform = transform.inverse();

  tf::StampedTransform transformMsg (transform, time, worldFrame_, odomFrame_);
  tfBroadcaster_.sendTransform (transformMsg);

  // publish pose2D

  //posePublisher_.publish(estimatedPose);
}

void ScanMatcherNode::tokenize(const std::string& str, std::vector<std::string>& tokens)
{
  std::string::size_type lastPos = str.find_first_not_of (" ", 0);
  std::string::size_type pos = str.find_first_of (" ", lastPos);

  while (std::string::npos != pos || std::string::npos != lastPos)
  {
    std::string stringToken = str.substr (lastPos, pos - lastPos);
    tokens.push_back (stringToken);
    lastPos = str.find_first_not_of (" ", pos);
    pos = str.find_first_of (" ", lastPos);
  }
}

bool ScanMatcherNode::initialize (const sensor_msgs::LaserScan& scanMsg)
{
  // **** get base to laser tf

  tf::StampedTransform baseToLaserTf;
  try
  {
   tfListener_.waitForTransform(baseFrame_, scanMsg.header.frame_id, scanMsg.header.stamp, ros::Duration(1.0));
   tfListener_.lookupTransform (baseFrame_, scanMsg.header.frame_id, scanMsg.header.stamp, baseToLaserTf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("ScanMatcherNode: Could get initial laser transform, skipping scan (%s)", ex.what());
    return false;
  }

  geometry_msgs::Pose2D laserPose;
  laserPose.x = baseToLaserTf.getOrigin().x();
  laserPose.y = baseToLaserTf.getOrigin().y();
  laserPose.theta = tf::getYaw(baseToLaserTf.getRotation());

  ROS_INFO("ScanMatcherNode: Received transform: %f, %f, %f",
            laserPose.x, laserPose.y, laserPose.theta);

  // **** set up karto

  dataset_.reset(new karto::Dataset());

  matchers_.resize(searchSizes_.size());
  mappers_.resize(searchSizes_.size());

  for (size_t i=0; i < searchSizes_.size(); ++i) 
  {
    if (i>0) 
    {
      if (searchSizes_[i] >= searchSizes_[i-1])
        ROS_WARN ("ScanMatcherNode: Matcher %zu search size was %f and %zu was %f, which look out of order",
                        i-1, searchSizes_[i-1], i, searchSizes_[i]);
      if (resolutions_[i] >= resolutions_[i-1])
        ROS_WARN ("ScanMatcherNode: Matcher %zu search resolution was %f and %zu was %f, which look out of order",
                        i-1, resolutions_[i-1], i, resolutions_[i]);
    }

    mappers_[i].reset(new karto::Mapper());
    mappers_[i]->SetParameter("DistanceVariancePenalty", distanceVariancePenalty_);

    matchers_[i].reset(karto::ScanMatcher::Create(mappers_[i].get(), searchSizes_[i], resolutions_[i],
                                                  smearDeviation_, DEFAULT_RANGE_THRESHOLD));

    ROS_INFO ("ScanMatcherNode: matcher %zu: searchspace is %.2f; resolution is %.2f", i, searchSizes_[i], resolutions_[i]);
  }

  // This is copied over from slam_karto.cpp
  // Create a laser range finder device and copy in data from the first scan

  std::string name = scanMsg.header.frame_id;

  laser_ = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, karto::Name(name));
  laser_->SetOffsetPose(karto::Pose2(laserPose.x, laserPose.y, laserPose.theta));
  laser_->SetMinimumRange(scanMsg.range_min);
  laser_->SetMaximumRange(scanMsg.range_max);
  laser_->SetMinimumAngle(scanMsg.angle_min);
  laser_->SetMaximumAngle(scanMsg.angle_max);
  laser_->SetAngularResolution(scanMsg.angle_increment);
  laser_->SetRangeThreshold(DEFAULT_RANGE_THRESHOLD);

  dataset_->Add(laser_);

  return true;
}


} // namespace karto_scan_matcher

int main (int argc, char** argv)
{

  ros::init(argc, argv, "ScanMatcherNode");
  karto_scan_matcher::ScanMatcherNode node;
  ros::spin();
}
