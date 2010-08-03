#include "ccny_karto/scan_matcher_node.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "ScanMatcherNode");
  ScanMatcherNode node;
  ros::spin();
}

ScanMatcherNode::ScanMatcherNode()
{
  ROS_INFO("Starting ScanMatcherNode");

  initialized_    = false;
  scansReceived_  = 0;
  historyIndex_   = 0;
  initPose_.x     = 0.0;
  initPose_.y     = 0.0;
  initPose_.theta = 0.0;

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string searchSizesString;
  std::string resolutionsString;
  std::string smearDeviationsString;
  std::vector<std::string> searchSizesTokens;
  std::vector<std::string> resolutionsTokens;
  std::vector<std::string> smearDeviationsTokens;

  int historyLength;

  if (!nh_private.getParam ("history_length", historyLength))
    historyLength = 15;
  if (!nh_private.getParam ("history_skip", historySkip_))
    historySkip_ = 0;
  if (!nh_private.getParam ("distance_variance_penalty", distanceVariancePenalty_))
    distanceVariancePenalty_ = 1.0;
  if (!nh_private.getParam ("search_sizes", searchSizesString))
    searchSizesString = "0.50";
  if (!nh_private.getParam ("resolutions", resolutionsString))
    resolutionsString = "0.01";
  if (!nh_private.getParam ("smear_deviations", smearDeviationsString))
    smearDeviationsString = "0.01";
  if (!nh_private.getParam ("odom_frame", odomFrame_))
    odomFrame_ = "odom";
  if (!nh_private.getParam ("world_frame", worldFrame_))
    worldFrame_ = "world";
  if (!nh_private.getParam ("base_frame", baseFrame_))
    baseFrame_ = "base_link";
  if (!nh_private.getParam ("publish_tf", publishTf_))
    publishTf_ = true;
  if (!nh_private.getParam ("publish_pose", publishPose_))
    publishPose_ = true;

  historyLength_ = historyLength;

  tokenize(searchSizesString, searchSizesTokens);
  tokenize(resolutionsString, resolutionsTokens);
  tokenize(smearDeviationsString, smearDeviationsTokens);

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

  for(unsigned int i = 0; i < smearDeviationsTokens.size(); ++i)
  {
    double dev = std::strtod(smearDeviationsTokens[i].c_str(), NULL);
    smearDeviations_.push_back(dev);
  }

  ROS_ASSERT (resolutions_.size() == searchSizes_.size());
  ROS_ASSERT (resolutions_.size() == smearDeviations_.size());
  ROS_ASSERT (historyLength_ > 1);

  scan_sub_ = nh.subscribe("scan", 10, &ScanMatcherNode::scanCallback, this);
  posePublisher_ = nh.advertise<geometry_msgs::Pose2D>("pose2D", 10);
}

ScanMatcherNode::~ScanMatcherNode()
{
  ROS_INFO("Destroying ScanMatcherNode");
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
    if (initialized_)
    { 
      ROS_INFO("ScanMatcherNode: Initialized successfully.");

      // Fill up the history
      historyMutex_.lock();
      while (localizedReferenceScans_.size() < historyLength_)
      {
        karto::RangeReadingsVector kartoReadings(scanMsg.ranges.begin(), scanMsg.ranges.end());
        karto::LocalizedRangeScan* localizedScan = new karto::LocalizedRangeScan(laser_->GetName(), kartoReadings);

        karto::Pose2 kartoPose(initPose_.x, initPose_.y, initPose_.theta);
        localizedScan->SetOdometricPose(kartoPose);
        localizedScan->SetCorrectedPose(kartoPose);

        localizedReferenceScans_.push_back(localizedScan);
      }
      historyMutex_.unlock();      
    }
    else
    { 
      ROS_WARN("ScanMatcherNode: Failed to initialize. Skipping scan");
      return;
    }
  }

  // **** get estimate for current pose

  geometry_msgs::Pose2D currentEstimate;
  getCurrentEstimatedPose(currentEstimate, scanMsg);

  // **** scan match

  karto::RangeReadingsVector kartoReadings(scanMsg.ranges.begin(), scanMsg.ranges.end());
  karto::LocalizedRangeScan* localizedScan = new karto::LocalizedRangeScan(laser_->GetName(), kartoReadings);

  for (unsigned int i = 0; i < matchers_.size(); ++i)
  {
    karto::Pose2 kartoPose(currentEstimate.x, currentEstimate.y, currentEstimate.theta);
    localizedScan->SetOdometricPose(kartoPose);
    localizedScan->SetCorrectedPose(kartoPose);

    karto::Pose2 mean;
    karto::Matrix3 cov;

    historyMutex_.lock();
    double lastResponse = matchers_[i]->MatchScan(localizedScan, localizedReferenceScans_, 
                                                  mean, cov, false, true);
    historyMutex_.unlock();

    currentEstimate = subtractLaserOffset(mean, laser_->GetOffsetPose());
  }

  lastEstimate_ = currentEstimate;

  karto::Pose2 kartoPose(currentEstimate.x, currentEstimate.y, currentEstimate.theta);
  localizedScan->SetOdometricPose(kartoPose);
  localizedScan->SetCorrectedPose(kartoPose);

  // **** add to history, if we're not skipping this one

  if (historySkip_ < 1 || scansReceived_ % historySkip_ == 0)
    addToHistory(localizedScan);

  // **** publish tf & pose to ros if needed

  if (publishTf_)   publishTf(currentEstimate, scanMsg.header.stamp);
  if (publishPose_) publishPose(currentEstimate);

  // **** timing
  
  gettimeofday(&end, NULL);
  double dur = (end.tv_sec * 1000000 + end.tv_usec) - (start.tv_sec * 1000000 + start.tv_usec);
  printf("dur:\t %d ms\n", (int)(dur/1000.0));
}

void ScanMatcherNode::getCurrentEstimatedPose(geometry_msgs::Pose2D& currentEstimate, 
                                              const sensor_msgs::LaserScan& scanMsg)
{
  tf::StampedTransform odomToBaseTf;
  try
  {
     tfListener_.lookupTransform (worldFrame_, baseFrame_, scanMsg.header.stamp, odomToBaseTf);
  }
  catch (tf::TransformException ex)
  {
    // transform unavailable - use the pose from our last estimation
    ROS_WARN("Transform unavailable, using last estimated pose (%s)", ex.what());
    currentEstimate = lastEstimate_;
    return;
  }

  btTransform odomToBase = odomToBaseTf;

  btMatrix3x3 m(odomToBase.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  currentEstimate.x = odomToBase.getOrigin().getX();
  currentEstimate.y = odomToBase.getOrigin().getY();
  currentEstimate.theta = yaw;
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

void ScanMatcherNode::addToHistory(karto::LocalizedRangeScan* localizedScan)
{
  historyMutex_.lock();

  delete localizedReferenceScans_[historyIndex_]; 
  localizedReferenceScans_[historyIndex_] = localizedScan;

  historyIndex_++;    
  if (historyIndex_ == historyLength_) historyIndex_ = 0;

  historyMutex_.unlock();
}

void ScanMatcherNode::publishTf(const geometry_msgs::Pose2D& estimatedPose, const ros::Time& time)
{
  btTransform transform;

  btQuaternion rotation;
  rotation.setRPY (0.0, 0.0, estimatedPose.theta);
  transform.setRotation (rotation);

  btVector3 origin;
  origin.setValue (estimatedPose.x, estimatedPose.y, 0.0);
  transform.setOrigin (origin);

  tf::StampedTransform transformMsg (transform, time, worldFrame_, odomFrame_);
  tfBroadcaster_.sendTransform (transformMsg);
}

void ScanMatcherNode::publishPose(const geometry_msgs::Pose2D& estimatedPose)
{
  posePublisher_.publish(estimatedPose);
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
                                                  smearDeviations_[i], DEFAULT_RANGE_THRESHOLD));

    ROS_INFO ("ScanMatcherNode: matcher %zu:", i);
    ROS_INFO ("\tsearchspace is %.3f", searchSizes_[i]);
    ROS_INFO ("\tresolution is %.3f", resolutions_[i]);
    ROS_INFO ("\tsmear deviation is %.3f", smearDeviations_[i]);   

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


