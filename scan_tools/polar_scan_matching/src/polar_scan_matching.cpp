#include "polar_scan_matching/polar_scan_matching.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "PolarScanMatching");
  PolarScanMatching matcher;
  ros::spin();
}

PolarScanMatching::PolarScanMatching()
{
  ROS_INFO("Creating PolarScanMatching node");

  initialized_   = false;
  totalDuration_ = 0.0;
  scansCount_    = 0;

  prevWorldToBase_.setIdentity();

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // **** get parameters
  
  if (!nh_private.getParam ("min_valid_points", minValidPoints_))
    minValidPoints_ = 200;
  if (!nh_private.getParam ("search_window", searchWindow_))
    searchWindow_ = 40;
  if (!nh_private.getParam ("max_error", maxError_))
    maxError_ = 0.20;
  if (!nh_private.getParam ("max_iterations", maxIterations_))
    maxIterations_ = 20;
  if (!nh_private.getParam ("stop_condition", stopCondition_))
    stopCondition_ = 0.01;
  if (!nh_private.getParam ("world_frame", worldFrame_))
    worldFrame_ = "world";
  if (!nh_private.getParam ("base_frame", baseFrame_))
    baseFrame_ = "base_link";
  if (!nh_private.getParam ("publish_tf", publishTf_))
    publishTf_ = true;
  if (!nh_private.getParam ("publish_pose", publishPose_))
    publishPose_ = true;

  // **** subscribe to laser scan messages
  scanSubscriber_ = nh.subscribe (scanTopic_, 10, &PolarScanMatching::scanCallback, this);

  // **** advertise pose messages
  posePublisher_ = nh.advertise<geometry_msgs::Pose2D>(poseTopic_, 10);
}

PolarScanMatching::~PolarScanMatching()
{
  ROS_INFO("Destroying PolarScanMatching node");
}

bool PolarScanMatching::initialize(const sensor_msgs::LaserScan& scan)
{
  laserFrame_ = scan.header.frame_id;

  // **** get base to laser tf

  tf::StampedTransform baseToLaserTf;
  try
  {
   tfListener_.waitForTransform(baseFrame_, scan.header.frame_id, scan.header.stamp, ros::Duration(1.0));
   tfListener_.lookupTransform (baseFrame_, scan.header.frame_id, scan.header.stamp, baseToLaserTf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("ScanMatcherNode: Could get initial laser transform, skipping scan (%s)", ex.what());
    return false;
  }
  baseToLaser_ = baseToLaserTf;
  laserToBase_ = baseToLaser_.inverse();

  // **** pass parameters to matcher and initialise

  matcher_.PM_L_POINTS         = scan.ranges.size();

  matcher_.PM_FOV              = (scan.angle_max - scan.angle_min) * 180.0 / M_PI;
  matcher_.PM_MAX_RANGE        = scan.range_max * ROS_TO_PM;

  matcher_.PM_TIME_DELAY       = 0.00;

  matcher_.PM_MIN_VALID_POINTS = minValidPoints_;
  matcher_.PM_SEARCH_WINDOW    = searchWindow_;
  matcher_.PM_MAX_ERROR        = maxError_ * ROS_TO_PM;

  matcher_.PM_MAX_ITER         = maxIterations_;
  matcher_.PM_MAX_ITER_ICP     = maxIterations_;
  matcher_.PM_STOP_COND        = stopCondition_ * ROS_TO_PM;
  matcher_.PM_STOP_COND_ICP    = stopCondition_ * ROS_TO_PM;

  matcher_.pm_init();

  // **** get the initial worldToBase tf

  getCurrentEstimatedPose(prevWorldToBase_, scan);

  // **** create the first pm scan from the laser scan message

  btTransform t;
  t.setIdentity();
  prevPMScan_ = new PMScan(scan.ranges.size());
  rosToPMScan(scan, t, prevPMScan_);

  return true;
}

void PolarScanMatching::scanCallback (const sensor_msgs::LaserScan& scan)
{
  ROS_INFO("Received scan");
  scansCount_++;

  struct timeval start, end;
  gettimeofday(&start, NULL);

  // **** if this is the first scan, initialize and leave the function here

  if (!initialized_)
  {   
    initialized_ = initialize(scan);
    if (initialized_) ROS_INFO("Matcher initialized");
    return;
  }
  
  // **** get the current position of the base in the world frame

  // if no transofrm is available, we'll use the last known transform
  btTransform currWorldToBase;
  getCurrentEstimatedPose(currWorldToBase, scan);

  // **** attmempt to match the two scans

  // PM scan matcher is used in the following way:
  // The reference scan (prevPMScan_) always has a pose of 0
  // The new scan (currPMScan) has a pose equal to the movement
  // of the laser in the world frame since the last scan (btTransform change)
  // The computed correction is then propagated using the tf machinery

  prevPMScan_->rx = 0;
  prevPMScan_->ry = 0;
  prevPMScan_->th = 0; 

  btTransform change = 
    laserToBase_ * prevWorldToBase_.inverse() * currWorldToBase * baseToLaser_;

  PMScan * currPMScan = new PMScan(scan.ranges.size());
  rosToPMScan(scan, change, currPMScan);
  
  try
  {         
    matcher_.pm_psm(prevPMScan_, currPMScan);                         
  }
  catch(int err)
  {
    ROS_WARN("Error in scan matching");
    delete prevPMScan_;
    prevPMScan_ = currPMScan;
    return;
  };    

  // **** calculate change in position

  // rotate by -90 degrees, since polar scan matcher assumes different laser frame
  // and scale down by 100
  double dx =  currPMScan->ry / ROS_TO_PM;
  double dy = -currPMScan->rx / ROS_TO_PM;
  double da =  currPMScan->th; 

  // change = scan match result for how much laser moved between scans, 
  // in the world frame
  change.setOrigin(btVector3(dx, dy, 0.0));
  btQuaternion q;
  q.setRPY(0, 0, da);
  change.setRotation(q);
  
  // **** publish the new estimated pose as a tf
   
  currWorldToBase = currWorldToBase * baseToLaser_ * change * laserToBase_;

  if (publishTf_  ) publishTf  (currWorldToBase, scan.header.stamp);
  if (publishPose_) publishPose(currWorldToBase);

  printf("x, y: %f, %f\n", currWorldToBase.getOrigin().getX(), 
                           currWorldToBase.getOrigin().getY());

  // **** swap old and new

  delete prevPMScan_;
  prevPMScan_      = currPMScan;
  prevWorldToBase_ = currWorldToBase;

  // **** timing information - needed for profiling only

  gettimeofday(&end, NULL);
  double dur = ((end.tv_sec   * 1000000 + end.tv_usec  ) - 
                (start.tv_sec * 1000000 + start.tv_usec)) / 1000.0;
  totalDuration_ += dur;
  double ave = totalDuration_/scansCount_;

  printf("dur:\t %.3f ms \t ave:\t %.3f ms\n", dur, ave);
}

void PolarScanMatching::publishTf(const btTransform& transform, 
                                  const ros::Time& time)
{
  ROS_INFO("Publishing TF");

  tf::StampedTransform transformMsg (transform, time, worldFrame_, baseFrame_);
  tfBroadcaster_.sendTransform (transformMsg);
}

void PolarScanMatching::publishPose(const btTransform& transform)
{
  ROS_INFO("Publishing POSE");

  geometry_msgs::Pose2D pose;
  tfToPose2D(transform, pose);

  posePublisher_.publish(pose);
}

void PolarScanMatching::rosToPMScan(const sensor_msgs::LaserScan& scan, 
                                    const btTransform& change,
                                    PMScan* pmScan)
{
  geometry_msgs::Pose2D pose;
  tfToPose2D(change, pose);

  // FIXME: rotate x & y by 90 degree?

  pmScan->rx = pose.x * ROS_TO_PM;
  pmScan->ry = pose.y * ROS_TO_PM;
  pmScan->th = pose.theta;

  for (int i = 0; i < scan.ranges.size(); ++i)
  {
    if (scan.ranges[i] == 0) 
    {
      pmScan->r[i] = 99999;  // hokuyo uses 0 for out of range reading
    }
    else
    {
      pmScan->r[i] = scan.ranges[i] * ROS_TO_PM;
      pmScan->x[i] = (pmScan->r[i]) * matcher_.pm_co[i];
      pmScan->y[i] = (pmScan->r[i]) * matcher_.pm_si[i];
      pmScan->bad[i] = 0;
    }

    pmScan->bad[i] = 0;
  }

  matcher_.pm_median_filter  (pmScan);
  matcher_.pm_find_far_points(pmScan);
  matcher_.pm_segment_scan   (pmScan);  
}

void PolarScanMatching::getCurrentEstimatedPose(btTransform& worldToBase, 
                                                const sensor_msgs::LaserScan& scanMsg)
{
  tf::StampedTransform worldToBaseTf;
  try
  {
     tfListener_.lookupTransform (worldFrame_, baseFrame_, scanMsg.header.stamp, worldToBaseTf);
  }
  catch (tf::TransformException ex)
  {
    // transform unavailable - use the pose from our last estimation
    ROS_WARN("Transform unavailable, using last estimated pose (%s)", ex.what());
    worldToBase = prevWorldToBase_;
    return;
  }

  worldToBase = worldToBaseTf;

  // remove z, roll, pitch
  geometry_msgs::Pose2D pose;
  tfToPose2D(worldToBase, pose);
  pose2DToTf(pose, worldToBase);
}

void PolarScanMatching::pose2DToTf(const geometry_msgs::Pose2D& pose, btTransform& t)
{
  t.setOrigin(btVector3(pose.x, pose.y, 0.0));
  btQuaternion q;
  q.setRPY(0, 0, pose.theta);
  t.setRotation(q);
}

void PolarScanMatching::tfToPose2D(const btTransform& t, geometry_msgs::Pose2D& pose)
{
  btMatrix3x3 m(t.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  pose.x = t.getOrigin().getX();
  pose.y = t.getOrigin().getY();
  pose.theta = yaw;
}
