#include "canonical_scan_matcher/csm_node.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "CSMNode");
  
  CSMNode csmNode;

  ros::spin();

  return 0;
}

CSMNode::CSMNode()
{
  ROS_INFO("Creating CanonicalScanMatcher node");

  ros::NodeHandle nh;

  initialized_   = false;
  totalDuration_ = 0.0;
  scansCount_    = 0;

  prevWorldToBase_.setIdentity();

  getParams();  

  scanSubscriber_ = nh.subscribe (scanTopic_, 10, &CSMNode::scanCallback, this);
  posePublisher_  = nh.advertise<geometry_msgs::Pose2D>(poseTopic_, 10);
}

CSMNode::~CSMNode()
{
  ROS_INFO("Destroying CanonicalScanMatcher node");
}

void CSMNode::getParams()
{
  ros::NodeHandle nh_private("~");

  if (!nh_private.getParam ("world_frame", worldFrame_))
    worldFrame_ = "world";
  if (!nh_private.getParam ("base_frame", baseFrame_))
    baseFrame_ = "base_link";
  if (!nh_private.getParam ("publish_tf", publishTf_))
    publishTf_ = true;
  if (!nh_private.getParam ("publish_pose", publishPose_))
    publishPose_ = true;

  // **** CSM parameters

  // Maximum angular displacement between scans
  if (!nh_private.getParam ("max_angular_correction_deg", input_.max_angular_correction_deg))
    input_.max_angular_correction_deg = 45.0;

  // Maximum translation between scans (m)
  if (!nh_private.getParam ("max_linear_correction",      input_.max_linear_correction))
    input_.max_linear_correction = 0.50;

  // Maximum ICP cycle iterations
  if (!nh_private.getParam ("max_iterations", input_.max_iterations))
    input_.max_iterations = 2;

  // A threshold for stopping (m)
  if (!nh_private.getParam ("epsilon_xy", input_.epsilon_xy))
    input_.epsilon_xy = 0.001;

  // A threshold for stopping (rad)
  if (!nh_private.getParam ("epsilon_theta", input_.epsilon_theta))
    input_.epsilon_theta = 0.00872;

  // ???
  if (!nh_private.getParam ("max_correspondence_dist", input_.max_correspondence_dist))
    input_.max_correspondence_dist = 0.2;

  // Noise in the scan (m)
  if (!nh_private.getParam ("sigma", input_.sigma))
    input_.sigma = 0.010;

  // Use smart tricks for finding correspondences.
  if (!nh_private.getParam ("use_corr_tricks", input_.use_corr_tricks))
    input_.use_corr_tricks = 1;

  // Restart: Restart if error is over threshold
  if (!nh_private.getParam ("restart", input_.restart))
    input_.restart = 0;

  // Restart: Threshold for restarting
  if (!nh_private.getParam ("restart_threshold_mean_error", input_.restart_threshold_mean_error))
    input_.restart_threshold_mean_error = 0.01;

  // Restart: displacement for restarting. (m)
  if (!nh_private.getParam ("restart_dt", input_.restart_dt))
    input_.restart_dt = 1.0;

  // Restart: displacement for restarting. (rad)
  if (!nh_private.getParam ("restart_dtheta", input_.restart_dtheta))
    input_.restart_dtheta = 0.1;

  // Max distance for staying in the same clustering
  if (!nh_private.getParam ("clustering_threshold", input_.clustering_threshold))
    input_.clustering_threshold = 0.25;

  // Number of neighbour rays used to estimate the orientation
  if (!nh_private.getParam ("orientation_neighbourhood", input_.orientation_neighbourhood))
    input_.orientation_neighbourhood = 10;

  // If 0, it's vanilla ICP
  if (!nh_private.getParam ("use_point_to_line_distance", input_.use_point_to_line_distance))
    input_.use_point_to_line_distance = 1;

  // Discard correspondences based on the angles
  if (!nh_private.getParam ("do_alpha_test", input_.do_alpha_test))
    input_.do_alpha_test = 0;

  // ???
  if (!nh_private.getParam ("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
    input_.do_alpha_test_thresholdDeg = 20.0;

  // ???
  if (!nh_private.getParam ("outliers_maxPerc", input_.outliers_maxPerc))
    input_.outliers_maxPerc = 0.90;

  // ??? 
  if (!nh_private.getParam ("outliers_adaptive_order", input_.outliers_adaptive_order))
    input_.outliers_adaptive_order = 0.7;

  // ???
  if (!nh_private.getParam ("outliers_adaptive_mult", input_.outliers_adaptive_mult))
    input_.outliers_adaptive_mult = 2.0;

  // ???
  if (!nh_private.getParam ("do_visibility_test", input_.do_visibility_test))
    input_.do_visibility_test = 0;

  // no two points in laser_sens can have the same corr.
  if (!nh_private.getParam ("outliers_remove_doubles", input_.outliers_remove_doubles))
    input_.outliers_remove_doubles = 1;

  // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
  if (!nh_private.getParam ("do_compute_covariance", input_.do_compute_covariance))
    input_.do_compute_covariance = 0;

  // Checks that find_correspondences_tricks gives the right answer
  if (!nh_private.getParam ("debug_verify_tricks", input_.debug_verify_tricks))
    input_.debug_verify_tricks = 0;

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the 
  // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
  if (!nh_private.getParam ("use_ml_weights", input_.use_ml_weights))
    input_.use_ml_weights = 0;

  // If 1, the field 'readings_sigma' in the second scan is used to weight the 
  // correspondence by 1/sigma^2
  if (!nh_private.getParam ("use_sigma_weights", input_.use_sigma_weights))
    input_.use_sigma_weights = 0;
}

void CSMNode::scanCallback (const sensor_msgs::LaserScan& scan)
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

  geometry_msgs::Pose2D p;
  p.x = 0;
  p.y = 0;
  p.theta = 0;

  LDP currLDPScan = rosToLDPScan(scan, p);

  input_.laser_ref  = prevLDPScan_;
  input_.laser_sens = currLDPScan;
  input_.first_guess[0] = 0;
  input_.first_guess[1] = 0;
  input_.first_guess[2] = 0;

  sm_icp(&input_, &output_);

  if (!output_.valid) 
  {
    printf("************************\n");
    return;
  }

  btTransform change;
  change.setOrigin(btVector3(output_.x[0], output_.x[1], 0.0));
  btQuaternion q;
  q.setRPY(0.0, 0.0, output_.x[2]);
  change.setRotation(q);

  prevWorldToBase_ = prevWorldToBase_ * change;

  if (publishTf_  ) publishTf  (prevWorldToBase_, scan.header.stamp);
  if (publishPose_) publishPose(prevWorldToBase_);

  // **** swap old and new

  ld_free(prevLDPScan_);

  prevLDPScan_ = currLDPScan;

  // **** timing information - needed for profiling only

  gettimeofday(&end, NULL);
  double dur = ((end.tv_sec   * 1000000 + end.tv_usec  ) - 
                (start.tv_sec * 1000000 + start.tv_usec)) / 1000.0;
  totalDuration_ += dur;
  double ave = totalDuration_/scansCount_;

  ROS_INFO("dur:\t %.3f ms \t ave:\t %.3f ms,\t%d", dur, ave, output_.iterations);
}

LDP CSMNode::rosToLDPScan(const sensor_msgs::LaserScan& scan, 
                          const geometry_msgs::Pose2D& laserPose)
{
  unsigned int n = scan.ranges.size();
  
  LDP ld = ld_alloc_new(n);
	
  for (int i = 0; i < n; i++)
  {
    ld->readings[i] = scan.ranges[i];
    ld->theta[i]    = scan.angle_min + (double)i * scan.angle_increment;

    if (scan.ranges[i] == 0)  ld->valid[i] = 0;
    else                      ld->valid[i] = 1;
      
    ld->cluster[i]  = -1;
  }

  ld->min_theta = ld->theta[0];
  ld->max_theta = ld->theta[n-1];

  ld->odometry[0] = laserPose.x;
  ld->odometry[1] = laserPose.y;
  ld->odometry[2] = laserPose.theta;

	return ld;
}

bool CSMNode::initialize(const sensor_msgs::LaserScan& scan)
{
  printf("Initializing...\n");
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

  // **** create the first pm scan from the laser scan message

  geometry_msgs::Pose2D p;
  p.x = 0;
  p.y = 0;
  p.theta = 0;
  prevLDPScan_ = rosToLDPScan(scan, p);

  double x, y, theta, temp;
  x = baseToLaser_.getOrigin().getX();
  y = baseToLaser_.getOrigin().getY();
  baseToLaser_.getBasis().getRPY(temp, temp, theta);

  input_.laser[0] = x; 
  input_.laser[1] = y; 
  input_.laser[2] = theta; 

  input_.min_reading = scan.range_min;
  input_.max_reading = scan.range_max;

  printf("Done Initializing...\n");
  return true;
}

void CSMNode::publishTf(const btTransform& transform, 
                                  const ros::Time& time)
{
  tf::StampedTransform transformMsg (transform, time, worldFrame_, baseFrame_);
  tfBroadcaster_.sendTransform (transformMsg);
}

void CSMNode::publishPose(const btTransform& transform)
{
  geometry_msgs::Pose2D pose;
  tfToPose2D(transform, pose);

  posePublisher_.publish(pose);
}


void CSMNode::tfToPose2D(const btTransform& t, geometry_msgs::Pose2D& pose)
{
  btMatrix3x3 m(t.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  pose.x = t.getOrigin().getX();
  pose.y = t.getOrigin().getY();
  pose.theta = yaw;
}
