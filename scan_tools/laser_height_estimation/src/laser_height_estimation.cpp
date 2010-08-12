#include "laser_height_estimation/laser_height_estimation.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_height_estimation");
  LaserHeightEstimation laserHeightEstimation;
  ros::spin();
	return 0;
}

LaserHeightEstimation::LaserHeightEstimation()
{
	ROS_INFO("Starting LaserHeightEstimation"); 

  initialized_ = false;
  floorHeight_ = 0.0;

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // **** parameters
  
  if (!nh_private.getParam ("base_frame", baseFrame_))
    baseFrame_ = "base_link";
  if (!nh_private.getParam ("world_frame", worldFrame_))
    worldFrame_ = "world";

  // **** subscribers

  scanFilterSub_ = new message_filters::Subscriber <sensor_msgs::LaserScan> (nh, scanTopic_, 10);
  scanFilter_ = new tf::MessageFilter <sensor_msgs::LaserScan> (*scanFilterSub_, tfListener_, baseFrame_, 10);
  scanFilter_->registerCallback (boost::bind (&LaserHeightEstimation::scanCallback, this, _1));
  scanFilter_->setTolerance (ros::Duration(tfTolerance_));

  // **** publishers

  heightPublisher_ = nh_private.advertise<asctec_msgs::Height>(heightTopic_, 10);
}

LaserHeightEstimation::~LaserHeightEstimation()
{
	ROS_INFO("Destroying LaserHeightEstimation"); 
}

void LaserHeightEstimation::scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  ROS_DEBUG("Received scan");

  if (!initialized_)
  {
    // if this is the first scan, lookup the static base to lase tf
    // if tf is not available yet, skip this scan
    if (!setBaseToLaserTf(scan)) return;
  }

  tf::StampedTransform worldToBaseTf;
  try
  {
     tfListener_.lookupTransform (worldFrame_, baseFrame_, scan->header.stamp, worldToBaseTf);
  }
  catch (tf::TransformException ex)
  {
    // transform unavailable - skip scan
    ROS_WARN("Transform unavailable, skipping scan (%s)", ex.what());
    return;
  }
  btTransform worldToBase = worldToBaseTf;

  btTransform worldToLaser = worldToBase * baseToLaser_;

  btVector3 basePose  = worldToBase  * btVector3(0,0,0);

  // **** calculate
  
  double sum = 0.0;
  int validRanges = 0;

  for(unsigned int i = 0; i < scan->ranges.size(); i++)
  {
    if (scan->ranges[i] > scan->range_min && scan->ranges[i] < scan->range_max)
    {
      double angle = scan->angle_min + i * scan->angle_increment;
      btVector3 v(cos(angle)*scan->ranges[i], sin(angle)*scan->ranges[i], 0.0);
      btVector3 p = worldToLaser * v;
      
      double diff = basePose.getZ() -  p.getZ();

      validRanges++;
      sum += diff;
    }
  }

  // **** estimate height

  double rawHeight = sum / validRanges;
  double height;

  if (initialized_)
  {
    if (abs(rawHeight - prevHeight_) > 10)
      floorHeight_ += (prevHeight_ - rawHeight);
  }

  height = rawHeight - floorHeight_;
  prevHeight_ = height;

  // **** publish height message

  if (validRanges > 0)
  {
    asctec_msgs::Height heightMsg;

    heightMsg.height = height;
    heightMsg.height_variance = 0;
    heightPublisher_.publish(heightMsg);
  }

  initialized_ = true;
}

bool LaserHeightEstimation::setBaseToLaserTf(const sensor_msgs::LaserScanConstPtr& scan)
{
  // **** get transform 

  tf::StampedTransform baseToLaserTf;
  try
  {
     tfListener_.lookupTransform (baseFrame_, scan->header.frame_id, scan->header.stamp, baseToLaserTf);
  }
  catch (tf::TransformException ex)
  {
    // transform unavailable - skip scan
    ROS_WARN("LHE: Transform unavailable, skipping scan (%s)", ex.what());
    return false;
  }
  
  baseToLaser_ = baseToLaserTf;
  return true;
}
