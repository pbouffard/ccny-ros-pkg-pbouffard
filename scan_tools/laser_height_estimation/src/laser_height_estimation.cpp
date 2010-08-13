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

  initialized_        = false;
  haveFloorReference_ = false;
  floorHeight_        = 0.0;
  prevHeight_         = 0.0;

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
    initialized_ = true;
  }

  // **** get required transforms

  btTransform worldToBase;
  getWorldToBaseTf(scan, worldToBase);
  btVector3 basePose  = worldToBase  * btVector3(0,0,0);
  btTransform worldToLaser = worldToBase * baseToLaser_;

  // **** get vector of height values
  
  std::vector<double> values;
  for(unsigned int i = 0; i < scan->ranges.size(); i++)
  {
    if (scan->ranges[i] > scan->range_min && scan->ranges[i] < scan->range_max)
    {
      double angle = scan->angle_min + i * scan->angle_increment;
      btVector3 v(cos(angle)*scan->ranges[i], sin(angle)*scan->ranges[i], 0.0);
      btVector3 p = worldToLaser * v;
      
      double diff = basePose.getZ() -  p.getZ();

      values.push_back(diff);
    }
  }

  double rawHeight, stdev;
  getStats(values, rawHeight, stdev);
 
  ROS_INFO("Height: %f, %f, %f", rawHeight, prevHeight_, stdev);

  if (values.size() < 5 || stdev > 0.10)
  {
    ROS_WARN("Not enough information to determine height, skipping");
    return;
  }

  // **** estimate height
  
  if (!haveFloorReference_)
  {
    floorHeight_ = 0.0;
    haveFloorReference_ = true;
  }
  else
  {
    if (fabs(rawHeight - prevHeight_) > 0.05)
    {
      printf("*********************************************\n");
      floorHeight_ += (prevHeight_ - rawHeight);
    }
  }

  double height = rawHeight - floorHeight_;
  prevHeight_ = height;

  // **** publish height message

  if (values.size() > 0)
  {
    asctec_msgs::Height heightMsg;

    heightMsg.height = height;
    heightMsg.height_variance = 0;
    heightPublisher_.publish(heightMsg);
  }
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

void LaserHeightEstimation::getStats(const std::vector<double> values, double& ave, double& stdev)
{
  double sum   = 0.0;
  double sumsq = 0.0;

  for (size_t i = 0; i < values.size(); ++i)
    sum += values[i];

  ave = sum/values.size();

  for (size_t i = 0; i < values.size(); ++i)
    sumsq += (values[i] - ave) * (values[i] - ave);

  stdev = sqrt(sumsq/values.size());
}

void LaserHeightEstimation::getWorldToBaseTf(const sensor_msgs::LaserScanConstPtr& scan,
                                                    btTransform& worldToBase)
{
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
  worldToBase = worldToBaseTf;
}
