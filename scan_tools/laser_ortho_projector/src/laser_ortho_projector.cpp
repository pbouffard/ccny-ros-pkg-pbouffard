#include "laser_ortho_projector/laser_ortho_projector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_ortho_projector");
  LaserOrthoProjector filter;
  ros::spin();
	return 0;
}

LaserOrthoProjector::LaserOrthoProjector()
{
	ROS_INFO("Starting LaserOrthoProjector"); 

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // **** paramters

  if(!nh_private.getParam("world_frame", worldFrame_))
    worldFrame_ = "world";
  if(!nh_private.getParam("laser_ortho_frame", laserOrthoFrame_))
    laserOrthoFrame_ = "laser_ortho";

  if(!nh_private.getParam("scan_topic", scanTopic_))
    scanTopic_ = "scan";
  if(!nh_private.getParam("scan_ortho_topic", scanOrthoTopic_))
    scanOrthoTopic_ = "scan_ortho";
  if(!nh_private.getParam("cloud_ortho_topic", cloudOrthoTopic_))
    cloudOrthoTopic_ = "cloud_ortho";

  if(!nh_private.getParam("publish_cloud", publishCloud_))
    publishCloud_ = true;
  if(!nh_private.getParam("tf_tolerace", tfTolerance_))
    tfTolerance_ = 0.01;

  // **** subscribe to laser scan messages

  scanFilterSub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, scanTopic_, 10);
  scanFilter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scanFilterSub_, tfListener_, worldFrame_, 10);
  scanFilter_->registerCallback(boost::bind(&LaserOrthoProjector::scanCallback, this, _1));
  scanFilter_->setTolerance(ros::Duration(tfTolerance_));
	ROS_INFO("LaserOrthoProjector listening on topic: %s", scanTopic_.c_str()); 

  // **** advertise orthogonal scan
  scanPublisher_ = nh_private.advertise<laser_ortho_projector::LaserScanWithAngles>(scanOrthoTopic_, 10);
	ROS_INFO("LaserOrthoProjector publishing on topic: %s", scanOrthoTopic_.c_str()); 

  // **** advertise point cloud representation of orthogonal scan
  cloudPublisher_ = nh_private.advertise<sensor_msgs::PointCloud>(cloudOrthoTopic_, 10);
	ROS_INFO("LaserOrthoProjector publishing on topic: %s", cloudOrthoTopic_.c_str()); 
}

LaserOrthoProjector::~LaserOrthoProjector()
{
  delete scanFilterSub_;
  delete scanFilter_;
}

void LaserOrthoProjector::scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  ROS_DEBUG("Received scan");

  // **** obtain transform between world and laser frame
  
  try
  {
    tfListener_.lookupTransform (worldFrame_, scan->header.frame_id, scan->header.stamp, worldToLaser_);
  }
  catch (tf::TransformException ex)
  {
    // transform unavailable - skip scan
    ROS_WARN("Skipping scan (%s)", ex.what());
    return;
  }

  btMatrix3x3 m(worldToLaser_.getRotation());
  m.getRPY(roll_, pitch_, yaw_);

  // **** calculate and publish transform bewtween world and laserOrtho frames

  publishWorldToLaserOrthoTf(scan->header.stamp);

  // **** build and publish projected scan

  laser_ortho_projector::LaserScanWithAngles scanOrtho;
  publishScanOrtho(scan, scanOrtho);
  
  // **** build and publish point from projected cloud scan

  if (publishCloud_) publishCloudOrtho(scanOrtho);
}

void LaserOrthoProjector::publishWorldToLaserOrthoTf(ros::Time time)
{
  // **** calculate a transform by throwing away the roll, pitch, and z 

  btTransform t;

  btQuaternion rotation;
  rotation.setRPY(0.0, 0.0, yaw_);
  t.setRotation(rotation);

  btVector3 origin;
  origin.setValue(worldToLaser_.getOrigin().getX(), worldToLaser_.getOrigin().getY(), 0.0);
  t.setOrigin(origin);

  tf::StampedTransform worldToLaserOrtho(t, time, worldFrame_, laserOrthoFrame_);
  tfBroadcaster_.sendTransform(worldToLaserOrtho);
}

void LaserOrthoProjector::publishScanOrtho(const sensor_msgs::LaserScanConstPtr& scan,
                                           laser_ortho_projector::LaserScanWithAngles& scanOrtho)
{
  // **** set header and misc information

  scanOrtho.header.frame_id = laserOrthoFrame_;
  scanOrtho.header.stamp    = scan->header.stamp;
  scanOrtho.time_increment  = scan->time_increment;
  scanOrtho.scan_time       = scan->scan_time;
  scanOrtho.range_min       = scan->range_min;
  scanOrtho.range_max       = scan->range_max;
  scanOrtho.intensities     = scan->intensities;

  scanOrtho.angles.resize(scan->ranges.size());
  scanOrtho.ranges.resize(scan->ranges.size());

  // **** set the array of angles and array of readings

  // calculate a transform based on roll and pitch only

  btTransform t;
  btQuaternion rotation;
  rotation.setRPY(roll_, pitch_, 0.0);
  t.setRotation(rotation);
  t.setOrigin(btVector3(0.0, 0.0, 0.0));

  // calculate unit vector corresponing to 0-deg reading
  btVector3 p1 = t * btVector3(1.0, 0.0, 0.0);
  double x1 = p1.getX();
  double y1 = p1.getY();

  for (unsigned int i = 0;  i < scan->ranges.size(); i++)
  {
    double sinAngle = sin(scan->angle_min + i * scan->angle_increment);
    double cosAngle = cos(scan->angle_min + i * scan->angle_increment);

    // compute unit vector corresponding to the projected laser ray
    btVector3 p2 = t * btVector3(cosAngle, sinAngle, 0.0);

    double x2 = p2.getX();
    double y2 = p2.getY();

    // compute angle between unit vector, and 0-deg reading unit vector
    double cos_da = (x1*x2 + y1*y2)/sqrt((x1*x1 + y1*y1)*(x2*x2  + y2*y2));

    if (y2 >= 0.0) scanOrtho.angles[i] =  acos(cos_da);
    else           scanOrtho.angles[i] = -acos(cos_da);

    // scale the length of the unit vector by the range to get projected reading

    double p2_range= sqrt(x2*x2 + y2*y2);
    scanOrtho.ranges[i] = p2_range * scan->ranges[i];
  } 
  
  // **** publish the projected scan with angles

  scanPublisher_.publish(scanOrtho);
}

void LaserOrthoProjector::publishCloudOrtho(const laser_ortho_projector::LaserScanWithAngles& scanOrtho)
{
  // **** build the point cloud from the LaserScanWithAngles

  sensor_msgs::PointCloud cloud;
  cloud.header.stamp    = scanOrtho.header.stamp;
  cloud.header.frame_id = scanOrtho.header.frame_id;

  for (unsigned int i = 0; i < scanOrtho.ranges.size(); i++)
  {
    double angle = scanOrtho.angles[i];
    double range = scanOrtho.ranges[i];

    geometry_msgs::Point32 p;
    p.x = cos(angle)*range;
    p.y = sin(angle)*range;
    p.z = 0.0;
    
    cloud.points.push_back(p);
  }

  cloudPublisher_.publish(cloud);
}

