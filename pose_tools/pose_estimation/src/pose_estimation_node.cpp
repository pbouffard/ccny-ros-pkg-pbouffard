#include "pose_estimation/pose_estimation_node.h"

PoseEstimationNode::PoseEstimationNode()
{
	// **** set up filter

	arInitialized      = false;
  imuInitialized_    = false;
  heightInitialized_ = false;

  // inital values

  velX = 0.0;
  velY = 0.0;
  velZ = 0.0;

  rollPrev  = 0.0;
  pitchPrev = 0.0;
  yawPrev   = 0.0;

  roll_ = 0;
  pitch_ = 0;
  yaw_ = 0;

  x_ = 0;
  y_ = 0;
  z_ = 0;

	// **** set up ROS

	ROS_INFO("Starting PoseEstimationNode"); 
  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh;

  // paramters
  double freq;
  if (!nh_private.getParam ("freq", freq))
    freq = 40.0;
  if (!nh_private.getParam ("base_frame", baseFrame_))
    baseFrame_ = "base_link";
  if (!nh_private.getParam ("world_frame", worldFrame_))
    worldFrame_ = "world";

	// set up timer
  timer_ = nh_private.createTimer(ros::Duration(1.0/std::max(freq,1.0)), &PoseEstimationNode::spin, this);

  // subscribe to imu messages
  imuSubscriber_ = nh.subscribe(imuTopicName_, 100,  &PoseEstimationNode::imuCallback, this);

  // subscribe to height messages
  heightSubscriber_ = nh.subscribe(heightTopicName_, 100,  &PoseEstimationNode::heightCallback, this);

  // subscribe to laser scanmatch messages
  pose2DSubscriber_ = nh.subscribe(pose2DTopicName_, 100,  &PoseEstimationNode::pose2DCallback, this);

  // subscribe to ar toolkit messages
  //arToolkitSubscriber = nh.subscribe(arToolkitTopicName, 100,  &PoseEstimationNode::arToolkitCallback, this);
	//ROS_INFO("Subscribing to topic: %s", arToolkitTopicName); 
}

PoseEstimationNode::~PoseEstimationNode()
{
  ROS_INFO("Destroying PoseEstimationNode");
}

// callback function for pose2D data
void PoseEstimationNode::pose2DCallback(const geometry_msgs::Pose2DConstPtr& pose2D)
{
  ROS_DEBUG("Pose2DCallback");
/*
  btTransform worldToBase;
  worldToBase.setOrigin(btVector3(pose2D->x, pose2D->y, 0.0));
  btQuaternion q;
  q.setRPY(0, 0, pose2D->theta);
  worldToBase.setRotation(q);

  btMatrix3x3 m(worldToBase.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  //TODO - what base should angels be really in?
  fixAngle(yaw,   360.0);

  BFL::ColumnVector measurement(MEAS_SIZE);

  measurement(1) = worldToBase.getOrigin().getX();
  measurement(2) = worldToBase.getOrigin().getY();
  measurement(3) = 0.0;
  measurement(4) = 0.0;
  measurement(5) = 0.0;
  measurement(6) = 0.0;

  BFL::ColumnVector measNoiseMu(MEAS_SIZE);
  measNoiseMu(1) = 0.0;
  measNoiseMu(2) = 0.0;
  measNoiseMu(3) = 0.0;
  measNoiseMu(4) = 0.0;
  measNoiseMu(5) = 0.0;
  measNoiseMu(6) = 0.0;

  BFL::SymmetricMatrix measNoiseCov(MEAS_SIZE);
  measNoiseCov = 0.0;

  measNoiseCov(1,1) = 0.0001;
  measNoiseCov(2,2) = 0.0001;
  measNoiseCov(3,3) = 99999.0;
  measNoiseCov(4,4) = 99999.0;
  measNoiseCov(5,5) = 99999.0;
  measNoiseCov(6,6) = 99999.0;

  // update the filter

  filter_.updateFromMeasurement(measurement, measNoiseMu, measNoiseCov);
*/
  x_ = pose2D->x;
  y_ = pose2D->y;

  //printf("x, y: %f, %f\n", pose2D->x, pose2D->y);

  yaw_ = pose2D->theta;
}

// callback function for imu data
void PoseEstimationNode::imuCallback(const sensor_msgs::ImuConstPtr& imuData)
{
  boost::mutex::scoped_lock imuLock(imuMutex_);
  ros::Duration imuDeltaT;

	if (!imuInitialized_)
	{
    ROS_INFO("Received first imu reading");

		imuInitialized_ = true;
		
    imuDeltaT = ros::Duration(0.0);

    setImuToBaseTf(imuData);
  }
  else
  {
    ROS_DEBUG("Received imu reading");

	  imuDeltaT = imuData->header.stamp - lastIMUReadingTime_;
    lastIMUReadingTime_ = imuData->header.stamp;
  }

  //  set position to fake 0, 0, 0 - only interested in orientation

  btTransform worldToImu;
  worldToImu.setOrigin(btVector3(0.0, 0.0, 0.0));
  worldToImu.setRotation(btQuaternion(imuData->orientation.x,
                                      imuData->orientation.y,
                                      imuData->orientation.z,
                                      imuData->orientation.w));

  btTransform worldToBase = worldToImu * imuToBase_;

  btMatrix3x3 m(worldToBase.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll,pitch, yaw);

  //printf("PEN: %f %f %f\n",roll,pitch,yaw);  

  //TODO - what base should angels be really in?
/*
  fixAngle(roll,  360.0);
  fixAngle(pitch, 360.0);
  fixAngle(yaw,   360.0);

  BFL::ColumnVector measurement(MEAS_SIZE);

  measurement(1) = 0.0;
  measurement(2) = 0.0;
  measurement(3) = 0.0;
  measurement(4) = roll;
  measurement(5) = pitch;
  measurement(6) = yaw;

  BFL::ColumnVector measNoiseMu(MEAS_SIZE);
  measNoiseMu(1) = 0.0;
  measNoiseMu(2) = 0.0;
  measNoiseMu(3) = 0.0;
  measNoiseMu(4) = 0.0;
  measNoiseMu(5) = 0.0;
  measNoiseMu(6) = 0.0;

  BFL::SymmetricMatrix measNoiseCov(MEAS_SIZE);
  measNoiseCov = 0.0;

  measNoiseCov(1,1) = 99999.0;
  measNoiseCov(2,2) = 99999.0;
  measNoiseCov(3,3) = 99999.0;
  measNoiseCov(4,4) = 0.0001;
  measNoiseCov(5,5) = 0.0001;
  measNoiseCov(6,6) = 0.0001;
*/
  // update the filter

  //filter_.updateFromMeasurement(measurement, measNoiseMu, measNoiseCov);
    roll_ = roll;
    pitch_ = pitch;
  //  yaw_ = yaw;
}

void PoseEstimationNode::heightCallback(const asctec_msgs::HeightConstPtr& heightData)
{
  boost::mutex::scoped_lock heightLock(heightMutex_);

  if (!heightInitialized_)
	{
		heightInitialized_ = true;

    ROS_DEBUG("Received first height reading");
  }
  else
  {
    ROS_DEBUG("Received height reading");

    // update filter with given variance on z, and variance on all other = 99999
/*
    BFL::ColumnVector measurement(MEAS_SIZE);
    measurement = 0.0;

    measurement(1) = 0.0;
    measurement(2) = 0.0;
    measurement(3) = heightData->height;
    measurement(4) = 0.0;
    measurement(5) = 0.0;
    measurement(6) = 0.0;

    //TODO - use covariance from message instead of hardcoded

    BFL::ColumnVector measNoiseMu(MEAS_SIZE);
    measNoiseMu = 0.0;
  
    BFL::SymmetricMatrix measNoiseCov(MEAS_SIZE);
    measNoiseCov = 0.0;

    measNoiseCov(1,1) = 99999.0;
    measNoiseCov(2,2) = 99999.0;  
    measNoiseCov(3,3) = 0.0001;
    measNoiseCov(4,4) = 99999.0;
    measNoiseCov(5,5) = 99999.0;
    measNoiseCov(6,6) = 99999.0;
*/
    //filter_.updateFromMeasurement(measurement, measNoiseMu, measNoiseCov);

    z_ = heightData->height;
  }
}

// callback function for arToolkit data
void PoseEstimationNode::arToolkitCallback(const geometry_msgs::PoseConstPtr& arData)
{
/*
  boost::mutex::scoped_lock arToolkitlock(arToolkitMutex);

	if (arInitialized)
	{
    ROS_INFO("Received arPose");

    // TODO - transform from camera frame to correct frame ?
    //      - use first reading instead of previous reading when correcting

		// create pose from ar data

    btTransform arPose;
		getBtTransformFromPose(arPose, arData);

    // create measurement vector from pose differences

    ColumnVector measurement(MEAS_SIZE);

    toColumnVector(measurement, arPose, arPosePrev);
  
    // update filter with pose vector

    filter_.updateFromMeasurement(measurement);

    ROS_DEBUG("MEAS: dx: %f, dy: %f, dz: %f, dr: %f, dp: %f, dy: %f\n", 
              measurement(1), measurement(2), measurement(3), 
              measurement(4) * SIMD_DEGS_PER_RAD, 
              measurement(5) * SIMD_DEGS_PER_RAD, 
              measurement(6) * SIMD_DEGS_PER_RAD); 
	}
	else 
	{
    ROS_INFO("Received first arPose");

		arInitialized = true;

    //create inital ar pose
		
		getBtTransformFromPose(arPosePrev, arData);
	}
*/
}

void PoseEstimationNode::sendFakeMotion()
{
  BFL::ColumnVector inputNoiseMu(INPUT_SIZE);
  inputNoiseMu = 0.0;
    
  BFL::SymmetricMatrix inputNoiseCov(INPUT_SIZE);
  inputNoiseCov = 0.0;

  inputNoiseCov(1,1) = 1.0;
  inputNoiseCov(2,2) = 1.0;
  inputNoiseCov(3,3) = 1.0;
  inputNoiseCov(4,4) = 1.0;
  inputNoiseCov(5,5) = 1.0;
  inputNoiseCov(6,6) = 1.0;
  
  BFL::ColumnVector input(INPUT_SIZE);
  input(1) = 0.0;
  input(2) = 0.0;
  input(3) = 0.0;
  input(4) = 0.0;
  input(5) = 0.0;
  input(6) = 0.0;

	filter_.updateFromMotion(input, inputNoiseMu, inputNoiseCov);
}

// filter loop
void PoseEstimationNode::spin(const ros::TimerEvent& e)
{
  ROS_DEBUG("Pose estimation: spin");

  boost::mutex::scoped_lock imuLock(imuMutex_);
  boost::mutex::scoped_lock arToolkitlock(arToolkitMutex);
  boost::mutex::scoped_lock heightLock(heightMutex_);
  boost::mutex::scoped_lock pose2Dlock(pose2DMutex_);

/*
  sendFakeMotion();
 
	BFL::ColumnVector mean(STATE_SIZE);
	filter_.getPosteriorMean(mean);

	BFL::SymmetricMatrix covariance(STATE_SIZE);
	filter_.getPosteriorCovariance(covariance);

	//printState(mean, covariance);

  // **** create message from column vector

  btVector3 pos;
  pos.setValue(mean(1), mean(2), mean(3));

  btQuaternion rot;
  rot.setRPY(mean(4), mean(5), mean(6));
*/

  btVector3 pos;
  pos.setValue(x_, y_, z_);

  btQuaternion rot;
  rot.setRPY(roll_, pitch_, yaw_);

  // **** publish odometry transform

  tf::Transform navToBaseLink;

  navToBaseLink.setOrigin(pos);
  navToBaseLink.setRotation(rot);

  odomBroadcaster_.sendTransform(tf::StampedTransform(
    navToBaseLink, ros::Time::now(), worldFrame_, baseFrame_));
}

void PoseEstimationNode::printState(const BFL::ColumnVector& m, const BFL::SymmetricMatrix& c)
{
	ROS_INFO("STATE: mean [%f %f %f %f %f %f]", m(1), m(2), m(3), m(4), m(5), m(6));
	ROS_INFO("     : cov  [%f %f %f %f %f %f]", c(1,1), c(2,2), c(3,3), c(4,4), c(5,5), c(6,6));
}

void PoseEstimationNode::getBtTransformFromPose(btTransform& t, const geometry_msgs::PoseConstPtr& pose)
{
	btVector3 pos(pose->position.x, 
							  pose->position.y, 
							  pose->position.z);

	btQuaternion rot(pose->orientation.x,
									 pose->orientation.y,
									 pose->orientation.z,
									 pose->orientation.w);

  t.setOrigin(pos);
  t.setRotation(rot);
}

void PoseEstimationNode::fixAngle(double& da, double base)
{
  // normalize da to (-180 deg, 180 deg]

  while (da > base * SIMD_RADS_PER_DEG) 
    da -= base * SIMD_RADS_PER_DEG;
  while (da <= -base * SIMD_RADS_PER_DEG)
    da += base * SIMD_RADS_PER_DEG;
}

void PoseEstimationNode::setImuToBaseTf(const sensor_msgs::ImuConstPtr& imuMsg)
{
  tf::StampedTransform imuToBaseTf;
  try
  {
     tfListener_.waitForTransform(imuMsg->header.frame_id, baseFrame_, imuMsg->header.stamp, ros::Duration(2.0));
     tfListener_.lookupTransform (imuMsg->header.frame_id, baseFrame_, imuMsg->header.stamp, imuToBaseTf);
  }
  catch (tf::TransformException ex)
  {
    // transform unavailable - skip scan
    ROS_WARN("PoseEstimation: Could not cache initial static transform.(%s)", ex.what());
  }
  imuToBase_ = imuToBaseTf;
}
