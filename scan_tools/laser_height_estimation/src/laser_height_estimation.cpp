#include "laser_height_estimation/laser_height_estimation.h"

using namespace MatrixWrapper;
using namespace BFL;

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
  heightInitialized_  = false;
  floorHeight_        = 0.0;

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // **** parameters
  
  if (!nh_private.getParam ("base_frame", baseFrame_))
    baseFrame_ = "base_link";
  if (!nh_private.getParam ("world_frame", worldFrame_))
    worldFrame_ = "world";
  if (!nh_private.getParam ("min_values", minValues_))
    minValues_ = 5;
  if (!nh_private.getParam ("max_stdev", maxStdev_))
    maxStdev_ = 0.10;
  if (!nh_private.getParam ("max_height_jump", maxHeightJump_))
    maxHeightJump_ = 0.25;
  if (!nh_private.getParam ("use_kf", useKF_))
    useKF_ = false;
  if (!nh_private.getParam ("initial_height", initialHeight_))
    initialHeight_ = 0.14;
  
  // **** subscribers

  scanFilterSub_ = new message_filters::Subscriber <sensor_msgs::LaserScan> (nh, scanTopic_, 10);
  scanFilter_ = new tf::MessageFilter <sensor_msgs::LaserScan> (*scanFilterSub_, tfListener_, baseFrame_, 10);
  scanFilter_->registerCallback (boost::bind (&LaserHeightEstimation::scanCallback, this, _1));
  scanFilter_->setTolerance (ros::Duration(tfTolerance_));

  pHeightSubscriber_ = nh.subscribe(pHeightTopic_, 10, &LaserHeightEstimation::pHeightCallback, this);

  // **** publishers

  heightPublisher_ = nh_private.advertise<asctec_msgs::Height>(heightTopic_, 10);

  if (useKF_)
    timer_ = nh_private.createTimer(ros::Duration(1.0/20.0), &LaserHeightEstimation::spin, this);

  initializeFilter();
}

LaserHeightEstimation::~LaserHeightEstimation()
{
	ROS_INFO("Destroying LaserHeightEstimation"); 
}

void LaserHeightEstimation::initializeFilter()
{
  if (useKF_)
  {
    ROS_INFO("Laser Height Estimation: Using Kalman Filter");
    ColumnVector prior_Mu(1);
    prior_Mu(1) = initialHeight_;

    SymmetricMatrix prior_Cov(1);
    prior_Cov(1,1) = 1000000.0;

    Gaussian prior(prior_Mu, prior_Cov);

    filter_ = new ExtendedKalmanFilter(&prior);
  }
  else
  {
    ROS_INFO("Laser Height Estimation: NOT Using Kalman Filter");
    prevHeight_ = initialHeight_;
  }
}

void LaserHeightEstimation::pHeightCallback (const asctec_msgs::Height& heightMsg)
{
  //ROS_INFO("** pHeight msg received");

  if (!heightInitialized_)
  {
    lastHeightMsg_ = heightMsg;
     
    heightInitialized_ = true;
    return;
  }

  if (useKF_)
  {
    double v  = heightMsg.height - lastHeightMsg_.height;

    filterMutex_.lock();

    Pdf<ColumnVector> * posterior = filter_->PostGet();
    ColumnVector state = posterior->ExpectedValueGet();

    Matrix H_height(1,1);
    H_height(1,1) = 1;

    ColumnVector measNoise_Mu_height(1);
    measNoise_Mu_height(1) = 0.0;

    SymmetricMatrix measNoise_Cov_height(1);
    measNoise_Cov_height(1,1) = std::pow(10.0, 2);        // variance of laser

    ColumnVector measurement(1);
    measurement(1) = state(1) + v;       // height reading in message

    Gaussian measurement_Uncertainty_height(measNoise_Mu_height, measNoise_Cov_height);
    LinearAnalyticConditionalGaussian meas_pdf_height(H_height, measurement_Uncertainty_height);
    LinearAnalyticMeasurementModelGaussianUncertainty meas_model_height(&meas_pdf_height);

    // **** update the filter

    filter_->Update(&meas_model_height, measurement);

    posterior = filter_->PostGet();
    state = posterior->ExpectedValueGet();

    asctec_msgs::Height stateMsg;
    stateMsg.header = heightMsg.header;
    stateMsg.height = state(1);

    heightPublisher_.publish(stateMsg);

    filterMutex_.unlock();
  }

  lastHeightMsg_ = heightMsg;
}

void LaserHeightEstimation::spin(const ros::TimerEvent& e)
{
  Matrix A(1,1);
  A(1, 1) = 1.0;

  Matrix B(1,1);
  B(1, 1) = 1;

  vector<Matrix> AB(2);
  AB[0] = A;
  AB[1] = B;

  // **** create SYSTEM MODEL

  ColumnVector sysNoise_Mu(1);  
  sysNoise_Mu(1) = 0;

  SymmetricMatrix sysNoise_Cov(1); 
  sysNoise_Cov(1,1) = pow(1.00, 2);

  ColumnVector input(1);
  input(1) = 0.0;                 
     
  Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
  LinearAnalyticConditionalGaussian sys_pdf(AB, system_Uncertainty);
  LinearAnalyticSystemModelGaussianUncertainty sys_model(&sys_pdf);

  // **** update the filter

  filterMutex_.lock();
  filter_->Update(&sys_model, input);
  filterMutex_.unlock();
}
void LaserHeightEstimation::scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  if (!initialized_)
  {
    // if this is the first scan, lookup the static base to lase tf
    // if tf is not available yet, skip this scan
    if (!setBaseToLaserTf(scan)) return;
    initialized_ = true;
  }

  // **** get required transforms

  btTransform worldToBase;
  if (!getWorldToBaseTf(scan, worldToBase)) return;
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
      
      double diff = basePose.getZ() - p.getZ();

      values.push_back(diff);
    }
  }

  // **** get mean and standard dev

  double rawHeight, stdev;
  getStats(values, rawHeight, stdev);
 
  if (values.size() < minValues_)
  {
    ROS_WARN("Not enough valid values to determine height, skipping (%d collected, %d needed)",
      values.size(), minValues_);
    return;
  }

  if (stdev > maxStdev_)
  {
    ROS_WARN("Stdev of height readings too big to determine height, skipping (stdev is %f, max is %f)",
      stdev, maxStdev_);
    return;
  }

  // **** estimate height
  
  double prevHeight;
  Pdf<ColumnVector> * posterior;
  ColumnVector state;

  if (useKF_)
  {
    posterior = filter_->PostGet();
    state = posterior->ExpectedValueGet();
    prevHeight = state(1);
  }
  else
    prevHeight = prevHeight_;
  
  double height = rawHeight + floorHeight_;
  double heightJump = prevHeight - height;

  if (fabs(heightJump) > maxHeightJump_)
  {
    ROS_INFO("Laser Height Estimation: Floor Discontinuity detected");
    floorHeight_ += heightJump;
    height += heightJump;
  }

  asctec_msgs::Height stateMsg;
  stateMsg.header.stamp = ros::Time::now();

  if (useKF_)
  {
    // **** create MEASUREMENT MODEL HEIGHT (from laser)

    Matrix H_laser(1,1);
    H_laser(1,1) = 1;

    ColumnVector measNoise_Mu_laser(1);
    measNoise_Mu_laser(1) = 0.0;

    SymmetricMatrix measNoise_Cov_laser(1);
    measNoise_Cov_laser(1,1) = std::pow(0.01, 2);        // variance of laser

    ColumnVector measurement(1);
    measurement(1) = height; 

    Gaussian measurement_Uncertainty_laser(measNoise_Mu_laser, measNoise_Cov_laser);
    LinearAnalyticConditionalGaussian meas_pdf_laser(H_laser, measurement_Uncertainty_laser);
    LinearAnalyticMeasurementModelGaussianUncertainty meas_model_laser(&meas_pdf_laser);

    // **** update the filter

    filterMutex_.lock();
    filter_->Update(& meas_model_laser, measurement);

    posterior = filter_->PostGet();
    state = posterior->ExpectedValueGet();

    stateMsg.height       = state(1);

    filterMutex_.unlock();
  }
  else
  {
    stateMsg.height = height;
    prevHeight_ = height;
  }

  // **** publish height message

  heightPublisher_.publish(stateMsg);
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

bool LaserHeightEstimation::getWorldToBaseTf(const sensor_msgs::LaserScanConstPtr& scan,
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
    ROS_WARN("Laser Height Estimation: Transform unavailable, skipping scan (%s)", ex.what());
    return false;
  }
  worldToBase = worldToBaseTf;  
  return true;
}
