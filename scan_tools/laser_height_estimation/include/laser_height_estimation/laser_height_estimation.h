#ifndef LASER_HEIGHT_ESTIMATION_LASER_HEIGHT_ESTIMATION_H
#define LASER_HEIGHT_ESTIMATION_LASER_HEIGHT_ESTIMATION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <boost/thread/mutex.hpp>
#include <asctec_msgs/Height.h>

const std::string imuTopic_    = "imu";
const std::string scanTopic_   = "scan";
const std::string heightTopic_ = "height";

const double tfTolerance_ = 0.10;

class LaserHeightEstimation
{
	private:

    boost::mutex imuMutex_;
    sensor_msgs::Imu lastImuMsg_;

    bool initialized_;
    bool haveFloorReference_;
    double prevHeight_;
    double floorHeight_;

    btTransform baseToLaser_;

    message_filters::Subscriber<sensor_msgs::LaserScan>* scanFilterSub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scanFilter_;

    tf::TransformListener tfListener_;
    geometry_msgs::PoseWithCovarianceStamped poseMsg_;

    // parameters
  
    std::string baseFrame_;
    std::string worldFrame_;
    int minValues_;
    double maxStdev_;
    double maxHeightJump_;

		// publishers & subscirbers

    ros::Subscriber scanSubscriber_;
		ros::Publisher  heightPublisher_;

		void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);

    bool setBaseToLaserTf(const sensor_msgs::LaserScanConstPtr& scan);

    void getStats(const std::vector<double> values, double& ave, double& stdev);

    void getWorldToBaseTf(const sensor_msgs::LaserScanConstPtr& scan,
                                btTransform& worldToLaser);

	public:
  
    LaserHeightEstimation();
    virtual ~LaserHeightEstimation();
};

#endif // LASER_HEIGHT_ESTIMATION_LASER_HEIGHT_ESTIMATION_H

