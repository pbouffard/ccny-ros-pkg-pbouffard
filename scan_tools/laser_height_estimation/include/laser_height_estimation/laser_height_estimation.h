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

// bfl
#include <filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>

const std::string pHeightTopic_ = "pressure_height";
const std::string scanTopic_    = "scan";
const std::string heightTopic_  = "height";

const double tfTolerance_ = 0.10;

class LaserHeightEstimation
{
	private:

    bool initialized_;
    ros::Timer timer_;
    bool useKF_;

    boost::mutex filterMutex_;
    BFL::ExtendedKalmanFilter* filter_;

    bool heightInitialized_;
    asctec_msgs::Height lastHeightMsg_;
    double floorHeight_;
    double prevHeight_;
    double initialHeight_;

    btTransform baseToLaser_;

    message_filters::Subscriber<sensor_msgs::LaserScan>* scanFilterSub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scanFilter_;

    tf::TransformListener tfListener_;
    geometry_msgs::PoseWithCovarianceStamped poseMsg_;

    // **** parameters
  
    std::string baseFrame_;
    std::string worldFrame_;
    int minValues_;
    double maxStdev_;
    double maxHeightJump_;

		// **** publishers & subscirbers

    ros::Subscriber imuSubscriber_;
    ros::Subscriber scanSubscriber_;
    ros::Subscriber pHeightSubscriber_;
		ros::Publisher  heightPublisher_;

    void initializeFilter();

		void scanCallback    (const sensor_msgs::LaserScanConstPtr& scan);
    void pHeightCallback (const asctec_msgs::Height& heightMsg);
    
    bool setBaseToLaserTf(const sensor_msgs::LaserScanConstPtr& scan);

    void getStats(const std::vector<double> values, double& ave, double& stdev);

    bool getWorldToBaseTf(const sensor_msgs::LaserScanConstPtr& scan,
                                btTransform& worldToLaser);

    void spin(const ros::TimerEvent& e);

	public:
  
    LaserHeightEstimation();
    virtual ~LaserHeightEstimation();
};

#endif // LASER_HEIGHT_ESTIMATION_LASER_HEIGHT_ESTIMATION_H

