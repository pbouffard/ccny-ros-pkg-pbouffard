#ifndef POSE_ESTIMATION_POSE_ESTIMATION_NODE_H
#define POSE_ESTIMATION_POSE_ESTIMATION_NODE_H

// **** ROS includes
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <btBulletDynamicsCommon.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <asctec_msgs/Height.h>

// **** BFL includes

#include <filter/extendedkalmanfilter.h>

#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>

// **** Other includes
#include "pose_estimation/config.h"
#include "pose_estimation/pose_estimation_filter.h"

//using namespace std;

const std::string imuTopicName_      = "imu";
const std::string heightTopicName_   = "height";
const std::string arToolkitTopicName = "arToolkitPose";
const std::string pose2DTopicName_   = "pose2D";

class PoseEstimationNode
{

	private:

    double roll_, pitch_, yaw_;
    double x_, y_, z_;

    std::string baseFrame_;
    std::string worldFrame_;

    btTransform imuToBase_;

		// ros
		ros::Publisher posePublisher;
		ros::Subscriber imuSubscriber_, arToolkitSubscriber, heightSubscriber_;
    ros::Subscriber pose2DSubscriber_;
  	ros::Timer timer_;

    // transfor boradcasters
    tf::TransformBroadcaster odomBroadcaster_;
    tf::TransformListener tfListener_;

		// mutex
		boost::mutex imuMutex_, arToolkitMutex, heightMutex_, pose2DMutex_;

		// filter
		PoseEstimationFilter filter_;

		// measurements
		bool arInitialized, imuInitialized_, heightInitialized_;

    ros::Time lastIMUReadingTime_;

		btTransform arPosePrev;

    double velX, velY, velZ;

    double rollPrev, pitchPrev, yawPrev;

    double heightFirst;

    void sendFakeMotion();

    void setImuToBaseTf(const sensor_msgs::ImuConstPtr& imuMsg);

	  void imuCallback(const sensor_msgs::ImuConstPtr& imuData);

		void heightCallback(const asctec_msgs::HeightConstPtr& heightData);
  
    void pose2DCallback(const geometry_msgs::Pose2DConstPtr& pose2D);

		void arToolkitCallback(const geometry_msgs::PoseConstPtr& arData);

		static void getBtTransformFromPose(btTransform& t, const geometry_msgs::PoseConstPtr& pose);

		static void printState(const BFL::ColumnVector& m, const BFL::SymmetricMatrix& c);
    static void fixAngle(double& da, double base);

	public:

		PoseEstimationNode();
		virtual ~PoseEstimationNode();

		// the main filter loop that will be called periodically
		void spin(const ros::TimerEvent& e);
};

#endif // POSE_ESTIMATION_POSE_ESTIMATION_NODE_H
