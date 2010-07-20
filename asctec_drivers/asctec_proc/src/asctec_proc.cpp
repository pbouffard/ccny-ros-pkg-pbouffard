#include "asctec_proc/asctec_proc.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "AsctecProc");
  asctec::AsctecProc asctecProc;
  ros::spin();
	return 0;
}

namespace asctec
{

AsctecProc::AsctecProc()
{
  ROS_INFO("Starting AsctecProc"); 

  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh;

  // **** subscribe to point cloud messages
  imuCalcDataSubscriber_ = nh.subscribe(imuCalcDataTopic_, 10,  &AsctecProc::imuCalcDataCallback, this);

  // **** advertise published filtered pointcloud
  imuPublisher_ = nh_private.advertise<sensor_msgs::Imu>(imuTopic_, 10);

}

AsctecProc::~AsctecProc()
{
  ROS_INFO("Destroying AsctecProc"); 

}

void AsctecProc::imuCalcDataCallback(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg)
{
  ROS_INFO("imuCalcDataCallback");



}

} // end namespace asctec
