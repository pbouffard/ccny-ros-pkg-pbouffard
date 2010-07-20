#ifndef ASCTEC_PROC_ASCTEC_PROC_H
#define ASCTEC_PROC_ASCTEC_PROC_H

#include <ros/ros.h>
#include <asctec_msgs/IMUCalcData.h>
#include <sensor_msgs/Imu.h>

const std::string imuCalcDataTopic_ = "/autopilot/IMU_CALCDATA";
const std::string imuTopic_         = "imu";

namespace asctec
{
class AsctecProc
{
  private:

    ros::Subscriber imuCalcDataSubscriber_;
    ros::Publisher  imuPublisher_;

    void imuCalcDataCallback(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg);

  public:

    AsctecProc();
    virtual ~AsctecProc();



};

} // end namespace asctec


#endif //ASCTEC_PROC_ASCTEC_PROC_H
