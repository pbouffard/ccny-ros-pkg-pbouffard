#include "asctec_proc/asctec_proc.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "AsctecProc");
  asctec::AsctecProc asctecProc;
  ros::spin ();
  return 0;
}

namespace asctec
{

  AsctecProc::AsctecProc ()
  {
    ROS_INFO ("Starting AsctecProc");

    ros::NodeHandle nh_private ("~");
    ros::NodeHandle nh;

    // **** subscribe to point cloud messages
    imuCalcDataSubscriber_ = nh.subscribe (imuCalcDataTopic_, 10, &AsctecProc::imuCalcDataCallback, this);

    // **** advertise published filtered pointcloud
    imuPublisher_ = nh_private.advertise < sensor_msgs::Imu > (imuTopic_, 10);

  }

  AsctecProc::~AsctecProc ()
  {
    ROS_INFO ("Destroying AsctecProc");

  }

  void AsctecProc::imuCalcDataCallback (const asctec_msgs::IMUCalcDataConstPtr & imuCalcDataMsg)
  {
    // publish imu message
    sensor_msgs::Imu imuMsg;
    createImuMsg (imuCalcDataMsg, imuMsg);
    imuPublisher_.publish (imuMsg);

    // publish tf for testing

    btTransform t;

    btQuaternion orientation;
    orientation.setRPY (imuCalcDataMsg->angle_roll * ASC_TO_ROS_ANGLE,
                        imuCalcDataMsg->angle_nick * ASC_TO_ROS_ANGLE, imuCalcDataMsg->angle_yaw * ASC_TO_ROS_ANGLE);

    t.setRotation (orientation);
    t.setOrigin (btVector3 (0, 0, 0));

    tf::StampedTransform worldToOdomTf (t, ros::Time::now (), "navigation", "imu_raw");
    tfBroadcaster_.sendTransform (worldToOdomTf);

  }

  void AsctecProc::createImuMsg (const asctec_msgs::IMUCalcDataConstPtr & imuCalcDataMsg, sensor_msgs::Imu & imuMsg)
  {
    // set header info
    imuMsg.header.stamp = ros::Time::now ();
    imuMsg.header.frame_id = "imu";

    // copy over linear acceleration
    imuMsg.linear_acceleration.x = imuCalcDataMsg->acc_x_calib * ASC_TO_ROS_ACC;
    imuMsg.linear_acceleration.y = imuCalcDataMsg->acc_y_calib * ASC_TO_ROS_ACC;
    imuMsg.linear_acceleration.z = imuCalcDataMsg->acc_z_calib * ASC_TO_ROS_ACC;

    // define linear acceleration variance
    imuMsg.linear_acceleration_covariance[0] = 1.0;
    imuMsg.linear_acceleration_covariance[1] = 0.0;
    imuMsg.linear_acceleration_covariance[2] = 0.0;
    imuMsg.linear_acceleration_covariance[3] = 0.0;
    imuMsg.linear_acceleration_covariance[4] = 1.0;
    imuMsg.linear_acceleration_covariance[5] = 0.0;
    imuMsg.linear_acceleration_covariance[6] = 0.0;
    imuMsg.linear_acceleration_covariance[7] = 0.0;
    imuMsg.linear_acceleration_covariance[8] = 1.0;

    // copy over angular_velocity
    imuMsg.angular_velocity.x = imuCalcDataMsg->angvel_roll * ASC_TO_ROS_ANGVEL;
    imuMsg.angular_velocity.y = imuCalcDataMsg->angvel_nick * ASC_TO_ROS_ANGVEL;
    imuMsg.angular_velocity.z = imuCalcDataMsg->angvel_yaw * ASC_TO_ROS_ANGVEL;

    // define angular_velocity variance
    imuMsg.angular_velocity_covariance[0] = 1.0;
    imuMsg.angular_velocity_covariance[1] = 0.0;
    imuMsg.angular_velocity_covariance[2] = 0.0;
    imuMsg.angular_velocity_covariance[3] = 0.0;
    imuMsg.angular_velocity_covariance[4] = 1.0;
    imuMsg.angular_velocity_covariance[5] = 0.0;
    imuMsg.angular_velocity_covariance[6] = 0.0;
    imuMsg.angular_velocity_covariance[7] = 0.0;
    imuMsg.angular_velocity_covariance[8] = 1.0;

    // calculate quaternion orientation
    btQuaternion orientation;
    orientation.setRPY (imuCalcDataMsg->angle_roll * ASC_TO_ROS_ANGLE,
                        imuCalcDataMsg->angle_nick * ASC_TO_ROS_ANGLE, imuCalcDataMsg->angle_yaw * ASC_TO_ROS_ANGLE);

    imuMsg.orientation.x = orientation.getX ();
    imuMsg.orientation.y = orientation.getY ();
    imuMsg.orientation.z = orientation.getZ ();
    imuMsg.orientation.w = orientation.getW ();
  }

}                               // end namespace asctec
