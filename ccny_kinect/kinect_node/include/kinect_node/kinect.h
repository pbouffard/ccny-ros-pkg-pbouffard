#ifndef KINECT_NODE_KINECT_H
#define KINECT_NODE_KINECT_H

#include <libusb.h>
#include <algorithm>

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

extern "C" {
#include "libfreenect.h"
}

class Kinect
{
  private:

    boost::mutex bufferMutex_;

    uint16_t *depthBuf_;
    uint8_t  *rgbBuf_;

    bool depthSent_;
    bool rgbSent_; 

    //ros::Publisher rgbImagePub;
    image_transport::CameraPublisher rgb_image_pub_;
    ros::Publisher pointCloudPub;

    std::string kinectFrame_;
    double maxRange_;

    double horizontalFOV_;
	  double verticalFOV_;
    
    int width_;
    int height_;

    void publish();

  public:

    std::string cam_name_;
    std::string cam_info_url_;
    sensor_msgs::CameraInfo cam_info_;
    CameraInfoManager *cam_info_manager_;

    boost::mutex rgbMutex_;
    boost::mutex depthMutex_;

    Kinect();
    virtual ~Kinect();

    void depthImgCb(uint16_t *buf);
    void rgbImgCb  (uint8_t  *buf);
};

#endif //KINECT_NODE_KINECT_H
