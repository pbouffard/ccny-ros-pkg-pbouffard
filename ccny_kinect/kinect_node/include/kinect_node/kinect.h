/*
 *  Kinect Driver Header (based on libfreenect)
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  William Morris <morris@ee.ccny.cuny.edu>
 *  Stéphane Magnenat <stephane.magnenat@mavt.ethz.ch>
 *
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
