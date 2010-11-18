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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <geometry_msgs/Point32.h>

#include <tf/transform_listener.h>

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include <pcl/point_types.h>
#include <pcl/ros/publisher.h>

#include <image_geometry/pinhole_camera_model.h>

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
    image_transport::CameraPublisher rgbImagePub_;
    image_transport::CameraPublisher depthImagePub_;
    //image_transport::CameraPublisher depthImagePub_;
    point_cloud::Publisher<pcl::PointXYZRGB> pointCloudPub_;

    //ros::Publisher depthImagePub_;

    std::string kinectRGBFrame_;
    std::string kinectDepthFrame_;
    double maxRange_;

    double horizontalFOV_;
	  double verticalFOV_;
    
    int width_;
    int height_;

    void publish();

    void depthBufferTo8BitImage(const uint16_t *depthBuf, sensor_msgs::Image& image);
    double readingToRange(int reading) const;
    uint8_t rangeTo8BitColor(double range) const;

    void createRectificationMatrix();
    bool haveMatrix_;
    cv::Point3d * rectMatrix_;

  public:

    std::string cam_name_;
    std::string rgb_cam_info_url_;
    std::string depth_cam_info_url_;
    sensor_msgs::CameraInfo rgb_cam_info_;
    CameraInfoManager *rgb_cam_info_manager_;
    sensor_msgs::CameraInfo depth_cam_info_;
    CameraInfoManager *depth_cam_info_manager_;

    boost::mutex rgbMutex_;
    boost::mutex depthMutex_;

    Kinect();
    virtual ~Kinect();

    void depthImgCb(uint16_t *buf);
    void rgbImgCb  (uint8_t  *buf);
};

#endif //KINECT_NODE_KINECT_H
