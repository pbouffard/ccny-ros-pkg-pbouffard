/*
 *  Kinect Driver (based on libfreenect)
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

#include "kinect_node/kinect.h"

Kinect::Kinect()
{
  ROS_INFO("Creating Kinect");
  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh_private_rgb("~/rgb");
  ros::NodeHandle nh_private_depth("~/depth");
  rgb_cam_info_manager_ = new CameraInfoManager(nh_private_rgb);
  depth_cam_info_manager_ = new CameraInfoManager(nh_private_depth);

  // **** constants

  double horizontalFOVd = 57.0;
	double verticalFOVd   = 43.0;
	double deg2rad = M_PI/180.0;

	horizontalFOV_ = horizontalFOVd * deg2rad;
	verticalFOV_   = verticalFOVd   * deg2rad;

  rgbBuf_ = NULL;
  depthBuf_ = NULL;

  depthSent_ = false;
  rgbSent_ = false; 

  haveMatrix_ = false;

  // **** parameters

  if (!nh_private.getParam ("kinect_rgb_frame", kinectRGBFrame_))
    kinectRGBFrame_ = "/kinect_rgb";
  if (!nh_private.getParam ("kinect_depth_frame", kinectDepthFrame_))
    kinectDepthFrame_ = "/kinect_depth";
  if (!nh_private.getParam ("max_range", maxRange_))
    maxRange_ = 5.0;
  if (!nh_private.getParam ("width", width_))
    width_ = 640;
  if (!nh_private.getParam ("height", height_))
    height_ = 480;
  nh_private.param("camera_name", cam_name_, std::string("camera"));
  nh_private.param("rgb_camera_info_url",rgb_cam_info_url_,std::string("auto"));
  nh_private.param("depth_camera_info_url",depth_cam_info_url_,std::string("auto"));
  if (rgb_cam_info_url_.compare("auto") == 0) {
    rgb_cam_info_url_ = std::string("file://")+ros::package::getPath(ROS_PACKAGE_NAME)+std::string("/info/rgb_calibration.yaml");
  }
  if (depth_cam_info_url_.compare("auto") == 0) {
    depth_cam_info_url_ = std::string("file://")+ros::package::getPath(ROS_PACKAGE_NAME)+std::string("/info/depth_calibration.yaml");
  }
  ROS_INFO("RGB Calibration URL: %s",rgb_cam_info_url_.c_str());
  ROS_INFO("Depth Calibration URL: %s",depth_cam_info_url_.c_str());
  if (rgb_cam_info_manager_->validateURL(rgb_cam_info_url_)) {
    rgb_cam_info_manager_->loadCameraInfo(rgb_cam_info_url_);
  } else {
    ROS_ERROR("Invalid RGB Calibration URL");
    ROS_BREAK();
  }
  if (depth_cam_info_manager_->validateURL(depth_cam_info_url_)) {
    depth_cam_info_manager_->loadCameraInfo(depth_cam_info_url_);
  } else {
    ROS_ERROR("Invalid Depth Calibration URL");
    ROS_BREAK();
  }
  rgb_cam_info_manager_->setCameraName(cam_name_);
  depth_cam_info_manager_->setCameraName(cam_name_);


  // **** publishers and subscribers

  image_transport::ImageTransport it(nh_private);
  rgbImagePub_ = it.advertiseCamera("rgb/image_raw", 1);
  depthImagePub_ = it.advertiseCamera("depth/image_raw", 1);
  //depthImagePub_ = nh_private.advertise<sensor_msgs::Image>("depth_image_raw", 1);

  //point_cloud::Publisher<pcl::PointXYZRGB> pub(nh_private, "cloud", 16);

  //pointCloudPub_ = nh_private.advertise<pcl::PointXYZRGB>("cloud", 16);
  pointCloudPub_.advertise (nh_private, "cloud", 16);
}

Kinect::~Kinect()
{
  ROS_INFO("Destroying Kinect");
}

void Kinect::depthImgCb(uint16_t *buf)
{
  bufferMutex_.lock();

  ROS_DEBUG("Depth Img Callback");

  depthSent_ = false;

  if (depthBuf_) delete depthBuf_;
  depthBuf_ = new uint16_t[width_*height_];
  memcpy(depthBuf_, buf, width_*height_*sizeof(uint16_t));

  if (rgbBuf_ && !rgbSent_) publish();

  bufferMutex_.unlock();
}

void Kinect::rgbImgCb(uint8_t *buf)
{
  bufferMutex_.lock();

  ROS_DEBUG("RGB Img Callback");

  rgbSent_ = false;
  if (rgbBuf_) delete rgbBuf_;
  rgbBuf_ = new uint8_t[width_*height_*3];
  memcpy(rgbBuf_, buf, width_*height_*3);

  if (depthBuf_ && !depthSent_) publish();

  bufferMutex_.unlock();
}

void Kinect::depthBufferTo8BitImage(const uint16_t *depthBuf, sensor_msgs::Image& image)
{
  image.header.frame_id = kinectDepthFrame_;
	image.height = height_;
	image.width = width_;
	image.encoding = "rgb8";
	image.step = width_ * 3;
	for (int y=0; y<height_; y+=1)
  for (int x=0; x<width_; x+=1) 

  {
    int index(y*width_ + x);
    int reading = depthBuf_[index];
    double range = readingToRange(reading);

    if (range > maxRange_ || range < 0) 
      range = 0;

    uint8_t color = rangeTo8BitColor(range);
    image.data.push_back(color);
    image.data.push_back(color);
    image.data.push_back(color);
  }
}

double Kinect::readingToRange(int reading) const
{
  double range = -325.616 / ((double)reading + -1084.61);
  return range;
}

uint8_t Kinect::rangeTo8BitColor(double range) const
{
  uint8_t color = 255 * range / maxRange_;
  if (color == 0) color = 255;
  return color;
}

void Kinect::publish()
{
  if (!haveMatrix_)
  {
    createRectificationMatrix();
    haveMatrix_ = true;
  }

  ros::Time time = ros::Time::now();



  // **** publish depth image

  sensor_msgs::Image depthImage;
  depthImage.header.stamp = time;
  depthBufferTo8BitImage(depthBuf_, depthImage);

  // **** publish point cloud

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.header.stamp = time;
	cloud.header.frame_id = kinectDepthFrame_;

  cloud.width = width_/2;
  cloud.height = height_/2;
  cloud.points.resize(cloud.width * cloud.height);

	for (int y=0; y<height_; y+=2)
	for (int x=0; x<width_; x+=2) 
  {
    int index(y*width_ + x);
    int reading = depthBuf_[index];

    if (reading  >= 2048 || reading <= 0) continue;

    double range = readingToRange(reading);
  
    cv::Point3d rectRay = rectMatrix_[(y/2)*(width_/2) + x/2];

    rectRay *= range;

    if (range > maxRange_ || range <= 0) continue;

    // **** project cloud point onto rgb image frame

    btTransform depthToRgbTf;
    depthToRgbTf.setIdentity();
    depthToRgbTf.setOrigin(btVector3(-0.03, 0, 0));

    // ****

    pcl::PointXYZRGB point;
    point.x = rectRay.x;
    point.y = rectRay.y;
    point.z = rectRay.z;

    uint8_t r = rgbBuf_[index*3+0];
    uint8_t g = rgbBuf_[index*3+1];
    uint8_t b = rgbBuf_[index*3+2];
    int32_t rgb = (r << 16) | (g << 8) | b;
    point.rgb = *(float*)(&rgb);

  	cloud.points[(y/2)*(width_/2) + x/2] = point;
  }

  // **** publish RGB Image

	sensor_msgs::Image image;
	image.header.stamp = time;
	image.header.frame_id = kinectRGBFrame_;
	image.height = height_;
	image.width = width_;
	image.encoding = "rgb8";
	image.step = width_ * 3;
	image.data.reserve(width_*height_*3);
  
	copy(rgbBuf_, rgbBuf_ + width_*height_*3, back_inserter(image.data));

  // **** publish RGB Camera Info
  rgb_cam_info_ = rgb_cam_info_manager_->getCameraInfo();
  if (rgb_cam_info_.height != (unsigned int)image.height
      || rgb_cam_info_.width != (unsigned int)image.width) {
    ROS_DEBUG_THROTTLE(60,"Uncalibrated RGB Camera");
  }
  rgb_cam_info_.header.stamp = image.header.stamp;
  rgb_cam_info_.height = image.height;
  rgb_cam_info_.width = image.width;
  rgb_cam_info_.header.frame_id = image.header.frame_id;

  // **** publish Depth Camera Info
  depth_cam_info_ = depth_cam_info_manager_->getCameraInfo();
  if (depth_cam_info_.height != (unsigned int)image.height
      || depth_cam_info_.width != (unsigned int)image.width) {
    ROS_DEBUG_THROTTLE(60,"Uncalibrated Depth Camera");
  }
  depth_cam_info_.header.stamp = image.header.stamp;
  depth_cam_info_.height = image.height;
  depth_cam_info_.width = image.width;
  depth_cam_info_.header.frame_id = kinectDepthFrame_;

  // **** publish the messages

	pointCloudPub_.publish(cloud);
	rgbImagePub_.publish(image, rgb_cam_info_); 
	depthImagePub_.publish(depthImage, depth_cam_info_); 
  // depthImagePub_.publish(depthImage);

  rgbSent_   = true;
  depthSent_ = true;
}

void Kinect::createRectificationMatrix()
{
  image_geometry::PinholeCameraModel pcm;
  pcm.fromCameraInfo(depth_cam_info_manager_->getCameraInfo());

  rectMatrix_ = new cv::Point3d[height_/2 * width_/2];

  for (int y=0; y<height_; y+=2)
	for (int x=0; x<width_; x+=2) 
  {
    int index = y*width_ + x;

    cv::Point2d rawPoint(x, y);
    cv::Point2d rectPoint;
    cv::Point3d rectRay;

    pcm.rectifyPoint(rawPoint, rectPoint);
    pcm.projectPixelTo3dRay(rectPoint, rectRay);

    rectRay *= 1.0/rectRay.z;

    rectMatrix_[(y/2)*(width_/2) + x/2] = rectRay;
  }
}
