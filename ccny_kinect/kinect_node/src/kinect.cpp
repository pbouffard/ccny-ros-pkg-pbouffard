#include "kinect_node/kinect.h"

Kinect::Kinect()
{
  ROS_INFO("Creating Kinect");
  ros::NodeHandle nh_private("~");

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

  // **** parameters

  if (!nh_private.getParam ("kinect_frame", kinectFrame_))
    kinectFrame_ = "/kinect";
  if (!nh_private.getParam ("kinect_frame", maxRange_))
    maxRange_ = 5.0;
  if (!nh_private.getParam ("width", width_))
    width_ = 640;
  if (!nh_private.getParam ("height", height_))
    height_ = 480;

  // **** publishers and subscribers

  rgbImagePub   = nh_private.advertise<sensor_msgs::Image>("rgb_image", 16);
  pointCloudPub = nh_private.advertise<sensor_msgs::PointCloud>("cloud", 16);
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

void Kinect::publish()
{
  ros::Time time = ros::Time::now();

  // **** publish point cloud

	sensor_msgs::PointCloud cloud;
	cloud.header.stamp = time;
	cloud.header.frame_id = kinectFrame_;

	for (int x=0; x<width_; x+=2) 
	for (int y=0; y<height_; y+=2)
  {
    int index(y*width_ + x);
    int reading = depthBuf_[index];

    if (reading  >= 2048 || reading <= 0) continue;

    int px = x - width_/2;
    int py = y - height_/2;
  
    double wx = px * (horizontalFOV_ / (double)width_);
    double wy = py * (horizontalFOV_ / (double)height_);
    double wz = 1.0;
  
    double range = -325.616 / ((double)reading + -1084.61);
    
    if (range > maxRange_ || range <= 0) continue;

    wx*=range;
    wy*=range;
    wz*=range;

	  geometry_msgs::Point32 point;
    point.x = wx;
    point.y = wy;
    point.z = wz;
  	cloud.points.push_back(point);
  }

  // **** publish RGB Image

	sensor_msgs::Image image;
	image.header.stamp = time;
	image.header.frame_id = kinectFrame_;
	image.height = height_;
	image.width = width_;
	image.encoding = "rgb8";
	image.step = width_ * 3;
	image.data.reserve(width_*height_*3);
  
	copy(rgbBuf_, rgbBuf_ + width_*height_*3, back_inserter(image.data));

  // **** publish the messages

	pointCloudPub.publish(cloud);
	rgbImagePub.publish(image); 

  rgbSent_   = true;
  depthSent_ = true;
}
