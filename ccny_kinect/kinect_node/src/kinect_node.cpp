#include "kinect_node/kinect.h"

// TODO: get rid of non-member callbacks
//       boost::bind complains 

Kinect * kinect;
void depthimg(uint16_t *buf, int width, int height);
void rgbimg(uint8_t *buf, int width, int height);

int main (int argc, char **argv)
{
  ros::init (argc, argv, "kinect_node");

  libusb_init(NULL);
  libusb_device_handle *dev = libusb_open_device_with_vid_pid(NULL, 0x45e, 0x2ae);
  if (!dev)
  {
    ROS_WARN("Could not open Kinect device, exiting");
    return 1;
  }

  kinect = new Kinect();

  // start acquisition
  cams_init(dev, depthimg, rgbimg);

  // ros loop
  while (ros::ok() && (libusb_handle_events(NULL) == 0))
    ros::spinOnce();

  return 0;
}

void depthimg(uint16_t *buf, int width, int height)
{
  kinect->depthImgCb(buf);
}

void rgbimg(uint8_t *buf, int width, int height)
{
  kinect->rgbImgCb(buf);
}

