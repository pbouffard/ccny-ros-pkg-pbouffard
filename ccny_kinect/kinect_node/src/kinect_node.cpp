/*
 *  Kinect Driver Node (based on libfreenect)
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

