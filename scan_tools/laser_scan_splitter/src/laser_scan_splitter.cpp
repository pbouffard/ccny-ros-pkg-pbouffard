/*
*  Laser Scan Splitter
*  Copyright (C) 2010, CCNY Robotics Lab
*  Ivan Dryanovski <ivan.dryanovski@gmail.com>
*  William Morris <morris@ee.ccny.cuny.edu>
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

#include "laser_scan_splitter/laser_scan_splitter.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "laser_scan_splitter");
  LaserScanSplitter laserScanSplitter;
  ros::spin ();
  return 0;
}

LaserScanSplitter::LaserScanSplitter ()
{
  ROS_INFO ("Starting LaserScanSplitter");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private ("~");

  // **** get paramters

  std::string sizesString;
  std::string framesString;
  std::string topicsString;

  if (!nh_private.getParam ("topics", topicsString))
    topicsString = "scan1 scan2";
  if (!nh_private.getParam ("frames", framesString))
    framesString = "laser laser";
  if (!nh_private.getParam ("sizes", sizesString))
    sizesString = "256 256";

  // **** tokenize inputs
  tokenize (topicsString, publishedScanTopics_);
  tokenize (framesString, publishedLaserFrames_);

  std::vector < std::string > sizesTokens;
  tokenize (sizesString, sizesTokens);

  sizeSum_ = 0;
  for (unsigned int i = 0; i < sizesTokens.size (); i++)
  {
    sizes_.push_back (atoi (sizesTokens[i].c_str ()));
    sizeSum_ += sizes_[i];

    ROS_ASSERT_MSG ((sizes_[i] > 0), "LaserScanSplitter: Scan size cannot be zero. Quitting.");
  }

  // **** check that topic, frames, and sizes vectors have same sizes

  ROS_ASSERT_MSG ((publishedScanTopics_.size () == publishedLaserFrames_.size ()) &&
                  (sizes_.size () == publishedLaserFrames_.size ()),
                  "LaserScanSplitter: Invalid parameters. Quitting.");

  // **** subscribe to laser scan messages
  scanSubscriber_ = nh.subscribe (scanTopic_, 100, &LaserScanSplitter::scanCallback, this);

  // **** advertise topics
  for (unsigned int i = 0; i < publishedScanTopics_.size (); i++)
  {
    scanPublishers_.push_back (ros::Publisher ());
    scanPublishers_[i] = nh_private.advertise < sensor_msgs::LaserScan > (publishedScanTopics_[i], 10);
  }
}

LaserScanSplitter::~LaserScanSplitter ()
{

}

void LaserScanSplitter::scanCallback (const sensor_msgs::LaserScanConstPtr & scan)
{

  // **** check for scan size

  if (sizeSum_ != scan->ranges.size ())
  {
    ROS_WARN ("LaserScanSplitter: Received a laser scan with size (%d) incompatible to input parameters. Skipping scan.", scan->ranges.size());
    return;
  }

  // **** copy information over
  int r = 0;

  for (unsigned int i = 0; i < publishedScanTopics_.size (); i++)
  {
    sensor_msgs::LaserScan scanSegment;

    scanSegment.header = scan->header;
    scanSegment.range_min = scan->range_min;
    scanSegment.range_max = scan->range_max;
    scanSegment.angle_increment = scan->angle_increment;
    scanSegment.time_increment = scan->time_increment;
    scanSegment.scan_time = scan->scan_time;
    scanSegment.header.frame_id = publishedLaserFrames_[i];

    scanSegment.angle_min = scan->angle_min + (scan->angle_increment * r);
    scanSegment.angle_max = scan->angle_min + (scan->angle_increment * (r + sizes_[i] - 1));

    // TODO - also copy intensity values

    for (int ii = 0; ii < sizes_[i]; ii++)
    {
      scanSegment.ranges.push_back (scan->ranges[r]);
      r++;
    }

    scanPublishers_[i].publish (scanSegment);
  }
}

void LaserScanSplitter::tokenize (const std::string & str, std::vector < std::string > &tokens)
{
  std::string::size_type lastPos = str.find_first_not_of (" ", 0);
  std::string::size_type pos = str.find_first_of (" ", lastPos);

  while (std::string::npos != pos || std::string::npos != lastPos)
  {
    std::string stringToken = str.substr (lastPos, pos - lastPos);
    tokens.push_back (stringToken);
    lastPos = str.find_first_not_of (" ", pos);
    pos = str.find_first_of (" ", lastPos);
  }
}
